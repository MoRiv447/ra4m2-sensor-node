/*
 * hal_entry.c —— 双无线环境监测节点主程序（RA4M2-SENSOR）。
 *
 * 五路数据源：BH1750（I2C 光照）/ BMP280（I2C 气压）/ DHT22（单总线）/
 * P014 分压基准（ADC）/ 片内温度 TSN；三路出口：串口、蓝牙、WiFi-MQTT。
 *
 * 节拍：GPT 500ms 中断推格 —— ADC 0.5s / I2C 3s / DHT 4s / 汇总 10s；
 * tick 之间由 power_gear 决定忙等 / WFI / 软件待机。
 *
 * 注意：`R_BSP_WarmStart()` 在 src/hal_warmstart.c（FSP flat 工程约定），
 * 本文件不要再定义一份，否则链接期重复符号。
 *
 * © 2026 MoRiv447 · MIT License
 */

#include "hal_data.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <sys/stat.h>

#include "app_config.h"
#include "app_util.h"
#include "sensors.h"
#include "wifi_bridge.h"
#include "lpm_test.h"
#include "power_gear.h"

/* ==================================================================
 * CTSU 触摸：第 2 篇的主要内容；第 4 篇"触摸换挡"重新启用 ——
 * 两个焊盘（TS1/TS2）现在是功耗档位的"换挡拨片"，逻辑在 power_gear.c。
 * 不想要触摸时改回 0（qe_gen 的文件仍在工程里，随开随用）。
 * ================================================================== */
#define APP_ENABLE_CTSU     1

#if APP_ENABLE_CTSU
#include "qe_touch_config.h"
extern uint64_t button_status;
#endif

/* ==================================================================
 * newlib syscall stub
 *
 * 一旦自己实现了 _write，newlib 就认为用户接管了 syscall，
 * 于是连带要求下面这几个符号也存在。它们一个都用不到（只走串口输出），
 * 但链接器不管"用没用到"，符号找不到就报 undefined reference。
 * 两种解法：① 链接选项加 -lnosys；② 代码里给空实现。
 * 这里选 ② —— 不用动 IDE 的链接配置，换台机器也好使。
 * ================================================================== */
int _close (int fd);
int _lseek (int fd, int offset, int whence);
int _read (int fd, char * buf, int len);
int _fstat (int fd, struct stat * st);
int _isatty (int fd);

int _close (int fd)
{
    FSP_PARAMETER_NOT_USED(fd);
    return -1;
}

int _lseek (int fd, int offset, int whence)
{
    FSP_PARAMETER_NOT_USED(fd);
    FSP_PARAMETER_NOT_USED(offset);
    FSP_PARAMETER_NOT_USED(whence);
    return 0;
}

int _read (int fd, char * buf, int len)
{
    FSP_PARAMETER_NOT_USED(fd);
    FSP_PARAMETER_NOT_USED(buf);
    FSP_PARAMETER_NOT_USED(len);
    return 0;
}

int _fstat (int fd, struct stat * st)
{
    FSP_PARAMETER_NOT_USED(fd);
    /* 伪装成字符设备，printf 靠这个判断要不要做缓冲 */
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty (int fd)
{
    FSP_PARAMETER_NOT_USED(fd);
    return 1;
}

/* ==================================================================
 * printf 重定向（走 g_uart9）
 *
 * 依赖 FSP 里 UART 的 Callback = user_uart_callback，没有它这个 while 会死等。
 * 注意：本工程的 printf 是阻塞的，每发一个字节等一次 TX 完成中断 ——
 * 输出过多会拖慢主循环。所以汇总打印周期取 10s，不是每格都打。
 * ================================================================== */
static volatile bool g_uart_tx_done = false;

void user_uart_callback (uart_callback_args_t * p_args)
{
    if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        g_uart_tx_done = true;
    }
}

int __io_putchar (int ch);
int _write (int fd, char * pBuffer, int size);

int __io_putchar (int ch)
{
    fsp_err_t err = R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *) &ch, 1);
    if (FSP_SUCCESS != err)
    {
        __BKPT();
    }
    while (false == g_uart_tx_done)
    {
        ;
    }
    g_uart_tx_done = false;
    return ch;
}

int _write (int fd, char * pBuffer, int size)
{
    FSP_PARAMETER_NOT_USED(fd);
    for (int i = 0; i < size; i++)
    {
        __io_putchar(*pBuffer++);
    }
    return size;
}

/* ==================================================================
 * 中断回调
 * ================================================================== */

/* 非 static：sensor_ldr.c 里的 adc_burst_read 要用这个标志等转换完成 */
volatile bool g_adc_done = false;

void adc_callback (adc_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_adc_done = true;
}

static volatile bool     g_rtc_tick  = false;   /* RTC 1Hz，用来打时间戳 */
static volatile uint32_t g_gpt_tick  = 0U;      /* GPT 0.5s，采集节拍 */

void rtc_callback (rtc_callback_args_t * p_args)
{
    if (RTC_EVENT_PERIODIC_IRQ == p_args->event)
    {
        g_rtc_tick = true;
    }
}

void gpt_callback (timer_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    g_gpt_tick++;
}

/* ==================================================================
 * 片内温度（TSN）—— 第 2 篇的内容，这里保留当第 5 路数据源
 * ================================================================== */
#define TSN_SLOPE_MV_PER_C  4.0f        /* 4000 μV/℃，见 bsp_feature.h */
#define TSN_CAL_TEMP_C      127.0f      /* TSCDR 对应的温度点 */

static bool read_chip_temp (float * out_c)
{
    uint16_t adc = 0U;

    g_adc_done = false;
    if (FSP_SUCCESS != R_ADC_ScanStart(&g_adc0_ctrl))
    {
        return false;
    }

    uint32_t waited = 0U;
    while ((false == g_adc_done) && (waited < 100U))
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        waited++;
    }
    if (false == g_adc_done)
    {
        return false;
    }

    if (FSP_SUCCESS != R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_TEMPERATURE, &adc))
    {
        return false;
    }

    /* 每 LSB 的电压 × 读数差 = 电压差，再除以斜率得温差 */
    uint32_t cal     = R_TSN_CAL->TSCDR & 0xFFFFUL;
    float    mv_lsb  = APP_ADC_VREF_MV / APP_ADC_FULL_SCALE;
    float    delta_c = ((float) adc - (float) cal) * mv_lsb / TSN_SLOPE_MV_PER_C;

    *out_c = TSN_CAL_TEMP_C + delta_c;
    return true;
}

/* ==================================================================
 * 全局数据快照
 * ================================================================== */

static bh1750_data_t g_light = { .valid = false, .lux = 0.0f, .addr = 0U };
static bmp280_data_t g_press = { .valid = false };
static dht_data_t    g_dht   = { .valid = false };
static ldr_data_t    g_ldr   = { .valid = false };
static float         g_chip_temp_c = 0.0f;
static bool          g_chip_temp_ok = false;

static uint32_t g_sample_no = 0U;
static uint32_t g_dht_fail  = 0U;      /* DHT 累计失败次数，用来判断接线 */

/* 模拟通道 B（P015/AN013）—— 预留给 MQ-2 等第二路模拟量。
 * 没接线时读数会漂，所以单独记一个 ok 标志，不接线就报 n/a 而不是乱给数字。 */
static uint16_t g_adc_b_raw   = 0U;
static float    g_adc_b_mv    = 0.0f;
static float    g_adc_b_pp    = 0.0f;
static float    g_adc_b_sigma = 0.0f;
static bool     g_adc_b_ok    = false;
/* P015 上挂的是光敏电阻（LDR）分压：3V3─[LDR]─┬─P015 / └─[10k]─GND，
 * 与通道 A 的固定分压并存：A 继续当"已知基准"，B 这一路给光照。 */
static float    g_adc_b_r_ohm = -1.0f;
static float    g_adc_b_lux   = 0.0f;
static bool     g_adc_b_used  = (1 == APP_ADC_B_ENABLE);

/* ==================================================================
 * 蓝牙（BLE）链路 —— SCI0：P101(TXD0) → 模块 RXD、P100(RXD0) ← 模块 TXD
 * 铁律：发给模块的数据不能以 "AT" 开头（会被当命令吃掉）；本链路不用 printf。
 * 协议：每 5s 播报 [RA4M2] lux=.. ldr=.. T=.. RH=..；read→回数据；ping→pong；其它→echo。
 * ================================================================== */
#define BT_LINE_MAX         90U

static volatile bool    g_bt_tx_busy    = false;    /* 上一笔发送未完成 */
static uint8_t          g_bt_rx_byte    = 0U;       /* 单字节接收落点 */
static volatile char    g_bt_line[BT_LINE_MAX + 1U];/* 行组装缓冲（中断写） */
static volatile uint8_t g_bt_line_len   = 0U;
static volatile bool    g_bt_line_ready = false;    /* 收到整行，等主循环处理 */
static uint32_t         g_bt_drop       = 0U;       /* busy 丢帧计数 */

/**
 * @brief 非阻塞发一行。忙则丢弃本帧 —— 周期播报场景可以接受，
 *        远好过在主循环里等待（会拖慢整个采集节拍）。
 *
 * ⚠️ FSP 的 UART 写是"中断驱动 + 引用传入缓冲"的：发送完成前那块内存必须
 *    保持有效。调用方给的常是栈上缓冲，函数一返回就被后续代码覆写 ——
 *    实测症状是"前半句正常、后半句乱码"。所以这里先拷贝进静态缓冲再发；
 *    忙时丢帧的策略顺手保证上一帧没发完时不会被新帧覆盖。
 */
static void bt_send (const char * str)
{
    static char s_tx_buf[128];
    size_t      len = strlen(str);

    if (g_bt_tx_busy)
    {
        g_bt_drop++;
        return;
    }
    if (len > (sizeof(s_tx_buf) - 1U))
    {
        len = sizeof(s_tx_buf) - 1U;    /* 超长截断（正常路径到不了这里） */
    }
    memcpy(s_tx_buf, str, len);
    s_tx_buf[len] = '\0';

    /* 先置忙、再启动发送：保证 TX_COMPLETE 回调一定发生在置忙之后 */
    g_bt_tx_busy = true;
    if (FSP_SUCCESS != R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t const *) s_tx_buf,
                                        (uint32_t) len))
    {
        g_bt_tx_busy = false;           /* 没启动成功：立刻释放，下帧再试 */
    }
}

/**
 * @brief 把当前数据快照格式化一行发出去。
 *        buf 尺寸按最长输出估：头部 + 各字段 + 结尾，104 足够。
 */
static void bt_report (void)
{
    char   buf[104];
    size_t n = (size_t) snprintf(buf, sizeof(buf), "[RA4M2]");

    if (g_light.valid)
    {
        int32_t lx10 = (int32_t) (g_light.lux * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, " lux=%ld.%01ld",
                               (long) (lx10 / 10), (long) (lx10 % 10));
    }
    else
    {
        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, " lux=na");
    }

    /* LDR（P015 模拟通道 B）的相对照度，与 BH1750 同发一份便于对照量级 */
    if (g_adc_b_used && g_adc_b_ok)
    {
        int32_t lx10 = (int32_t) (g_adc_b_lux * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, " ldr=%ld.%01ld",
                               (long) (lx10 / 10), (long) (lx10 % 10));
    }
    else
    {
        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, " ldr=na");
    }

    if (g_dht.valid)
    {
        int32_t      t10 = (int32_t) (g_dht.temp_c * 10.0f +
                                      ((g_dht.temp_c >= 0.0f) ? 0.5f : -0.5f));
        const char * ts  = (t10 < 0) ? "-" : "";
        int32_t      ta  = (t10 < 0) ? -t10 : t10;
        int32_t      h10 = (int32_t) (g_dht.rh_pct * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n,
                               " T=%s%ld.%01ldC RH=%ld.%01ld%%",
                               ts, (long) (ta / 10), (long) (ta % 10),
                               (long) (h10 / 10), (long) (h10 % 10));
    }

    if (n < (sizeof(buf) - 3U))
    {
        buf[n]     = '\r';
        buf[n + 1] = '\n';
        buf[n + 2] = '\0';
    }
    bt_send(buf);
}

/**
 * @brief 处理一帧手机下发的命令（主循环上下文，line 已是私有拷贝）
 */
static void bt_handle_line (const char * line)
{
    if (0 == strcmp(line, "read"))
    {
        bt_report();
    }
    else if (0 == strcmp(line, "ping"))
    {
        bt_send("pong\r\n");
    }
    else
    {
        /* 回显：联调时用来确认"这条线是谁发的" */
        char buf[BT_LINE_MAX + 12U];

        (void) snprintf(buf, sizeof(buf), "echo: %s\r\n", line);
        bt_send(buf);
    }
}

/**
 * @brief SCI0（蓝牙）收发回调。接收是"单字节重挂"模式：
 *        每收到 1 字节触发一次，回调里把下一字节的读取重新挂上。
 */
void bt_uart_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_COMPLETE:
        {
            char c = (char) g_bt_rx_byte;

            if (('\r' != c) && ('\n' != c))
            {
                if (g_bt_line_ready)
                {
                    /* 上一行还没被主循环取走：重启组行（丢旧保新） */
                    g_bt_line_ready = false;
                    g_bt_line_len   = 0U;
                }
                if (g_bt_line_len < BT_LINE_MAX)
                {
                    g_bt_line[g_bt_line_len] = c;
                    g_bt_line_len++;
                }
            }
            else if ((0U != g_bt_line_len) && (!g_bt_line_ready))
            {
                g_bt_line[g_bt_line_len] = '\0';    /* 整行就绪 */
                g_bt_line_ready = true;
            }

            (void) R_SCI_UART_Read(&g_uart0_ctrl, &g_bt_rx_byte, 1U);
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            g_bt_tx_busy = false;
            break;
        }

        default:
        {
            /* 错误类事件：清忙标志 + 重挂接收，别让链路卡死 */
            g_bt_tx_busy = false;
            (void) R_SCI_UART_Read(&g_uart0_ctrl, &g_bt_rx_byte, 1U);
            break;
        }
    }
}

/* ==================================================================
 * 自检
 * ================================================================== */

static void print_banner (void)
{
    printf("\r\n\r\n");
    printf("========================================\r\n");
    printf("  RA4M2-SENSOR  Stage 3: Multi-Sensor\r\n");
    printf("  Build %s %s\r\n", __DATE__, __TIME__);
    printf("========================================\r\n");
}

/**
 * @brief 上电自检：把每一条链路的"通/不通"暴露出来
 *
 * 这一段的输出就是帖子里「接线检查」那几张截图 ——
 * 与其等读数不对了再逐个排查，不如开机就把结论打出来。
 */
static void self_test (void)
{
    /* ---- I2C 总线：接线对不对，一次扫描就能看出来 ---- */
    if (FSP_SUCCESS != i2c_bus_init())
    {
        printf("[I2C] bus open FAILED\r\n");
    }
    else
    {
        (void) i2c_bus_scan();
        i2c_diag();     /* 软件 I2C 逐段诊断（每项 3 遍，看稳定性） */

        if (FSP_SUCCESS != bh1750_init())
        {
            printf("[BH1750] unavailable (will report n/a)\r\n");
        }
        if (FSP_SUCCESS != bmp280_init())
        {
            printf("[BMP280] unavailable (will report n/a)\r\n");
        }
        else
        {
            /* 读数离谱时靠这段定位：数据寄存器、校准系数、20 位结果全打出来。
             * 正常时它这几行也是"器件真的在回数据"的证据，留着不碍事。 */
            bmp280_dump_debug();
        }
    }

    /* ---- 单总线 ---- */
    (void) dht_init();

    /* ---- 模拟通道 ---- */
    (void) ldr_init();

    /* 逐条通道单独测一遍。放在 ldr_init 之后不是多此一举：
     * ldr_init 只报 P014 一条，而"线插错脚"这种情况必须两条一起看
     * 才分得清 —— 哪条显示"接上了"，线就在那个脚上。 */
    adc_probe_channels();

    printf("----------------------------------------\r\n");
}

/* ==================================================================
 * 上报
 * ================================================================== */

#if APP_OUT_CSV
/* CSV 模式：一行一条，表头只打一次。
 * 直接贴进 Excel / 上位机就能画曲线，适合长时间稳定性记录。 */
static void print_header_csv (void)
{
    printf("ms,sample,adc_a_raw,adc_a_mv,adc_a_err_lsb,adc_a_pp,adc_a_sigma,"
           "adc_b_raw,adc_b_mv,adc_b_lux,"
           "bh1750_lux,bmp_t_c,bmp_p_hpa,dht_t_c,dht_rh_pct,chip_t_c\r\n");
}

static void report_csv (uint32_t tick)
{
    printf("%lu,%lu,", (unsigned long) (tick * APP_TICK_MS),
           (unsigned long) g_sample_no);

    printf("%u,", (unsigned) g_ldr.raw);
    app_print_f1(g_ldr.volt * 1000.0f);
    printf(",");
    app_print_f2(g_ldr.error_lsb);
    printf(",");
    app_print_f1(g_ldr.pp);
    printf(",");
    app_print_f2(g_ldr.sigma);
    printf(",");

    if (g_adc_b_used && g_adc_b_ok)
    {
        printf("%u,", (unsigned) g_adc_b_raw);
        app_print_f1(g_adc_b_mv);
        printf(",");
        app_print_f1(g_adc_b_lux);
    }
    else
    {
        printf(",,");
    }
    printf(",");

    if (g_light.valid)
    {
        app_print_f1(g_light.lux);
    }
    printf(",");

    if (g_press.valid)
    {
        app_print_f2(g_press.temp_c);
        printf(",");
        app_print_f2(g_press.press_hpa);
    }
    else
    {
        printf(",");
    }
    printf(",");

    if (g_dht.valid)
    {
        app_print_f1(g_dht.temp_c);
        printf(",");
        app_print_f1(g_dht.rh_pct);
    }
    else
    {
        printf(",");
    }
    printf(",");

    if (g_chip_temp_ok)
    {
        app_print_f2(g_chip_temp_c);
    }

    printf("\r\n");
}
#endif /* APP_OUT_CSV */

/* 表格模式：人类可读，适合截图。每 10s 打一整块 —— 见 report_all() */

static void report_all (uint32_t tick)
{
#if APP_OUT_CSV
    report_csv(tick);
#else
    printf("\r\n--- sample #%lu   t = %lu.%01lus ---\r\n",
           (unsigned long) g_sample_no,
           (unsigned long) ((tick * APP_TICK_MS) / 1000U),
           (unsigned long) (((tick * APP_TICK_MS) % 1000U) / 100U));

    /* ---- 模拟通道 A：raw / 电压 /（固定分压时）精度误差 ---- */
#if (0 == APP_ADC_A_INPUT)
    printf("[ADC-A ] P014 raw=%u  ", (unsigned) g_ldr.raw);
    app_print_f1(g_ldr.volt * 1000.0f);
    printf(" mV   expect ");
    app_print_f1(g_ldr.expect_mv);
    printf(" mV   err ");
    app_print_f2(g_ldr.error_lsb);
    printf(" LSB\r\n");
#else
    printf("[ADC-A ] P014 raw=%u  ", (unsigned) g_ldr.raw);
    app_print_f1(g_ldr.volt * 1000.0f);
    printf(" mV  ");
    if (g_ldr.r_ohm > 0.0f)
    {
        app_print_f1(g_ldr.r_ohm / 1000.0f);
        printf(" kOhm");
    }
    else
    {
        printf("open");
    }
    printf("  lux~");
    app_print_f1(g_ldr.lux);
    printf("  [%s]",
           (LDR_LEVEL_DARK == g_ldr.level)   ? "dark" :
           (LDR_LEVEL_BRIGHT == g_ldr.level) ? "bright" : "mid");
    printf("\r\n");
#endif

    /* 噪声指标单独一行：这是 ADC 噪声抑制实测的原始数据。
     * 注意 σ 的含义取决于 FSP 里的平均次数 —— 一并打出来，免得帖子写混 */
    printf("        noise: pp=");
    app_print_f1(g_ldr.pp);
    printf(" LSB   sigma=");
    app_print_f2(g_ldr.sigma);
    printf(" LSB   (%u conversions", (unsigned) APP_ADC_BURST);
    if (APP_ADC_AVERAGE_TIMES > 1)
    {
        printf(", hw-average x%d", (int) APP_ADC_AVERAGE_TIMES);
    }
    else
    {
        printf(", no averaging");
    }
    printf(")\r\n");

    /* ---- 模拟通道 B（P015）---- */
    if (!g_adc_b_used)
    {
        printf("[ADC-B ] P015 n/a   (APP_ADC_B_ENABLE = 0)\r\n");
    }
    else if (g_adc_b_ok)
    {
        printf("[ADC-B ] P015 raw=%u  ", (unsigned) g_adc_b_raw);
        app_print_f1(g_adc_b_mv);
        printf(" mV  LDR=");
        if (g_adc_b_r_ohm > 0.0f)
        {
            app_print_f1(g_adc_b_r_ohm / 1000.0f);
            printf("k  ");
            app_print_f1(g_adc_b_lux);
            printf(" lx");
        }
        else
        {
            printf("n/a");
        }
        printf("   pp=");
        app_print_f1(g_adc_b_pp);
        printf(" sigma=");
        app_print_f2(g_adc_b_sigma);
        printf("\r\n");
    }
    else
    {
        printf("[ADC-B ] P015 read failed\r\n");
    }

    /* 采样速率三口径：us/sample（吞吐）、us/AD-conv（单次转换）、kSps。
     * ⚠️ 单次转换耗时 = 周期数 ÷（通道数 × 平均次数）——scan_mask 勾几路就全转几路；
     *    只除平均次数会算大 3 倍，且不报错。 */
    {
        uint32_t cyc = adc_last_burst_cycles();
        printf("[ADC   ] burst %u samples in %lu cycles -> ",
               (unsigned) APP_ADC_BURST, (unsigned long) cyc);
        if (0U != cyc)
        {
            float us_per_sample = ((float) cyc / (float) APP_ADC_BURST)
                                  / ((float) SystemCoreClock / 1000000.0f);
            float conv_per_sample = (float) (APP_ADC_SCAN_CHANNELS
                                             * APP_ADC_AVERAGE_TIMES);
            float us_per_conv = us_per_sample / conv_per_sample;

            app_print_f2(us_per_sample);
            printf(" us/sample   ");
            app_print_f2(us_per_conv);
            printf(" us/AD-conv  (%u ch x avg%d = %u conv/sample)   ",
                   (unsigned) APP_ADC_SCAN_CHANNELS, (int) APP_ADC_AVERAGE_TIMES,
                   (unsigned) (APP_ADC_SCAN_CHANNELS * APP_ADC_AVERAGE_TIMES));
            app_print_f1(1000.0f / us_per_sample);
            printf(" kSps\r\n");
        }
        else
        {
            printf("n/a (DWT off)\r\n");
        }
    }

    /* ---- I2C 两器件 ---- */
    if (g_light.valid)
    {
        printf("[BH1750] lux = ");
        app_print_f1(g_light.lux);
        printf("\r\n");
    }
    else
    {
        printf("[BH1750] n/a\r\n");
    }

    if (g_press.valid)
    {
        printf("[BMP280] ");
        app_print_f2(g_press.temp_c);
        printf(" C   ");
        app_print_f2(g_press.press_hpa);
        printf(" hPa\r\n");
    }
    else if (g_press.not_ready)
    {
        /* 和"读失败"分开报：这是器件回了未就绪码，驱动已经自动重配过，
         * 通常等一两轮就恢复；要是几十轮都这样，那就是供电/接线的问题。 */
        printf("[BMP280] n/a (器件报未就绪 0x80000，已自动重配，等下一轮)\r\n");
    }
    else
    {
        printf("[BMP280] n/a\r\n");
    }

    /* ---- 单总线 ---- */
    if (g_dht.valid)
    {
        printf("[DHT%-3d] ", (int) APP_DHT_TYPE);
        app_print_f1(g_dht.temp_c);
        printf(" C   ");
        app_print_f1(g_dht.rh_pct);
        printf(" %%RH   raw=%02X %02X %02X %02X %02X\r\n",
               (unsigned) g_dht.raw[0], (unsigned) g_dht.raw[1],
               (unsigned) g_dht.raw[2], (unsigned) g_dht.raw[3],
               (unsigned) g_dht.raw[4]);
    }
    else
    {
        printf("[DHT%-3d] n/a   (fail x%lu)\r\n",
               (int) APP_DHT_TYPE, (unsigned long) g_dht_fail);
    }

    /* ---- 片内温度 ---- */
    if (g_chip_temp_ok)
    {
        printf("[TSN   ] ");
        app_print_f2(g_chip_temp_c);
        printf(" C\r\n");
    }
#endif
}

/* ==================================================================
 * 主流程
 * ================================================================== */

void hal_entry (void)
{
    fsp_err_t err;

    /* ---- 串口最先打开，之后所有日志都靠它 ---- */
    err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    assert(FSP_SUCCESS == err);

    print_banner();

    /* ---- RTC：1Hz，用来打时间戳 ---- */
    static rtc_time_t init_time =
    {
        .tm_sec  = 0,
        .tm_min  = 0,
        .tm_hour = 12,
        .tm_mday = 6,
        .tm_mon  = 9,       /* 0~11，9 = 10月 */
        .tm_year = 126,     /* 自 1900 起算，126 = 2026 */
        .tm_wday = 2,
    };

    err = R_RTC_Open(&g_rtc0_ctrl, &g_rtc0_cfg);
    assert(FSP_SUCCESS == err);
    R_RTC_ClockSourceSet(&g_rtc0_ctrl);
    R_RTC_CalendarTimeSet(&g_rtc0_ctrl, &init_time);       /* 必须调一次，RTC 才开始走 */
    R_RTC_PeriodicIrqRateSet(&g_rtc0_ctrl, RTC_PERIODIC_IRQ_SELECT_1_SECOND);

    /* ---- GPT：0.5s 节拍，采集的时间基准 ---- */
    err = R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    assert(FSP_SUCCESS == err);
    err = R_GPT_Start(&g_timer0_ctrl);
    assert(FSP_SUCCESS == err);

    /* ---- ADC：TSN 通道 + 两路模拟输入，一次 ScanCfg 全配好 ---- */
    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    assert(FSP_SUCCESS == err);

    /* ⚠️ 这里依赖 FSP 里已把 AN012/AN013 与 Temperature 一起勾上。
     * 只勾了 Temperature 的话，scan_mask 里没有 AN012/AN013，
     * R_ADC_Read 会返回错误 —— 自检那行会直接暴露出来。 */
    err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);

    printf("[ADC] ScanCfg %s (need AN012 + AN013 + Temperature)\r\n",
           (FSP_SUCCESS == err) ? "ok" : "FAILED");

    /* ---- 蓝牙串口（SCI0）：打开并挂上单字节接收 ---- */
    err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    assert(FSP_SUCCESS == err);
    err = R_SCI_UART_Read(&g_uart0_ctrl, &g_bt_rx_byte, 1U);
    assert(FSP_SUCCESS == err);

    /* ---- WiFi 串口（SCI2）：打开 + 探测（AT）---- */
    wifi_bridge_init();

    /* ---- DWT 计时基准（DHT 时序 + 采样速率都靠它） ---- */
    if (!app_dwt_init())
    {
        printf("[DWT] unavailable -- DHT timing and rate measurement disabled\r\n");
    }

#if APP_ENABLE_CTSU
    err = RM_TOUCH_Open(g_qe_touch_instance_config01.p_ctrl,
                        g_qe_touch_instance_config01.p_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[CAP] RM_TOUCH_Open FAILED, err = %d\r\n", (int) err);
    }
#endif

#if APP_OUT_CSV
    print_header_csv();
#endif

    /* ---- 自检放在外设都开好之后 ---- */
    self_test();

#if APP_LPM_TEST_ENABLE
    /* ---- 低功耗三档测试（第 4 篇）：打开 LPM + 打印测试计划 ---- */
    lpm_test_init();
#endif

#if APP_POWER_GEAR_ENABLE
    /* ---- 触摸换挡（第 4 篇·成品）：两个触摸焊盘切换功耗档位 ---- */
    power_gear_init();
#endif

    uint32_t tick_ctr   = 0U;
    uint32_t last_tick  = 0U;
    uint32_t rep_ctr    = 0U;

    while (1)
    {
        /* ---- 等 GPT 推进一格（0.5s）。这里不软延时，CPU 空闲等中断 ---- */
        if (last_tick == g_gpt_tick)
        {
#if APP_LPM_TEST_ENABLE
            lpm_test_wait_hook();   /* 低功耗测试：待机段在这里用 WFI 真正睡下 */
#endif
#if APP_POWER_GEAR_ENABLE
            power_gear_wait_hook(); /* 触摸换挡：SLEEP/STANDBY 档在这里睡下 */
#endif
            continue;
        }
        last_tick = g_gpt_tick;
        tick_ctr++;

#if APP_LPM_TEST_ENABLE
        lpm_test_tick_hook();       /* 低功耗测试：运行/待机两段计时与切换 */
#endif
#if APP_POWER_GEAR_ENABLE
        power_gear_tick_hook();     /* 触摸换挡：扫描触摸、执行换挡 */
#endif

        /* ---- 蓝牙：手机命令的即时处理（行就绪由中断置位） ---- */
        if (g_bt_line_ready)
        {
            char    line[BT_LINE_MAX + 1U];
            uint8_t len = g_bt_line_len;

            for (uint8_t i = 0U; i < len; i++)
            {
                line[i] = g_bt_line[i];
            }
            line[len] = '\0';

            g_bt_line_len   = 0U;
            g_bt_line_ready = false;    /* 先清标志：中断可立刻收下一行 */

            bt_handle_line(line);
        }

        /* ---- WiFi：模块响应行处理 ---- */
        wifi_bridge_poll();

        /* ---- 0.5s：模拟通道 A 一批独立转换 ---- */
        if (0U == (tick_ctr % APP_TICKS_FAST_ADC))
        {
            (void) ldr_read(&g_ldr);

            /* 模拟通道 B（P015/AN013）：没接线就不读，免得把悬空脚的
             * 乱漂读数当成数据报出去 */
            if (g_adc_b_used)
            {
                uint16_t b[APP_ADC_BURST];

                if (FSP_SUCCESS == adc_burst_read(APP_ADC_CH_B, b, APP_ADC_BURST))
                {
                    app_stats_t b_st;

                    app_stats_calc(b, APP_ADC_BURST, &b_st);
                    g_adc_b_raw   = (uint16_t) (b_st.mean + 0.5f);
                    g_adc_b_mv    = b_st.mean / APP_ADC_FULL_SCALE * APP_ADC_VREF_MV;
                    g_adc_b_pp    = b_st.pp;
                    g_adc_b_sigma = b_st.sigma;
                    /* P015 = 光敏电阻分压：mV → 阻值 → 照度（幂律估算） */
                    g_adc_b_r_ohm = ldr_volt_to_ohm(g_adc_b_mv / 1000.0f);
                    g_adc_b_lux   = (g_adc_b_r_ohm > 0.0f)
                                    ? ldr_ohm_to_lux(g_adc_b_r_ohm) : 0.0f;
                    g_adc_b_ok    = true;
                }
                else
                {
                    g_adc_b_ok = false;
                }
            }
        }

        /* ---- 3s：I2C 两器件 ---- */
        if (0U == (tick_ctr % APP_TICKS_SLOW_I2C))
        {
            if (bh1750_ready())
            {
                (void) bh1750_read(&g_light);
            }
            if (bmp280_ready())
            {
                (void) bmp280_read(&g_press);
            }
        }

        /* ---- 5s：蓝牙上行播报（手机近场直连的数据流） ---- */
        if (0U == (tick_ctr % APP_TICKS_BT_REPORT))
        {
            bt_report();
        }

        /* ---- 10s：WiFi 云端上报（AT+NWMQMSG） ---- */
        if (0U == (tick_ctr % APP_TICKS_WIFI_REPORT))
        {
            wifi_bridge_report(g_light.valid ? g_light.lux : -1.0f,
                               (g_adc_b_used && g_adc_b_ok) ? g_adc_b_lux : -1.0f,
                               g_dht.valid ? g_dht.temp_c : -100.0f,
                               g_dht.valid ? g_dht.rh_pct : -1.0f);
        }

        /* ---- 4s：单总线 ---- */
        if (0U == (tick_ctr % APP_TICKS_DHT))
        {
            fsp_err_t derr = dht_read(&g_dht);
            if (FSP_SUCCESS != derr)
            {
                /* 保留上一次的有效值，不清零 —— 0℃/0%RH 看着像数据其实不是 */
                g_dht_fail++;
                if (0U == (g_dht_fail % 5U))
                {
                    printf("[DHT] read failed %lu times in a row (err=%d), "
                           "check wiring / pull-up\r\n",
                           (unsigned long) g_dht_fail, (int) derr);
                }
            }
        }

        /* ---- 10s：汇总上报 ---- */
        rep_ctr++;
        if (0U == (rep_ctr % APP_TICKS_REPORT))
        {
            g_sample_no++;

            /* 片内温度借同一节拍读一次 */
            g_chip_temp_ok = read_chip_temp(&g_chip_temp_c);

            report_all(tick_ctr);
        }

        /* ---- 每秒更新一次时间戳（可选，保持 RTC 在跑） ---- */
        if (g_rtc_tick)
        {
            g_rtc_tick = false;
        }
    }

#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif
}

/* ==================================================================
 * ⚠️ 这里**故意不包含** R_BSP_WarmStart()
 *
 * FSP 的 flat 工程把它单独放在 src/hal_warmstart.c 里
 * （内容：RESET 事件打开 data flash；POST_C 事件里 R_IOPORT_Open 配引脚）。
 * 本文件如果也定义一份，链接期会报
 *     multiple definition of `R_BSP_WarmStart'
 * 所以复制文件时**保留工程的 hal_warmstart.c 原样，不要动它**。
 * ================================================================== */

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
