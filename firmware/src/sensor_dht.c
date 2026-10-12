/*
 * sensor_dht.c —— DHT22 单总线温湿度驱动（DHT11 兼容）。
 *
 * 时序：主机拉低 ≥1ms 起始，其后靠高电平**宽度**区分 0/1（26~28us=0，70us=1）；
 * 一次完整读取约 25ms，期间用 PRIMASK 关中断保护（保存/恢复，不无脑开中断）。
 * 计时用 DWT 周期计数（ICLK=96MHz，分辨率 ~10.4ns），不占用定时器外设。
 *
 * © 2026 MoRiv447 · MIT License
 */

#include "sensors.h"
#include "app_config.h"
#include "app_util.h"

#include <stdio.h>
#include <string.h>

/* ================================================================== */
/*                             时序参数                                */
/* ================================================================== */

#if (11 == APP_DHT_TYPE)
    /* DHT11：手册要求主机拉低 ≥18ms，采样间隔 ≥1s */
    #define DHT_START_LOW_MS        20U
#elif (22 == APP_DHT_TYPE)
    /* DHT22：主机拉低 1~10ms 即可，采样间隔 ≥2s */
    #define DHT_START_LOW_MS        2U
#else
    #error "APP_DHT_TYPE 只能是 11 或 22"
#endif

#define DHT_RELEASE_US              30U     /* 释放后保持高电平的时间，手册建议 20~40us */
#define DHT_RESP_TIMEOUT_US         200U    /* 等从机应答边沿的超时 */
#define DHT_BIT_LOW_TIMEOUT_US      100U    /* 每位开头的 50us 低电平 */
#define DHT_BIT_HIGH_TIMEOUT_US     150U    /* 每位的高电平（最多 ~70us） */
#define DHT_BIT_ONE_THRESHOLD_US    40U     /* 高电平超过这个宽度判为 1 */

#define DHT_WAIT_FAIL               0xFFFFFFFFUL

/* ================================================================== */
/*                            内部状态                                 */
/* ================================================================== */

static bool s_dht_ready = false;

/* DWT 周期计数器本身在 app_util.c 里统一管理（ADC 测采样速率也要用），
 * 这里只借用它的取周期数 / 微秒换算两个接口。 */

/* ================================================================== */
/*                            引脚操作                                 */
/* ================================================================== */

static inline void dht_pin_output (void)
{
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_DHT_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT
                           | (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH);
}

static inline void dht_pin_input (void)
{
    /* 内部上拉只能算兜底（RA 的内部上拉在几十 kΩ 量级）。
     * DHT 手册要求 4.7k~10k 外部上拉，模块版一般自带；
     * 裸传感器必须自己加，否则读到的永远是 0。 */
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_DHT_PIN,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT
                           | (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
}

static inline void dht_pin_write (bsp_io_level_t level)
{
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl, APP_DHT_PIN, level);
}

static inline bsp_io_level_t dht_pin_read (void)
{
    bsp_io_level_t lv = BSP_IO_LEVEL_LOW;

    (void) R_IOPORT_PinRead(&g_ioport_ctrl, APP_DHT_PIN, &lv);
    return lv;
}

bool dht_pin_idle_level (void)
{
    return (BSP_IO_LEVEL_HIGH == dht_pin_read());
}

/**
 * @brief 等引脚变成指定电平
 *
 * @return 从开始等到变化所花的周期数；超时返回 DHT_WAIT_FAIL
 *
 * 注意 CYCCNT 是 32 位、会自然回绕，用无符号相减差值比较即可，
 * 不需要处理溢出。
 */
static uint32_t dht_wait_level (bsp_io_level_t want, uint32_t timeout_us)
{
    uint32_t t0    = app_dwt_cycles();
    uint32_t limit = app_dwt_us_to_cycles(timeout_us);

    while (dht_pin_read() != want)
    {
        if ((uint32_t) (app_dwt_cycles() - t0) > limit)
        {
            return DHT_WAIT_FAIL;
        }
    }

    return (uint32_t) (app_dwt_cycles() - t0);
}

/* ================================================================== */
/*                            读原始 5 字节                             */
/* ================================================================== */

/**
 * @brief 完成一次完整时序读取，填充 5 个原始字节
 *
 * 前提：调用者已关中断；DWT 已初始化。
 * 拆成独立函数是为了让「关中断 / 恢复中断」在调用点成对出现，
 * 中间不管从哪条路径 return，恢复语句都不会被跳过。
 */
static bool dht_read_raw (uint8_t * out5)
{
    /* ---- 1) 起始信号：拉低 → 释放 ---- */
    dht_pin_output();
    dht_pin_write(BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(DHT_START_LOW_MS, BSP_DELAY_UNITS_MILLISECONDS);

    dht_pin_write(BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(DHT_RELEASE_US, BSP_DELAY_UNITS_MICROSECONDS);

    /* 释放总线：切成输入，靠外部上拉把线拉高 */
    dht_pin_input();

    /* ---- 2) 从机应答：80us 低 → 80us 高 → 进入数据 ---- */
    if (DHT_WAIT_FAIL == dht_wait_level(BSP_IO_LEVEL_LOW,  DHT_RESP_TIMEOUT_US)) { return false; }
    if (DHT_WAIT_FAIL == dht_wait_level(BSP_IO_LEVEL_HIGH, DHT_RESP_TIMEOUT_US)) { return false; }
    if (DHT_WAIT_FAIL == dht_wait_level(BSP_IO_LEVEL_LOW,  DHT_RESP_TIMEOUT_US)) { return false; }

    /* ---- 3) 40 位数据，MSB 先出 ---- */
    uint8_t buf[5] = { 0U, 0U, 0U, 0U, 0U };

    for (uint32_t i = 0U; i < 40U; i++)
    {
        /* 每位的低电平段（约 50us） */
        if (DHT_WAIT_FAIL == dht_wait_level(BSP_IO_LEVEL_HIGH, DHT_BIT_LOW_TIMEOUT_US))
        {
            return false;
        }

        /* 量这一位的高电平有多宽 —— 宽度决定 0 还是 1 */
        uint32_t high_cycles = dht_wait_level(BSP_IO_LEVEL_LOW, DHT_BIT_HIGH_TIMEOUT_US);
        if (DHT_WAIT_FAIL == high_cycles)
        {
            return false;
        }

        uint32_t idx = i / 8U;
        buf[idx] = (uint8_t) (buf[idx] << 1U);

        if (high_cycles > app_dwt_us_to_cycles(DHT_BIT_ONE_THRESHOLD_US))
        {
            buf[idx] |= 1U;
        }
    }

    memcpy(out5, buf, 5U);
    return true;
}

/* ================================================================== */
/*                            对外接口                                 */
/* ================================================================== */

fsp_err_t dht_init (void)
{
    s_dht_ready = app_dwt_init();

    if (!s_dht_ready)
    {
        /* DWT 不可用的话时序无从判起，直接报不支持，
         * 而不是跑一个靠猜的时序然后在帖子里写一堆"读数不稳定" */
        printf("[DHT] DWT cycle counter unavailable\r\n");
        return FSP_ERR_UNSUPPORTED;
    }

    /* 起始状态必须让总线空闲为高，否则从机一直以为主机在发起始信号 */
    dht_pin_input();

    printf("[DHT%d] init ok, idle level = %d\r\n",
           (int) APP_DHT_TYPE,
           (int) dht_pin_idle_level());

    if (!dht_pin_idle_level())
    {
        printf("[DHT] idle level is LOW -- check pull-up resistor and wiring\r\n");
    }

    return FSP_SUCCESS;
}

fsp_err_t dht_read (dht_data_t * out)
{
    if (NULL == out)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!s_dht_ready)
    {
        return FSP_ERR_NOT_OPEN;
    }

    uint8_t raw[5] = { 0U, 0U, 0U, 0U, 0U };

    /* ---- 关中断 → 读时序 → 恢复中断（单点恢复） ---- */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool ok = dht_read_raw(raw);

    __set_PRIMASK(primask);

    if (!ok)
    {
        out->valid = false;
        out->err_streak++;
        return FSP_ERR_TIMEOUT;
    }

    /* ---- 校验：前 4 字节之和的低 8 位应该等于第 5 字节 ---- */
    uint8_t sum = (uint8_t) (raw[0] + raw[1] + raw[2] + raw[3]);
    if (sum != raw[4])
    {
        /* 校验失败通常意味着时序被干扰或线太长。把原始字节留下来，
         * 帖子里正好可以展示"校验和不匹配"这个现象本身。 */
        out->valid = false;
        out->err_streak++;
        memcpy(out->raw, raw, 5U);
        return FSP_ERR_INVALID_DATA;
    }

#if (11 == APP_DHT_TYPE)
    /* DHT11：整数 8 位 + 小数 8 位（小数位器件上恒为 0，保留是为了兼容） */
    float rh = (float) raw[0] + ((float) raw[1] / 10.0f);
    float tc = (float) raw[2] + ((float) (raw[3] & 0x7FU) / 10.0f);
    if (0U != (raw[3] & 0x80U))
    {
        tc = -tc;
    }
#else
    /* DHT22：16 位湿度（含小数，÷10）、16 位温度（最高位为符号） */
    uint16_t rh_raw = (uint16_t) (((uint16_t) raw[0] << 8) | (uint16_t) raw[1]);
    uint16_t t_raw  = (uint16_t) (((uint16_t) raw[2] << 8) | (uint16_t) raw[3]);

    float rh = (float) rh_raw / 10.0f;
    float tc = (float) (t_raw & 0x7FFFU) / 10.0f;
    if (0U != (t_raw & 0x8000U))
    {
        tc = -tc;
    }
#endif

    out->temp_c   = tc;
    out->rh_pct   = rh;
    out->valid    = true;
    out->err_streak = 0U;
    memcpy(out->raw, raw, 5U);

    return FSP_SUCCESS;
}
