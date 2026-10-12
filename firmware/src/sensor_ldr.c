/*
 * sensor_ldr.c —— 模拟通道 A（P014/AN012）+ ADC 噪声/精度/速率实测。
 *
 * 输入源由 APP_ADC_A_INPUT 切换：0 = 固定分压（默认，干净基准） / 1 = 光敏分压。
 * 三个注意点：
 *   ① FSP 里 ADC 配了 clear_after_read → 本模块每个样本都重新 ScanStart；
 *   ② scan_mask 必须包含 AN012/AN013（原本只有 Temperature）；
 *   ③ 分压上下臂接反会把读数变成 3V3 − 预期值，精度对比会直接暴露。
 *
 * © 2026 MoRiv447 · MIT License
 */

#include "sensors.h"
#include "app_config.h"
#include "app_util.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* 由 stage3_main.c 定义；FSP 里 ADC Stack 的 Callback 就填 adc_callback */
extern volatile bool g_adc_done;

static uint32_t s_last_burst_cycles = 0U;

uint32_t adc_last_burst_cycles (void)
{
    return s_last_burst_cycles;
}

/* ================================================================== */
/*                     ADC 连采（每点独立转换）                         */
/* ================================================================== */

fsp_err_t adc_burst_read (adc_channel_t ch, uint16_t * samples, uint32_t n)
{
    if ((NULL == samples) || (0U == n))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    uint32_t t0 = app_dwt_cycles();

    for (uint32_t i = 0U; i < n; i++)
    {
        g_adc_done = false;

        fsp_err_t err = R_ADC_ScanStart(&g_adc0_ctrl);
        FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

        /* 等扫描结束中断：用裸轮询 DWT 周期数，不用 R_BSP_SoftwareDelay(1us)。
         *
         * ⚠️ 更正一条我先前写错的判断：原来这里写着"24us/sample 里绝大部分是
         *    软延时轮询的开销"。换成裸轮询后实测 37175 → 36189 周期，**几乎没变**
         *    —— 说明那个数是真的，我猜错了。
         *    真正的构成在 app_config.h 的 APP_ADC_SCAN_CHANNELS 里写着：
         *    每次 ScanStart 会把 scan_mask 里 3 个通道全转一遍（×平均4次 = 12 次），
         *    12 次转换就要十几微秒，剩下的才是 ScanStart/Read 的软件开销。
         *
         * 换裸轮询仍然值得：粒度从 1us 降到 1 个周期，也不依赖软延时的校准系数，
         * 量出来的时间才干净。 */
        uint32_t t_scan = app_dwt_cycles();
        uint32_t limit  = app_dwt_us_to_cycles(APP_ADC_SCAN_TIMEOUT_US);

        while (!g_adc_done)
        {
            if ((uint32_t) (app_dwt_cycles() - t_scan) > limit)
            {
                return FSP_ERR_TIMEOUT;
            }
        }

        err = R_ADC_Read(&g_adc0_ctrl, ch, &samples[i]);
        FSP_ERROR_RETURN(FSP_SUCCESS == err, err);
    }

    s_last_burst_cycles = (uint32_t) (app_dwt_cycles() - t0);
    return FSP_SUCCESS;
}

/* ================================================================== */
/*                             换算                                    */
/* ================================================================== */

float adc_a_expected_mv (void)
{
#if (0 == APP_ADC_A_INPUT)
    /* 分压：V = VREF * R_LOW / (R_HIGH + R_LOW) */
    float total = APP_FIXED_R_HIGH_OHM + APP_FIXED_R_LOW_OHM;

    if (total <= 0.0f)
    {
        return -1.0f;
    }
    return APP_ADC_VREF_MV * (APP_FIXED_R_LOW_OHM / total);
#else
    /* 光敏电阻阻值随光照变，没有固定期望值 */
    return -1.0f;
#endif
}

float ldr_volt_to_ohm (float volt)
{
    const float vref = APP_ADC_VREF_MV / 1000.0f;

    /* 电压为 0 → 光敏电阻开路，阻值无穷大，数学上无定义 */
    if (volt <= 0.0f)
    {
        return -1.0f;
    }

    /* 电压顶到电源 → 阻值趋近 0（强光） */
    if (volt >= vref)
    {
        return 0.0f;
    }

    return APP_LDR_R_SERIES_OHM * ((vref / volt) - 1.0f);
}

float ldr_ohm_to_lux (float r_ohm)
{
    if (r_ohm <= 0.0f)
    {
        return 0.0f;
    }

    /* 幂律：lux = lux_ref * (R / R_ref)^(-1/γ)
     * γ 和参考点都是 GL5528 的典型值，属于量级可用，不是标定结果。 */
    float ratio = r_ohm / APP_LDR_R_REF_OHM;

    return APP_LDR_LUX_REF * powf(ratio, -1.0f / APP_LDR_GAMMA);
}

#if (1 == APP_ADC_A_INPUT)
static ldr_level_t ldr_classify (float lux)
{
    if (lux < APP_LDR_LUX_DARK)
    {
        return LDR_LEVEL_DARK;
    }
    if (lux > APP_LDR_LUX_BRIGHT)
    {
        return LDR_LEVEL_BRIGHT;
    }
    return LDR_LEVEL_MID;
}
#endif

/* ================================================================== */
/*                   逐通道探针（接线排查用）                          */
/* ================================================================== */

/**
 * @brief 单独测每一个模拟通道，打印各自的均值与噪声
 *
 * 解决什么问题：实机遇到过"分压明明确认接好了，但 P014 读出来还是悬空"。
 * 这种时候光看 P014 一条通道分不清是
 *   ① 分压没接好   ② 跳线插错排针脚（比如插到 P015 去了）   ③ 杜邦线内部断
 * 把两条模拟通道各自单独测一遍就清楚了：
 * **线插在哪个脚上，那个脚就会显示"正常"**，另一个继续悬空。
 *
 * 判据和 ldr_init 里一致：接上低阻源时 sigma 是个位数 LSB，悬空则上百。
 */
void adc_probe_channels (void)
{
    static const struct
    {
        adc_channel_t ch;
        const char *  name;
    } k_list[] =
    {
        { APP_ADC_CH_A, "P014 / AN012" },
        { APP_ADC_CH_B, "P015 / AN013" },
    };

    uint32_t floating_cnt = 0U;

    printf("[ADC probe] 两条模拟通道分别单独测一遍（用来判断跳线插在哪个脚上）：\r\n");

    for (uint32_t i = 0U; i < sizeof(k_list) / sizeof(k_list[0]); i++)
    {
        uint16_t    s[APP_ADC_BURST];
        app_stats_t st;

        if (FSP_SUCCESS != adc_burst_read(k_list[i].ch, s, APP_ADC_BURST))
        {
            printf("  %-13s read failed\r\n", k_list[i].name);
            continue;
        }

        app_stats_calc(s, APP_ADC_BURST, &st);

        bool floating = (st.sigma > 20.0f);
        if (floating)
        {
            floating_cnt++;
        }

        printf("  %-13s mean = ", k_list[i].name);
        app_print_f1(st.mean);
        printf(" LSB (");
        app_print_f1(st.mean / APP_ADC_FULL_SCALE * APP_ADC_VREF_MV);
        printf(" mV)   sigma = ");
        app_print_f2(st.sigma);
        printf(" LSB   -> %s\r\n",
               (floating) ? "悬空（这脚没接东西）" : "接上了 ✓");
    }

    /* 两条都悬空的话，能再缩小一次范围：
     * 如果跳线只是插错了脚，那它插的那个脚必然显示"接上了"。
     * 两条都没接上 ⇒ 问题不在排针那一端，而在面包板这一端。 */
    if (floating_cnt == 2U)
    {
        printf("  两条都悬空 ⇒ 跳线**没有落在任何一个模拟脚上**，问题在面包板这一侧：\r\n");
        printf("    先查分压的插孔有没有跨过中间槽（上下两半不连通）。\r\n");
    }
    else
    {
        printf("  ↑ 哪一行显示\"接上了\"，就说明跳线插在那个脚上。\r\n");
    }
}

/* ================================================================== */
/*                        跨轮次的滚动滤波                              */
/* ================================================================== */

/* 每轮 burst 先出一个代表值（burst 均值），再把这个代表值送进
 * 「中值 → 滑动平均」两级滚动滤波。顺序不能反：
 * 先平均会把离群点摊薄到整个窗口里，中值就再也救不回来了。 */
static app_median_t  s_med;
static app_average_t s_avg;
static bool          s_ready = false;

fsp_err_t ldr_init (void)
{
    app_median_init(&s_med, APP_MEDIAN_WIN);
    app_average_init(&s_avg, APP_AVERAGE_WIN);

    /* 先空跑一轮丢掉。
     *
     * ADC 在 Open/ScanCfg 之后的第一次扫描，采样保持电路还没稳定 ——
     * 实机量的：第一轮 sigma = 2.95 LSB / pp = 12，而进入稳态后是
     * sigma = 0.78 LSB / pp = 3，差 3~4 倍。
     * 这个基线数字是要写进报告的，不能拿一个过渡过程的数去代表噪声底。 */
    uint16_t warm[APP_ADC_BURST];
    (void) adc_burst_read(APP_ADC_CH_A, warm, APP_ADC_BURST);

    /* ADC 的 Open / ScanCfg 由 stage3_main.c 统一做（它要同时管温度通道）。
     * 这里只做一次上电基线检查，把原始读数、噪声、以及（固定分压时的）
     * 理论值 vs 实测值一起打出来 —— 接线对不对、精度如何，一眼就有。 */
    uint16_t  s[APP_ADC_BURST];
    fsp_err_t err = adc_burst_read(APP_ADC_CH_A, s, APP_ADC_BURST);
    if (FSP_SUCCESS != err)
    {
        printf("[ADC-A] baseline read failed: %d\r\n", (int) err);
        return err;
    }

    app_stats_t st;
    app_stats_calc(s, APP_ADC_BURST, &st);

    float mean_mv = st.mean / APP_ADC_FULL_SCALE * APP_ADC_VREF_MV;

    /* ⚠️ 这里不能用 %.0f —— 工程链的是 newlib-nano，浮点格式化被裁掉了，
     * 表现是"不报错、不打印、那个位置直接空白"。实机就打出过
     * `R_H=k R_L=k` 这种怪句子。所有浮点都得走 app_print_fix。 */
#if (0 == APP_ADC_A_INPUT)
    printf("[ADC-A] P014/AN012 fixed divider R_H=");
    app_print_fix(APP_FIXED_R_HIGH_OHM / 1000.0f, 0U);
    printf("k R_L=");
    app_print_fix(APP_FIXED_R_LOW_OHM / 1000.0f, 0U);
    printf("k, %u conversions\r\n", (unsigned) APP_ADC_BURST);
#else
    printf("[ADC-A] P014/AN012 photoresistor divider R_S=");
    app_print_fix(APP_LDR_R_SERIES_OHM / 1000.0f, 0U);
    printf("k, %u conversions\r\n", (unsigned) APP_ADC_BURST);
#endif

    printf("        mean = ");
    app_print_f1(st.mean);
    printf(" LSB  (");
    app_print_f1(mean_mv);
    printf(" mV)   pp = ");
    app_print_f1(st.pp);
    printf(" LSB   sigma = ");
    app_print_f2(st.sigma);
    printf(" LSB\r\n");

    if (st.mean < 8.0f)
    {
        printf("[ADC-A] mean ~ 0 -> 输入没接上 / 开路（分压点没有电压）\r\n");
    }
    else if (st.mean > (APP_ADC_FULL_SCALE - 8.0f))
    {
        printf("[ADC-A] mean ~ full-scale -> 输入顶到 3V3，检查分压上下臂\r\n");
    }

    /* ---- 悬空检测：这是"均值看着像回事、但其实是没接"的那种情况 ----
     *
     * 实机踩过：P014 悬空时读到 mean≈1440 LSB，均值落在量程中段，
     * 光看均值完全看不出问题；但 pp=564、sigma=148 LSB 一眼就是悬空 ——
     * 悬空的 CMOS 输入会把工频干扰整片收进来。
     *
     * 判据怎么定的：接好 10k+10k 分压时，输入是个几十欧姆级的低阻节点，
     * sigma 通常只有个位数 LSB；悬空时轻松上百。20 LSB 这个门限远离两边。 */
    if (st.sigma > 20.0f)
    {
        printf("[ADC-A] sigma too large -> 输入很可能**悬空**（P014 没接到分压点）\r\n");
        printf("        接好 10k+10k 时 sigma 应是个位数 LSB；现在是 ");
        app_print_f1(st.sigma);
        printf(" LSB\r\n");

        /* 按实机踩过的概率把原因列出来 —— 泛泛说"检查接线"没用，
         * 这三个是真正会发生的，而且按可能性排序。 */
        printf("        常见原因（按可能性）：\r\n");
        printf("          1) 跳线不在丝印印着 P014 的那个孔上\r\n");
        printf("             **标 3 是从上往下第 3 个孔**（丝印上 P013/P014 那一行）；\r\n");
        printf("             注意标 3/标 4 的接插件引脚 1 在最下面，别按脚号数\r\n");
        printf("          2) 分压的插孔跨过了面包板**中间槽**\r\n");
        printf("             中间槽两侧（a~e 与 f~j）在板内是**不连通**的，\r\n");
        printf("             两个电阻腿和中点跳线必须全在同一侧\r\n");
        printf("          3) 杜邦线内部断（换一根试）\r\n");
        printf("        万用表最直接：量分压中点那一列对 GND，应读到 1650 mV 左右。\r\n");
    }

#if (0 == APP_ADC_A_INPUT)
    {
        float expect  = adc_a_expected_mv();
        float err_lsb = (mean_mv - expect) / (APP_ADC_VREF_MV / APP_ADC_FULL_SCALE);

        printf("[ADC-A] precision: expect ");
        app_print_f1(expect);
        printf(" mV, measured ");
        app_print_f1(mean_mv);
        printf(" mV, error ");
        app_print_f2(err_lsb);
        printf(" LSB\r\n");

        /* 误差超过 ±40 LSB（约 ±32mV）通常不是 ADC 的问题，而是
         * 电阻实际阻值偏了，或者分压上下臂接反（那会偏到反方向一大截） */
        if ((err_lsb > 40.0f) || (err_lsb < -40.0f))
        {
            printf("[ADC-A] error too large -> 量一下两颗电阻的实际阻值，"
                   "并确认上下臂没接反\r\n");
        }
    }
#endif

    s_ready = true;
    return FSP_SUCCESS;
}

fsp_err_t ldr_read (ldr_data_t * out)
{
    if (NULL == out)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!s_ready)
    {
        return FSP_ERR_NOT_OPEN;
    }

    uint16_t  s[APP_ADC_BURST];
    fsp_err_t err = adc_burst_read(APP_ADC_CH_A, s, APP_ADC_BURST);
    if (FSP_SUCCESS != err)
    {
        out->valid = false;
        return err;
    }

    app_stats_t st;
    app_stats_calc(s, APP_ADC_BURST, &st);

    /* 两级滚动滤波：中值剔离群点，滑均压随机噪声 */
    float med_val = (float) app_median_push(&s_med, (uint16_t) (st.mean + 0.5f));
    float filt    = app_average_push(&s_avg, med_val);

    out->raw  = (uint16_t) (filt + 0.5f);
    out->volt = filt / APP_ADC_FULL_SCALE * (APP_ADC_VREF_MV / 1000.0f);

    float filt_mv = out->volt * 1000.0f;

#if (0 == APP_ADC_A_INPUT)
    /* 固定分压：只报电压和精度误差，不编造照度 */
    out->expect_mv = adc_a_expected_mv();
    out->error_lsb = (filt_mv - out->expect_mv)
                     / (APP_ADC_VREF_MV / APP_ADC_FULL_SCALE);
    out->r_ohm     = -1.0f;
    out->lux       = 0.0f;
#else
    out->expect_mv = -1.0f;
    out->error_lsb = 0.0f;
    out->r_ohm     = ldr_volt_to_ohm(out->volt);
    out->lux       = (out->r_ohm > 0.0f) ? ldr_ohm_to_lux(out->r_ohm) : 0.0f;
    out->level     = ldr_classify(out->lux);
#endif

    /* 噪声指标刻意用未滤波的原始 burst 值算 ——
     * 滤波后的数字是"我们处理得多好"，原始 pp/sigma 才是"ADC 本身有多吵"。
     * 帖子里要做噪声抑制对比，靠的就是这两个数。 */
    out->pp    = st.pp;
    out->sigma = st.sigma;

    out->valid = true;
    return FSP_SUCCESS;
}
