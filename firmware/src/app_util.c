/*
 * app_util.c —— 打印格式化 + 中值/滑动平均滤波 + 统计
 */

#include "app_util.h"
#include "app_config.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ================================================================== */
/*                         浮点打印（nano 安全）                        */
/* ================================================================== */

void app_print_fix (float v, uint8_t decimals)
{
    if (decimals > 4U)
    {
        decimals = 4U;
    }

    /* 手工构造 10^decimals。不用 powf() 是为了避免拉进多余的库函数 */
    int32_t scale = 1;
    for (uint8_t i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    /* 先四舍五入到目标精度，再拆整数/小数两段 */
    float   rounded = v * (float) scale;
    int32_t scaled  = (int32_t) (rounded + ((v >= 0.0f) ? 0.5f : -0.5f));

    int32_t ip = scaled / scale;
    int32_t fp = scaled % scale;

    if (fp < 0)
    {
        fp = -fp;
    }

    /* -0.30 会算出 ip==0 / fp==30，直接打印会丢掉负号，这里补回来 */
    if ((scaled < 0) && (0 == ip))
    {
        printf("-");
    }

    /* 固定写死每种精度，不用 "%0*ld" 的动态宽度 ——
     * 少依赖一个格式化特性，nano 上更稳 */
    switch (decimals)
    {
        case 0U:
        {
            printf("%ld", (long) ip);
            break;
        }
        case 1U:
        {
            printf("%ld.%01ld", (long) ip, (long) fp);
            break;
        }
        case 2U:
        {
            printf("%ld.%02ld", (long) ip, (long) fp);
            break;
        }
        case 3U:
        {
            printf("%ld.%03ld", (long) ip, (long) fp);
            break;
        }
        default:
        {
            printf("%ld.%04ld", (long) ip, (long) fp);
            break;
        }
    }
}

/* ================================================================== */
/*                    DWT 周期计数器（微秒级计时）                      */
/* ================================================================== */

static bool     s_dwt_ok         = false;
static uint32_t s_cycles_per_us  = 96U;     /* 兜底值，init 时按实际时钟重算 */

bool app_dwt_init (void)
{
    /* TRCENA 是 DWT 的总开关，不置位 CYCCNT 不会计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_ok = (0U != (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk));

    /* SystemCoreClock 由 FSP 的 SystemInit 设置为 ICLK。
     * 不写死 96 是为了以后改时钟树时这里能自动跟上。 */
    if (SystemCoreClock > 0U)
    {
        s_cycles_per_us = (uint32_t) (SystemCoreClock / 1000000U);
    }
    if (0U == s_cycles_per_us)
    {
        s_cycles_per_us = 1U;
    }

    return s_dwt_ok;
}

uint32_t app_dwt_cycles (void)
{
    return s_dwt_ok ? DWT->CYCCNT : 0U;
}

uint32_t app_dwt_us_to_cycles (uint32_t us)
{
    return us * s_cycles_per_us;
}

/* ================================================================== */
/*                            中值滤波                                 */
/* ================================================================== */

void app_median_init (app_median_t * m, uint8_t win)
{
    if (NULL == m)
    {
        return;
    }

    if (win < 1U)
    {
        win = 1U;
    }
    if (win > APP_MEDIAN_MAX)
    {
        win = APP_MEDIAN_MAX;
    }
    if (0U == (win & 1U))
    {
        win++;      /* 中值滤波窗口必须是奇数 */
    }
    if (win > APP_MEDIAN_MAX)
    {
        win = APP_MEDIAN_MAX;
    }

    memset(m, 0, sizeof(*m));
    m->win = win;
}

uint16_t app_median_push (app_median_t * m, uint16_t v)
{
    if (NULL == m)
    {
        return v;
    }

    m->buf[m->idx] = v;
    m->idx = (uint8_t) ((m->idx + 1U) % m->win);
    if (m->count < m->win)
    {
        m->count++;
    }

    /* 未填满时，缓冲区前 count 个正是已填入的样本（idx 尚未回绕），
     * 所以直接取前 count 个排序即可，不会把初值 0 算进来。
     * 窗口最多 9 个，插入排序的开销可以忽略。 */
    uint16_t tmp[APP_MEDIAN_MAX];
    for (uint8_t i = 0U; i < m->count; i++)
    {
        tmp[i] = m->buf[i];
    }

    for (uint8_t i = 1U; i < m->count; i++)
    {
        uint16_t key = tmp[i];

        /* j 用 int 而不是 int8_t：下标若写成 `j + 1U`，j 会先被提升为 int
         * 再转成 unsigned，工程开了 -Wsign-conversion 就会告警。
         * 用有符号 int 做下标则完全不涉及这个转换（下标允许任意整型）。 */
        int j = (int) i - 1;

        while ((j >= 0) && (tmp[j] > key))
        {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    return tmp[m->count / 2U];
}

/* ================================================================== */
/*                            滑动平均                                 */
/* ================================================================== */

void app_average_init (app_average_t * a, uint8_t win)
{
    if (NULL == a)
    {
        return;
    }

    if (win < 1U)
    {
        win = 1U;
    }
    if (win > APP_AVERAGE_MAX)
    {
        win = APP_AVERAGE_MAX;
    }

    memset(a, 0, sizeof(*a));
    a->win = win;
}

float app_average_push (app_average_t * a, float v)
{
    if (NULL == a)
    {
        return v;
    }

    if (a->count < a->win)
    {
        /* 预热期：只累加，不移除 */
        a->buf[a->idx] = v;
        a->sum += v;
        a->count++;
    }
    else
    {
        /* 稳态：减去被顶替的那个样本，避免每次重算整窗 */
        a->sum -= a->buf[a->idx];
        a->buf[a->idx] = v;
        a->sum += v;
    }

    a->idx = (uint8_t) ((a->idx + 1U) % a->win);

    if (0U == a->count)
    {
        return v;
    }

    return a->sum / (float) a->count;
}

/* ================================================================== */
/*                              统计                                   */
/* ================================================================== */

void app_stats_calc (const uint16_t * samples, uint32_t n, app_stats_t * out)
{
    if ((NULL == samples) || (NULL == out) || (0U == n))
    {
        return;
    }

    float sum  = 0.0f;
    float vmin = (float) samples[0];
    float vmax = (float) samples[0];

    for (uint32_t i = 0U; i < n; i++)
    {
        float f = (float) samples[i];

        sum += f;
        if (f < vmin)
        {
            vmin = f;
        }
        if (f > vmax)
        {
            vmax = f;
        }
    }

    float mean = sum / (float) n;

    /* 两遍算法求方差：先求均值再求偏差平方和。
     * 单遍的 sum(x^2)-mean^2 在定点 ADC 值上会有抵消误差，两遍更稳。 */
    float var = 0.0f;
    for (uint32_t i = 0U; i < n; i++)
    {
        float d = (float) samples[i] - mean;
        var += d * d;
    }
    var /= (float) n;

    out->mean  = mean;
    out->vmin  = vmin;
    out->vmax  = vmax;
    out->pp    = vmax - vmin;
    out->sigma = sqrtf(var);
}
