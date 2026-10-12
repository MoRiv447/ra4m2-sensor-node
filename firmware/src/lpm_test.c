/*
 * lpm_test.c —— 低功耗四平台测量固件（单发阶梯，盲测）。
 *
 *   上电自动走：RUN(忙等) → SLEEP(WFI) → SW-STANDBY(RTC 秒醒) → DEEP(保持，RESET 醒)。
 *   分流电阻 + 万用表读四段台阶即可，不用接串口；测量配方见 docs/。
 *
 * 口径：读数含 D5/D8 指示灯、CH340C 与 LDO 静态（免焊剥不掉），
 * 所以"差值"才是芯片本体功耗；深待机地板主要来自 CH340（未连接态 4-12mA）。
 *
 * 由 APP_LPM_TEST_ENABLE / ONESHOT 开关切换（默认关，成品固件走 power_gear）。
 *
 * © 2026 MoRiv447 · MIT License
 */
#include "lpm_test.h"
#include "hal_data.h"
#include "app_config.h"
#include "sensors.h"
#include <stdio.h>

#if APP_LPM_TEST_ENABLE

#define LPM_TICKS_PER_SEC   2U      /* GPT 0.5s 一格 */

typedef enum
{
    LPM_ST_RUN = 0,                 /* 运行段（正常采集/上报） */
    LPM_ST_SLEEP,                   /* Sleep 档：等待点 WFI */
    LPM_ST_SOFT,                    /* Software Standby 档 */
    LPM_ST_DEEP,                    /* Deep Standby 档（RESET 唤醒） */
} lpm_state_t;

static bool        s_ok    = false; /* LPM 打开成功才启用 */
static lpm_state_t s_state = LPM_ST_RUN;
static uint32_t    s_ticks = 0U;    /* 当前段已过 tick 数 */
static uint32_t    s_wakes = 0U;    /* 待机段已醒次数（≈ 秒） */

/* ================================================================== */
/*                              辅助                                   */
/* ================================================================== */

/* 待机前让高耗外设睡下：BH1750 连续测量模式约 0.12mA，不睡会污染读数 */
static void lpm_quiet_peripherals (void)
{
    uint8_t cmd = 0x00U;    /* BH1750 power down */

    (void) i2c_bus_write(APP_ADDR_BH1750_GND, &cmd, 1U);
}

#if !APP_LPM_TEST_ONESHOT
/* 唤醒后把外设恢复回来（仅循环演示模式用：单发模式一睡到底不回头） */
static void lpm_restore_peripherals (void)
{
    (void) bh1750_init();   /* 重新上电 + 连续高分辨率模式 */
}
#endif

/* 运行段 → Sleep 档（WFI）：最浅的一档，时钟全在，靠 GPT tick 唤醒 */
static void lpm_enter_sleep (void)
{
    lpm_cfg_t cfg = g_lpm0_cfg;

    cfg.low_power_mode = LPM_MODE_SLEEP;
    (void) R_LPM_LowPowerReconfigure(&g_lpm0_ctrl, &cfg);

    s_state = LPM_ST_SLEEP;
    s_ticks = 0U;

    printf("[LPM] -> SLEEP (WFI idle)\r\n");
}

/* → Software Standby：停 GPT，RTC 秒中断唤醒计数 */
static void lpm_enter_soft (void)
{
    lpm_cfg_t cfg = g_lpm0_cfg;

    printf("[LPM] -> Software Standby (%us): RTC 1s wake, GPT stopped\r\n",
           (unsigned) APP_LPM_TEST_STBY_S);

    lpm_quiet_peripherals();
    (void) R_GPT_Stop(&g_timer0_ctrl);

    cfg.low_power_mode = LPM_MODE_STANDBY;
    cfg.standby_wake_sources = (lpm_standby_wake_source_bits_t)
                               LPM_STANDBY_WAKE_SOURCE_RTCPRD;
    (void) R_LPM_LowPowerReconfigure(&g_lpm0_ctrl, &cfg);

    s_state = LPM_ST_SOFT;
    s_wakes = 0U;
}

/* → Deep Standby：只有 RESET 键能把我们弄出来（复位重启） */
static void lpm_enter_deep (void)
{
    lpm_cfg_t cfg = g_lpm0_cfg;

    printf("[LPM] -> Deep Standby (hold): press RESET to wake\r\n");

    (void) R_GPT_Stop(&g_timer0_ctrl);

    cfg.low_power_mode = LPM_MODE_DEEP;
    cfg.deep_standby_cancel_source = (lpm_deep_standby_cancel_source_bits_t)
                                     LPM_DEEP_STANDBY_CANCEL_SOURCE_RESET_ONLY;
    cfg.io_port_state      = LPM_IO_PORT_NO_CHANGE;
    cfg.power_supply_state = LPM_POWER_SUPPLY_DEEPCUT0;
    (void) R_LPM_LowPowerReconfigure(&g_lpm0_ctrl, &cfg);

    s_state = LPM_ST_DEEP;
}

/* ================================================================== */
/*                              对外                                   */
/* ================================================================== */

void lpm_test_init (void)
{
    fsp_err_t err = R_LPM_Open(&g_lpm0_ctrl, &g_lpm0_cfg);

    if (FSP_SUCCESS != err)
    {
        printf("[LPM] open failed: %d (test disabled)\r\n", (int) err);
        return;
    }

    /* 深待机复位唤醒检测（RESET 键唤醒不置此位，只有深待机取消会置） */
    if (R_SYSTEM->RSTSR0_b.DPSRSTF)
    {
        printf("[LPM] <- reset cause: Deep Standby wake\r\n");
        R_SYSTEM->RSTSR0 = 0U;      /* 清复位标志 */
    }

#if APP_LPM_TEST_ONESHOT
    printf("[LPM] one-shot ladder: RUN %us -> SLEEP %us -> SW-STANDBY %us -> DEEP (hold)\r\n",
           (unsigned) APP_LPM_TEST_RUN_S, (unsigned) APP_LPM_TEST_SLEEP_S,
           (unsigned) APP_LPM_TEST_STBY_S);
#else
    printf("[LPM] loop test: mode=%u, RUN=%us, STBY=%us\r\n",
           (unsigned) APP_LPM_TEST_MODE,
           (unsigned) APP_LPM_TEST_RUN_S,
           (unsigned) APP_LPM_TEST_STBY_S);
#endif

    s_ok    = true;
    s_state = LPM_ST_RUN;
    s_ticks = 0U;
}

void lpm_test_wait_hook (void)
{
    if (!s_ok)
    {
        return;                     /* 没启用 → 保持原忙等行为 */
    }

#if APP_LPM_TEST_ONESHOT
    /* ---------- 单发阶梯：RUN → SLEEP → SOFT → DEEP ---------- */
    switch (s_state)
    {
        case LPM_ST_SLEEP:
        {
            (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* WFI：下一个中断才醒 */
            break;
        }

        case LPM_ST_SOFT:
        {
            if (s_wakes >= (uint32_t) APP_LPM_TEST_STBY_S)
            {
                /* 软待机时段满：转深待机（一去不回，直到按 RESET） */
                lpm_enter_deep();
            }
            else if (FSP_SUCCESS == R_LPM_LowPowerModeEnter(&g_lpm0_ctrl))
            {
                s_wakes++;          /* 每醒一次 ≈ RTC 的 1 秒 */
            }
            else
            {
                s_wakes++;          /* 进不去也计数，防止死循环卡死 */
            }
            break;
        }

        case LPM_ST_DEEP:
        {
            (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* 只有 RESET 键能弄出来 */
            break;
        }

        default:                    /* LPM_ST_RUN：维持原样（忙等） */
        {
            break;
        }
    }
#else
    /* ---------- 循环演示：RUN ↔ MODE 所选档 反复 ---------- */
    switch (s_state)
    {
        case LPM_ST_SLEEP:
        {
            (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* WFI：下一个 GPT tick 才醒 */
            break;
        }

        case LPM_ST_SOFT:
        {
            if (s_wakes >= (uint32_t) APP_LPM_TEST_STBY_S)
            {
                /* 待机段结束：恢复 GPT 与外设，回运行段 */
                (void) R_GPT_Start(&g_timer0_ctrl);
                lpm_restore_peripherals();
                s_state = LPM_ST_RUN;
                s_ticks = 0U;
                printf("[LPM] <- woke (%lu s); back to RUN\r\n",
                       (unsigned long) s_wakes);
            }
            else if (FSP_SUCCESS == R_LPM_LowPowerModeEnter(&g_lpm0_ctrl))
            {
                s_wakes++;          /* 每醒一次 ≈ RTC 的 1 秒 */
            }
            else
            {
                s_wakes++;          /* 进不去也计数，防止死循环卡死 */
            }
            break;
        }

        case LPM_ST_DEEP:
        {
            (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* RESET 键才出得来 */
            break;
        }

        default:                    /* LPM_ST_RUN：维持原样（忙等） */
        {
            break;
        }
    }
#endif
}

void lpm_test_tick_hook (void)
{
    if (!s_ok)
    {
        return;
    }

    s_ticks++;

#if APP_LPM_TEST_ONESHOT
    /* ---------- 单发阶梯的计时与推进 ---------- */
    if (LPM_ST_RUN == s_state)
    {
        if (s_ticks >= ((uint32_t) APP_LPM_TEST_RUN_S * LPM_TICKS_PER_SEC))
        {
            lpm_enter_sleep();
        }
    }
    else if (LPM_ST_SLEEP == s_state)
    {
        if (s_ticks >= ((uint32_t) APP_LPM_TEST_SLEEP_S * LPM_TICKS_PER_SEC))
        {
            lpm_enter_soft();
        }
    }
    else
    {
        /* SOFT/DEEP 段 GPT 已停，tick 本来也不会来；SOFT 的推进在 wait_hook */
    }
#else
    /* ---------- 循环演示的计时与推进 ---------- */
    if (LPM_ST_RUN == s_state)
    {
        if (s_ticks >= ((uint32_t) APP_LPM_TEST_RUN_S * LPM_TICKS_PER_SEC))
        {
            s_ticks = 0U;
            s_wakes = 0U;

            if (1U == APP_LPM_TEST_MODE)
            {
                lpm_enter_sleep();
            }
            else if (2U == APP_LPM_TEST_MODE)
            {
                lpm_enter_soft();
            }
            else
            {
                lpm_enter_deep();
            }
        }
    }
    else if (LPM_ST_SLEEP == s_state)
    {
        if (s_ticks >= ((uint32_t) APP_LPM_TEST_STBY_S * LPM_TICKS_PER_SEC))
        {
            s_state = LPM_ST_RUN;
            s_ticks = 0U;
            printf("[LPM] <- sleep phase done; back to RUN\r\n");
        }
    }
    else
    {
        /* SOFT/DEEP 段由 wait_hook 负责推进（GPT 已停，tick 本来也不会来） */
    }
#endif
}

#endif /* APP_LPM_TEST_ENABLE */
