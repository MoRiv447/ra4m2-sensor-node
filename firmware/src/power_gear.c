/*
 * power_gear.c —— 触摸换挡：左右触摸焊盘切换三档功耗模式。
 *
 *   左键（TS2/P409）= 更省电：RUN → SLEEP → STANDBY
 *   右键（TS1/P206）= 更活跃：STANDBY → SLEEP → RUN
 *   STANDBY 档数据暂停（软件待机 + RTC 每秒轮询触摸，摸到即唤醒回 RUN）。
 *
 * 两个已处理的坑：① CTSU 自动调谐 —— 上电先跑"调谐热身"（gear_touch_warmup）；
 * ② 进/出同为触摸手势 —— 进待机后先等手指松开、再开放唤醒（s_wake_armed）。
 *
 * © 2026 MoRiv447 · MIT License
 */
#include "power_gear.h"

#include "hal_data.h"
#include "app_config.h"
#include "sensors.h"
#include "qe_touch_config.h"    /* QE 生成的触摸实例（RM_TOUCH） */
#include <stdio.h>

#if APP_POWER_GEAR_ENABLE

/* 触摸扫描等待上限：1ms 轮询、最多等 200ms（正常扫描只需几 ms，超时保护用） */
#define GEAR_TOUCH_WAIT_MS      (200U)

/* 调谐热身上限（次）；实测本板 227 次完成，留足余量 */
#define GEAR_TUNE_WARMUP_MAX    (400U)

/* RUN 档补扫节流：忙等里每 ~100ms 补扫一次，让"轻按"也能抓到
 *（主 tick 0.5s 一拍对快速点按太粗；SLEEP/STANDBY 档保持慢节奏） */
#define GEAR_RUN_RESCAN_MS      (100U)

/* 按键位图：bit0 = button00 = TS1/P206（右焊盘）；bit1 = button01 = TS2/P409（左焊盘） */
#define GEAR_PAD_RIGHT          (1ULL << 0)
#define GEAR_PAD_LEFT           (1ULL << 1)

typedef enum e_power_gear
{
    GEAR_RUN = 1,               /* 全速（忙等） */
    GEAR_SLEEP,                 /* WFI 空闲 */
    GEAR_STANDBY,               /* 软件待机 + 触摸轮询唤醒 */
} power_gear_t;

static bool         s_ok       = false;
static power_gear_t s_gear     = GEAR_RUN;
static uint64_t     s_buttons  = 0U;    /* 上一次扫描的按键位图（做"新按下"边沿） */
static bool         s_touch_ok = true;  /* 待机中扫描连续失败后置 false，停用轮询 */
static uint8_t      s_touch_fail = 0U;
static uint8_t      s_dbg_prints = 0U;  /* 异常打印计数（防刷屏） */
static uint32_t     s_last_scan_cyc = 0U;   /* RUN 档补扫计时基准（DWT cycles） */
static bool         s_wake_armed   = false; /* STANDBY：等手指松开后，才开放"触摸唤醒" */

/* ================================================================== */
/*                              触摸                                   */
/* ================================================================== */

/* 扫一次。返回：
 *   FSP_SUCCESS                    —— 拿到结果（out = 按键位图）
 *   FSP_ERR_CTSU_INCOMPLETE_TUNING —— 初始调谐进行中（正常，调用方自行忽略）
 *   其它                            —— 真异常（已打诊断打印，并已尝试解卡） */
static fsp_err_t gear_touch_scan_ex (uint64_t * out)
{
    fsp_err_t err;
    uint32_t  waited_ms = 0U;

    g_qe_touch_flag = 0U;

    err = RM_TOUCH_ScanStart(g_qe_touch_instance_config01.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        if (s_dbg_prints < 8U)
        {
            s_dbg_prints++;
            printf("[GEAR][dbg] ScanStart 失败 err=%d\r\n", (int) err);
        }
        (void) RM_TOUCH_ScanStop(g_qe_touch_instance_config01.p_ctrl);  /* 解卡 */
        return err;
    }

    while (0U == g_qe_touch_flag)
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        if (++waited_ms > GEAR_TOUCH_WAIT_MS)
        {
            break;
        }
    }

    if (0U == g_qe_touch_flag)
    {
        if (s_dbg_prints < 8U)
        {
            s_dbg_prints++;
            printf("[GEAR][dbg] 扫描超时 %lums：CTSUST=%02X CR0=%02X CR1=%02X ERRS=%02X\r\n",
                   (unsigned long) waited_ms,
                   (unsigned) R_CTSU->CTSUST, (unsigned) R_CTSU->CTSUCR0,
                   (unsigned) R_CTSU->CTSUCR1, (unsigned) R_CTSU->CTSUERRS);
            printf("[GEAR][dbg] NVIC(en/pri)：W=%u/%u RD=%u/%u END=%u/%u\r\n",
                   (unsigned) NVIC_GetEnableIRQ(CTSU_WRITE_IRQn), (unsigned) NVIC_GetPriority(CTSU_WRITE_IRQn),
                   (unsigned) NVIC_GetEnableIRQ(CTSU_READ_IRQn),  (unsigned) NVIC_GetPriority(CTSU_READ_IRQn),
                   (unsigned) NVIC_GetEnableIRQ(CTSU_END_IRQn),   (unsigned) NVIC_GetPriority(CTSU_END_IRQn));
        }
        (void) RM_TOUCH_ScanStop(g_qe_touch_instance_config01.p_ctrl);  /* 解卡 */
        return FSP_ERR_CTSU_NOT_GET_DATA;   /* 超时：数据没来 */
    }
    g_qe_touch_flag = 0U;

    if (CTSU_EVENT_SCAN_COMPLETE != g_qe_ctsu_event)
    {
        if (s_dbg_prints < 8U)
        {
            s_dbg_prints++;
            printf("[GEAR][dbg] 事件异常 event=%d（0=SCAN_COMPLETE 正常）\r\n", (int) g_qe_ctsu_event);
        }
        return FSP_ERR_CTSU_NOT_GET_DATA;
    }

    err = RM_TOUCH_DataGet(g_qe_touch_instance_config01.p_ctrl, out, NULL, NULL);
    if ((FSP_SUCCESS != err) && (FSP_ERR_CTSU_INCOMPLETE_TUNING != err) && (s_dbg_prints < 8U))
    {
        s_dbg_prints++;
        printf("[GEAR][dbg] DataGet 失败 err=%d\r\n", (int) err);
    }
    return err;
}

/* 调谐热身：快扫直到 DataGet 第一次成功（烧穿 CTSU 初始偏移调谐） */
static void gear_touch_warmup (void)
{
    uint32_t i;
    uint64_t dummy = 0U;

    for (i = 0U; i < GEAR_TUNE_WARMUP_MAX; i++)
    {
        fsp_err_t err = gear_touch_scan_ex(&dummy);

        if (FSP_SUCCESS == err)
        {
            printf("[GEAR] 触摸调谐完成（%lu 次扫描）\r\n", (unsigned long) (i + 1U));
            return;
        }
        if (FSP_ERR_CTSU_INCOMPLETE_TUNING != err)
        {
            printf("[GEAR] 调谐热身中断（第 %lu 次，err=%d），稍后按正常节奏重试\r\n",
                   (unsigned long) (i + 1U), (int) err);
            return;
        }
    }
    printf("[GEAR] 注意：调谐未在 %u 次内完成\r\n", (unsigned) GEAR_TUNE_WARMUP_MAX);
}

/* ================================================================== */
/*                              档位                                   */
/* ================================================================== */

static const char * gear_name (power_gear_t g)
{
    switch (g)
    {
        case GEAR_RUN:     return "RUN";
        case GEAR_SLEEP:   return "SLEEP";
        case GEAR_STANDBY: return "STANDBY";
        default:           return "?";
    }
}

/* LPM 配置回 SLEEP 档（WFI 用）；离开待机后必须回一次，
 * 否则 SBY 位还立着，SLEEP 档的 WFI 会直接进软件待机 */
static void gear_lpm_cfg_sleep (void)
{
    lpm_cfg_t cfg = g_lpm0_cfg;

    cfg.low_power_mode = LPM_MODE_SLEEP;
    (void) R_LPM_LowPowerReconfigure(&g_lpm0_ctrl, &cfg);
}

/* 进入 STANDBY：安静外设 → 停 GPT → 配好 RTC 唤醒源（真正睡下在 wait_hook） */
static void gear_enter_standby (void)
{
    uint8_t cmd = 0x00U;        /* BH1750 power down：连续测量 ~0.12mA，睡了它 */

    printf("[GEAR] -> STANDBY：数据暂停；松开手指后，触摸任意键唤醒\r\n");

    (void) i2c_bus_write(APP_ADDR_BH1750_GND, &cmd, 1U);
    (void) R_GPT_Stop(&g_timer0_ctrl);

    {
        lpm_cfg_t cfg = g_lpm0_cfg;

        cfg.low_power_mode = LPM_MODE_STANDBY;
        cfg.standby_wake_sources = (lpm_standby_wake_source_bits_t) LPM_STANDBY_WAKE_SOURCE_RTCPRD;
        (void) R_LPM_LowPowerReconfigure(&g_lpm0_ctrl, &cfg);
    }

    s_gear       = GEAR_STANDBY;
    s_wake_armed = false;       /* 先屏蔽唤醒：等手指松开（首次扫到 0）再开放 */
}

/* 从 STANDBY 醒来（摸到了）：恢复外设与数据流，回到 RUN 档 */
static void gear_wake_to_run (uint64_t held_buttons)
{
    printf("[GEAR] <- 触摸唤醒：回到 RUN 档\r\n");

    (void) bh1750_init();           /* 待机前 power down 过，这里重新上电 + 连续模式 */
    gear_lpm_cfg_sleep();           /* LPM 配置回 SLEEP 档（清掉 SBY 位） */
    (void) R_GPT_Start(&g_timer0_ctrl);

    s_gear    = GEAR_RUN;
    s_buttons = held_buttons;       /* 手指还按着的话，不算"新按下"，防抖 */
}

/* 真正执行换挡；到端点时提示不动作 */
static void gear_goto (power_gear_t want)
{
    if ((want < GEAR_RUN) || (want > GEAR_STANDBY) || (want == s_gear))
    {
        printf("[GEAR] 已在端档 %s：没有更%s的档位了\r\n",
               gear_name(s_gear), (want > s_gear) ? "省电" : "活跃");
        return;
    }

    if (GEAR_STANDBY == want)
    {
        gear_enter_standby();       /* 自己会置 s_gear，并在 wait_hook 里真正睡下 */
    }
    else if (GEAR_SLEEP == want)
    {
        s_gear = GEAR_SLEEP;
        gear_lpm_cfg_sleep();
        printf("[GEAR] -> SLEEP（WFI 空闲，数据节奏不变）\r\n");
    }
    else /* GEAR_RUN */
    {
        s_gear = GEAR_RUN;
        gear_lpm_cfg_sleep();
        printf("[GEAR] -> RUN（全速）\r\n");
    }
}

/* 处理一次按键位图：只响应"新按下"（边沿），左右键各换一个方向 */
static void gear_handle_buttons (uint64_t now_buttons)
{
    uint64_t press = now_buttons & ~s_buttons;

    /* 诊断：位图有变化留一行（限量），现场确认"按到了没有" */
    if ((now_buttons != s_buttons) && (s_dbg_prints < 20U))
    {
        s_dbg_prints++;
        printf("[GEAR][dbg] 按键位图 %llX -> %llX\r\n",
               (unsigned long long) s_buttons, (unsigned long long) now_buttons);
    }

    s_buttons = now_buttons;

    if (0U == press)
    {
        return;
    }

    /* ⚠ 方向映射：左键 = 更省电，右键 = 更活跃（要互换就调这两行） */
    if (0U != (press & GEAR_PAD_LEFT))
    {
        gear_goto((power_gear_t) ((int) s_gear + 1));
    }
    if (0U != (press & GEAR_PAD_RIGHT))
    {
        gear_goto((power_gear_t) ((int) s_gear - 1));
    }
}

/* 扫一次并按结果处理（主 tick 与 RUN 档补扫共用） */
static void gear_poll_touch_once (void)
{
    uint64_t  buttons = 0U;
    fsp_err_t err = gear_touch_scan_ex(&buttons);

    if (FSP_SUCCESS == err)
    {
        s_touch_fail = 0U;
        gear_handle_buttons(buttons);
    }
    else if (FSP_ERR_CTSU_INCOMPLETE_TUNING == err)
    {
        /* 调谐中（理论上热身已烧穿；真出现也属"进行中"，忽略） */
    }
    else if (s_touch_fail < 255U)
    {
        s_touch_fail++;
        if (3U == s_touch_fail)
        {
            printf("[GEAR] 触摸扫描连续异常——检查 CTSU 配置 / 触摸焊盘\r\n");
        }
    }
}

/* ================================================================== */
/*                              对外                                   */
/* ================================================================== */

void power_gear_init (void)
{
    fsp_err_t err;
    uint64_t  dummy = 0U;

    /* 现场自检信息（保留一行，方便排查） */
    printf("[GEAR] CTSU 中断(en/pri)：W=%u/%u RD=%u/%u END=%u/%u\r\n",
           (unsigned) NVIC_GetEnableIRQ(CTSU_WRITE_IRQn), (unsigned) NVIC_GetPriority(CTSU_WRITE_IRQn),
           (unsigned) NVIC_GetEnableIRQ(CTSU_READ_IRQn),  (unsigned) NVIC_GetPriority(CTSU_READ_IRQn),
           (unsigned) NVIC_GetEnableIRQ(CTSU_END_IRQn),   (unsigned) NVIC_GetPriority(CTSU_END_IRQn));

    /* 保险：确保三个 CTSU 中断在 NVIC 里是使能的（正常 Open 已做过；重复使能无害） */
    NVIC_EnableIRQ(CTSU_WRITE_IRQn);
    NVIC_EnableIRQ(CTSU_READ_IRQn);
    NVIC_EnableIRQ(CTSU_END_IRQn);

    /* ---- 调谐热身（详见文件头注释；上电多花几十毫秒到 ~1 秒） ---- */
    gear_touch_warmup();

    err = R_LPM_Open(&g_lpm0_ctrl, &g_lpm0_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[GEAR] LPM open failed: %d（换挡功能停用）\r\n", (int) err);
        return;
    }

    printf("[GEAR] 触摸换挡已就绪：左键=更省电（RUN->SLEEP->STANDBY），右键=更活跃\r\n");
    printf("[GEAR] 当前档位：RUN（STANDBY 档触摸任意键唤醒）\r\n");

    s_ok         = true;
    s_gear       = GEAR_RUN;
    s_buttons    = 0U;
    s_touch_ok   = true;
    s_touch_fail = 0U;

    err = gear_touch_scan_ex(&dummy);
    if (FSP_SUCCESS == err)
    {
        s_buttons = dummy;      /* 基线：避免把初始化时"正按着"的键当新按下 */
        printf("[GEAR] 触摸自检 OK（TS1/TS2 在线）\r\n");
    }
    else if (FSP_ERR_CTSU_INCOMPLETE_TUNING == err)
    {
        printf("[GEAR] 触摸仍在调谐中（正常运行后会完成）\r\n");
    }
    else
    {
        printf("[GEAR] 触摸自检失败——检查 RM_TOUCH_Open 与 CTSU 配置\r\n");
    }
}

void power_gear_wait_hook (void)
{
    if (!s_ok)
    {
        return;
    }

    if (GEAR_SLEEP == s_gear)
    {
        (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* WFI：等下一个 tick */
        return;
    }

    if (GEAR_STANDBY == s_gear)
    {
        uint64_t  buttons = 0U;
        fsp_err_t err;

        (void) R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);   /* 睡到 RTC 1s 唤醒 */

        if (!s_touch_ok)
        {
            return;                                     /* 轮询停用：守觉，等 RESET */
        }

        err = gear_touch_scan_ex(&buttons);
        if (FSP_SUCCESS == err)
        {
            s_touch_fail = 0U;

            if (!s_wake_armed)
            {
                /* 刚进待机时手指可能还按着（进待机就是按住触发的）——
                 * 先观察到一次"干净"（松开）才算就绪，防止自己唤醒自己 */
                if (0U == buttons)
                {
                    s_wake_armed = true;
                }
            }
            else if (0U != buttons)
            {
                gear_wake_to_run(buttons);
            }
        }
        else if (FSP_ERR_CTSU_INCOMPLETE_TUNING == err)
        {
            /* 调谐中：静默，继续睡 */
        }
        else
        {
            if (s_touch_fail < 255U)
            {
                s_touch_fail++;
            }
            if (s_touch_fail >= 5U)
            {
                s_touch_ok = false;
                printf("[GEAR] 待机中触摸扫描连续失败——轮询停用（按 RESET 重来）\r\n");
            }
        }
        return;
    }

    /* GEAR_RUN：忙等 + ~100ms 节流补扫（让"轻按"也能抓到） */
    {
        uint32_t now_cyc = DWT->CYCCNT;
        uint32_t gap_cyc = SystemCoreClock / (1000U / GEAR_RUN_RESCAN_MS);

        if ((uint32_t) (now_cyc - s_last_scan_cyc) >= gap_cyc)
        {
            s_last_scan_cyc = now_cyc;
            gear_poll_touch_once();
        }
    }
}

void power_gear_tick_hook (void)
{
    if (!s_ok)
    {
        return;
    }
    if (GEAR_STANDBY == s_gear)
    {
        return;                     /* 待机段 GPT 已停，tick 不会来；保险起见直接返回 */
    }

    s_last_scan_cyc = DWT->CYCCNT;  /* 计时基准：避免 tick 与 RUN 补扫连打 */
    gear_poll_touch_once();
}

#endif /* APP_POWER_GEAR_ENABLE */
