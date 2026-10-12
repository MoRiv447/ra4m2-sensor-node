/***********************************************************************************************************************
* Copyright (c) 2020 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
************************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : qe_touch_config.c
* Description  : This file includes module implementations.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History      : MM/DD/YYYY Version Description
*              : 09/02/2019 1.00    First Release
*              : 10/07/2019 1.00a   Provisional Version for RA2L1
*              : 12/26/2019 1.10    Corresponding for FSP V0.10.0
*              : 02/20/2020 1.20    Corresponding for FSP V0.12.0
*              : 02/26/2020 1.21    Adding information for Temperature Correction
*              : 03/04/2020 1.30    Corresponding for FSP V1.0.0 RC0
*              : 03/10/2020 1.31    Corresponding for FSP V1.0.0 RC1
*              : 09/10/2020 1.40    Corresponding for FSP V2.0.0 Beta2
*              : 05/26/2021 1.50    Adding Diagnosis Supporting
*              : 06/01/2021 1.51    Fixing a Little
*              : 07/15/2021 1.52    Fixing a Little
*              : 08/20/2021 1.53    Fixing a Little
*              : 11/13/2021 1.60    Adding information for Initial Offset Tuning
*              : 03/31/2023 1.61    Improving Traceability
*              : 04/18/2023 1.70    Adding 3 Frequency Judgement Supporting
*              : 07/30/2024 1.80    Adding Auto Judgement Supporting
*              : 10/03/2025 1.90    Improving Diagnosis function
***********************************************************************************************************************/
/***********************************************************************************************************************
* Touch I/F Configuration File  : ra4m2_flat.tifcfg
***********************************************************************************************************************/

#include "qe_touch_config.h"

volatile uint8_t      g_qe_touch_flag;
volatile ctsu_event_t g_qe_ctsu_event;

void qe_touch_callback(touch_callback_args_t * p_args)
{
    g_qe_touch_flag = 1;
    g_qe_ctsu_event = p_args -> event;
}



















/***********************************************************************************************************************
CTSU Related Information for [CONFIG01] configuration.
***********************************************************************************************************************/

const ctsu_element_cfg_t g_qe_ctsu_element_cfg_config01[] =
{
    /* 电极调谐参数（ssdiv/so/snum/sdpa）：从官方例程 RA4M2_Project20 移植
     * （本工程没跑 QE 自动调参）—— P206(TS02) so=0x052、P409(TS05) so=0x06E；
     * 中间两个电极（P407/P408）为插值估算，某一路不灵敏时优先怀疑它们。 */
    { .ssdiv = CTSU_SSDIV_0670, .so = 0x052, .snum = 0x01, .sdpa = 0x01 },   /* TS02 (P206) — 右焊盘 TS1 */
    { .ssdiv = CTSU_SSDIV_0670, .so = 0x06E, .snum = 0x01, .sdpa = 0x01 },   /* TS05 (P409) — 左焊盘 TS2 */
};

const ctsu_cfg_t g_qe_ctsu_cfg_config01 =
{
    .cap = CTSU_CAP_SOFTWARE,

    .txvsel = CTSU_TXVSEL_VCC,

    .atune1 = CTSU_ATUNE1_NORMAL,

    .md     = CTSU_MODE_SELF_MULTI_SCAN,




    .ctsuchac0  = 0x24, /* ch0-ch7 enable mask */
    .ctsuchac1  = 0x00, /* ch8-ch15 enable mask */
    .ctsuchac2  = 0x00, /* ch16-ch23 enable mask */
    .ctsuchac3  = 0x00, /* ch24-ch31 enable mask */
    .ctsuchac4  = 0x00, /* ch32-ch39 enable mask */
    .ctsuchtrc0 = 0x00, /* ch0-ch7 mutual tx mask */
    .ctsuchtrc1 = 0x00, /* ch8-ch15 mutual tx mask */
    .ctsuchtrc2 = 0x00, /* ch16-ch23 mutual tx mask */
    .ctsuchtrc3 = 0x00, /* ch24-ch31 mutual tx mask */
    .ctsuchtrc4 = 0x00, /* ch32-ch39 mutual tx mask */
    .num_rx     = 2,
    .num_tx     = 0,
    .p_elements = g_qe_ctsu_element_cfg_config01,

#if (CTSU_TARGET_VALUE_CONFIG_SUPPORT == 1)
    .tuning_self_target_value   = 15360,
    .tuning_mutual_target_value = 10240,
#endif

    /* 平滑窗口 4→1：慢节奏扫描下 4 点平滑要按足 3 拍才越阈值；原始噪声只有几十 LSB，取消平滑。 */
    .num_moving_average = 1,
    .tunning_enable     = true,
    .p_callback    = &qe_touch_callback,
#if (CTSU_CFG_DTC_SUPPORT_ENABLE == 1)
    .p_transfer_tx = &g_transfer0,
    .p_transfer_rx = &g_transfer1,
#else
    .p_transfer_tx = NULL,
    .p_transfer_rx = NULL,
#endif

    .write_irq = CTSU_WRITE_IRQn,
    .read_irq  = CTSU_READ_IRQn,
    .end_irq   = CTSU_END_IRQn,

};

ctsu_instance_ctrl_t g_qe_ctsu_ctrl_config01;

const ctsu_instance_t g_qe_ctsu_instance_config01 =
{
    .p_ctrl = &g_qe_ctsu_ctrl_config01,
    .p_cfg  = &g_qe_ctsu_cfg_config01,
    .p_api  = &g_ctsu_on_ctsu,
};

/***********************************************************************************************************************
Touch Related Information for [CONFIG01] configuration.
***********************************************************************************************************************/

#define QE_TOUCH_CONFIG01_NUM_BUTTONS (2)
#define QE_TOUCH_CONFIG01_NUM_SLIDERS (0)
#define QE_TOUCH_CONFIG01_NUM_WHEELS  (0)
#define QE_TOUCH_CONFIG01_NUM_TOUCH_PADS (0)

/* Button configurations */
#if (QE_TOUCH_CONFIG01_NUM_BUTTONS != 0)
const touch_button_cfg_t g_qe_touch_button_cfg_config01[] =
{


    /* button00 → elem 0 = P206（右焊盘 TS1）：不碰 ~2.6k / 按下 ~18k，阈值取中点偏下 */
    {
        .elem_index = 0,
        .threshold  = 10700,
        .hysteresis = 500,
    },
    /* button01 → elem 1 = P409（左焊盘 TS2）：不碰 ~3.2k / 按下 ~14k+，阈值取中点偏下 */
    {
        .elem_index = 1,
        .threshold  = 8700,
        .hysteresis = 400,
    },
};
#endif

/* Slider configurations */
#if (QE_TOUCH_CONFIG01_NUM_SLIDERS != 0)
const touch_slider_cfg_t g_qe_touch_slider_cfg_config01[] =
{
    NULL
};
#endif

/* Wheel configurations */
#if (QE_TOUCH_CONFIG01_NUM_WHEELS != 0)
const touch_wheel_cfg_t g_qe_touch_wheel_cfg_config01[] =
{
    NULL
};
#endif

/* Touch pad configurations */
#if (QE_TOUCH_CONFIG01_NUM_TOUCH_PADS != 0)
const touch_pad_cfg_t g_qe_touch_touch_pad_cfg_config01 =
{
    NULL
};
#endif

/* Touch configurations */
const touch_cfg_t g_qe_touch_cfg_config01 =
{
    .p_buttons   = g_qe_touch_button_cfg_config01,
    .p_sliders   = NULL,
    .p_wheels    = NULL,
#if (TOUCH_CFG_PAD_ENABLE != 0)
    .p_pad       = NULL,
#endif
    .num_buttons = QE_TOUCH_CONFIG01_NUM_BUTTONS,
    .num_sliders = QE_TOUCH_CONFIG01_NUM_SLIDERS,
    .num_wheels  = QE_TOUCH_CONFIG01_NUM_WHEELS,

    .number        = 0,
#if ((TOUCH_CFG_UART_MONITOR_SUPPORT == 1) || (TOUCH_CFG_UART_TUNING_SUPPORT == 1))
    .p_uart_instance = &g_uart_qe,
#else
    .p_uart_instance = NULL,
#endif

    /* 扫描节奏 0.5s/格：on/off_freq=3 需连续 3 次判定（1.5s）太迟钝，改 1 —— 按一下即触发。 */
    .on_freq     = 1,
    .off_freq    = 1,
    .drift_freq  = 255,
    .cancel_freq = 0,

    .p_ctsu_instance = &g_qe_ctsu_instance_config01,
};

touch_instance_ctrl_t g_qe_touch_ctrl_config01;

const touch_instance_t g_qe_touch_instance_config01 =
{
    .p_ctrl = &g_qe_touch_ctrl_config01,
    .p_cfg  = &g_qe_touch_cfg_config01,
    .p_api  = &g_touch_on_ctsu,
};

