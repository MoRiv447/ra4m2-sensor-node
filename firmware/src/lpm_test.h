/*
 * lpm_test.h —— 低功耗四平台测量固件（单发阶梯 RUN→SLEEP→SW-STANDBY→DEEP）。
 * 开关在 app_config.h（APP_LPM_TEST_ENABLE / ONESHOT）。
 *
 * © 2026 MoRiv447 · MIT License
 */
#pragma once

/**
 * @brief 初始化（在 self_test() 之后、进主循环之前调用一次）
 */
void lpm_test_init (void);

/**
 * @brief 主循环空闲分支调用：Sleep/待机段在这里真正睡下（WFI）
 */
void lpm_test_wait_hook (void);

/**
 * @brief 每 0.5s 一格调用：各段的计时与切换
 */
void lpm_test_tick_hook (void);
