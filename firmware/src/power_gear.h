/*
 * power_gear.h —— 触摸换挡：左右触摸焊盘切换三档功耗模式（RUN / SLEEP / STANDBY）。
 * 挂钩方式与 lpm_test 相同：init 一次 + wait_hook / tick_hook 进出主循环。
 *
 * © 2026 MoRiv447 · MIT License
 */
#pragma once

/**
 * @brief 初始化（R_LPM 打开、触摸自检、打印换挡说明）
 */
void power_gear_init (void);

/**
 * @brief 主循环空闲分支调用：SLEEP 档 WFI、STANDBY 档进软件待机 + 醒来扫触摸
 */
void power_gear_wait_hook (void);

/**
 * @brief 每 0.5s 一格调用：扫描触摸、执行换挡
 */
void power_gear_tick_hook (void);
