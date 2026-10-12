/*
 * app_util.h —— 浮点打印（nano 安全）+ 滤波 + 统计。
 *
 * ⚠️ 本工程链接 newlib-nano，默认关浮点格式化：printf("%f") 不报错、直接打印空白。
 * 所有浮点输出走本文件的格式化函数，新代码禁止直接写 %f
 * （要标准 %f 需链接 -u _printf_float，代码体积 +10KB，不划算）。
 *
 * © 2026 MoRiv447 · MIT License
 */

#pragma once

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================== 浮点打印（nano 安全） ===================== */

/**
 * @brief 按指定小数位打印一个浮点数（不用 %f）
 *
 * @param v         待打印的值
 * @param decimals  小数位数，0~4
 *
 * 内部先四舍五入到目标精度再拆成整数/小数两段打印，
 * 避免 25.999 被打印成 25.99。
 */
void app_print_fix(float v, uint8_t decimals);

/* 常用精度快捷方式 */
static inline void app_print_f1 (float v) { app_print_fix(v, 1U); }
static inline void app_print_f2 (float v) { app_print_fix(v, 2U); }
static inline void app_print_f3 (float v) { app_print_fix(v, 3U); }

/* ================== DWT 周期计数器（微秒级计时） ==================
 * 用途有两处：
 *   1. DHT 单总线要判别 26us / 70us 的脉宽 —— 软延时精度不够
 *   2. ADC 采样速率实测 —— 量一段连续转换花了多少周期
 * 对外只暴露「取当前周期数」和「微秒换算」，寄存器细节收在 .c 里。
 */

/**
 * @brief 使能 DWT 周期计数器（可重复调用，幂等）
 *
 * @return true 表示 CYCCNT 已开始计数；false 表示该芯片/调试配置下不可用
 */
bool app_dwt_init (void);

/** @brief 当前 CPU 周期数。未初始化时返回 0 */
uint32_t app_dwt_cycles (void);

/** @brief 微秒 → CPU 周期数（按 SystemCoreClock 换算） */
uint32_t app_dwt_us_to_cycles (uint32_t us);

/* ===================== 中值滤波 ===================== */

#define APP_MEDIAN_MAX  9U

typedef struct
{
    uint16_t buf[APP_MEDIAN_MAX];
    uint8_t  win;       /* 实际窗口长度，奇数 */
    uint8_t  idx;
    uint8_t  count;     /* 已填入的样本数，未填满时用现有值排序 */
} app_median_t;

/**
 * @brief 初始化中值滤波窗口
 *
 * @param win 窗口长度，必须为奇数且 ≤ APP_MEDIAN_MAX
 */
void app_median_init (app_median_t * m, uint8_t win);

/**
 * @brief 投入一个样本并返回当前中值
 *
 * 用途：剔掉单次离群点。比如 ADC 被触摸动作/继电器吸合干扰时
 * 会跳出一个孤立的高值，中值滤波能整条剔掉，而滑动平均只会把它摊薄。
 */
uint16_t app_median_push (app_median_t * m, uint16_t v);

/* ===================== 滑动平均 ===================== */

#define APP_AVERAGE_MAX 32U

typedef struct
{
    float   buf[APP_AVERAGE_MAX];
    uint8_t win;
    uint8_t idx;
    uint8_t count;
    float   sum;
} app_average_t;

void  app_average_init (app_average_t * a, uint8_t win);
float app_average_push (app_average_t * a, float v);

/* ===================== 统计（噪声评估用） ===================== */

typedef struct
{
    float mean;     /* 均值 */
    float vmin;     /* 最小值 */
    float vmax;     /* 最大值 */
    float pp;       /* 峰峰值 = vmax - vmin，最直观的噪声指标 */
    float sigma;    /* 标准差（RMS 噪声） */
} app_stats_t;

/**
 * @brief 对一组原始 ADC 值算统计量
 *
 * 这是「ADC 噪声抑制实测」那一节的原始数据来源：
 *   - pp（峰峰值）看最坏跳变
 *   - sigma（标准差）看整体抖动
 *   - 开/关 FSP 里的 ADC 硬件平均（add_average_count）再各测一遍，
 *     两个 sigma 一对比，就是「噪声抑制」的量化证据。
 */
void app_stats_calc (const uint16_t * samples, uint32_t n, app_stats_t * out);
