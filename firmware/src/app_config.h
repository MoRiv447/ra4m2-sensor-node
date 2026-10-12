/*
 * app_config.h —— 全局配置：引脚 / 采集节奏 / ADC / 传感器 / 功能开关。
 * 接线细节与说明文档见 docs/；本文件只留"代码与 FSP 配置之间的契约"。
 *
 * © 2026 MoRiv447 · MIT License
 */

#pragma once

#include "hal_data.h"

/* ===================== 调试串口（沿用第 2 篇配置） ===================== */

#define APP_UART_BAUD               115200

/* 输出格式：0 = 人类可读表格（截图用）/ 1 = CSV 单行（上位机画曲线用） */
#define APP_OUT_CSV                 0

/* ========================== 采集节奏 ==========================
 * 基准节拍 = GPT Ch0 的 500ms 周期中断，各传感器按 tick 计数换算。
 * 间隔下限由器件决定：DHT22 ≥2s、BH1750 转换 ~120ms；只有 ADC 走高节奏。 */
#define APP_TICK_MS                 500U    /* 与 FSP 里 GPT 的 period 保持一致 */
#define APP_TICKS_FAST_ADC          1U      /* 每 1 个 tick(0.5s) 采一轮 ADC */
#define APP_TICKS_SLOW_I2C          6U      /* 每 6 个 tick(3s) 读 BH1750 + BMP280 */
#define APP_TICKS_DHT               8U      /* 每 8 个 tick(4s) 读 DHT（≥2s 要求） */
#define APP_TICKS_REPORT            20U     /* 每 20 个 tick(10s) 打一行汇总 */
#define APP_TICKS_BT_REPORT         10U     /* 每 10 个 tick(5s) 蓝牙播报一行 */
#define APP_TICKS_WIFI_REPORT       20U     /* 每 20 个 tick(10s) WiFi 云端上报一帧 */

/* ===================== 低功耗测量 / 触摸换挡（二选一） =====================
 * LPM_TEST_ENABLE=1：测量固件 —— 上电自动走"功耗阶梯"（盲测）：
 *     RUN 15s → SLEEP 15s → SW-STANDBY 60s → DEEP（保持，RESET 醒）；
 *     分流电阻上的表读数逐段下台阶；配方见 docs/低功耗三档-测量配方.md。
 * POWER_GEAR_ENABLE=1：成品固件 —— 触摸换挡手动切档（见 power_gear.c）。
 * 常态：阶梯关、换挡开；要重测阶梯就两个宏对调。
 * MODE（仅循环模式用）：1=Sleep / 2=SW-Standby / 3=Deep Standby */
#define APP_LPM_TEST_ENABLE     0
#define APP_LPM_TEST_ONESHOT    1
#define APP_LPM_TEST_MODE       2
#define APP_LPM_TEST_RUN_S      15U
#define APP_LPM_TEST_SLEEP_S    15U
#define APP_LPM_TEST_STBY_S     60U

/* 触摸换挡：左焊盘=更省电 / 右焊盘=更活跃（见 power_gear.c） */
#define APP_POWER_GEAR_ENABLE   1

/* ADC 采样组：每组连续采 APP_ADC_BURST 次，算出 mean / pp / sigma（噪声评估三件套）。 */
#define APP_ADC_BURST               16U

/* 模拟通道：P014=AN012（通道 A）/ P015=AN013（通道 B）。
 * ⚠️ MQ-2 之类 0~5V 输出必须先分压到 3.3V 以内再进 ADC。 */
#define APP_ADC_CH_A                ADC_CHANNEL_12   /* P014 */
#define APP_ADC_CH_B                ADC_CHANNEL_13   /* P015 */

#define APP_ADC_VREF_MV             3300.0f
#define APP_ADC_FULL_SCALE          4096.0f

/* 等扫描结束的超时（µs）：转换实际只需几 µs，2ms 是"防死等"的宽松兜底。 */
#define APP_ADC_SCAN_TIMEOUT_US     2000U

/* 每次 ScanStart 会转换 scan_mask 里的全部通道（AN012+AN013+TSN = 3 路）——
 * 算采样速率时按"通道数 × 平均次数"除；改 FSP 的 scan_mask 后同步改这个数。 */
#define APP_ADC_SCAN_CHANNELS       3U

/* ADC 硬件平均次数（与 FSP add_average_count 一致：1 / 4 / 16 / 64）。
 * 影响两处：①单次转换耗时 = 周期数 ÷(通道数×N)；②σ 的含义（N=1 原始噪声，N=4 已抑制）。 */
#define APP_ADC_AVERAGE_TIMES       4

/* 通道 A 输入源：0 = 固定分压（默认，干净基准，可算绝对精度）/ 1 = 光敏电阻分压。 */
#define APP_ADC_A_INPUT             0       /* 0 = 固定分压（默认）  1 = 光敏电阻 */

/* 固定分压：3V3─[R_HIGH]─┬─P014，└─[R_LOW]─GND；同阻值最省事（预期 1650 mV）。 */
#define APP_FIXED_R_HIGH_OHM        10000.0f
#define APP_FIXED_R_LOW_OHM         10000.0f

/* 通道 B（P015）是否接线：不接就报 n/a（悬空引脚读数会乱漂，不给假数字）。 */
#define APP_ADC_B_ENABLE            1       /* 2026-10-09：已接光敏电阻（LDR）分压到 P015 */

/* LDR 标定（仅 APP_ADC_A_INPUT==1）：
 * R_ldr = R10×(3V3/V−1)；lux = lux_ref×(R/R_ref)^(−1/γ)，γ≈0.8（GL5528 典型值）。
 * 属"量级可用"的相对照度，要准就用自己的两点实测替换。 */
#define APP_LDR_R_SERIES_OHM        10000.0f  /* 上拉/下拉的那个固定电阻 */
#define APP_LDR_R_REF_OHM           10000.0f  /* 参考照度对应的电阻 */
#define APP_LDR_LUX_REF             10.0f     /* 参考照度 (lx) */
#define APP_LDR_GAMMA               0.8f      /* 幂律指数 γ */

/* 判定阈值（相对照度，单位 lx），只为演示「模拟量分档」这个动作 */
#define APP_LDR_LUX_DARK            10.0f
#define APP_LDR_LUX_BRIGHT          200.0f

/* FSP 生成物命名契约：各 Stack 的 Name 属性会生成 g_uart9 / g_rtc0 / g_timer0 /
 * g_adc0 / g_ioport 等符号（ra_gen/hal_data.c）；名字不符 = undefined reference，
 * 对不上时改 FSP 的 Name，而不是改代码。 */

/* 软件 I2C：SCL = P000、SDA = P001（标 2 第 6、7 孔；硬件 SCI2 让给了 WiFi）。
 * ⚠️ P000/P001 无板载上拉 → 释放时内部上拉兜底，建议外并 4.7k~10k（宁慢勿错）。 */
#define APP_SW_I2C_SCL_PIN      BSP_IO_PORT_00_PIN_00
#define APP_SW_I2C_SDA_PIN      BSP_IO_PORT_00_PIN_01
#define APP_SW_I2C_HALF_US      20U     /* 半周期 20us ≈ 25kHz（保守；SDA 靠内部上拉爬升，宁慢勿错） */

/* 器件地址（7bit，按 ADDR / SDO 接法二选一）： */
#define APP_ADDR_BH1750_GND         0x23U   /* BH1750 ADDR 接 GND */
#define APP_ADDR_BH1750_VCC         0x5CU   /* BH1750 ADDR 接 VCC */
#define APP_ADDR_BMP280_GND         0x76U   /* BMP280 SDO 接 GND */
#define APP_ADDR_BMP280_VCC         0x77U   /* BMP280 SDO 接 VCC */

/* DHT 型号（11 或 22）：协议相同、数据格式不同，驱动按宏切解析分支。
 * 本板实测判定为 22 / AM2302 —— 判据：DHT11 小数位恒 0，而本板小数位非零、
 * 解出的温湿度与 TSN 对照吻合（详见 docs/ 里的记录）。 */
#define APP_DHT_TYPE                22

/* DHT 数据脚 = P402（标 2 第 8 孔）：已按官方原理图 netlist 核实（无串阻/复用）。
 * 数据线需 4.7k~10k 上拉到 3V3（模块版通常自带）；启动会打印空闲电平自检。 */
#define APP_DHT_PIN                 BSP_IO_PORT_04_PIN_02   /* P402 */

/* 滤波窗口（只对 ADC 生效）：中值窗（奇数，剔离群毛刺）+ 滑动平均窗（压随机噪声）。 */
#define APP_MEDIAN_WIN              5U
#define APP_AVERAGE_WIN             8U
