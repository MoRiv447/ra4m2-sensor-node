/*
 * sensors.h —— 传感器与总线的统一接口（BH1750 / BMP280 / DHT22 / ADC 通道）。
 *
 * 总线：软件 I2C（P000/P001）挂 BH1750 + BMP280；单总线 P402 挂 DHT22；ADC 用 P014/P015。
 * 取值约定：read() 填充结构体，第一个成员是 valid；读失败**不清零**旧数据，
 * 只置 valid=false —— 0 对温度/气压来说都是"看起来合理但错误"的值。
 *
 * © 2026 MoRiv447 · MIT License
 */

#pragma once

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================================== */
/*                          I2C 总线封装                               */
/* ================================================================== */

/**
 * @brief 初始化软件 I2C 总线（SCL=P000 / SDA=P001）
 */
fsp_err_t i2c_bus_init (void);

/** @brief 探测某地址是否有器件应答（读 1 字节，对传感器无副作用） */
bool i2c_bus_probe (uint8_t dev_addr);

/** @brief 扫描 0x08~0x77 并打印结果，返回找到的器件数 */
int i2c_bus_scan (void);

/** @brief 软件 I2C 逐段诊断（第 4 篇引入；上电自检时打印每拍 ACK 与读值） */
void i2c_diag (void);

fsp_err_t i2c_bus_write (uint8_t dev_addr, const uint8_t * buf, uint32_t len);
fsp_err_t i2c_bus_read (uint8_t dev_addr, uint8_t * buf, uint32_t len);
fsp_err_t i2c_bus_write_reg (uint8_t dev_addr, uint8_t reg, const uint8_t * buf, uint32_t len);
fsp_err_t i2c_bus_read_reg (uint8_t dev_addr, uint8_t reg, uint8_t * buf, uint32_t len);

/* ================================================================== */
/*                      BH1750 —— 数字光照（I2C）                      */
/* ================================================================== */

typedef struct
{
    bool    valid;
    float   lux;
    uint8_t addr;
} bh1750_data_t;

fsp_err_t bh1750_init (void);
fsp_err_t bh1750_read (bh1750_data_t * out);
bool      bh1750_ready (void);

/* ================================================================== */
/*                   BMP280 —— 气压 / 温度（I2C）                      */
/* ================================================================== */

typedef struct
{
    bool    valid;
    float   temp_c;     /* 摄氏度 */
    float   press_pa;   /* 帕斯卡 */
    float   press_hpa;  /* 百帕，= press_pa / 100 */

    /* 器件回了"未就绪"码（0x80000）时为 true —— 通常意味着它被复位过、
     * 驱动已经自动重配过一轮。和"读失败"分开，因为处理方式不同：
     * 读失败要查接线，未就绪往往等下一轮自己就好了。 */
    bool    not_ready;
} bmp280_data_t;

fsp_err_t bmp280_init (void);
fsp_err_t bmp280_read (bmp280_data_t * out);
bool      bmp280_ready (void);

/**
 * @brief 把 BMP280 的原始寄存器、校准系数、20 位换算结果全部打印出来
 *
 * 读回来的温压离谱时用它定位 —— 光看结果分不清是"读错寄存器位置"、
 * "校准系数没读上"、"器件不是 BMP280" 还是"数据线没被驱动"。
 * 判据写在实现里，见 sensor_i2c.c。
 */
void bmp280_dump_debug (void);

/* ================================================================== */
/*                   DHT11 / DHT22 —— 温湿度（单总线）                  */
/* ================================================================== */

typedef struct
{
    bool    valid;
    float   temp_c;
    float   rh_pct;     /* 相对湿度 %RH */
    uint8_t raw[5];     /* 原始 5 字节，帖子里展示校验过程要用 */
    uint32_t err_streak;/* 连续失败次数，用于判断接线是否真的有问题 */
} dht_data_t;

/**
 * @brief 初始化 DHT 数据脚（先配成输入，靠板外上拉保持高电平）
 */
fsp_err_t dht_init (void);

/**
 * @brief 读一次温湿度（整轮耗时约 25ms，期间关中断）
 *
 * 阻塞实现。DHT 的时序要求是高电平持续 26us / 70us 两种宽度，
 * 必须靠 DWT 周期计数器计时，不能用 R_BSP_SoftwareDelay。
 */
fsp_err_t dht_read (dht_data_t * out);

/** @brief 数据脚当前闲置电平（1=高）。上电自检用它确认接线和上拉 */
bool dht_pin_idle_level (void);

/* ================================================================== */
/*              模拟通道 A —— 固定分压基准 / 光敏电阻                   */
/* ================================================================== */

/* 照度分档，仅供演示"模拟量分档"这个动作（仅光敏电阻模式下有意义） */
typedef enum
{
    LDR_LEVEL_DARK = 0,
    LDR_LEVEL_MID,
    LDR_LEVEL_BRIGHT
} ldr_level_t;

typedef struct
{
    bool     valid;
    uint16_t raw;       /* 滤波后的代表值 */
    float    volt;      /* 引脚电压 */
    ldr_level_t level;  /* 仅在光敏电阻模式下有意义 */

    /* ↓ 下面两个字段只在「固定分压」模式下填（APP_ADC_A_INPUT == 0）
     *   阻值已知 ⇒ 能算出理论电压，两者一比就是 12 位 ADC 的绝对误差。
     *   这是「片上 12 位 ADC 精度实测」那一节最硬的数字。 */
    float    expect_mv; /* 由分压电阻算出的理论电压 */
    float    error_lsb; /* 实测 − 理论，单位 LSB */

    /* ↓ 下面两个字段只在「光敏电阻」模式下填（APP_ADC_A_INPUT == 1） */
    float    r_ohm;     /* 反解出的光敏电阻阻值 */
    float    lux;       /* 相对照度（幂律估算，非标定值） */

    /* 本轮的噪声指标。刻意用**未滤波**的原始 burst 计算：
     * 滤波后的数字说明"处理得多好"，这两个才是"ADC 本身有多吵" ——
     * 噪声抑制实测要对比的就是它们。 */
    float    pp;        /* 峰峰值 */
    float    sigma;     /* 标准差 */
} ldr_data_t;

fsp_err_t ldr_init (void);
fsp_err_t ldr_read (ldr_data_t * out);

/**
 * @brief 当前配置下模拟通道 A 的理论输入电压（mV）
 *
 * 固定分压模式下由 APP_FIXED_R_HIGH/LOW 算出，用于精度比对；
 * 光敏电阻模式下没有固定期望值，返回 -1。
 */
float adc_a_expected_mv (void);

/**
 * @brief 由引脚电压反解光敏电阻阻值（仅光敏电阻模式用）
 *
 * 接线为「3V3 -[光敏]- 引脚 -[Rseries]- GND」，所以越亮阻值越小、电压越高。
 *   V = 3V3 * Rseries / (R_ldr + Rseries)
 *   R_ldr = Rseries * (3V3 / V - 1)
 */
float ldr_volt_to_ohm (float volt);

/**
 * @brief 由阻值幂律估算相对照度（仅光敏电阻模式用）
 *
 *   lux = lux_ref * (R / R_ref)^(-1/γ)
 *
 * γ 与参考点是 GL5528 的典型值，属于量级可用，不是标定结果。
 */
float ldr_ohm_to_lux (float r_ohm);

/* ================================================================== */
/*                        ADC 原始采样（噪声评估）                      */
/* ================================================================== */

/**
 * @brief 对一个通道连采 n 次，结果写进 samples
 *
 * 内部对每个样本都重新 ScanStart —— 因为 FSP 里这个 ADC 配的是
 * clearing = clear_after_read_on，读一次就清零，"一次扫描连读 n 次"
 * 会得到 1 个真值加 n-1 个 0。每次重新转换也才是噪声评估要的独立样本。
 */
fsp_err_t adc_burst_read (adc_channel_t ch, uint16_t * samples, uint32_t n);

/**
 * @brief 上一次 adc_burst_read 花掉的 CPU 周期数
 *
 * 用来反推采样速率。注意这个数是**吞吐量**：里面含 ScanStart + 中断等待 +
 * R_ADC_Read 三步的软件开销，不是 ADC 的原生转换时间。
 */
uint32_t adc_last_burst_cycles (void);

/**
 * @brief 逐条模拟通道单独测一遍，打印各自的均值与噪声
 *
 * 排查"分压确认接好了但读数还是悬空"用它 —— 线插在哪个脚上，
 * 那个脚就会显示"接上了"，从而区分出是接线问题还是插错脚。
 */
void adc_probe_channels (void);
