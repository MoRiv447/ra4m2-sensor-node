/*
 * sensor_i2c.c —— I2C 总线封装（软件 I2C，GPIO 位操作）+ BH1750 + BMP280。
 *
 * 总线：SCL=P000 / SDA=P001（标 2 第 6、7 孔）；硬件 SCI2 让给了 WiFi 模块。
 * 地址：7bit 直接传入（BH1750=0x23 / BMP280=0x76），本层负责左移并附加读写位。
 *
 * © 2026 MoRiv447 · MIT License
 */

#include "hal_data.h"
#include "sensors.h"
#include "app_config.h"
#include "app_util.h"

#include <stdio.h>
#include <string.h>

/* ==================================================================
 * 软件 I2C（GPIO 位操作）：SCL=P000、SDA=P001。
 * ⚠️ P000/P001 无板载上拉（R26=4K7 挂在 P301/P302 上）——释放时用内部上拉兜底，
 *    建议外并 4.7k~10k。SCL 推挽驱动；SDA 输出低 / 释放（输入 + 内部上拉）。
 * 时序基准 = DWT 周期计数（与 DHT 驱动同一套）；不实现 clock-stretching 等待
 * （BH1750/BMP280 用不到，且这样接线异常时也不会把主机卡死）。
 * ================================================================== */

/* ===================== 器件自愈用的重试计数 =====================
 * 放这里（文件顶部）而不是各自驱动段里，是因为 BH1750 和 BMP280
 * 两段代码在文件里的先后顺序不固定，计数器一旦挪动位置就会变成
 * "使用在定义之前"。这两个计数器都只在本文件内用。 */
static uint32_t s_bh_zero_cnt       = 0U;   /* BH1750 连续读到 0 的次数 */
static uint32_t s_bmp_not_ready_cnt = 0U;   /* BMP280 连续"未就绪"次数 */

/* ---------------- 位延时与引脚原语 ---------------- */

static inline void sw_i2c_delay_half (void)
{
    uint32_t start = app_dwt_cycles();
    uint32_t wait  = app_dwt_us_to_cycles(APP_SW_I2C_HALF_US);
    uint32_t guard = 0U;

    /* guard 兜底：即使时基异常也不会死循环（正常时第一个条件即刻满足） */
    while (((app_dwt_cycles() - start) < wait) && (guard++ < 1000U))
    {
        ;
    }
}

static inline void sw_i2c_scl_low (void)
{
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_SW_I2C_SCL_PIN,
                           IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);
}

/**
 * @brief SCL 拉高 —— 推挽驱动（不是"释放"）。
 *
 * SCL 只由主机单方驱动，且本工程从机（BH1750/BMP280）都不做 clock
 * stretching ⇒ 推挽安全。这样时钟边沿不依赖上拉，把"上拉强度"的
 * 瓶颈只留给 SDA 一侧（从机开漏输出，必须靠上拉拉高，协议层面无解）。
 */
static inline void sw_i2c_scl_high (void)
{
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_SW_I2C_SCL_PIN,
                           IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);
}

static inline void sw_i2c_sda_low (void)
{
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_SW_I2C_SDA_PIN,
                           IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);
}

static inline void sw_i2c_sda_release (void)
{
    /* 释放 = 输入 + 内部上拉。
     * ⚠️ 从机（BH1750/BMP280）的 SDA 是开漏输出，只能拉低、不能拉高；
     * "高"必须由某种上拉提供 —— 而 P000/P001 板上没有 I2C 上拉
     * （板载 R26 = 4K7 挂在 P301/P302 网络上，实机踩过），故这里启用
     * 内部上拉（~30k）兜底；半周期 15us 给足边沿时间。
     * 若现场稳定性仍不足，唯一的物理解是外并 4.7k~10k 上拉。 */
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, APP_SW_I2C_SDA_PIN,
                           IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE);
}

static inline bool sw_i2c_sda_read (void)
{
    bsp_io_level_t lv = BSP_IO_LEVEL_LOW;

    (void) R_IOPORT_PinRead(&g_ioport_ctrl, APP_SW_I2C_SDA_PIN, &lv);
    return (BSP_IO_LEVEL_HIGH == lv);
}

/* ---------------- 总线时序原语 ---------------- */

static void sw_i2c_start (void)
{
    /* 先释放 SDA 并等它爬稳，再抬起 SCL —— restart 场景下 SDA 可能刚从
     * 从机的 ACK 拉低里释放出来，不给爬升时间会被从机误看成"上升沿"。 */
    sw_i2c_sda_release();
    sw_i2c_delay_half();
    sw_i2c_scl_high();
    sw_i2c_delay_half();

    sw_i2c_sda_low();                   /* SCL 高时 SDA 下降沿 = START */
    sw_i2c_delay_half();
    sw_i2c_scl_low();
    sw_i2c_delay_half();
}

static void sw_i2c_stop (void)
{
    sw_i2c_sda_low();
    sw_i2c_delay_half();

    sw_i2c_scl_high();
    sw_i2c_delay_half();
    sw_i2c_sda_release();               /* SCL 高时 SDA 上升沿 = STOP */
    sw_i2c_delay_half();
    sw_i2c_delay_half();                /* 再给一段：让 STOP 的上升沿彻底爬稳 */
}

/**
 * @brief 写 1 字节，返回是否收到 ACK（true = 从机应答）
 */
static bool sw_i2c_write_byte (uint8_t b)
{
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (0U != (b & 0x80U))
        {
            sw_i2c_sda_release();
        }
        else
        {
            sw_i2c_sda_low();
        }
        b = (uint8_t) (b << 1);

        sw_i2c_delay_half();
        sw_i2c_scl_high();
        sw_i2c_delay_half();
        sw_i2c_scl_low();
        sw_i2c_delay_half();
    }

    /* 第 9 拍：释放 SDA，采样从机应答（低 = ACK） */
    sw_i2c_sda_release();
    sw_i2c_delay_half();
    sw_i2c_scl_high();
    sw_i2c_delay_half();

    bool ack = !sw_i2c_sda_read();

    sw_i2c_scl_low();
    sw_i2c_delay_half();

    return ack;
}

/**
 * @brief 读 1 字节；send_ack=true 回 ACK（非最后一字节），false 回 NACK（最后一字节）
 */
static uint8_t sw_i2c_read_byte (bool send_ack)
{
    uint8_t v = 0U;

    sw_i2c_sda_release();               /* 主机全程释放 SDA */
    for (uint8_t i = 0U; i < 8U; i++)
    {
        v = (uint8_t) (v << 1);

        sw_i2c_scl_high();
        sw_i2c_delay_half();
        if (sw_i2c_sda_read())
        {
            v |= 1U;
        }
        sw_i2c_delay_half();
        sw_i2c_scl_low();
        sw_i2c_delay_half();
    }

    /* 第 9 拍：主机发 ACK / NACK */
    if (send_ack)
    {
        sw_i2c_sda_low();
    }
    else
    {
        sw_i2c_sda_release();
    }
    sw_i2c_delay_half();
    sw_i2c_scl_high();
    sw_i2c_delay_half();
    sw_i2c_scl_low();
    sw_i2c_delay_half();
    sw_i2c_sda_release();

    return v;
}

/* ================================================================== */
/*                            总线接口                                 */
/* ================================================================== */

fsp_err_t i2c_bus_init (void)
{
    /* 上电先把两条线置为释放态（输入 + 内部上拉） */
    sw_i2c_scl_high();
    sw_i2c_sda_release();
    sw_i2c_delay_half();

    /* 空闲电平自检：两根都应为 1（高）。
     * 读到 0 = 上拉缺失 / 线被拉死 / 孔位错 —— 这是接线层问题最直接的证据。 */
    {
        bsp_io_level_t scl_lv = BSP_IO_LEVEL_LOW;
        bsp_io_level_t sda_lv = BSP_IO_LEVEL_LOW;

        (void) R_IOPORT_PinRead(&g_ioport_ctrl, APP_SW_I2C_SCL_PIN, &scl_lv);
        (void) R_IOPORT_PinRead(&g_ioport_ctrl, APP_SW_I2C_SDA_PIN, &sda_lv);
        printf("[SW-I2C] idle SCL=%d SDA=%d (both should be 1)\r\n",
               (int) scl_lv, (int) sda_lv);
    }

    /* 总线恢复：若从机把 SDA 拉低（上一次传输被 MCU 复位打断），
     * 补最多 9 个时钟让它把数据吐完，再发 STOP 复位其状态机。 */
    if (!sw_i2c_sda_read())
    {
        for (uint8_t i = 0U; i < 9U; i++)
        {
            sw_i2c_scl_low();
            sw_i2c_delay_half();
            sw_i2c_scl_high();
            sw_i2c_delay_half();
            if (sw_i2c_sda_read())
            {
                break;
            }
        }
        sw_i2c_stop();
    }

    return FSP_SUCCESS;
}

fsp_err_t i2c_bus_write (uint8_t dev_addr, const uint8_t * buf, uint32_t len)
{
    if ((NULL == buf) || (0U == len))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((dev_addr << 1) | 0U)))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;         /* 从机不应答：地址错 / 没接线 */
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        if (!sw_i2c_write_byte(buf[i]))
        {
            sw_i2c_stop();
            return FSP_ERR_ABORTED;
        }
    }
    sw_i2c_stop();

    return FSP_SUCCESS;
}

fsp_err_t i2c_bus_read (uint8_t dev_addr, uint8_t * buf, uint32_t len)
{
    if ((NULL == buf) || (0U == len))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((dev_addr << 1) | 1U)))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        buf[i] = sw_i2c_read_byte(i < (len - 1U));  /* 最后一字节回 NACK */
    }
    sw_i2c_stop();

    return FSP_SUCCESS;
}

fsp_err_t i2c_bus_write_reg (uint8_t dev_addr, uint8_t reg,
                             const uint8_t * buf, uint32_t len)
{
    if ((NULL == buf) || (0U == len) || (len > 8U))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 寄存器地址和数据拼成一帧连续发送（与原硬件版语义一致） */
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((dev_addr << 1) | 0U)))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }
    if (!sw_i2c_write_byte(reg))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        if (!sw_i2c_write_byte(buf[i]))
        {
            sw_i2c_stop();
            return FSP_ERR_ABORTED;
        }
    }
    sw_i2c_stop();

    return FSP_SUCCESS;
}

fsp_err_t i2c_bus_read_reg (uint8_t dev_addr, uint8_t reg,
                            uint8_t * buf, uint32_t len)
{
    if ((NULL == buf) || (0U == len))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 写阶段：地址 + 寄存器号，不发 STOP，用重复起始进入读阶段 */
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((dev_addr << 1) | 0U)))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }
    if (!sw_i2c_write_byte(reg))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((dev_addr << 1) | 1U)))
    {
        sw_i2c_stop();
        return FSP_ERR_ABORTED;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        buf[i] = sw_i2c_read_byte(i < (len - 1U));
    }
    sw_i2c_stop();

    return FSP_SUCCESS;
}

bool i2c_bus_probe (uint8_t dev_addr)
{
    /* 在场探测 = 完整做一次「读 1 字节」（末尾 NACK + STOP），器件在则成功。
     *
     * ⚠️ 不要用「只发地址就 STOP」的短探测：从机被"读地址"点名后进入
     *    数据输出状态，半截读的 STOP 收尾会与它的输出打架，把它卡在
     *    输出模式 —— 实测后果：紧随其后的事务必被污染（写命令返回
     *    FSP_ERR_ABORTED、读 ID 得到 0x03 这类错值，且 1ms 间隔也救不回）。
     *    完整读（含数据拍与 NACK）能让从机状态机干净收尾。 */
    uint8_t dummy = 0U;

    return (FSP_SUCCESS == i2c_bus_read(dev_addr, &dummy, 1U));
}

int i2c_bus_scan (void)
{
    int found = 0;

    printf("--- I2C bus scan (SW: P000=SCL / P001=SDA) ---\r\n");

    for (uint8_t addr = 0x08U; addr < 0x78U; addr++)
    {
        if (i2c_bus_probe(addr))
        {
            const char * tag = "unknown";

            if ((APP_ADDR_BH1750_GND == addr) || (APP_ADDR_BH1750_VCC == addr))
            {
                tag = "BH1750 (light)";
            }
            if ((APP_ADDR_BMP280_GND == addr) || (APP_ADDR_BMP280_VCC == addr))
            {
                tag = "BMP280 (press)";
            }

            printf("  found 0x%02X  %s\r\n", (unsigned) addr, tag);
            found++;
        }
    }

    printf("--- scan done, %d device(s) ---\r\n", found);
    return found;
}

/**
 * @brief 软件 I2C 逐段诊断 v2（上电自检跑一遍；只读/只发传感器命令）
 *
 * 背景：v1 显示"diag 里同操作全成功、init 里全失败"——变量收窄到
 * "紧邻前置动作"与"边沿时序"。本版六路分流：
 *   ① SDA 上升时间实测（DWT 计时，µs 级）
 *   ② probe 后"立即"写        ← 复现 init 的真实序列
 *   ③ probe 后隔 1ms 再写      ← 对照：隔离"间隔"因素
 *   ④⑤ BMP280 同构（立即 / 隔 1ms）
 *   ⑥ 无 probe 直接写          ← 基线对照
 */
void i2c_diag (void)
{
    uint8_t cmd = 0x01U;                /* BH1750 PowerOn 命令 */

    /* ① SDA 上升时间：拉低 → 释放 → 数到"读到高"的微秒数 */
    sw_i2c_sda_low();
    sw_i2c_delay_half();
    {
        uint32_t t0   = app_dwt_cycles();
        uint32_t t_us = 9999U;

        sw_i2c_sda_release();
        for (uint32_t i = 0U; i < 20000U; i++)
        {
            if (sw_i2c_sda_read())
            {
                t_us = (app_dwt_cycles() - t0) / (SystemCoreClock / 1000000U);
                break;
            }
        }
        printf("[DIAG] SDA rise ~ %lu us%s\r\n", (unsigned long) t_us,
               (9999U == t_us) ? "  (TIMEOUT!)" : "");
    }

    /* ② probe 后立即写 */
    bool      p1 = i2c_bus_probe(APP_ADDR_BH1750_GND);
    fsp_err_t e1 = i2c_bus_write(APP_ADDR_BH1750_GND, &cmd, 1U);
    printf("[DIAG] probe-nowrite: p=%d write=%d\r\n", (int) p1, (int) e1);

    /* ③ probe 后隔 1ms 再写（对照） */
    bool p2 = i2c_bus_probe(APP_ADDR_BH1750_GND);
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    fsp_err_t e2 = i2c_bus_write(APP_ADDR_BH1750_GND, &cmd, 1U);
    printf("[DIAG] probe-1ms-write: p=%d write=%d\r\n", (int) p2, (int) e2);

    /* ④ probe 后立即读 BMP280 ID */
    uint8_t   id  = 0U;
    bool      p3  = i2c_bus_probe(APP_ADDR_BMP280_GND);
    fsp_err_t e3  = i2c_bus_read_reg(APP_ADDR_BMP280_GND, 0xD0U, &id, 1U);
    printf("[DIAG] probe-nowreadID: p=%d err=%d id=0x%02X\r\n",
           (int) p3, (int) e3, (unsigned) id);

    /* ⑤ probe 后隔 1ms 再读（对照） */
    id = 0U;
    bool p4 = i2c_bus_probe(APP_ADDR_BMP280_GND);
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    fsp_err_t e4 = i2c_bus_read_reg(APP_ADDR_BMP280_GND, 0xD0U, &id, 1U);
    printf("[DIAG] probe-1ms-readID: p=%d err=%d id=0x%02X\r\n",
           (int) p4, (int) e4, (unsigned) id);

    /* ⑥ 无 probe 直接写（基线） */
    fsp_err_t e5 = i2c_bus_write(APP_ADDR_BH1750_GND, &cmd, 1U);
    printf("[DIAG] nowrite(noprobe): write=%d\r\n", (int) e5);
}

/* ================================================================== */
/*                              BH1750                                 */
/* ================================================================== */

#define BH1750_CMD_POWER_ON     0x01U
#define BH1750_CMD_RESET        0x07U
#define BH1750_CMD_CONT_H_RES   0x10U

/* 连续高分辨率模式转换时间典型 120ms / 最大 180ms，留余量 */
#define BH1750_CONV_DELAY_MS    200U

static uint8_t s_bh1750_addr  = 0U;
static bool    s_bh1750_ready = false;

bool bh1750_ready (void)
{
    return s_bh1750_ready;
}

fsp_err_t bh1750_init (void)
{
    s_bh1750_ready = false;

    /* 地址由 ADDR 引脚决定：接 GND → 0x23，接 VCC → 0x5C，两个都试一遍 */
    if (i2c_bus_probe(APP_ADDR_BH1750_GND))
    {
        s_bh1750_addr = APP_ADDR_BH1750_GND;
    }
    else if (i2c_bus_probe(APP_ADDR_BH1750_VCC))
    {
        s_bh1750_addr = APP_ADDR_BH1750_VCC;
    }
    else
    {
        printf("[BH1750] not found at 0x23 / 0x5C\r\n");
        return FSP_ERR_NOT_FOUND;
    }

    uint8_t   cmd;
    fsp_err_t err;

    cmd = BH1750_CMD_POWER_ON;
    err = i2c_bus_write(s_bh1750_addr, &cmd, 1U);
    if (FSP_SUCCESS != err)
    {
        printf("[BH1750] power on failed: %d\r\n", (int) err);
        return err;
    }

    cmd = BH1750_CMD_RESET;
    (void) i2c_bus_write(s_bh1750_addr, &cmd, 1U);
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);

    cmd = BH1750_CMD_CONT_H_RES;
    err = i2c_bus_write(s_bh1750_addr, &cmd, 1U);
    if (FSP_SUCCESS != err)
    {
        printf("[BH1750] set mode failed: %d\r\n", (int) err);
        return err;
    }

    /* 等第一帧转换完成，否则首次读到的可能是 0 */
    R_BSP_SoftwareDelay(BH1750_CONV_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    s_bh1750_ready = true;
    printf("[BH1750] init ok, addr=0x%02X, continuous H-res\r\n",
           (unsigned) s_bh1750_addr);
    return FSP_SUCCESS;
}

fsp_err_t bh1750_read (bh1750_data_t * out)
{
    if (NULL == out)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!s_bh1750_ready)
    {
        return FSP_ERR_NOT_OPEN;
    }

    uint8_t raw[2] = { 0U, 0U };

    /* 连续模式下数据寄存器持续更新，不需要先写寄存器地址，
     * 直接读 2 字节即可 —— 这一点和寄存器型的 BMP280 不同 */
    fsp_err_t err = i2c_bus_read(s_bh1750_addr, raw, 2U);
    if (FSP_SUCCESS != err)
    {
        out->valid = false;
        return err;
    }

    uint16_t value = ((uint16_t) raw[0] << 8) | (uint16_t) raw[1];

    out->addr = s_bh1750_addr;

    /* ---- 连续读到 0 就重发一次测量命令 ----
     *
     * 正常室内 0 lx 其实极少见（关灯的房间也有个位数 lx），而 0 更像
     * **器件被复位后的状态**：BH1750 掉电或复位后会回到 Power Down，
     * 之后数据寄存器一直是 0。实机遇到过一次 —— BMP280 同时也在报
     * "未就绪"，两块器件的供电被同一个动作扰动过。
     *
     * 判据取连续 3 次：单次 0 可能是真的全黑，连续 3 次（采样间隔 3s）
     * 才归为异常并重发命令。 */
    if (0U == value)
    {
        s_bh_zero_cnt++;

        if (s_bh_zero_cnt >= 3U)
        {
            if (1U == (s_bh_zero_cnt % 10U))
            {
                printf("[BH1750] lux 连续 %lu 次为 0 -> 器件可能被复位，重发测量命令\r\n",
                       (unsigned long) s_bh_zero_cnt);
            }

            uint8_t cmd = BH1750_CMD_POWER_ON;
            (void) i2c_bus_write(s_bh1750_addr, &cmd, 1U);
            cmd = BH1750_CMD_CONT_H_RES;
            (void) i2c_bus_write(s_bh1750_addr, &cmd, 1U);
        }
    }
    else
    {
        s_bh_zero_cnt = 0U;
    }

    out->lux   = ((float) value) / 1.2f;    /* 手册：lux = raw / 1.2 */
    out->valid = true;
    return FSP_SUCCESS;
}

/* ================================================================== */
/*                              BMP280                                 */
/* ================================================================== */

#define BMP280_REG_CALIB    0x88U
#define BMP280_REG_ID       0xD0U
#define BMP280_REG_CONFIG   0xF5U
#define BMP280_REG_CTRL     0xF4U
#define BMP280_REG_DATA     0xF7U
#define BMP280_CHIP_ID      0x58U

/* 数据寄存器读出这个值 = "这一轮没有转换出数据"（手册里的未就绪标志）。
 * 实机抓到过：读数冻在 22.19C / 705.60hPa，反推原始值正好是 0x80000 ——
 * 不是"等的时间不够"，而是**器件被复位了**：VDD 瞬间跌落或总线毛刺
 * 都会让它回到 sleep（mode=00），此后数据寄存器就永远停在这个码上。
 * 两个寄存器同时是这个值，就能确定是器件状态问题而不是换算问题。 */
#define BMP280_NOT_READY    0x80000L

/* 标定系数。全部是 16 位，其中大部分为有符号 */
typedef struct
{
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib_t;

static bmp280_calib_t s_cal;
static uint8_t        s_bmp_addr  = 0U;
static bool           s_bmp_ready = false;

/* t_fine 是补偿公式的中间量：温度先算，气压的补偿要用同一个 t_fine，
 * 所以必须是全局的 —— 这也是「一次读 6 字节」的原因：
 * 气压和温度必须来自同一轮转换，否则 t_fine 与 adc_P 不对应。 */
static double s_t_fine = 0.0;

/* ⚠️ BMP280 字节序"混着"（手册写得很隐晦）：输出数据 0xF7~0xFC 是**大端**，
 * 校准寄存器 0x88~0xA1 是**小端** —— 坑已踩实：大端解校准会得到
 * 温度 -59℃ / 气压 18815hPa 这种"离谱但不报错"的数；小端才对（室温 28℃ / 1000hPa）。
 * 所以：校准用小端（le_i16/le_u16），数据用大端。 */
static int16_t le_i16 (const uint8_t * p)
{
    return (int16_t) (((uint16_t) p[1] << 8) | (uint16_t) p[0]);
}

static uint16_t le_u16 (const uint8_t * p)
{
    return (uint16_t) (((uint16_t) p[1] << 8) | (uint16_t) p[0]);
}

bool bmp280_ready (void)
{
    return s_bmp_ready;
}

/* 定义在下面；init 和自愈重配都要用，所以先声明 */
static fsp_err_t bmp280_write_mode (void);

fsp_err_t bmp280_init (void)
{
    s_bmp_ready = false;

    if (i2c_bus_probe(APP_ADDR_BMP280_GND))
    {
        s_bmp_addr = APP_ADDR_BMP280_GND;
    }
    else if (i2c_bus_probe(APP_ADDR_BMP280_VCC))
    {
        s_bmp_addr = APP_ADDR_BMP280_VCC;
    }
    else
    {
        printf("[BMP280] not found at 0x76 / 0x77\r\n");
        return FSP_ERR_NOT_FOUND;
    }

    /* 先认 ID。0x58 是 BMP280；若读出 0x60 那是 BME280（多了湿度），
     * 温压算法通用，湿度寄存器这里没用上 */
    uint8_t   id  = 0U;
    fsp_err_t err = i2c_bus_read_reg(s_bmp_addr, BMP280_REG_ID, &id, 1U);
    if (FSP_SUCCESS != err)
    {
        printf("[BMP280] read ID failed: %d\r\n", (int) err);
        return err;
    }
    if (BMP280_CHIP_ID != id)
    {
        printf("[BMP280] unexpected chip id 0x%02X (expect 0x58)\r\n", (unsigned) id);
        return FSP_ERR_NOT_FOUND;
    }

    /* 读 24 字节标定系数（0x88~0x9F）。每个芯片出厂单独标定，
     * 没有这组系数，补偿公式算出来的数字毫无意义 */
    uint8_t cal[24] = { 0 };
    err = i2c_bus_read_reg(s_bmp_addr, BMP280_REG_CALIB, cal, sizeof(cal));
    if (FSP_SUCCESS != err)
    {
        printf("[BMP280] read calib failed: %d\r\n", (int) err);
        return err;
    }

    /* 校准系数用小端解（见上面 le_i16/le_u16 的长注释 —— 这里就是那个坑的位置） */
    s_cal.T1 = le_u16(&cal[0]);
    s_cal.T2 = le_i16(&cal[2]);
    s_cal.T3 = le_i16(&cal[4]);
    s_cal.P1 = le_u16(&cal[6]);
    s_cal.P2 = le_i16(&cal[8]);
    s_cal.P3 = le_i16(&cal[10]);
    s_cal.P4 = le_i16(&cal[12]);
    s_cal.P5 = le_i16(&cal[14]);
    s_cal.P6 = le_i16(&cal[16]);
    s_cal.P7 = le_i16(&cal[18]);
    s_cal.P8 = le_i16(&cal[20]);
    s_cal.P9 = le_i16(&cal[22]);

    /* 上电自检：系数解错的话量级会明显跑偏，与其等到读数离谱再查，
     * 不如开机就判一下 —— 这几条范围是按手册典型值放的宽边界。 */
    if ((s_cal.P1 < 10000U) || (s_cal.P1 > 60000U) || (s_cal.P5 > 1000) || (s_cal.P5 < -1000))
    {
        printf("[BMP280] calib looks wrong: P1=%u P5=%d (expect P1~36000, P5 很小)\r\n",
               (unsigned) s_cal.P1, (int) s_cal.P5);
    }

    err = bmp280_write_mode();
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* 等第一轮转换完成 */
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);

    s_bmp_ready = true;
    printf("[BMP280] init ok, addr=0x%02X, T1=%u P1=%u\r\n",
           (unsigned) s_bmp_addr, (unsigned) s_cal.T1, (unsigned) s_cal.P1);
    return FSP_SUCCESS;
}

/**
 * @brief 把 config + ctrl_meas 写一遍（初始化与自愈重配共用）
 *
 * 单独抽出来是因为它不只在上电时用：器件一旦被复位（VDD 跌落 / 总线毛刺），
 * mode 会回到 sleep，之后再读数据寄存器就一直返回 0x80000。
 * 这时候把这两个寄存器重写一遍就能救回来，不必掉电重插。
 */
static fsp_err_t bmp280_write_mode (void)
{
    /* config (0xF5)：t_sb=0(0.5ms) / filter=4(×16) / spi3w=0  → 0b000_100_00 */
    uint8_t   config = 0x10U;
    fsp_err_t err = i2c_bus_write_reg(s_bmp_addr, BMP280_REG_CONFIG, &config, 1U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* ctrl_meas (0xF4)：osrs_t=2(×2) / osrs_p=5(×16) / mode=3(normal)
     *                          → 0b010_101_11 = 0x57
     * 气压过采样开到 ×16 是因为气压对噪声最敏感；
     * IIR 滤波器 ×16 再压一层，代价是响应变慢 —— 环境监测场景完全可以接受 */
    uint8_t ctrl = 0x57U;
    return i2c_bus_write_reg(s_bmp_addr, BMP280_REG_CTRL, &ctrl, 1U);
}

/**
 * @brief 温度补偿（Bosch 手册 3.11 节浮点版本）
 *
 * @param adc_T 20 位原始温度值
 * @return 摄氏度
 */
static double bmp280_compensate_temp (int32_t adc_T)
{
    double var1;
    double var2;

    var1 = (((double) adc_T) / 16384.0 - ((double) s_cal.T1) / 1024.0)
           * ((double) s_cal.T2);

    var2 = ((((double) adc_T) / 131072.0 - ((double) s_cal.T1) / 8192.0)
            * (((double) adc_T) / 131072.0 - ((double) s_cal.T1) / 8192.0))
           * ((double) s_cal.T3);

    /* t_fine 供气压补偿复用，必须先于气压计算 */
    s_t_fine = var1 + var2;

    return (var1 + var2) / 5120.0;
}

/**
 * @brief 气压补偿（Bosch 手册 3.11 节浮点版本）
 *
 * @param adc_P 20 位原始气压值
 * @return 帕斯卡；var1 为 0 时（理论上不会）返回 0 避免除零
 */
static double bmp280_compensate_press (int32_t adc_P)
{
    double var1;
    double var2;
    double p;

    var1 = ((double) s_t_fine / 2.0) - 64000.0;

    var2 = var1 * var1 * ((double) s_cal.P6) / 32768.0;
    var2 = var2 + var1 * ((double) s_cal.P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) s_cal.P4) * 65536.0);

    var1 = (((double) s_cal.P3) * var1 * var1 / 524288.0
            + ((double) s_cal.P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) s_cal.P1);

    /* 手册这里写的是 `if (var1 == 0) return 0;`，只是防除零。
     * 改成"接近零就拿零"有两个好处：
     *   ① 工程开了 -Wfloat-equal，浮点用 == 比较会告警；
     *   ② 更实际的是，var1 趋近 0 时下面的 p = ... / var1 会放大成天文数字，
     *      这种结果本来就没有意义，阈值判定比"恰好为 0"更贴近真实意图。 */
    if ((var1 > -1e-12) && (var1 < 1e-12))
    {
        return 0.0;
    }

    p = 1048576.0 - (double) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;

    var1 = ((double) s_cal.P9) * p * p / 2147483648.0;
    var2 = p * ((double) s_cal.P8) / 32768.0;

    p = p + (var1 + var2 + ((double) s_cal.P7)) / 16.0;

    return p;
}

fsp_err_t bmp280_read (bmp280_data_t * out)
{
    if (NULL == out)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!s_bmp_ready)
    {
        return FSP_ERR_NOT_OPEN;
    }

    /* 一次读 6 字节：press_msb/lsb/xlsb + temp_msb/lsb/xlsb。
     * 必须一次读完 —— 分两次读会跨转换周期，气压与温度就不同源了，
     * 而气压补偿依赖同一轮的 t_fine。 */
    uint8_t   d[6] = { 0 };
    fsp_err_t err  = i2c_bus_read_reg(s_bmp_addr, BMP280_REG_DATA, d, 6U);
    if (FSP_SUCCESS != err)
    {
        out->valid = false;
        return err;
    }

    /* 20 位：高 8 + 中 8 + 低 4（低 4 位在高半字节） */
    int32_t adc_P = (int32_t) (((uint32_t) d[0] << 12)
                               | ((uint32_t) d[1] << 4)
                               | ((uint32_t) d[2] >> 4));

    int32_t adc_T = (int32_t) (((uint32_t) d[3] << 12)
                               | ((uint32_t) d[4] << 4)
                               | ((uint32_t) d[5] >> 4));

    /* ---- 先判"器件没在转换"，别急着代入公式 ----
     *
     * 两个寄存器同时读出 0x80000 = 手册的"未就绪"码。
     * 实机遇到过一次：读数冻在 22.19C / 705.60hPa（反推原始值正好 0x80000），
     * 而且 40 秒里一个字节都不变 —— 那是**器件被复位过**，
     * mode 回到 sleep 之后数据寄存器就永远停在这个码上。
     * 不判这一条的话，公式会把 0x80000 当成真实测量值算出一堆
     * 看着像模像样的垃圾（22.19C 还挺"合理"，705hPa 才露馅）。
     *
     * 处理：重写 config + ctrl_meas 当作自愈，并把这一轮标为无效。 */
    if ((BMP280_NOT_READY == adc_P) || (BMP280_NOT_READY == adc_T))
    {
        out->valid     = false;
        out->not_ready = true;
        s_bmp_not_ready_cnt++;

        /* 只在开始时和之后每隔 20 次各报一次，免得刷屏 */
        if ((s_bmp_not_ready_cnt <= 3U) || (0U == (s_bmp_not_ready_cnt % 20U)))
        {
            printf("[BMP280] data=0x80000 not-ready -> 器件被复位过，第 %lu 次尝试重配\r\n",
                   (unsigned long) s_bmp_not_ready_cnt);
        }

        fsp_err_t rc = bmp280_write_mode();
        if (FSP_SUCCESS != rc)
        {
            /* 连寄存器都写不进去，说明器件真的掉线了（供电或接线） */
            s_bmp_ready = false;
            printf("[BMP280] reconfig FAILED (%d) -> 器件掉线，检查 VCC/GND/接线\r\n", (int) rc);
            return FSP_ERR_NOT_OPEN;
        }

        return FSP_ERR_INVALID_DATA;
    }

    s_bmp_not_ready_cnt = 0U;

    double temp = bmp280_compensate_temp(adc_T);
    double pres = bmp280_compensate_press(adc_P);

    out->temp_c     = (float) temp;
    out->press_pa   = (float) pres;
    out->press_hpa  = (float) (pres / 100.0);
    out->valid      = true;
    out->not_ready  = false;    /* 读到有效数据就把未就绪标记清掉 */

    return FSP_SUCCESS;
}

/* ================================================================== */
/*                      BMP280 原始数据转储（排错用）                   */
/* ================================================================== */

/**
 * @brief 把 BMP280 的原始寄存器、校准系数、20 位换算结果全打出来
 *
 * 为什么要这个：读回来的温压值离谱时，光看结果**定位不到原因** ——
 * 可能是数据寄存器读错位置、校准系数没读上、器件不是 BMP280、或者总线
 * 根本没在驱动数据线，这几种的"输出数字"长得都差不多。把原始字节和
 * 系数一起打出来，一眼就能分开。
 *
 * 判据（写在这里免得每次重推）：
 *   - 校准系数全 0xFF / 全 0x00 → 数据线没被真正驱动
 *   - 室温下 adc_T 应在 50 万附近、adc_P 在 30~60 万；差一个数量级就是读错位置
 *   - T1 典型值 2.7 万上下、P1 典型值 3.6 万上下；和这个差太远说明系数读歪了
 */
void bmp280_dump_debug (void)
{
    if (!s_bmp_ready)
    {
        printf("[BMP280] not ready, skip dump\r\n");
        return;
    }

    uint8_t   data[6]  = { 0 };
    uint8_t   cal[24]  = { 0 };
    uint8_t   id       = 0U;

    (void) i2c_bus_read_reg(s_bmp_addr, BMP280_REG_ID,    &id,      1U);
    fsp_err_t e1 = i2c_bus_read_reg(s_bmp_addr, BMP280_REG_DATA,  data, 6U);
    fsp_err_t e2 = i2c_bus_read_reg(s_bmp_addr, BMP280_REG_CALIB, cal, 24U);

    printf("[BMP280] dump addr=0x%02X id=0x%02X (expect 0x58)  data_rd=%d calib_rd=%d\r\n",
           (unsigned) s_bmp_addr, (unsigned) id, (int) e1, (int) e2);

    printf("[BMP280] data :");
    for (uint32_t i = 0U; i < 6U; i++)
    {
        printf(" %02X", (unsigned) data[i]);
    }
    printf("   (press_msb lsb xlsb | temp_msb lsb xlsb)\r\n");

    printf("[BMP280] calib:");
    for (uint32_t i = 0U; i < 24U; i++)
    {
        printf(" %02X", (unsigned) cal[i]);
    }
    printf("\r\n");

    bool all_ff = true;
    bool all_00 = true;
    for (uint32_t i = 0U; i < 24U; i++)
    {
        if (0xFFU != cal[i])
        {
            all_ff = false;
        }
        if (0x00U != cal[i])
        {
            all_00 = false;
        }
    }
    if (all_ff)
    {
        printf("[BMP280] calib 全 0xFF -> 器件没在驱动数据线（总线被拉高 / 地址不是它）\r\n");
    }
    if (all_00)
    {
        printf("[BMP280] calib 全 0x00 -> 读到的是空数据\r\n");
    }

    int32_t adc_p = (int32_t) (((uint32_t) data[0] << 12)
                               | ((uint32_t) data[1] << 4)
                               | ((uint32_t) data[2] >> 4));
    int32_t adc_t = (int32_t) (((uint32_t) data[3] << 12)
                               | ((uint32_t) data[4] << 4)
                               | ((uint32_t) data[5] >> 4));

    printf("[BMP280] adc_P=%ld  adc_T=%ld   (室温期望 adc_T 约 50 万)\r\n",
           (long) adc_p, (long) adc_t);

    printf("[BMP280] T1=%u T2=%d T3=%d\r\n",
           (unsigned) le_u16(&cal[0]), (int) le_i16(&cal[2]), (int) le_i16(&cal[4]));
    printf("[BMP280] P1=%u P2=%d P3=%d P4=%d P5=%d  (校准按小端解)\r\n",
           (unsigned) le_u16(&cal[6]), (int) le_i16(&cal[8]), (int) le_i16(&cal[10]),
           (int) le_i16(&cal[12]), (int) le_i16(&cal[14]));
    printf("[BMP280] P6=%d P7=%d P8=%d P9=%d\r\n",
           (int) le_i16(&cal[16]), (int) le_i16(&cal[18]),
           (int) le_i16(&cal[20]), (int) le_i16(&cal[22]));

    /* 参考量级（手册典型值）：T1≈27500 T2≈26400 T3 很小；P1≈36500 P2≈-10700
     * P3≈3000 P4≈2900 P5 很小 P6≈-7 P7≈15500 P8≈-14600 P9≈6000
     * 差一个数量级或者符号反了 → 字节序解错了（这个已实机验证过） */
    if ((le_u16(&cal[6]) < 10000U) || (le_i16(&cal[8]) > 0))
    {
        printf("[BMP280] 系数不在典型量级 -> 字节序或读取位置有问题\r\n");
    }

    if ((adc_t < 300000L) || (adc_t > 800000L))
    {
        printf("[BMP280] adc_T 偏离室温合理范围 -> 数据寄存器读的位置不对\r\n");
    }

    /* 0x80000 = 手册的"未就绪"码。开机就出现说明器件刚被复位过，
     * 或者 init 里那两个配置寄存器没写进去。 */
    if ((BMP280_NOT_READY == adc_p) || (BMP280_NOT_READY == adc_t))
    {
        printf("[BMP280] !! 数据是 0x80000 = 未就绪码 —— 器件没在转换\r\n");
        printf("         `bmp280_write_mode()` 会尝试重配；一直这样就是供电/接线问题\r\n");
    }
}
