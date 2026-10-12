/*
 * wifi_bridge.c —— DA16200 WiFi 桥（AT-over-UART，非透传）。
 *
 * 要点：payload 不能含逗号（模块限制）→ 用分号；"轻启动" + 自愈重试
 * （每 10s 重发 AT+NWMQCL=1 直到 +NWMQCL:1），连上后才发数据帧；
 * 发送必须用静态缓冲 —— FSP 非阻塞发送引用传入地址，栈缓冲会发出"半截乱码"。
 *
 * © 2026 MoRiv447 · MIT License
 */
#include "wifi_bridge.h"
#include "hal_data.h"

#include <stdio.h>
#include <string.h>

/* FSP 里 UART Stack 的 Callback 要填这个名字（生成后 hal_data.h 里也会有声明） */
void wifi_uart_callback (uart_callback_args_t * p_args);

#define WIFI_LINE_MAX       128U
#define WIFI_TX_BUF_SIZE    192U

static volatile bool     g_wifi_tx_busy    = false;
static uint8_t           g_wifi_rx_byte    = 0U;
static volatile char     g_wifi_line[WIFI_LINE_MAX + 1U];
static volatile uint16_t g_wifi_line_len   = 0U;
static volatile bool     g_wifi_line_ready = false;
static bool              g_wifi_mqtt_connected = false;  /* 见到 +NWMQCL:1 后置位（仅主循环上下文用） */

/* ================================================================== */
/*                              发送                                   */
/* ================================================================== */

static void wifi_send_raw (const char * str)
{
    static char s_tx_buf[WIFI_TX_BUF_SIZE];

    if (g_wifi_tx_busy)
    {
        return;             /* 忙丢帧：周期上报场景可接受 */
    }

    size_t len = strlen(str);
    if (len > (sizeof(s_tx_buf) - 1U))
    {
        len = sizeof(s_tx_buf) - 1U;
    }
    memcpy(s_tx_buf, str, len);
    s_tx_buf[len] = '\0';

    /* 先置忙、再启动发送：保证 TX_COMPLETE 一定发生在置忙之后 */
    g_wifi_tx_busy = true;
    if (FSP_SUCCESS != R_SCI_UART_Write(&g_uart2_ctrl, (uint8_t const *) s_tx_buf,
                                        (uint32_t) len))
    {
        g_wifi_tx_busy = false;
    }
}

/** @brief 发一条 AT 指令（自动补 \r\n） */
static void wifi_send_cmd (const char * cmd)
{
    char buf[WIFI_TX_BUF_SIZE];

    (void) snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    wifi_send_raw(buf);
}

/** @brief 有界等待 TX 发完（仅初始化用；几毫秒上界，防死等） */
static void wifi_wait_tx_idle (void)
{
    for (uint32_t i = 0U; (i < 200000U) && g_wifi_tx_busy; i++)
    {
        ;
    }
}

/* ================================================================== */
/*                           收发回调                                  */
/* ================================================================== */

void wifi_uart_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_COMPLETE:
        {
            char c = (char) g_wifi_rx_byte;

            if (('\r' != c) && ('\n' != c))
            {
                if (g_wifi_line_ready)
                {
                    g_wifi_line_ready = false;      /* 上一条未处理：丢旧保新 */
                    g_wifi_line_len   = 0U;
                }
                if (g_wifi_line_len < WIFI_LINE_MAX)
                {
                    g_wifi_line[g_wifi_line_len] = c;
                    g_wifi_line_len++;
                }
            }
            else if ((0U != g_wifi_line_len) && (!g_wifi_line_ready))
            {
                g_wifi_line[g_wifi_line_len] = '\0';
                g_wifi_line_ready = true;
            }

            (void) R_SCI_UART_Read(&g_uart2_ctrl, &g_wifi_rx_byte, 1U);
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            g_wifi_tx_busy = false;
            break;
        }

        default:
        {
            /* 错误类事件：清忙标志 + 重挂接收，别让链路卡死 */
            g_wifi_tx_busy = false;
            (void) R_SCI_UART_Read(&g_uart2_ctrl, &g_wifi_rx_byte, 1U);
            break;
        }
    }
}

/* ================================================================== */
/*                              对外                                   */
/* ================================================================== */

void wifi_bridge_init (void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[WiFi] UART open failed: %d\r\n", (int) err);
        return;
    }
    err = R_SCI_UART_Read(&g_uart2_ctrl, &g_wifi_rx_byte, 1U);
    if (FSP_SUCCESS != err)
    {
        printf("[WiFi] RX arm failed: %d\r\n", (int) err);
        return;
    }

    printf("[WiFi] bridge up (SCI2, 115200); probe + MQTT attach...\r\n");

    /* 轻启动：探测 + 尝试恢复 MQTT 连接（假定模块已配网并保存）。
     * ⚠️ 两条命令不能背靠背连发：发送是异步的，忙保护会把前一条还没发完时
     *    进来的下一条直接丢掉 —— 实测 `AT+NWMQCL=1` 就这样被丢过，后果是
     *    MQTT 从未连上、之后所有上报都被模块以"未连接"拒绝（ERROR）。 */
    wifi_send_cmd("AT");
    wifi_wait_tx_idle();
    wifi_send_cmd("AT+NWMQCL=1");
}

void wifi_bridge_poll (void)
{
    if (g_wifi_line_ready)
    {
        char     line[WIFI_LINE_MAX + 1U];
        uint16_t len = g_wifi_line_len;

        for (uint16_t i = 0U; i < len; i++)
        {
            line[i] = g_wifi_line[i];
        }
        line[len] = '\0';

        g_wifi_line_len   = 0U;
        g_wifi_line_ready = false;

        /* 捕获连接事件：之后上报才从"重试连接"切换为数据帧 */
        if (NULL != strstr(line, "+NWMQCL:1"))
        {
            g_wifi_mqtt_connected = true;
        }

        printf("[WiFi] << %s\r\n", line);
    }
}

void wifi_bridge_report (float lux, float ldr_lux, float temp_c, float rh_pct)
{
    /* 还没连上 broker：这一格不发数据，改成发一次连接重试（每 10s 一次，
     * 直到看到 +NWMQCL:1）——模块重启 / 配网还没就绪时会自动续上 */
    if (!g_wifi_mqtt_connected)
    {
        wifi_send_cmd("AT+NWMQCL=1");
        return;
    }

    char   buf[WIFI_TX_BUF_SIZE];
    size_t n = (size_t) snprintf(buf, sizeof(buf), "AT+NWMQMSG=RA4M2");

    if (lux >= 0.0f)
    {
        int32_t x10 = (int32_t) (lux * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, ";lux=%ld.%01ld",
                               (long) (x10 / 10), (long) (x10 % 10));
    }
    if (ldr_lux >= 0.0f)
    {
        int32_t x10 = (int32_t) (ldr_lux * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, ";ldr=%ld.%01ld",
                               (long) (x10 / 10), (long) (x10 % 10));
    }
    if (temp_c > -50.0f)
    {
        int32_t      x10 = (int32_t) (temp_c * 10.0f +
                                      ((temp_c >= 0.0f) ? 0.5f : -0.5f));
        const char * sgn = (x10 < 0) ? "-" : "";
        int32_t      ax  = (x10 < 0) ? -x10 : x10;

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, ";T=%s%ld.%01ld",
                               sgn, (long) (ax / 10), (long) (ax % 10));
    }
    if (rh_pct >= 0.0f)
    {
        int32_t h10 = (int32_t) (rh_pct * 10.0f + 0.5f);

        n += (size_t) snprintf(&buf[n], sizeof(buf) - n, ";RH=%ld.%01ld",
                               (long) (h10 / 10), (long) (h10 % 10));
    }

    wifi_send_cmd(buf);
}
