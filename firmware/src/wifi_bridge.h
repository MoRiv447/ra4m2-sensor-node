/*
 * wifi_bridge.h —— DA16200 WiFi 桥（第 4 篇）
 *
 * 模块走 AT-over-UART：SCI2（P301 = RXD2 / P302 = TXD2），115200 8N1。
 * 行为：①启动探测（AT）并尝试恢复 MQTT 连接（未连上时每 10s 自动重试，直到 +NWMQCL:1）；
 *       ②连上后周期把传感器快照以 AT+NWMQMSG 发上云端；
 *       ③模块的所有响应行以 "[WiFi] <<" 前缀透传到调试串口（联调观察用）。
 */
#pragma once

/**
 * @brief 打开 SCI2 UART 并挂上接收（在 hal_entry 初始化阶段调用一次）
 */
void wifi_bridge_init (void);

/**
 * @brief 每格（0.5s tick）调用：处理模块响应行（打印到调试串口）
 */
void wifi_bridge_poll (void);

/**
 * @brief 发一帧传感器数据上云（建议 10s 调用一次）
 *
 * payload 形如：AT+NWMQMSG=RA4M2;lux=12.3;ldr=8.4;T=26.1;RH=48.0
 *
 * @param lux     光照 lx（BH1750 标定值；< 0 表示无效，跳过该字段）
 * @param ldr_lux 光敏电阻（P015）相对照度（< 0 表示无效，跳过该字段）
 * @param temp_c  温度 ℃（<= -50 表示无效）
 * @param rh_pct  湿度 %RH（< 0 表示无效）
 *
 * 注意：MQTT payload 不能含逗号（模块限制）——本函数用分号分隔。
 */
void wifi_bridge_report (float lux, float ldr_lux, float temp_c, float rh_pct);
