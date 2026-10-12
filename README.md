# RA4M2 双无线环境监测节点（Coremaker RA4M2-SENSOR 评测配套）

> 瑞萨 CoreMaker RA4M2-SENSOR 双无线环境监测节点：串口 / 蓝牙 BLE / WiFi-MQTT 三路数据访问 + 触摸换挡低功耗。

基于瑞萨 **CoreMaker RA4M2-SENSOR** 开发板的双无线环境监测节点：同一份传感器数据，三种访问方式
（串口本地 / 蓝牙近场 / WiFi MQTT 云端），外加两个电容触摸焊盘做的"功耗档位换挡"。
本仓库是 EEWorld 瑞萨 CoreMaker RA4M2-SENSOR 评测（第 1~4 篇）的配套源码与文档。

## 硬件

- 主控：CoreMaker RA4M2-SENSOR（R7FA4M2AD3CFL，Arm Cortex-M33）
- 无线：瑞萨 DA14531 蓝牙转接板（AT 固件）+ DA16200 WiFi 转接板
- 传感：DHT22（单总线）、BH1750（I2C 光照）、BMP280（I2C 气压）、10k+10k 分压基准（ADC）
- 工具：USB-TTL（供电/测量）、万用表、面包板、杜邦线

## 功能一览

| 功能 | 说明 |
|---|---|
| 三路数据访问 | 串口（每 10s 一帧）／蓝牙 BLE（每 5s 播报 + `read`/`ping` 命令）／WiFi MQTT（每 10s 发到 broker.emqx.io） |
| 触摸换挡 | 左键=更省电（RUN→SLEEP→STANDBY）、右键=更活跃；STANDBY 档数据暂停、触摸任意键唤醒 |
| 低功耗实测 | 整板口径：26 → 23 → 15.5 → 15.0 mA（详见 `docs/`） |
| 网页控制台 | `web/ra4m2-console.html`：一页集成蓝牙直连 + 云端 MQTT 订阅 + 离线看门狗 |

## 目录

```
firmware/src/          应用层源码（入口 hal_entry.c；power_gear.c = 触摸换挡）
firmware/qe_gen/       QE for Capacitive Touch 生成的触摸配置（2 电极 TS1/TS2）
firmware/configuration.xml   FSP 配置（各 Stack 与中断优先级可对照重建）
docs/                  接线表 / 操作清单 / 测量配方 / 各类记录文档
web/                   网页控制台（单文件）
images/                系统框图
```

## 构建

1. e² studio + FSP 6.3.0（器件选 **R7FA4M2AD3CFL**）；
2. 新建 flat 工程 → 把 `firmware/src` 与 `firmware/qe_gen` 拷入 → 对照 `firmware/configuration.xml`
   配好各 Stack（RTC / GPT / ADC / SCI ×3 / CTSU + rm_touch / LPM 等）与中断优先级 →
   Refresh → Clean → Build；
3. SWD 烧录；串口 115200 8N1。

> ⚠️ 本仓库不包含 FSP 源码（`ra/fsp`），构建需自行安装 FSP；`configuration.xml` 供对照参考。
> 触摸（CTSU）开了自动调谐：上电会先跑一段"调谐热身"（十几毫秒到约 1 秒），属正常行为。

## 串口速查

- 采样帧：`--- sample #N ---`（每 10 s，含 ADC / BH1750 / BMP280 / DHT22 / 片内温度）
- 触摸换挡：`[GEAR] -> SLEEP` ／ `-> STANDBY：数据暂停…` ／ `<- 触摸唤醒：回到 RUN 档`
- 蓝牙帧：`[RA4M2] lux=.. ldr=.. T=.. RH=..`，命令 `read` / `ping`
- WiFi：`AT` → `+NWMQCL:1`（MQTT 已连）→ `+NWMQMSG:...`（订阅回环）

## 低功耗实测（整板口径，不含无线模块）

| 状态 | 读数 |
|---|---|
| 运行 | 26 mA |
| Sleep | 23 mA |
| Software Standby | 15.5 mA |
| Deep Software Standby | 15.0 mA |

地板 ≈15 mA 的构成：CH340C 未连接态（进不了挂起，手册操作态 4-12 mA）+ 指示灯 1-3 mA。
测量方法（免焊接）与差值分析见 `docs/低功耗三档-测量配方.md` 与 `docs/第4篇-低功耗三档-测量记录-2026-10-11.md`。

## 网页控制台

桌面 Chrome / Edge 直接打开 `web/ra4m2-console.html`：

- 左半：**蓝牙近场**（Web Bluetooth 直连，连接时在弹窗里选 `NOTMESBE`）；
- 右半：**云端订阅**（MQTT over WebSocket，默认 `wss://broker.emqx.io:8084/mqtt`，主题 `notmes/ra4m2`；
  mqtt.js 自动从 CDN 加载，需联网）；
- 顶部"最新一帧"数据条 + 离线看门狗（任一路数据断流会在 25/35 秒后标红提醒）。

## 系列文章

- 第 1~4 篇（EEWorld 瑞萨 CoreMaker RA4M2-SENSOR 评测）：<链接待补>

## 许可

MIT License，见 `LICENSE`。第三方材料（芯片数据手册截图、FSP 源码等）不在本仓库内。
