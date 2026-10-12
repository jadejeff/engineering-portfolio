# 寝室用电安全 IoT 系统 — BLE p2pServer (STM32WBA65RI)

基于 STM32WBA65RI Nucleo 开发板的寝室用电安全监测系统。通过 ESP8266 采集多路传感器数据（温度、电流、功率），经 USART 传输至主控运行 NanoEdge AI 异常检测模型，检测到异常后通过 BLE 通知微信小程序，并回传告警指令至 ESP8266 执行断电保护。

## 系统架构

```
┌──────────────┐    USART/WiFi    ┌──────────────┐    BLE    ┌─────────────┐
│  ESP8266 #1  │ ◄──────────────► │  ESP8266 #2  │ ◄───────► │  STM32WBA   │
│  传感器采集   │    TCP 8266       │  中继转发     │  USART3   │  主控+AI推理 │
│  温度/电流/   │                  │  AP 热点      │           │  BLE p2pS   │
│  功率/烟雾    │                  │              │           │             │
└──────────────┘                  └──────────────┘           └──────┬──────┘
                                                                    │ BLE
                                                              ┌─────▼─────┐
                                                              │ 微信小程序  │
                                                              │ (BLE Client)│
                                                              └───────────┘
```

### 数据流

1. **ESP8266 #1**（采集端）：读取传感器数据，封装为 JSON 格式 `{"t":26.5,"v":220.0,"c":0.5,"p":110.0,...}`，通过 WiFi TCP 发送至中继
2. **ESP8266 #2**（中继端）：创建 AP 热点，接收 TCP 数据，通过 SoftwareSerial 转发至 STM32WBA 的 USART3
3. **STM32WBA**（主控端）：
   - USART3 中断接收 JSON 帧，解析提取 `t`（温度）、`c`（电流）、`p`（功率）三个特征
   - NanoEdge AI 先用正常数据学习建立基线（80 帧），再切换到检测模式
   - 检测到异常（相似度 < 50%）时：消抖 3 帧 + 5 秒冷却 → 发送 `"111"` 告警
   - 告警通过 USART3 回传 ESP8266（执行断电），同时通过 BLE 通知微信小程序
4. **微信小程序**：BLE 连接 p2pServer，实时接收传感器数据和 AI 告警通知

## NanoEdge AI 异常检测

| 参数 | 值 |
|------|-----|
| 模型类型 | 异常检测 (Anomaly Detection) |
| 输入特征 | 3 维 (温度 t, 电流 c, 功率 p) |
| 学习帧数 | 80 帧（实测 81 帧达 94.67% 准确率） |
| 学习超时 | 30 秒（超时强制进入检测模式） |
| 异常阈值 | 相似度 < 50% |
| 消抖策略 | 连续 3 帧异常才触发告警 |
| 告警冷却 | 两次告警最小间隔 5 秒 |
| 采样频率 | 48 Hz |

### AI 工作流程

```
┌─────────────────────────────────────────────────┐
│                NanoEdge AI 状态机                 │
├─────────────────────────────────────────────────┤
│                                                  │
│  [启动] → neai_anomalydetection_init(true)       │
│              ↓                                   │
│  ┌─── 学习阶段 (g_ai_learning_done=0) ───┐       │
│  │  每帧: neai_anomalydetection_learn()   │       │
│  │  计数 ≥ 80 → g_ai_learning_done = 1    │       │
│  │  超时 30s → 强制 g_ai_learning_done = 1│       │
│  └─────────────────┬─────────────────────┘       │
│                    ↓                             │
│  ┌─── 检测阶段 (g_ai_learning_done=1) ───┐       │
│  │  每帧: neai_anomalydetection_detect()  │       │
│  │  similarity < 50% → 异常              │       │
│  │  连续 3 帧异常 + 冷却 5s → 触发告警    │       │
│  │  → USART3 回传 "111" + BLE 通知手机    │       │
│  └───────────────────────────────────────┘       │
│                                                  │
└─────────────────────────────────────────────────┘
```

## 目录结构

```
BLE_p2pServer/
├── Core/
│   ├── Inc/
│   │   └── app_conf.h              # 新增 CFG_TASK_AI_ALERT_ID 任务
│   └── Src/
│       ├── main.c                  # 主循环：JSON解析 + AI推理 + 告警逻辑
│       ├── stm32wbaxx_it.c         # USART3 中断：手动TX环形缓冲 + ORE恢复
│       └── stm32wbaxx_hal_msp.c    # HAL MSP 配置
├── STM32_WPAN/App/
│   ├── app_ble.c                   # BLE应用 + JSON解析函数 parse_json_for_ai()
│   ├── p2p_server.c                # P2P Server特征值（缓冲区扩至128字节）
│   ├── p2p_server_app.c            # AI告警通知任务 + 通知长度优化
│   └── p2p_server_app.h            # AI学习帧数配置 AI_LEARNING_MIN_SAMPLES
├── esp8266_sketch/
│   └── esp8266_sketch.ino          # ESP8266 #2 中继端固件（AP+TCP+串口转发）
├── libneai_project-2026-06-21-13-03_2/
│   ├── libneai.a                   # NanoEdge AI 静态库
│   ├── NanoEdgeAI.h                # NanoEdge AI 头文件
│   ├── metadata.json               # 模型元数据
│   ├── docs/                       # 库文档
│   └── emulators/                  # 模拟器
├── MDK-ARM/
│   └── BLE_p2pServer.uvprojx       # Keil工程文件
├── BLE_p2pServer.ioc               # STM32CubeMX 工程配置
└── README.md
```

## 关键技术实现

### 1. USART3 非阻塞中断发送

为避免 BLE 事件处理被阻塞，USART3 发送改为手动环形缓冲 + 寄存器直写方式，绕过 HAL 状态机：

- `USART3_SendString_IT()`：数据入环形缓冲，直接写 TDR 寄存器启动发送
- `USART3_IRQHandler()`：手动处理 TXE 中断，逐字节发送，发完关闭中断
- 额外处理 ORE（溢出错误）恢复，防止 HAL 因 BASEPRI 屏蔽漏处理导致死锁

### 2. JSON 轻量解析

`parse_json_for_ai()` 使用 `strstr` + `atof` 从 JSON 中提取 `t`、`c`、`p` 三个浮点字段，避免引入完整 JSON 库，节省 Flash 空间。

### 3. AI 告警消抖与冷却

- **消抖**：连续 3 帧异常才触发告警，避免单帧抖动误报
- **冷却**：两次告警最小间隔 5 秒，避免同一波异常连续告警
- **学习超时保护**：30 秒内未完成学习则强制进入检测模式（应对 ESP8266 断连）

## 硬件平台

| 组件 | 型号 | 角色 |
|------|------|------|
| 主控 | STM32WBA65RI (Nucleo-WBA65RI) | AI 推理 + BLE p2pServer |
| 采集端 | ESP8266 #1 | 传感器采集（温度/电流/功率/烟雾/门磁/PIR） |
| 中继端 | ESP8266 #2 | AP 热点 + TCP 服务器 + 串口转发 |
| 负载 | 功率电阻 + 灯 + 扇叶 | 模拟寝室用电场景 |
| 客户端 | 微信小程序 | BLE Client，接收数据与告警 |

## 编译与烧录

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/BLE_p2pServer.uvprojx`
2. 确认 NanoEdge AI 库路径已配置（`libneai_project-2026-06-21-13-03_2/`）
3. 编译并烧录至 Nucleo-WBA65RI（Flash 地址 0x08000000）
4. ESP8266 #2 烧录 `esp8266_sketch/esp8266_sketch.ino`
5. 微信小程序扫描连接 `p2pS_xx`（xx 为 BD ADDRESS 末两位）

## 原始 ST 文档

本项目基于 ST 官方 BLE p2pServer 例程修改，原始文档参考：
- [STM32WBA Peer To Peer Server Applications](https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WBA_Peer_To_Peer#STM32WBA_Peer_to_Peer_Server_application)
- [STM32WBA Bluetooth LE - Peer 2 Peer Applications](https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WBA_Peer_To_Peer)

### Keywords

Connectivity, BLE, BLE protocol, BLE pairing, BLE profile, IoT, NanoEdge AI, Anomaly Detection, Power Safety
