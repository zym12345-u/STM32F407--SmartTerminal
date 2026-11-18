# STM32F407 环境监测智能终端

基于 **STM32F407VET6 + FreeRTOS + LVGL** 的多任务环境监测终端：温湿度/光照采集、触摸屏 UI、WiFi 配网对时、RS485 Modbus RTU 从站、CAN 总线节点、串口 IAP / WiFi OTA、SDIO/FatFS 文件存储、独立看门狗任务健康监控。

> 个人学习/求职项目。所有架构取舍均在代码注释中写明理由，欢迎 issue 交流。
>
> **版本说明**：下文 `V1.0~V2.4` 是**项目阶段号**（功能里程碑），与**固件版本号**（App 遥测 `fw` 字段）并行演进：V2.3 对应固件 1.8、V2.4 对应固件 1.9（当前封版）。两者不是同一计数，不存在"版本回退"。

## 硬件平台

| 模块 | 连接 | 说明 |
|---|---|---|
| MCU | STM32F407VET6 | 128KB SRAM + 64KB CCM，512KB Flash |
| LCD | ILI9341，FSMC | 240x320，背光 PA15 可控 |
| 触摸 | XPT2046，SPI 软件模拟 | 电阻屏 |
| 温湿度 | DHT11 | 单总线 |
| 光照 | BH1750 | I2C，寄存器直配 |
| WiFi | ESP8266 | USART2 (PA2/PA3)，AT 指令 |
| RS485 | 板载收发器，自动方向 | USART3 (PB10/PB11)，免跳线 |
| CAN | 板载收发器，CAN1 | PB8/PB9 (AF9)，500 kbps，板端 120Ω 终端 |
| 调试口 | 板载 USB-TTL | USART1，115200 |
| TF 卡 | SDIO 4-bit | PC8~PC11=D0~D3、PC12=CK、PD2=CMD，AF12；FAT16/32 |
| EEPROM | 24C02，I2C | OTA 搬运请求标志（`0xE8`）与元数据页 |

## 软件特性

**V1.0（已验证）**
- 10 个 FreeRTOS 任务，队列/任务通知/互斥锁分层通信
- LVGL 现代化 UI + 触摸，30s 无操作自动息屏（时间戳比对，零定时器开销）
- WiFi 配网 + SNTP 自动对时（V1.2 起凭据存 W25Q128/LittleFS）
- RS485 Modbus RTU 从站：9600-8-N-1 地址 1，功能码 03/06/16，异常码 01/02/03
- I2C 总线死锁恢复、SPI 触摸数据范围校验等总线容错

**V1.1（已验证）**
- 传感器周期任务改 `vTaskDelayUntil`，消除周期漂移
- RS485 接收改 **USART IDLE 硬件中断 + 任务通知**，无流量时任务挂起零 CPU
- **IWDG 独立看门狗 + 多任务心跳监控**：MonitorTask 汇总心跳、全员存活才喂狗
- **复位源诊断**（RCC->CSR）：boot 打印区分看门狗/软件/上电复位
- 线程安全 `UART_Printf`：互斥锁 + DMA + 完成信号量，整帧原子输出
- DWT 周期统计：任务栈水位 + 各任务 CPU 占比，5s 周期打印

### V1.1 实战踩坑（已修复）
事件驱动任务 `xTaskNotifyWait(portMAX_DELAY)` 死等通知，主站停轮询后心跳停发被误判卡死 → IWDG 误复位。修复：等待加 1s 超时兜底，心跳与外部流量解耦（`ModbusTask.c` 注释有完整记录）。

**V1.2（已验证）**
- W25Q128 SPI Flash（16MB）驱动：DWT 周期计数时基（boot 阶段零 tick 依赖）、字库区地址写保护、真机实测擦写时序
- LittleFS 文件系统：2559×4KB≈10MB 分区（V2.0 起缩至 2048×4KB=8MB 给固件槽让位，见下），二次冷上电免格式化直挂，掉电持久性验证通过
- LVGL 外挂 W25Q128 GBK 字库：RO-data 省 218KB，Unicode→GB2312 二分查表（6877 项），ASCII 回落内置字体
- WiFi 凭据从内部 Flash 迁移至 LittleFS，FreeRTOS 堆扩至 32KB

**V1.3（已验证）**
- CAN1 框架：500 kbps（42MHz/6 预分频，BS1=11/BS2=2/SJW=1，14tq，采样点 85.7%）
- RX ISR 只做搬运（排空 FIFO → `CanMsg_t` 入队），优先级 5/0 满足 FreeRTOS 临界区约束
- 发送**绝不阻塞**：无空闲邮箱即丢弃并计 `tx_drop`，无 `HAL_Delay`、无死等
- 总线状态机按 ESR 判定，**BOFF > EPVF > EWGF** 顺序；非对称去抖（恶化即时上报、恢复需连续 2 次稳定）+ 2s 限流
- 统计单写者设计：`rx_*` 仅 ISR 写、`tx_*` 仅任务写，32 位对齐写免锁
- 编译开关 `CAN_TEST_LOOPBACK` 支持不接线的控制器自测

**V1.4（已验证）**
- 应用层帧约定：`0x701` 心跳（计数/运行秒/TEC/REC/总线状态）、`0x181` 传感器广播（温湿度光照 ×10 定点大端 + 有效位 + 设备号）
- 下行命令 `0x201` + 应答 `0x281`：LED 开关、读版本，归一化结果码（0=OK / 1=NAK / 2=参数错误）
- **数据源单一化**：CAN / Modbus / LVGL 三通道共用 `g_sensorData`，同一份数据三个出口
- 无效值传递：传感器失效时填 `0xFFFF` 并清有效位，Modbus 与 CAN 两路同步生效

### V1.3/V1.4 实战踩坑（已修复）
- **CubeMX 脚本模式对 `.ioc` 手工新增外设识别不完整**：会生成 `can.c`、打开 `HAL_CAN_MODULE_ENABLED`、生成中断 handler，但**不把 `MX_CAN1_Init()` 写进 `main.c`**，也不应用位时序参数。症状是 `hcan1.Instance == NULL`，一烧录就 HardFault。教训：手工加外设后必须四项逐项核对（外设初始化参数 / HAL 模块开关 / 中断 handler / `main` 调用）。
- **ESR 位段读错**：误把低 8 位的标志字节（EWGF/EPVF/BOFF/LEC）当 TEC 打印。健康总线下全是 0 看不出来，只有做故障注入（拔线进 warning→passive）时才暴露——`tec=51`（0x33 = EWGF|EPVF|LEC）物理上不可能，正是这个"不可能的值"暴露了 bug。已改用 CMSIS 宏 `CAN_ESR_TEC/REC`。
- **接收结构体未清零**：`HAL_CAN_GetRxMessage` 只按 DLC 拷贝有效字节，`dlc<8` 时 `data[]` 尾部是栈垃圾；V1.4 命令处理器直接读 `data[0]/data[1]`，对端发 `dlc=1` 的帧可能误触发 LED。

**V2.0（已验证，2026-11-08）**
- **BootLoader**：内部 Flash 前 48KB，App 迁至 `0x08010000`；元数据 sector 3 改为追加记录（v3），掉电不留“已擦未写”裸窗
- **串口 IAP**：上电 3s 窗口 + 屏按钮进入 60s 升级窗口；帧协议、整包 CRC32、断点续传、受控擦写，PC 端工具 `Tools/iap_upgrade.py`
- **W25Q 双固件槽**：槽 A `0x00800000` 存当前 App 整区备份（448KB），槽 B `0x00871000` 存 OTA 镜像；32B 槽记录含 magic/格式/版本/大小/镜像 CRC/记录 CRC
- **WiFi OTA**：ESP8266 HTTP 流式下载 `app.meta` / `app.bin` 到槽 B，`+IPD` 二进制分块落盘，下载后整包 CRC 校验
- **搬运 / 回滚 / 确认**：BootLoader 先校验槽 A 整区 CRC，再把槽 B 搬到内部 App 区并回读双 CRC；连续 3 次上电未确认则自动回滚槽 A
- **App 侧**：OTA 页显示 Current / New，下载成功自动置 24C02 `0xE8` 搬运请求；MonitorTask 全任务健康 3s 后确认启动

### V2.0 实战踩坑（已修复）
- **槽 A/B 记录扇区重叠**：原槽 B 基址 `0x00870000` 会让槽 B 的 4KB 记录扇区压住槽 A 448KB 备份的最后 4KB，写 OTA 记录即破坏槽 A 整区 CRC，回滚基线永久失效。已把槽 B 上移到 `0x00871000`；旧槽 A 备份和旧槽 B 镜像必须重新生成。
- **Rollback 后元数据 size 语义**：回滚用的是槽 A 整区备份，`meta.size` 会记成 448KB 整区而不是实际 App 字节数；不影响启动，列入 V2.1 清理项。

**V2.2（已验证，2026-11-10）**
- **MQTT 上云**：Mosquitto 3.1.1，遥测 1Hz 上行（温湿度/光照/时间戳/固件版本），`status` retained + LWT 离线遗嘱
- **下行命令**：`ping` / `led` / `ota`，QoS1 + 环形队列；MQTT 会话层只搬运，业务解析全在 WifiTask
- **远程升级通知**：服务器向 `stm32/dev01/ota` 推 `{"ver":"1.6"}`，设备只更新本地“发现新版本”并回执，不发起 HTTP
- **远程检查/下载**：`check` 走一次 HTTP 读 `app.meta`，`download` 直接升级；操作前先 `Mqtt_Stop` 让出 ESP8266 单 TCP，结果在重连后补发
- **升级结果确认**：遥测带 `"fw"` 字段，远程不用额外查询就能确认设备已跑在新版本
- **Broker 运行时可配**：串口 `mqttcfg <host> [port]` 写 `/mqtt.conf`，无需重编

### V2.2 实战踩坑（已修复）
- **OTA 主题自发自收回环**：设备把回执也发到 `stm32/dev01/ota`，而它自己也订阅该主题，Broker 把回执投回后设备不认识就再回一条 `unknown`，形成无限回环。修复：带 `"ack"` 字段的消息一律视为本机回执直接丢弃，未知消息只记日志不回包。

**V2.3（已验证，2026-11-13，固件 1.8）**
- **SDIO 4-bit + FatFS**：2GB TF 卡挂载为 `0:/`，实测容量 1.84GiB，文件写/读/逐字节比对全部通过
- **低优先级延后初始化**：`SdCardTask` 在调度器启动 3s 后执行，`HAL_SD_Init` 再慢也不挡 GUI/WiFi
- **4-bit 按官方流程切换**：先 1-bit 完成卡识别，再 `HAL_SD_ConfigWideBusOperation()` 发 ACMD6，最后真读一个扇区确认数据线
- **写后等待**：轮询 `HAL_SD_GetCardState()` 直到卡退出 PROGRAMMING，再发起下一次访问
- **遥测带存储状态**：`"sd"`（是否就绪）、`"sdkb"`（容量 KB），联调期免串口观察
- **日志回传**：串口发送 `logdump`，`0:/LOG.TXT` 内容分块回传到串口，电脑没有读卡器也能查看

### V2.3 实战踩坑（已修复）
- **4-bit 只改主机不改卡**：把 `Init.BusWide = 4B` 直接传给 `HAL_SD_Init`，只写了主机 CLKCR.WIDBUS，卡仍工作在单线；主机按 4 线收，D1~D3 没有起始位，SDIO 报 **STBITERR**。正确做法是先 1-bit 识别，再调 `HAL_SD_ConfigWideBusOperation()` 发 ACMD6。
- **SD 初始化放 `main()` 会黑屏**：调度器启动前调用 `HAL_SD_Init`，无卡或异常卡时会长时间轮询，直接把 GUI 卡死。
- **写扇区后只查一次卡状态**：卡内部编程尚未结束就发起下一次访问，后续命令报 `CMD_RSP_TIMEOUT`，表现为 `f_mount` 成功但 `f_open` 返回 `FR_DISK_ERR`。

**V2.4 / 固件 1.9（封版验证，2026-11-18）**
- **日志异步落盘**：低优先级队列写 `0:/LOG.TXT`，64KB 滚动；传感器数值变化才记录，稳定时每 30s 兜底一条
- **OTA 备份补强**：远程下载前自动补槽 A 备份，独立 `BACKUP` 状态与错误码；串口 IAP 备份失败后仍可重试
- **Flash 容错**：W25Q 擦写超时软复位并重试，补充 SR1/地址/耗时日志；OTA 状态行使用 ASCII，避免外挂字库忙时黑块/白块
- **MQTT 小修**：SUBACK 被拒进入重连，缓冲满日志增加空帧越界保护
- **日志故障反馈**：打开失败打印 FatFS 错误并重挂载一次；`logdump` 未打开明确拒绝，读错误重试一次、仍失败明确中止，不作假成功
- **真机回归**：串口 IAP 与远程 OTA 均通过；远程链路完成槽 A 备份、双重 CRC、BootLoader 搬运和启动确认；`logdump` 完整回传 `34007/34007` 字节（期间曾出现一次 `FR_DISK_ERR`，固件明确中止并报错，未作假成功；再次回传完整恢复）

## 任务一览

| 任务 | 优先级 | 职责 |
|---|---|---|
| GUITask | 24 | LVGL 主循环 + 息屏管理 |
| SensorTask | 24 | DHT11/BH1750 周期采集 |
| WifiTask | 24 | ESP8266 状态机、配网、对时 |
| ModbusTask | 24 | Modbus 帧处理（IDLE 中断唤醒） |
| CanTask | 24 | CAN 收帧分发、心跳/传感器周期广播、总线状态轮询 |
| LEDTask | 24 | LED 队列消费 |
| KEYTask | 40 | 按键扫描 |
| MonitorTask | 40 | 心跳监控 + 喂狗（500ms） |
| CommandTask | 41 | 串口命令 |
| StatsTask | 8 | 栈水位/CPU 占比统计（编译开关 `STATS_TASK_ENABLE`，默认关闭） |
| SdCardTask | 8 | TF 卡延后初始化 + 文件读写自检 |

> 上表为当前 11 个业务任务（另有 Idle/Timer 系统任务）；历史阶段的"10 个任务"描述见各版本段，以本表为准。SensorTask 在采集瞬间临时提至 Realtime（48）保时序，稳态回落 Normal（24）。

## 构建

- Keil MDK5（AC5 编译器）
  - App 工程：`stm32f407ve/MDK-ARM/stm32f407ve.uvprojx`
  - BootLoader 工程：`stm32f407ve/MDK-ARM/BootLoader.uvprojx`
- 当前固件指标（V2.4 / 固件 1.9，2026-11-18 重测）：
  - App：`Code=241400 (235.7KB) / RO=80056 (78.2KB) / RW=1332 (1.3KB) / ZI=121256 (118.4KB)`
  - BootLoader：`Code=8140 (7.9KB) / RO=424 (0.4KB) / RW=20B / ZI=5476 (5.3KB)`，远低于 48KB 分区
  - FreeRTOS 堆 32KB
- OTA 工具：`stm32f407ve/Tools/iap_upgrade.py`（串口 IAP）、`stm32f407ve/Tools/ota_serve.py`（生成 `app.bin`/`app.meta` + 本地 HTTP 服务）；同目录另有 `ota_cutoff_server.py`（断流测试）、`ota_make_bad.py`（坏包测试）

## 调试入口

USART1（板载 USB-TTL，115200）输出全部运行日志：boot 复位源、稳态堆余量（所有任务建完后打一次 `[SYS] free heap ...`）、WiFi 状态机、Modbus 收发帧。

串口命令（`CommandTask`，回车结尾）：

| 命令 | 作用 |
|---|---|
| `logdump` | 回传 TF 卡 `0:/LOG.TXT` 到串口 |
| `otacfg <host> [port]` | 配置 OTA 服务器地址（端口缺省沿用现值） |
| `mqttcfg <host> [port]` | 配置 MQTT broker，写 `/mqtt.conf` |
| `at <原始指令>` | 透传 AT 指令给 ESP8266 排障 |

任务栈水位 / CPU 占比统计由编译开关 `STATS_TASK_ENABLE`（`freertos.c`）控制，默认关闭，排查性能时打开。

## 版本里程碑（均已验证，当前无进行中条目）

- V2.1：OTA 产品化（服务器地址可配置、启动/手动检查更新、一键升级并自动复位、失败重试）
- V2.2：MQTT 上云（传感器上报、下行命令、远程升级通知）
- V2.3：SDIO 4-bit + FatFS 挂载与读写
- V2.4：日志异步落盘 + 64KB 滚动 + logdump 回传

## 仓库结构

```
stm32f407ve/
  BootLoader/         BootLoader 工程（config/port/core 三层，无 HAL）
  Core/App/Tasks/     应用任务层
  Core/App/Bsp/       板级驱动（24C02/BH1750/CAN/DHT11/ESP8266/Font/I2CBus/IWDG/
                      LCD/LittleFS/MQTT/OTA/RS485/SDCard/W25Q128/WifiStore/XPT2046）
  Core/App/Types/     跨任务共享类型（OTA 状态等）
  Core/Src/           CubeMX 生成 + freertos.c 任务装配 + SDIO/FatFS 桥接
  Middlewares/Third_Party/littlefs/  LittleFS 文件系统
  Middlewares/Third_Party/FatFS/     FatFS（TF 卡）
  Tools/              iap_upgrade.py（串口 IAP）、ota_serve.py（OTA 包 + HTTP 服务）、
                      ota_cutoff_server.py / ota_make_bad.py（OTA 容错测试）
```
