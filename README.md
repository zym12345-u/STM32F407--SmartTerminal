# STM32F407 环境监测智能终端

基于 **STM32F407VET6 + FreeRTOS + LVGL** 的多任务环境监测终端：温湿度/光照采集、触摸屏 UI、WiFi 配网对时、RS485 Modbus RTU 从站、CAN 总线节点、独立看门狗任务健康监控。

> 个人学习/求职项目。所有架构取舍均在代码注释中写明理由，欢迎 issue 交流。

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
- LittleFS 文件系统：2559×4KB≈10MB 分区，二次冷上电免格式化直挂，掉电持久性验证通过
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
| StatsTask | 8 | 栈水位/CPU 占比统计 |

## 构建

- Keil MDK5（AC5 编译器），工程：`stm32f407ve/MDK-ARM/stm32f407ve.uvprojx`
- 当前固件指标（V1.2 基线，V1.4 接入 CAN 后待重测）：`Code≈213KB RO≈78KB RW≈1KB ZI≈92.5KB`，FreeRTOS 堆 32KB、稳态余量 ≈18KB

## 调试入口

USART1（板载 USB-TTL，115200）输出全部运行日志：boot 复位源、稳态堆余量、WiFi 状态机、Modbus 收发帧、任务列表与 CPU 占比。

## Roadmap（远景）

- OTA 升级：先串口 IAP（BootLoader / Flash 分区 / 固件校验 / 回滚），再叠加 MQTT 通道远程升级
- SDIO/TF 卡、MQTT 上云

## 仓库结构

```
stm32f407ve/          固件工程（CubeMX + Keil）
  Core/App/Tasks/     应用任务层
  Core/App/Bsp/       板级驱动（RS485/ESP8266/IWDG/Flash/CAN/LCD/触摸/传感器）
  Core/Src/           CubeMX 生成 + freertos.c 任务装配
  Middlewares/Third_Party/littlefs/  LittleFS 文件系统
```
