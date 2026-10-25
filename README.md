# STM32F407 环境监测智能终端

基于 **STM32F407VET6 + FreeRTOS + LVGL** 的多任务环境监测终端：温湿度/光照采集、触摸屏 UI、WiFi 配网对时、RS485 Modbus RTU 从站、独立看门狗任务健康监控。

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

## 任务一览

| 任务 | 优先级 | 职责 |
|---|---|---|
| GUITask | 24 | LVGL 主循环 + 息屏管理 |
| SensorTask | 24 | DHT11/BH1750 周期采集 |
| WifiTask | 24 | ESP8266 状态机、配网、对时 |
| ModbusTask | 24 | Modbus 帧处理（IDLE 中断唤醒） |
| LEDTask | 24 | LED 队列消费 |
| KEYTask | 40 | 按键扫描 |
| MonitorTask | 40 | 心跳监控 + 喂狗（500ms） |
| CommandTask | 41 | 串口命令 |
| StatsTask | 8 | 栈水位/CPU 占比统计 |

## 构建

- Keil MDK5（AC5 编译器），工程：`stm32f407ve/MDK-ARM/stm32f407ve.uvprojx`
- 当前固件指标（V1.2）：`Code≈213KB RO≈78KB RW≈1KB ZI≈92.5KB`，FreeRTOS 堆 32KB、稳态余量 ≈18KB

## 调试入口

USART1（板载 USB-TTL，115200）输出全部运行日志：boot 复位源、稳态堆余量、WiFi 状态机、Modbus 收发帧、任务列表与 CPU 占比。

## Roadmap（远景）

- CAN 总线（板载收发器，CAN1 = PB8/PB9）
- OTA 升级：先串口 IAP（BootLoader / Flash 分区 / 固件校验 / 回滚），再叠加 MQTT 通道远程升级
- SDIO/TF 卡、MQTT 上云

## 仓库结构

```
stm32f407ve/          固件工程（CubeMX + Keil）
  Core/App/Tasks/     应用任务层
  Core/App/Bsp/       板级驱动（RS485/ESP8266/IWDG/Flash/LCD/触摸/传感器）
  Core/Src/           CubeMX 生成 + freertos.c 任务装配
  Middlewares/Third_Party/littlefs/  LittleFS 文件系统
```
