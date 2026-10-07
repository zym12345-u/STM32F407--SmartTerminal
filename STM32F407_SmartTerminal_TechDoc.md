# STM32F407 智能终端 — 技术实现文档

> 基于 STM32F407VET6 + FreeRTOS + LVGL 的多传感器智能终端，集成环境感知、触摸交互、WiFi 联网对时、自动息屏与总线容错。

---

## 目录

1. [项目概述](#1-项目概述)
2. [技术栈](#2-技术栈)
3. [硬件平台与引脚分配](#3-硬件平台与引脚分配)
4. [系统架构](#4-系统架构)
5. [任务详解](#5-任务详解)
6. [核心模块实现](#6-核心模块实现)
7. [设计思路与关键决策](#7-设计思路与关键决策)
8. [易犯错误与踩坑记录](#8-易犯错误与踩坑记录)
9. [编译与烧录](#9-编译与烧录)

---

## 1. 项目概述

本项目是一个运行在 **STM32F407VET6** 上的嵌入式智能终端，主要功能：

| 功能 | 实现 |
|------|------|
| 环境监测 | BH1750 光照 + DHT11 温湿度，2s 周期采集，屏幕实时显示 |
| 触摸交互 | XPT2046 电阻屏 + LVGL v8.3，按钮控制 LED、WiFi 配网界面 |
| LED 控制 | 实体按键 / 屏幕按钮 / 串口命令三路统一控制绿/蓝 LED |
| 网络对时 | ESP8266 AT 固件连 WiFi + SNTP 北京时间，30 分钟自动重同步 |
| 凭据掉电保存 | WiFi SSID/密码写入内部 Flash Sector 11，开机自动连 |
| 自动息屏 | 超时关背光，触摸/按键唤醒，显存保持无需重绘 |
| 总线容错 | I2C 死锁自动恢复 + 重试；SPI 触摸 ADC 范围校验防误触 |
| 运行诊断 | 5s 周期打印任务栈水位 + CPU 占比（DWT 64 位累加） |

---

## 2. 技术栈

### 2.1 软件

| 组件 | 版本/说明 |
|------|-----------|
| 内核 | FreeRTOS（通过 CMSIS-RTOS2 API 封装） |
| GUI | LVGL v8.3.11，16 位色深，GB2312 中文字体 |
| HAL | STM32F4 HAL **精简子集**（不含 `stm32f4xx_hal_i2c`，I2C 直配寄存器） |
| 工具链 | Keil MDK-ARM v5 |
| 配置 | STM32CubeMX（生成 GPIO/UART/时钟骨架，外设驱动自写） |

### 2.2 硬件接口技术

| 外设 | 技术 | 说明 |
|------|------|------|
| LCD | FSMC Bank1 + 8080 并口 16bit | ILI9341，320×240 横屏 |
| 触摸 | **GPIO 模拟 SPI**（bit-bang） | XPT2046，非硬件 SPI |
| 光照 | **硬件 I2C1**（寄存器直配） | BH1750，100kHz 标准模式 |
| 温湿度 | **单总线**（GPIO 时序） | DHT11，us 级时序敏感 |
| WiFi | **USART3 + AT 指令** | ESP8266，中断接收环形缓冲 |
| LED/按键 | GPIO 轮询 | 积分式消抖 |
| 凭据存储 | **内部 Flash** | Sector 11 @ 0x080E0000，128KB |

---

## 3. 硬件平台与引脚分配

### 3.1 时钟

```
HSE = 25MHz  →  PLL (M=25, N=336, P=2)  →  SYSCLK = 168MHz
AHB  = 168MHz  (HCLK)
APB1 = 42MHz   (I2C1, USART3, TIM4 tick)
APB2 = 84MHz
```

### 3.2 引脚表

| 引脚 | 功能 | 备注 |
|------|------|------|
| PA0 | KEY_1 | 内部**下拉**，按下高电平（上升沿） |
| PA1 | KEY_2 | 内部**上拉**，按下低电平（下降沿） |
| PB2 | 蓝灯 LED | 低电平点亮（灌电流） |
| PC5 | 绿灯 LED | 低电平点亮（灌电流） |
| PB6 | I2C1_SCL | BH1750 |
| PB7 | I2C1_SDA | BH1750 |
| PB10 | USART3_TX | → ESP8266 RXD |
| PB11 | USART3_RX | ← ESP8266 TXD |
| PE4 | XPT2046 IRQ | 触摸按下检测（查询，非中断） |
| PD13 | XPT2046 CS | 模拟 SPI 片选 |
| PE0 | XPT2046 CLK | 模拟 SPI 时钟 |
| PE2 | XPT2046 MOSI | 模拟 SPI 主出从入 |
| PE3 | XPT2046 MISO | 模拟 SPI 主入从出 |
| PA15 | LCD 背光 | 低电平关背光（息屏） |
| PC1 | DHT11 DATA | 单总线 |

---

## 4. 系统架构

### 4.1 任务划分

```
┌──────────────────────────────────────────────────────────┐
│                      FreeRTOS 调度器                       │
├──────────┬──────────┬──────────┬──────────┬──────────────┤
│ KEYTask  │ LEDTask  │CommandTask│ GUITask  │  SensorTask  │
│ (High)   │ (Normal) │ (High1)  │ (Normal) │  (Normal)    │
│ 按键扫描  │ LED消费  │ 串口解析  │ LVGL渲染 │ BH1750+DHT11 │
├──────────┴──────────┴──────────┴──────────┴──────────────┤
│  WifiTask (Normal) — ESP8266 连网、SNTP 对时、本地走秒    │
├──────────────────────────────────────────────────────────┤
│  StatsTask (Low) — 栈水位 + CPU 占比（DWT）              │
└──────────────────────────────────────────────────────────┘
```

### 4.2 队列通信

```
KEYTask ──┐
GuiTask ──┼──► LEDQueue(16, LEDMessage*) ──► LEDTask
CommandTask┘

USART1中断 ──► CommandQueue(16, uint8_t) ──► CommandTask

GuiTask ──► cmdQ(2, WifiCmd_t) ──► WifiTask
WifiTask ──► evtQ(8, WifiEvent_t) ──► GuiTask
```

### 4.3 共享数据（非队列）

| 结构体 | 生产者 | 消费者 | 同步方式 |
|--------|--------|--------|----------|
| `g_sensorData` | SensorTask | GuiTask | `valid` 标志，float 允许撕裂读 |
| `g_clockData` | WifiTask | GuiTask | **临界区**保护结构体拷贝 |
| `g_wifiScan` | WifiTask | GuiTask | 事件同步（单写单读） |

---

## 5. 任务详解

### 5.1 KEYTask — 按键扫描（生产者）

- 10ms 周期轮询 KEY_1/KEY_2
- **积分式消抖**：连续 3 次（30ms）同电平才确认
- 仅在稳定**边沿**翻转对应 LED（上升沿→绿灯，下降沿→蓝灯）
- 翻转前**读 LED 引脚电平**为唯一真相，与其他控制源不冲突
- 按键活动触发 `PowerMgr_Wakeup()`（息屏唤醒/刷新计时）

### 5.2 LEDTask — LED 控制（消费者）

- `osWaitForever` 阻塞等 LEDQueue，不占 CPU
- 收到 `LEDMessage{color, state}` 后写对应引脚
- **低电平点亮**：On→RESET，Off→SET
- 消费完 `vPortFree(msg)`，生产者用 `pvPortMalloc` 分配

### 5.3 CommandTask — 串口命令解析

- 从 CommandQueue 逐字节取 USART1 数据
- 帧格式：`[0xAA][LEN][COLOR1][STATE1]...[CHECKSUM]`
- 校验和 = 前面 LEN-1 字节累加和（uint8 溢出截断）
- COLOR: 2→绿，3→蓝（1=红忽略，本板无红灯）
- 解析成功后打包 `LEDMessage*` 投递 LEDQueue

### 5.4 GUITask — LVGL 界面

- 初始化 ILI9341（FSMC）+ XPT2046（模拟 SPI）+ LVGL
- 创建两个按钮（左=绿灯，右=蓝灯），点击投递 LEDQueue
- LVGL 定时器周期刷新：温湿度/光照标签 + 网络时钟状态栏
- WiFi 配网页：热点列表 + 密码键盘，通过双向队列与 WifiTask 交互
- 每轮调用 `PowerMgr_Process()` 检查息屏超时
- LVGL tick 由 `xTaskGetTickCount()` 提供（`LV_TICK_CUSTOM=1`），无需额外定时器

### 5.5 SensorTask — 传感器采集

- 2s 周期采集 BH1750（I2C）+ DHT11（单总线）
- DHT11 读取期间**临时提升任务优先级到 Realtime**（osPriorityRealtime）
  - 原因：单总线时序窗口 70~100us，被任务抢占会导致 T:err
  - 只挡任务切换，不影响 UART/DMA 中断
- 串口打印用**整数+小数拆分**，避免 `%f` 浮点格式化（省栈省 CPU）
- 失败只置 `valid=0`，**不覆盖上次有效值**

### 5.6 WifiTask — WiFi 联网对时

**双队列模型：**
- `cmdQ`（深 2）：GuiTask → WifiTask（扫描/连接命令）
- `evtQ`（深 8）：WifiTask → GuiTask（扫描完成/连接结果）

**运行流程：**
1. 开机从 Flash 加载凭据 → 有则自动连 WiFi + SNTP 对时
2. 无凭据 → 等待屏幕配网
3. 已连接：每秒本地走秒（公历进位+星期），30 分钟 SNTP 重同步
4. 掉线：每 10s 用保存凭据自动重连

**AT 关键命令：**
- `AT+CIPSNTPCFG=1,8,"202.120.2.101","time.nist.gov","ntp.aliyun.com"` — 配置 SNTP（时区 8）
- `AT+CWLAP` — 扫描热点（解析 `+CWLAP:(ecn,"SSID",rssi,...)`）
- `AT+CWJAP="ssid","pwd"` — 连接 AP
- `AT+CIPSNTPTIME?` — 查询网络时间

### 5.7 StatsTask — 系统诊断

- 5s 周期打印：
  - `vTaskList`：任务名 / 状态 / 优先级 / **栈水位**（StackMin 剩余字）
  - `vTaskGetRunTimeStats`：各任务 CPU 占比
- **DWT 64 位累加**解决 CYCCNT 32 位回绕问题（见 §8.6）

---

## 6. 核心模块实现

### 6.1 I2C1 直配寄存器 + 总线恢复

**背景**：本工程 HAL 库为精简子集，未包含 `stm32f4xx_hal_i2c.c`，因此 I2C1 直接操作寄存器。

**初始化（`I2C1_HardwareInit`）：**
```c
I2C1->CR2   = 42U;    // FREQ = APB1 = 42MHz
I2C1->CCR   = 210U;   // 100kHz: 42M/(2*100k) = 210
I2C1->TRISE = 43U;    // 1000ns/23.8ns + 1 ≈ 43
```

**总线死锁恢复（`I2C1_BusRecovery`）：**

死锁根因：从机在传输中途掉电/复位，SDA 被持续拉低，I2C1 BUSY 位永远置位。

恢复步骤（必须按顺序）：
1. 关闭 I2C1 外设（`CR1 &= ~PE`）
2. PB6/PB7 切回 GPIO 开漏输出
3. 产生**最多 9 个 SCL 脉冲**，每发一个读 SDA，SDA 变高则跳出
   - 9 个脉冲足以让任何从机（8 位数据 + 1 位 ACK）释放 SDA
4. 产生 STOP 条件（SCL 高期间 SDA 低→高）
5. 重新调用 `I2C1_HardwareInit()` 恢复外设

**所有超时路径**（BUSY/SB/ADDR/TXE/RXNE/BTF）统一调用 `I2C1_BusRecovery()` 再返回失败。

**重试机制：**
- `BH1750_Init`：上电→复位→配置模式，最多重试 3 次
- `BH1750_Read`：读失败重试，全部失败置 `s_bh1750_ready=0`，下次自动重新初始化

### 6.2 XPT2046 触摸 + ADC 范围校验

**硬件 SPI vs 模拟 SPI**：本工程用 GPIO 模拟 SPI（bit-bang），引脚见 §3.2。

**`readAdcXY()` 采样策略：**
- 最多采样 4 次（IRQ 保持低电平时）
- 去掉最大值/最小值后平均
- 原函数固定 `return 1`，**已改造为范围校验后返回**

**ADC 范围校验：**
```c
#define XPT2046_ADC_MIN  50
#define XPT2046_ADC_MAX  4050   // 12 位 ADC 正常触摸范围
```

MISO 断线时 GPIO 输入读到 **0**，从机无响应时读到 **0 或 4095**，均落在范围外。

**`XPT2046_IsPressed()` 丢弃无效采样：**
- 两次 `readAdcXY()` 调用都检查返回值
- 任意一次校验失败 → `return 0`（视为未按下），不返回错误坐标给 LVGL

### 6.3 DHT11 单总线时序

- 起始信号：主机拉低 18ms 以上 → 释放 → DHT11 拉低 80us 响应 → 拉高 80us
- 40 位数据：每 bit 以 50us 低电平开始，高电平 26~28us=0，70us=1
- **读取期间任务优先级临时提升到 Realtime**，防止被其他任务抢占导致时序窗口错过

### 6.4 ESP8266 AT 驱动

**RX 架构：**
- USART3 中断逐字节进**环形缓冲**，ISR 不调用任何 RTOS API
- `ESP8266_Cmd`：任务上下文阻塞发送 + `osDelay` 轮询应答（让出 CPU，非忙等）
- `ESP8266_CopyResponse`：把环形缓冲拍快照成线性字符串供解析

**SNTP 对时：**
- `AT+CIPSNTPCFG` 配置 3 个服务器依次尝试
- `AT+CIPSNTPTIME?` 查询，解析 `+CIPSNTPTIME:Tue Oct 06 15:08:54 2026`
- 仍返回 1970 年 → 未同步，继续轮询（最多 20 次 × 1s）

### 6.5 Flash 凭据存储

**存储位置：** STM32F407 Sector 11 @ `0x080E0000`，128KB。

**关键设计：**
- Keil 工程 IROM 裁剪到 448KB（`0x08000000~0x080DFFFF`），程序不会与 Sector 11 冲突
- `WifiCred_t` 结构含 `magic=0x57494649`（"WIFI"）+ 校验和，判定是否写过
- **擦除 128KB 扇区期间内核停滞约 1~2 秒**（取指暂停），所有任务冻结
  - 保存动作安排在"连接成功后"执行，界面停在 Saving 状态，用户无感

### 6.6 电源管理（自动息屏）

**核心思路**：维护 `s_lastActiveTick`（最后活动 tick），超时关背光。

- `PowerMgr_Feed()`：刷新活动计时（触摸/按键/串口命令调用）
- `PowerMgr_Process()`：未息屏时检查 `now - lastActive >= 超时` → 关背光
- `PowerMgr_Wakeup()`：息屏时开背光 + 刷新计时

**只关背光**（PA15 低电平），不发 ILI9341 睡眠命令，显存/FSMC 保持，**唤醒后无需重绘**。

### 6.7 LVGL 中文字体

- 启用 `LV_FONT_GB2312_16`（GB2312 二级汉字，16 号字）
- 字体文件由 `lv_font_conv` 从 `simhei.ttf` 生成（注意：**simsun.ttc 不被支持**，TTC 格式需先转 TTF）
- 中文 SSID 在配网列表中可正常显示

---

## 7. 设计思路与关键决策

### 7.1 三路 LED 控制统一

实体按键、屏幕按钮、串口命令三路控制源**统一走 LEDQueue → LEDTask**。

**关键决策**：翻转前**读 LED 引脚电平**为唯一真相，而非维护软件状态变量。这样任何一方改灯，其他方读到的都是当前真实电平，不会出现"软件以为亮、实际灭"的错位。

### 7.2 传感器数据"失败不覆盖"

SensorTask 采集失败时只置 `valid=0`，**不覆盖 `g_sensorData` 中上次的有效值**。GuiTask 读到 `valid=0` 时显示 "err"，但数值仍为上次成功值，界面不会闪烁归零。

### 7.3 时钟数据用临界区而非互斥锁

时钟是多字段结构体（年/月/日/时/分/秒/星期），读取时可能恰好碰到整点走秒更新。

- 用 `taskENTER_CRITICAL` 包 `memcpy`（几 us），不影响实时性
- 不需要互斥锁（`ClockData_Set/Get` 是 `static inline`，调用点固定）
- 传感器 float 不加锁：显示场景对撕裂读容忍度高（最多一帧半旧半新）

### 7.4 WiFi 配网用双向队列 + 共享扫描结果

- 命令/事件走队列（小数据，频繁）
- 热点列表（8 个 AP × ~40 字节）走共享结构体 `g_wifiScan`
  - WifiTask 扫描完成后写入，GuiTask 收到 `SCAN_DONE` 事件后读取
  - 单写单读 + 事件同步，无需临界区（写完成后才发事件）

### 7.5 DHT11 读取期间提优先级

单总线时序窗口 70~100us，被任务级抢占会导致采样失败（T:err）。临时提到 Realtime 只挡任务切换，不影响中断（UART/DMA），整个读取约 5ms，代价可接受。

---

## 8. 易犯错误与踩坑记录

### 8.1 按键上下拉方向相反

**现象**：KEY_2（蓝灯键）按下无反应。

**原因**：CubeMX 生成的 GPIO 默认 `NOPULL`（浮空），PA1 空闲电平不确定。KEY_1 是内部下拉、KEY_2 是内部上拉，**方向相反**，必须在 App 层重配。

**解决**：KeyTask 启动时显式 `GPIO_PULLDOWN`（KEY_1）/ `GPIO_PULLUP`（KEY_2）。

### 8.2 LED 低电平点亮

**现象**：设置 GPIO_PIN_SET 灯不亮。

**原因**：板载 LED 是**灌电流**驱动，低电平点亮。

**解决**：On→`GPIO_PIN_RESET`，Off→`GPIO_PIN_SET`。main.c 调度器启动前先 `SET` 熄灭两灯，使物理状态与任务中的 `LEDState_Off` 一致。

### 8.3 HAL 库无 I2C 驱动

**现象**：`HAL_I2C_Init` 链接错误。

**原因**：工程 HAL 是精简子集，未包含 `stm32f4xx_hal_i2c.c`。

**解决**：直接操作 I2C1 寄存器（CR2/CCR/TRISE/CR1/DR/SR1/SR2），风格与 FSMC 直配一致。

### 8.4 DHT11 时序被抢占

**现象**：温湿度偶发 T:err。

**原因**：SensorTask 优先级 Normal，读单总线期间被其他任务抢占，错过 us 级时序窗口。

**解决**：读 DHT11 前后 `osThreadSetPriority(Realtime/Normal)`。

### 8.5 printf %f 栈开销

**现象**：SensorTask 栈水位低。

**原因**：`%f` 浮点格式化在 Cortex-M4 上栈开销大（需 8 字节对齐 + 临时缓冲）。

**解决**：拆成整数+小数两位分别打印：`T=%d.%dC`。

### 8.6 DWT CYCCNT 32 位回绕

**现象**：运行几分钟后 CPU 占比全部失真。

**原因**：`DWT->CYCCNT` @168MHz 是 32 位，**25.56 秒就回绕**。

**解决**：在 `vApplicationTickHook` 中每 1ms 做无符号差分累加进 64 位变量（`s_runCycles += now - last`），读出时右移 10 位（约 164kHz，32 位 7.3 小时才回绕）。

### 8.7 CMSIS-RTOS2 stack_size 单位

**现象**：任务栈溢出，但设置的值看起来够大。

**原因**：CMSIS-RTOS2 的 `stack_size` 单位是**字节**，不是字。`128 * 4` = 512 字节 = 128 字。

**解决**：大栈任务直接写字节数（如 `GUI_TASK_STACK_SIZE 2560U`），并通过 StatsTask 的 StackMin 列监控。

### 8.8 Flash 擦除停 CPU

**现象**：保存 WiFi 凭据时整个系统卡顿 1~2 秒。

**原因**：STM32F4 擦除 Flash 期间取指暂停，所有任务冻结。

**解决**：保存动作安排在"连接成功后"执行，界面显示 Saving 状态，用户无感等待。

### 8.9 lv_font_conv 不支持 TTC

**现象**：用 simsun.ttc 生成中文字体报错。

**原因**：`lv_font_conv` 不支持 TTC（TrueType Collection）格式。

**解决**：改用 `simhei.ttf`（单 TTF 文件）生成 GB2312 字体。

### 8.10 I2C 总线死锁

**现象**：BH1750 读取永远超时，BUSY 位不清除。

**原因**：从机在传输中途异常，SDA 被钳位在低电平。

**解决**：9 个 SCL 脉冲 + STOP + 重新初始化 I2C1（见 §6.1）。

### 8.11 SPI 触摸返回错误坐标

**现象**：MISO 断线时 LVGL 收到 (0,0) 坐标，按钮被误触。

**原因**：MISO 断线时 GPIO 输入读到 0，ADC 值为 0，未做范围校验。

**解决**：ADC 范围校验（50~4050），越界则 `readAdcXY()` 返回 0，`IsPressed` 丢弃该次采样。

---

### 8.12 LVGL 按钮按下缩放导致"消失"

**现象**：按钮按下时部分区域消失，只剩中间一小块。

**原因**：`lv_style_set_transform_zoom(&press, 235)`（缩放 92%）+ 180ms transition 动画。缩放以对象中心为原点，边缘露出背景，视觉上像"消失"。在小尺寸按钮（<50px）上尤其明显。

**解决**：**不要用 transform_zoom 做按钮按下反馈**。改用 `lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED)`（半透明变暗），无动画、无缩放、不会消失。

---

### 8.13 LVGL 控件背景透明只显示边框

**现象**：所有按钮、卡片只显示边框，背景完全透明。

**原因**：调用 `lv_obj_remove_style_all()` 后移除了默认 `bg_opa`，之后只设了 `bg_color` 但没设 `bg_opa=LV_OPA_COVER`。LVGL 默认 `bg_opa=0`（透明）。

**解决**：每个样式必须显式设置 `lv_style_set_bg_opa(&style, LV_OPA_COVER)`。

---

### 8.14 LVGL 开关被父容器裁剪

**现象**：开关底部被截掉一部分。

**原因**：`lv_obj_create` 创建的容器默认 `LV_OBJ_FLAG_SCROLLABLE`，可能裁剪超出边界的子控件。

**解决**：`lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE)`，并确保容器高度 > 子控件高度 + padding。

---

### 8.15 TFT 屏深底浅灰文字看不清

**现象**：深灰卡片(`#1e293b`) + 浅灰文字(`#94a3b8`)，文字几乎看不见。

**原因**：TFT 屏对比度有限，深色背景上的灰色文字亮度不足。

**解决**：改用**浅色卡片(`#f1f5f9`) + 深色文字(`#0f172a`)** 高对比方案，或用纯白(`#ffffff`)文字。

---

### 8.16 label 设置固定宽度 + CLIP 导致文字残留

**现象**：LED 开关旁边残留一个绿色的"N"字母。

**原因**：`lv_obj_set_width(lbl, 70)` + `LV_LABEL_LONG_CLIP` 模式下，文字渲染位置计算偏差，前面的字母被裁剪，只残留最后一个字母。

**解决**：不要给短文本 label 设固定宽度 + CLIP，让 label 按内容自适应宽度。

---

## 9. 编译与烧录

### 9.1 工程结构

```
stm32f407ve/
├── Core/
│   ├── App/
│   │   ├── Bsp/          # 外设驱动（BH1750/DHT11/ESP8266/LCD/XPT2046/Flash）
│   │   ├── Tasks/        # FreeRTOS 任务实现
│   │   └── Types/        # 共享数据结构
│   ├── Inc/              # CubeMX 生成的头文件
│   └── Src/              # main.c / freertos.c / gpio.c / usart.c
├── LVGL/                 # LVGL v8.3 源码 + lv_conf.h
├── Drivers/              # HAL + CMSIS
└── MDK-ARM/              # Keil 工程
```

### 9.2 编译

用 Keil MDK-ARM 打开 `MDK-ARM/stm32f407ve.uvprojx`，直接 Build。

### 9.3 烧录

- ST-Link 连接 SWD 接口
- Keil → Download（F8）
- 串口连接 USART1（115200 8N1）查看启动日志

### 9.4 验证清单

烧录后串口应看到：
```
[SYS] boot <date> <time>
[SENSOR] BH1750 init ok
[SENSOR] DHT11 init ok
[GUI] LCD init ok, 240x320
[GUI] touch init ok
[GUI] lv_init ok
[WIFI] ESP8266 USART3(PB10/PB11) init ok
[WIFI] saved credential: <SSID>, auto connecting...
[WIFI] joined, SNTP syncing...
[WIFI] synced: <date> <time>
[SENSOR] T=xx.xC H=xx.x% L=xxxlux  (T:ok L:ok)
```

---

## 10. 项目总结：模块硬件连接、驱动实现与核心代码逻辑

本章以**教程视角**逐模块讲解硬件连接方式、驱动分层设计与核心代码逻辑，帮助读者从"接线 → 驱动 → 应用"完整理解每个外设是如何跑起来的。

---

### 10.1 LCD 显示模块（ILI9341 + FSMC）

#### 硬件连接

ILI9341 通过 **FSMC Bank1** 与 STM32 相连，采用 8080 并口 16 位模式：

| STM32 引脚 | FSMC 功能 | LCD 信号 | 说明 |
|-----------|----------|---------|------|
| PD4~PD15  | FSMC_D0~D15 | D0~D15 | 16 位双向数据总线 |
| PD7  | FSMC_NE1 | CS | 片选（FSMC Bank1 Sector1） |
| PD11 | FSMC_A6  | RS/DC | 数据/命令选择线（A6 地址线） |
| PD5  | FSMC_NOE | RD  | 读使能 |
| PD12 | FSMC_NWE | WR  | 写使能 |
| PA15 | GPIO 输出 | BLK | 背光控制（高电平点亮） |
| NRST | MCU 复位 | RST | 硬件复位 |

**关键地址映射**：FSMC Bank1 起始 `0x60000000`，用 A6（位 6）区分命令/数据。
```
LCD_REG 地址 = 0x60000000 | (1 << 6) - 2 = 0x6001FFFE   // 写命令
LCD_RAM 地址 = 0x60000000 | (1 << 6)     = 0x60020000   // 写数据
```
代码中定义：
```c
volatile typedef struct { uint16_t LCD_REG; uint16_t LCD_RAM; } LCD_TypeDef;
#define LCD ((LCD_TypeDef *)0x6001FFFE)
```
写命令 `LCD->LCD_REG = 0x2A;` 写数据 `LCD->LCD_RAM = color;`，由 FSMC 硬件自动产生 CS/RD/WR/DC 时序，CPU 只需一次内存写。

#### 驱动实现

**初始化流程**（`LCD_Init`）：
1. 配置 FSMC Bank1 Sector1：16 位宽、地址建立时间、数据保持时间
2. 发送 ILI9341 初始化序列（Sleep Out → 像素格式 RGB565 → 显示方向 → Display On）
3. 初始化 **DMA2 Stream5**（Memory-to-Memory）用于像素批量搬运

**核心写入流程**：
```
设置窗口(0x2A 列地址 / 0x2B 行地址) → 写 0x2C(写显存) → 逐像素写 LCD_RAM
```

**DMA 加速 flush**：LVGL 调用 `LCD_DispFlush` 时：
1. 整区设置 ILI9341 窗口
2. 启动 DMA2 M2M，把像素缓冲搬到 `0x60020000`（LCD_RAM 地址）
3. DMA 完成中断释放信号量，LVGL 继续下一帧
- 省去 CPU 逐像素写循环，刷新速率显著提升
- `lcdDmaBusy` 标志防止 flush 冲突

#### 设计要点

- FSMC 把 LCD 当**外部 SRAM** 访问，写显存=写内存，效率远高于 GPIO 模拟
- 背光独立控制（PA15），息屏只关背光不关 ILI9341，显存保持
- DMA 传输期间 GuiTask 阻塞等信号量，不占 CPU

---

### 10.2 触摸模块（XPT2046 + GPIO 模拟 SPI）

#### 硬件连接

XPT2046 是电阻屏触摸控制芯片，本工程**不使用硬件 SPI**，而是用 GPIO 模拟 SPI 时序（bit-bang）：

| STM32 引脚 | XPT2046 信号 | 方向 | 说明 |
|-----------|-------------|------|------|
| PE4 | IRQ/INT | 输入 | 触摸按下指示（空闲高，按下低），查询方式 |
| PD13 | CS | 输出 | 片选（低有效） |
| PE0 | CLK/DCLK | 输出 | SPI 时钟 |
| PE2 | DIN/MOSI | 输出 | 主机→从机数据 |
| PE3 | DOUT/MISO | 输入 | 从机→主机数据 |

#### 驱动实现

**底层 SPI 时序**（用宏直接操作 BSRR 寄存器，速度最快）：
```c
#define CS_LOW    (CS_GPIO->BSRR = CS_PIN << 16)   // 复位位 = 拉低
#define CS_HIGH   (CS_GPIO->BSRR = CS_PIN)          // 置位位 = 拉高
#define MOSI_1    (MOSI_GPIO->BSRR = MOSI_PIN)
#define MOSI_0    (MOSI_GPIO->BSRR = MOSI_PIN << 16)
#define CLK_HIGH  (CLK_GPIO->BSRR = CLK_PIN)
#define CLK_LOW   (CLK_GPIO->BSRR = CLK_PIN << 16)
#define MISO      ((MISO_GPIO->IDR & MISO_PIN) ? 1 : 0)
```

**发送命令 `sendCMD`**：
- XPT2046 命令字 8 位，`0x90`=测 X 通道，`0xD0`=测 Y 通道
- 逐位发送：先置 MOSI，再 CLK 高→低，在时钟低电平时数据有效
- 发送完命令后同一次片选内连续读 12 位 ADC 结果

**读取 ADC `receiveData`**：
- 先发 1 个空闲时钟（等芯片内部 AD 转换，约 6us）
- 读 12 位：CLK 高时采样 MISO，逐位移入 `usBuf`

**采样与滤波 `readAdcXY`**：
```
while(IRQ==低 && cnt<4):
    xyArray[0][cnt] = readADC_X()
    xyArray[1][cnt] = readADC_Y()
    cnt++
→ 去掉 x/y 的最大最小值，剩余求平均
→ 范围校验 (50~4050)，越界返回 0
```

**坐标换算 `adcXYToLcdXY`**：
```
lcdX = adcX * xfac + xoff
lcdY = adcY * yfac + yoff
```
`xfac/yfac/xoff/yoff` 是校准系数（本工程用固定默认值，裁剪了外部 Flash 校准存储）。

**防误触 `XPT2046_IsPressed`**：
- 读 IRQ 判断是否按下
- 连续采样两次，比较 LCD 坐标差，差值 > 10 像素视为抖动丢弃
- 两次 `readAdcXY` 任一次返回 0（范围校验失败）→ 视为未按下

#### 设计要点

- 用 BSRR 寄存器单次写置位/复位引脚，避免读改写
- IRQ 仅查询不中断，简化驱动；触摸由 LVGL indev 定时轮询
- 采样+去极值+平均的滤波策略抑制电阻屏噪声
- ADC 范围校验防止 MISO 断线返回 (0,0) 误触

---

### 10.3 光照传感器（BH1750 + 硬件 I2C1 直配寄存器）

#### 硬件连接

| STM32 引脚 | BH1750 信号 | 说明 |
|-----------|------------|------|
| PB6 | SCL | I2C1 时钟（开漏+上拉） |
| PB7 | SDA | I2C1 数据（开漏+上拉） |
| 3.3V | VCC | 供电 |
| GND | ADDR | 地址选择脚，接 GND → 7 位地址 0x23 |

BH1750 的 8 位地址（含 R/W 位）= `0x23 << 1 = 0x46`。

#### 驱动实现

**为什么直配寄存器**：本工程 HAL 是精简子集，不含 `stm32f4xx_hal_i2c.c`，因此直接操作 I2C1 寄存器，风格与 FSMC 一致。

**I2C1 初始化 `I2C1_HardwareInit`**：
```c
I2C1->CR2   = 42;     // FREQ = APB1 时钟 = 42MHz
I2C1->CCR   = 210;    // 100kHz: 42MHz / (2 * 100kHz) = 210
I2C1->TRISE = 43;     // 1000ns / (1/42MHz) + 1 ≈ 43
I2C1->CR1  |= PE | ACK;  // 使能外设 + 应答
```

**主机写 `I2C1_MasterWrite` 流程**：
```
等待 BUSY 清零 → 置 START → 等 SB → 写从机地址+W → 等 ADDR → 读 SR1/SR2 清 ADDR
→ 逐字节写 DR（等 TXE）→ 等 BTF → 置 STOP
```

**主机读 `I2C1_MasterRead` 流程**：
```
START → SB → 写从机地址+R → ADDR → 读 SR1/SR2
→ 多字节：ACK 应答读 N-1 个，最后一个 NACK + STOP
→ 单字节：先清 ACK 再读 SR1/SR2，置 STOP，等 RXNE 读 DR
```

**总线死锁恢复 `I2C1_BusRecovery`**：
```
关 I2C1(CR1&=~PE) → PB6/PB7 切 GPIO 开漏输出 → 释放 SDA/SCL 高
→ 循环最多 9 次: SCL 低→高, 检测 SDA 是否变高
→ 发 STOP(SCL 高时 SDA 低→高) → 重新 I2C1_HardwareInit
```
9 个脉冲足以让任何从机（8 位数据 + 1 位 ACK）释放被钳位的 SDA。

**所有超时点统一恢复**：BUSY/SB/ADDR/TXE/RXNE/BTF 共 6 个等待点，超时都调用 `I2C1_BusRecovery()`。

**BH1750 协议（连续高分辨率模式）**：
```
Init:  POWER_ON(0x01) → RESET(0x07) → CONT_H_MODE(0x10) → 等 120ms
Read:  直接读 2 字节 → lux = (DataH<<8 | DataL) / 1.2
```

**重试机制**：
- `BH1750_Init`：上电→复位→配置，整条链路重试 3 次
- `BH1750_Read`：读重试 3 次，全失败置 `s_bh1750_ready=0`，下次自动重新初始化

#### 设计要点

- I2C 是**电平敏感**总线，从机异常会永久钳位 SDA，必须有恢复机制
- 超时计数器用 `0xFFFF`（约 10ms），配合 168MHz 主频足够
- `s_bh1750_ready` 标志实现"失败自愈"：读取失败后下次强制重新初始化

---

### 10.4 温湿度传感器（DHT11 + 单总线 + DWT 微秒计时）

#### 硬件连接

| STM32 引脚 | DHT11 信号 | 说明 |
|-----------|-----------|------|
| PC1 | DATA | 单总线，开漏+外部上拉电阻 |
| 3.3V | VCC | 供电 |
| GND | GND | 地 |

单总线只有一根线，主机和从机时分复用。

#### 驱动实现

**微秒计时（DWT 周期计数器）**：
```c
// 使能 DWT
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

// us 延时
ticks = us * (SystemCoreClock / 1000000);  // 168MHz → 1us = 168 cycles
while ((DWT->CYCCNT - start) < ticks);

// 测经过时间
elapsed_us = (DWT->CYCCNT - start) / 168;
```
用 DWT 而非 SysTick：DWT 是 Cortex-M 内核调试计数器，不占用系统定时器，精度 1 个周期（~6ns）。

**DHT11 通信时序 `DHT11_Read`**：

```
【主机起始】  DATA 输出低 ≥18ms → 拉高 20~40us → 切输入
【从机响应】  DHT11 拉低 80us → 拉高 80us （响应信号）
【数据传输】  40 bit: 每 bit = 50us 低 + (26~28us 高=0 / 70us 高=1)
【校验】      字节4 = 字节0+1+2+3 低 8 位
```

代码关键判断：
```c
// 每 bit 起始: 先等 50us 低电平过去
t = DWT_GetCycles();
while (ReadPin() == LOW) { if (elapsed > 70) return TIMEOUT; }

// 测高电平时长: 26~28us = 0, 70us = 1
t = DWT_GetCycles();
while (ReadPin() == HIGH) { if (elapsed > 100) return TIMEOUT; }
buf[i] <<= 1;
if (elapsed > 40) buf[i] |= 1;   // >40us 判为 1
```

**数据解析**：
```
湿度 = buf[0] + buf[1]*0.1   (整数.小数)
温度 = buf[2] + buf[3]*0.1
```
DHT11 小数位通常为 0，精度 ±1°C / ±1%RH。

#### 设计要点

- **任务优先级提升**：SensorTask 读 DHT11 前临时提到 `osPriorityRealtime`，防止被其他任务抢占错过 us 级窗口；读 40 位约 5ms，代价可接受
- 超时保护：每个等待循环都有 us 级超时（70~100us），防止从机无响应时死循环
- 校验和验证：4 字节数据 + 1 字节校验，失败返回 `CHECKSUM_ERR`
- `osDelay(20)` 让出 CPU：起始信号拉低 18ms 期间用 `osDelay` 而非忙等，其他任务可运行（引脚锁存低电平，时序不受影响）

---

### 10.5 WiFi 模块（ESP8266 + USART3 + AT 指令）

#### 硬件连接

| STM32 引脚 | ESP8266 信号 | 说明 |
|-----------|-------------|------|
| PB10 (USART3_TX) | RXD | STM32 发 → ESP 收 |
| PB11 (USART3_RX) | TXD | ESP 发 → STM32 收 |
| 3.3V | VCC | 供电（ESP8266 峰值电流 300mA，需足够电源） |
| GND | GND | 地 |
| 跳线帽 | 排针 2-3 行 | 板子上用跳线帽把 ESP8266 的 TX/RX 接到 USART3 |

波特率 115200，8N1。ESP8266 出厂 AT 固件需支持 `AT+CIPSNTPCFG`（v1.6+）。

#### 驱动实现

**RX 架构：中断环形缓冲**
```
USART3 中断 → 逐字节写入 s_rx[head] → head++
WifiTask    → 从 s_rx[tail] 读出       → tail++
```
- ISR 只操作环形缓冲，**不调用任何 RTOS API**（ISR 优先级 6，可被更高优先级中断抢占但不影响）
- 缓冲 2048 字节，可容纳一次 `AT+CWLAP` 扫描的二十多个热点应答
- 满时丢弃新字节（不覆盖旧数据）

**AT 命令 `ESP8266_Cmd`**：
```
清空环形缓冲 → HAL_UART_Transmit 发命令 → 循环:
    拍快照到 s_snap → strstr 匹配 "OK" → 成功返回 1
    检测 "\r\nFAIL"/"\r\nERROR" → 失败返回 0
    检测 "busy p..."/"busy s..." → 忙，返回 0 让调用方重试
    osDelay(10) 让出 CPU
超时返回 0
```
关键：等待期间 `osDelay(10)` 而非忙等，CPU 利用率从"对时独占十几秒"降到"<1%"。

**SNTP 对时 `ESP8266_FetchTime`**：
```
发 "AT+CIPSNTPTIME?\r\n" → 等 "OK"
→ 找 "+CIPSNTPTIME:" → 解析 "Tue Oct 06 15:08:54 2026"
→ 月份英文缩写→数字，星期英文→数字
→ 返回 1970 年 = 未同步
→ 连 "OK" 都没收到 = 模块掉线(返回2, 需重连)
```

**SNTP 配置**：
```
AT+CIPSNTPCFG=1,8,"202.120.2.101","time.nist.gov","ntp.aliyun.com"
              ↑ ↑   └─ 3 个服务器依次尝试
              │ └─ 时区 8 (北京)
              └─ 1=使能
```

#### 设计要点

- **环形缓冲单生产者单消费者**：ISR 写、任务读，无需锁（head/tail 各只有一个写者）
- 应答快照模式：`ESP_RingCopy` 不改读写位置，任务拿到完整应答再解析
- 三级错误处理：明确 FAIL/ERROR 立即失败、busy 快速重试、超时兜底
- 大缓冲 `s_resp[2048]` 用 `static` 不放栈（WifiTask 栈仅 3KB）

---

### 10.6 Flash 凭据存储（STM32 内部 Flash Sector 11）

#### 硬件原理

STM32F407VET6 内置 512KB Flash，划分为 11 个扇区（0~11）。本工程：
- **程序区**：Sector 0~10，共 448KB（Keil IROM 裁剪到 `0x08000000~0x080DFFFF`）
- **凭据区**：Sector 11，128KB，起始 `0x080E0000`

```
0x08000000 ┌─────────────────────────┐
           │   Sector 0~10 (448KB)   │  ← 程序代码
0x080E0000 ├─────────────────────────┤
           │   Sector 11  (128KB)    │  ← WiFi 凭据
0x08100000 └─────────────────────────┘
```

#### 驱动实现

**数据结构 `WifiCred_t`**：
```c
typedef struct {
    uint32_t magic;       // 'WIFI' = 0x57494649，判定是否写过
    char     ssid[33];    // SSID
    char     pwd[65];     // 密码
    uint32_t checksum;    // 前面所有字节的 32 位加和
} WifiCred_t;
```

**保存 `WifiStore_Save`**：
```
计算 checksum → HAL_FLASH_Unlock
→ 擦除 Sector 11 (FLASH_TYPEERASE_SECTORS, FLASH_SECTOR_11)
→ 逐 32 位字编程 (HAL_FLASH_Program, FLASH_TYPEPROGRAM_WORD)
→ HAL_FLASH_Lock
→ memcmp 回读校验
```
- 擦除 128KB 扇区期间 **CPU 取指暂停约 1~2 秒**（所有任务冻结）
- 保存动作安排在 WiFi 连接成功后，界面显示 Saving，用户无感

**加载 `WifiStore_Load`**：
```
读 0x080E0000 处的 WifiCred_t → magic != 'WIFI' → 未写过(返回0)
→ checksum 不匹配 → 数据损坏(清零返回0)
→ 合法 → 拷贝到 out(返回1)
```

#### 设计要点

- **IROM 裁剪**：Keil 工程 IROM 设为 448KB，程序绝不会写到 Sector 11
- **双校验**：magic 判定是否写过 + checksum 判定数据完整性
- **字对齐编程**：STM32F4 Flash 按 32 位字编程，结构体长度向上取整到 4 字节
- **回读校验**：写完 `memcmp` 确认真的写入（防止供电不足等异常）

---

### 10.7 LED 与按键（GPIO + 队列解耦）

#### 硬件连接

| STM32 引脚 | 功能 | 电平逻辑 |
|-----------|------|---------|
| PC5 | 绿灯 LED | 低电平点亮（灌电流） |
| PB2 | 蓝灯 LED | 低电平点亮 |
| PA0 | KEY_1 | 内部下拉，按下=高（上升沿） |
| PA1 | KEY_2 | 内部上拉，按下=低（下降沿） |

#### 驱动实现

**按键扫描 `StartKeyTask`**（10ms 周期，积分式消抖）：
```
每 10ms:
    raw = 读引脚
    if raw != stable:
        if --cnt == 0:   // 连续 3 次(30ms)确认
            stable = raw
            cnt = 3
            if 稳定到按下沿:
                PowerMgr_Wakeup()    // 息屏唤醒或刷新计时
                pin = 读 LED 当前电平
                LED_PostMessage(颜色, pin==亮 ? Off : On)  // 翻转
    else:
        cnt = 3
```

**LED 控制 `StartLEDTask`**（队列消费者）：
```
osMessageQueueGet(LEDQueue, &msg, osWaitForever):
    switch(msg->color):
        Green: 写 PC5 = (On ? RESET : SET)
        Blue:  写 PB2 = (On ? RESET : SET)
    vPortFree(msg)
```

**三路控制统一**：
- 生产者：KeyTask（实体键）/ GuiTask（屏幕按钮）/ CommandTask（串口）
- 统一投递 `LEDMessage{color, state}` 到 LEDQueue
- 消费者：LEDTask 唯一执行点灯

**关键：读引脚为真相**
翻转前 `HAL_GPIO_ReadPin(led_x_GPIO_Port, led_x_Pin)` 读当前电平：
- RESET（亮）→ 目标 Off
- SET（灭）→ 目标 On
不维护软件状态变量，任何控制源改灯后其他源都能读到真实状态。

#### 设计要点

- 按键上下拉方向**相反**（PA0 下拉/PA1 上拉），CubeMX 默认浮空必须重配
- 消抖用积分计数而非延时，不阻塞任务
- LED 消息用 `pvPortMalloc` 分配指针，队列只存指针（深 16 也只占 16×4 字节）
- 队列满时发送失败立即 `vPortFree`，防内存泄漏

---

### 10.8 LVGL GUI（FSMC LCD + XPT2046 触摸）

#### 硬件

复用 10.1 LCD（FSMC）和 10.2 触摸（XPT2046），无额外硬件。

#### LVGL 移植要点

**显示驱动 `lv_port_disp.c`**：
- 双缓冲或单缓冲 + `LV_DISP_REFR_MODE_PARTIAL` 局部刷新
- `disp_flush` 回调里调 `LCD_DispFlush`（DMA 搬运像素到 FSMC）
- DMA 完成后 `lv_disp_flush_ready(drv)` 通知 LVGL

**输入驱动 `lv_port_indev.c`**：
- `touchpad_is_pressed` → `XPT2046_IsPressed()`
- `touchpad_get_xy` → `XPT2046_GetX()/GetY()`

**Tick 来源（免定时器）**：
```c
// lv_conf.h
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "lv_tick_port.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (xTaskGetTickCount())
```
LVGL 直接用 FreeRTOS 的 tick count 作为时基，不需要 TIM6 或 `lv_tick_inc`。

**中文字体**：
```c
#define LV_FONT_GB2312_16  1   // 启用 GB2312 二级汉字 16 号字
```
字体文件 `lv_font_gb2312_16.c` 由 `lv_font_conv` 从 `simhei.ttf` 生成（simsun.ttc 不支持）。

#### 界面组成（2026-10-06 重设计）

**主屏布局**（320×240，暗色主题 + 浅色卡片高对比）：

```
y=0    状态栏:WiFi 状态图标 + 网络时钟
y=28   ┌─────────────────────────────┐
       │ ▎24.5 C               TEMP  │  ← 大滚动框(可上下拖动)
       │ ▎65.0 %               HUMI  │     3 个传感器数据无独立小框
       │ ▎15205 lux            LIGHT │     仅左侧色条 + 数值 + 标签
       └─────────────────────────────┘
       ┌────────┐         ┌─────────┐
y=138  │ WiFi   │         │ [绿开关]│  ← 左下:WiFi 按钮(150×60)
       │ Setup  │         ├─────────┤     右下:LED 开关竖向排列
       │        │         │ [蓝开关]│     开关颜色区分绿/蓝,无文字
       └────────┘         └─────────┘
```

**配网页**：热点列表（按信号强度分阶边框颜色）+ 密码输入弹层（textarea + 键盘，从底部滑入）。

#### UI 设计原则

1. **高对比度配色**：TFT 屏亮度有限，浅色卡片(`#f1f5f9`) + 深色文字(`#0f172a`)，避免深底浅灰文字看不清
2. **去框化**：传感器数据统一放入一个大滚动框，每个数据项不再单独画框，仅保留左侧色条区分
3. **颜色即语义**：LED 开关用开态颜色（绿/蓝）直接区分，不额外标注文字
4. **紧凑布局**：320×240 小屏，控件尺寸精确到像素，避免重叠和截断
5. **动效克制**：仅保留屏幕切换淡入淡出、密码面板滑入；**禁用按钮缩放动效**（见踩坑）

#### 关键样式与动效

| 元素 | 实现方式 |
|------|---------|
| 背景渐变 | `lv_style_set_bg_grad_color` + `LV_GRAD_DIR_VER` |
| 卡片圆角 | `lv_style_set_radius` |
| 传感器色条 | 4px 宽矩形，颜色对应传感器类型 |
| 屏幕切换 | `lv_scr_load_anim(..., LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false)` |
| 密码面板滑入 | `lv_anim` + `translate_y` 240→0（ease_out, 280ms） |
| 按钮按下反馈 | `bg_opa = LV_OPA_60`（半透明变暗，**不用 transform_zoom**） |

#### 样式复用策略

所有样式（背景/卡片/按钮/文字/开关）用 `lv_style_t` 创建一次后通过 `lv_obj_add_style` 复用到多个控件，避免每个控件单独创建样式浪费 48KB 内存池。

---

### 10.9 系统任务架构总览

```
                    ┌─────────────────────┐
                    │   FreeRTOS 调度器    │
                    └─────────────────────┘
          ┌──────────┬──────────┬──────────┬──────────┐
          │          │          │          │          │
     KEYTask     LEDTask   CommandTask  GUITask   SensorTask
     (High)     (Normal)   (High1)    (Normal)   (Normal)
          │          ▲          │          │          │
          │  LEDQueue│          │ LEDQueue │          │
          └──────────┘          └──────────┘          │
                                                       │
                                              g_sensorData(共享)
                                                       │
                                                       ▼
                                                  GuiTask 读显

     WifiTask ──evtQ──► GuiTask
        ▲
     cmdQ
        │
     GuiTask

     StatsTask(Low) — 5s 打印栈水位 + CPU 占比
```

**任务优先级设计**：
- CommandTask (High1) > KeyTask (High) > SensorTask/WifiTask/GUITask (Normal) > StatsTask (Low)
- DHT11 读取期间 SensorTask 临时提至 Realtime
- 串口命令 > 按键 > 传感器/网络 > 统计，保证人机交互响应

**队列深度设计**：
- LEDQueue 深 16：三路控制源高峰也不丢
- CommandQueue 深 16：串口命令字节缓冲
- cmdQ 深 2：WiFi 命令串行，2 个足够
- evtQ 深 8：WiFi 事件可能连发（扫描完成+连接结果）

---

### 10.10 容错设计总结

| 故障场景 | 检测方式 | 恢复策略 |
|---------|---------|---------|
| I2C 总线死锁 | BUSY/SB/ADDR 超时 | 9 个 SCL 脉冲 + STOP + 重初始化 |
| BH1750 读取失败 | 重试 3 次 | 置 ready=0，下次自动重初始化 |
| DHT11 校验失败 | checksum 比对 | 返回错误，不覆盖上次有效值 |
| XPT2046 MISO 断线 | ADC 值 0 或 4095 | 范围校验丢弃，视为未按下 |
| XPT2046 触摸抖动 | 两次采样差 > 10px | 丢弃该次，视为未按下 |
| ESP8266 掉线 | AT 无响应 | 每 10s 自动重连保存的凭据 |
| SNTP 对时失败 | 返回 1970 年 | 继续轮询，30 分钟后重同步 |
| Flash 数据损坏 | magic + checksum | 视为未配置，走屏幕配网 |
| 任务栈溢出 | vApplicationStackOverflowHook | 串口打印任务名 + 停机 |
| 堆耗尽 | vApplicationMallocFailedHook | 串口打印 + 停机 |

---

### 10.11 技术栈与工程价值总结

**技术栈全景**：
- **MCU**：STM32F407VET6 @ 168MHz，Cortex-M4F
- **RTOS**：FreeRTOS + CMSIS-RTOS2，8 任务 + 4 队列
- **GUI**：LVGL v8.3，GB2312 中文字体，FSMC DMA 刷新
- **通信接口**：FSMC(LCD) / 硬件 I2C(BH1750) / 模拟 SPI(XPT2046) / 单总线(DHT11) / USART AT(ESP8266) / 内部 Flash(凭据)
- **诊断**：DWT 运行时统计 + 栈水位监控

**工程价值**：
1. **总线容错**：I2C 死锁自动恢复 + SPI 范围校验，传感器断线不死机
2. **多源控制统一**：按键/屏幕/串口三路 LED 控制，读引脚为真相不冲突
3. **RTOS 友好**：所有等待用 `osDelay`/信号量，不忙等，CPU 空闲率 88%+
4. **数据一致性**：时钟临界区保护，传感器失败不覆盖有效值
5. **可诊断**：5s 周期打印栈水位/CPU 占比，故障可定位

---

## 11. 通信协议对比：六种接口的使用方式与优劣

本项目用到了 **6 种**不同的通信/存储接口，每种都有其适用场景。本章逐一讲解使用方式并做横向对比。

### 11.1 接口总览

| 接口 | 外设 | 协议类型 | 速率 | 引脚数 | 本工程用途 |
|------|------|---------|------|--------|-----------|
| FSMC | ILI9341 | 并行 8080 总线 | ~60MB/s | 20+ | LCD 显示 |
| I2C | BH1750 | 串行同步半双工 | 100kHz | 2 (SCL+SDA) | 光照传感器 |
| SPI | XPT2046 | 串行同步全双工 | 软件模拟 ~1Mbps | 4 (CS/CLK/MOSI/MISO) | 触摸 |
| 1-Wire | DHT11 | 单总线异步 | ~10kbps | 1 (DATA) | 温湿度 |
| UART+AT | ESP8266 | 异步串口 | 115200 | 2 (TX+RX) | WiFi 联网 |
| 内部 Flash | Sector 11 | 存储映射 | 字编程 | 0 (片内) | 凭据掉电保存 |

---

### 11.2 FSMC 并行总线（LCD）

**使用方式**：STM32 的 FSMC 外设把 LCD 映射为外部 SRAM，CPU 写内存 = 写 LCD 寄存器/显存。

```c
// 地址映射：用 A6 地址线区分命令/数据
#define LCD ((LCD_TypeDef *)0x6001FFFE)
LCD->LCD_REG = 0x2A;    // 写命令（A6=0）
LCD->LCD_RAM = color;   // 写数据（A6=1）
```

**优势**：
- 速度极快，FSMC 硬件自动产生 CS/RD/WR/DC 时序，CPU 零开销
- 支持 DMA 批量搬运像素（本工程用 DMA2 Stream5）
- 16 位宽一次传一个 RGB565 像素

**劣势**：
- 引脚占用多（PD4~PD15 + PD7/PD11/PD5/PD12），挤占 GPIO 资源
- 布线复杂，需要等长/阻抗控制

**适用场景**：高速大数据量传输，如 LCD 显存刷新。

---

### 11.3 I2C（BH1750）

**使用方式**：硬件 I2C1，本工程直配寄存器（HAL 无 I2C 驱动）。

```c
// 主机写：START → 从机地址+W → 数据 → STOP
I2C1->CR1 |= START;
while(!(I2C1->SR1 & SB));
I2C1->DR = addr & 0xFE;   // 写地址
// ...
I2C1->CR1 |= STOP;
```

**优势**：
- 只需 2 根线（SCL+SDA），支持多从机挂载（地址区分）
- 开漏输出 + 上拉电阻，允许多设备线与
- 协议有 ACK 应答，可靠性高

**劣势**：
- 半双工，同一时刻只能一个方向
- 总线死锁风险（从机异常钳位 SDA），需恢复机制
- 速率较低（标准 100kHz，快速 400kHz）
- 时序敏感，调试困难（需要逻辑分析仪）

**本工程容错**：6 个超时点统一调用 `I2C1_BusRecovery()`（9 脉冲 + STOP + 重初始化）。

**适用场景**：低速传感器、板载芯片间通信、引脚紧张场景。

---

### 11.4 SPI（XPT2046 触摸）

**使用方式**：本工程用 **GPIO 模拟 SPI**（bit-bang），非硬件 SPI。

```c
// 逐位发送命令
for (i = 0; i < 8; i++) {
    (cmd >> (7-i)) & 1 ? MOSI_1 : MOSI_0;
    CLK_HIGH; CLK_LOW;
}
// 逐位接收 12 位 ADC
for (i = 0; i < 12; i++) {
    CLK_HIGH;
    usBuf = (usBuf << 1) | MISO;
    CLK_LOW;
}
```

**优势**：
- 全双工，可同时收发
- 速率高（硬件 SPI 可达几十 MHz）
- 协议简单，无地址/应答开销
- 每个从机独立 CS，可挂多个设备

**劣势**：
- 引脚数较多（CS+CLK+MOSI+MISO = 4 线/设备，多设备共享 CLK/MOSI/MISO）
- 无内置应答，需软件校验
- 软件模拟速率受限（受 CPU 主频和 `delayUS` 精度影响）

**本工程容错**：ADC 范围校验（50~4050），MISO 断线时丢弃采样。

**适用场景**：高速数据传输、ADC/DAC、Flash、显示屏（非并口时）。

**为什么用软件 SPI 而非硬件 SPI**：触摸采样率低（LVGL 轮询 ~30Hz），软件 SPI 足够；且硬件 SPI 引脚被 LCD 的 FSMC 占用/冲突，软件 SPI 灵活选脚。

---

### 11.5 1-Wire 单总线（DHT11）

**使用方式**：单根线时分复用，主机和从机轮流控制。

```c
// 主机起始：拉低 18ms → 释放 → 切输入
DHT11_PinOutput();
DHT11_WritePin(LOW);
osDelay(20);
DHT11_WritePin(HIGH);
DWT_DelayUs(30);
DHT11_PinInput();

// 从机响应：拉低 80us → 拉高 80us
// 数据：每 bit = 50us 低 + (26us 高=0 / 70us 高=1)
```

**优势**：
- **只用 1 根线**，极大节省 GPIO
- 协议简单，无需时钟线

**劣势**：
- **时序极其敏感**（us 级窗口 70~100us），被任务抢占就失败
- 速率极低（~10kbps），一次读取 5ms
- 单总线只能一个从机（DHT11 无地址）
- 需要外部上拉电阻

**本工程应对**：读 DHT11 期间临时提升任务优先级到 `osPriorityRealtime`，防止被抢占。

**适用场景**：极低速传感器、引脚极度紧张场景。

---

### 11.6 UART + AT 指令（ESP8266）

**使用方式**：USART3 异步串口收发 AT 指令字符串。

```c
// 发命令 + 等应答
ESP8266_Cmd("AT+CWJAP=\"ssid\",\"pwd\"\r\n", "OK", 20000);

// RX 走中断环形缓冲
USART3_IRQHandler → s_rx[head++] = DR;
```

**优势**：
- 异步通信，无需时钟线，抗干扰强
- AT 指令是文本协议，可读性好、调试方便
- 支持长距离传输（TTL 电平数米）
- 通用标准，几乎所有 WiFi/蓝牙模块都支持

**劣势**：
- 文本协议解析开销大（`strstr`/`sscanf`）
- 应答有延迟，需要超时管理
- 无硬件流控时可能丢字节（需环形缓冲）

**本工程优化**：
- RX 中断环形缓冲（2KB），ISR 不调 RTOS API
- 等待应答用 `osDelay(10)` 让出 CPU，CPU 占用 <1%
- 三级错误处理：明确 FAIL 立即失败、busy 快速重试、超时兜底

**适用场景**：模块通信（WiFi/蓝牙/GPS）、串口调试、低速数据传输。

---

### 11.7 内部 Flash 存储（WiFi 凭据）

**使用方式**：直接读写 STM32 内部 Flash 扇区。

```c
// 擦除 + 32 位字编程
HAL_FLASH_Unlock();
HAL_FLASHEx_Erase(&erase, &err);      // 擦 Sector 11
HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);  // 逐字写
HAL_FLASH_Lock();
```

**优势**：
- 掉电不丢失，无需外部 EEPROM/Flash
- 零额外引脚和成本
- 容量大（本工程用 128KB 存 100 字节凭据，绰绰有余）

**劣势**：
- 擦除会停 CPU（1~2s），影响实时性
- 擦写次数有限（~10000 次）
- 必须按扇区擦除，不能单字节修改
- 需与程序分区隔离（本工程 IROM 裁剪到 448KB）

**适用场景**：配置参数、校准数据、用户凭据等少量非易失存储。

---

### 11.8 接口选择决策总结

| 需求特征 | 推荐接口 | 原因 |
|---------|---------|------|
| 高速大量数据 | FSMC/并口 SPI | 带宽高、CPU 零开销 |
| 多设备、引脚少 | I2C | 2 线 + 地址多从机 |
| 单从机、中速 | SPI | 简单高效、可加校验 |
| 极低速、引脚极度紧张 | 1-Wire | 1 根线 |
| 模块通信/调试 | UART | 通用、文本可读 |
| 掉电保存配置 | 内部 Flash | 零成本、非易失 |

---

## 12. FreeRTOS 功能详解：本项目用到的原语与作用

本项目基于 **FreeRTOS + CMSIS-RTOS2 API**，共用到以下原语。每个原语都标注了"在本项目中解决了什么问题"。

### 12.1 任务（Task）

**作用**：FreeRTOS 的基本调度单元，每个任务有独立栈和优先级。

**本项目使用**：8 个任务

| 任务 | 优先级 | 栈大小 | 职责 |
|------|--------|--------|------|
| KEYTask | High | 512B | 按键扫描消抖 |
| LEDTask | Normal | 512B | LED 队列消费 |
| CommandTask | High1 | 512B | 串口命令解析 |
| GUITask | Normal | 2560B | LVGL 渲染 |
| SensorTask | Normal | 1024B | BH1750+DHT11 采集 |
| WifiTask | Normal | 3072B | ESP8266 + SNTP |
| StatsTask | Low | 1024B | 栈水位/CPU 统计 |
| IDLE/TmrSvc | 系统 | - | 空闲/软件定时器 |

**创建方式**：CubeMX 生成 KEYTask/LEDTask/CommandTask，其余在 `MX_FREERTOS_Init` 中 `osThreadNew` 创建。

**调度模型**：抢占式优先级调度，同优先级时间片轮转。

---

### 12.2 消息队列（Message Queue）

**作用**：任务间/中断到任务的异步数据传递，自带阻塞/超时机制。

**本项目使用**：4 条队列

| 队列 | 深度 | 元素 | 生产者 → 消费者 | 解决的问题 |
|------|------|------|----------------|-----------|
| LEDQueue | 16 | `LEDMessage*` | KeyTask/GuiTask/CommandTask → LEDTask | 三路 LED 控制统一出口，解耦控制源与执行 |
| CommandQueue | 16 | `uint8_t` | USART1 ISR → CommandTask | 中断到任务的字节流缓冲，ISR 快速返回 |
| cmdQ | 2 | `WifiCmd_t` | GuiTask → WifiTask | GUI 发 WiFi 命令（扫描/连接） |
| evtQ | 8 | `WifiEvent_t` | WifiTask → GuiTask | WiFi 结果通知 GUI（扫描完成/连接结果） |

**核心代码示例**：
```c
// 生产者：分配消息 → 投递
LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));
msg->color = LEDColor_Green;
msg->state = LEDState_On;
osMessageQueuePut(LEDQueueHandle, &msg, 0, 0);  // 非阻塞

// 消费者：阻塞等消息
LEDMessage *msg;
osMessageQueueGet(LEDQueueHandle, &msg, NULL, osWaitForever);
// 执行...
vPortFree(msg);
```

**队列满处理**：发送失败立即 `vPortFree`，防内存泄漏。

**为什么用指针而非值**：`LEDMessage` 结构体小，传值也可以，但传指针队列只占 4 字节/元素，深 16 仅 64 字节堆；且生产者/消费者生命周期清晰（生产者 alloc，消费者 free）。

---

### 12.3 二值信号量（Binary Semaphore）

**作用**：任务间/中断到任务的**事件通知**与**资源互斥**（二值信号量可当互斥锁用，但无优先级继承）。

**本项目使用**：LCD DMA 完成同步

```c
// 创建：初值 1（DMA 空闲）
lcdDmaDoneSem = osSemaphoreNew(1U, 1U, NULL);

// flush 前：获取信号量（DMA 忙则阻塞等）
osSemaphoreAcquire(lcdDmaDoneSem, osWaitForever);
// 设置 ILI9341 窗口 + 启动 DMA...

// DMA 完成中断：释放信号量
DMA2_Stream5_IRQHandler:
    osSemaphoreRelease(lcdDmaDoneSem);
    lcdFlushDoneCb();   // 通知 LVGL flush 完成
```

**解决的问题**：
- 防止两次 flush 冲突（窗口设置必须在 DMA 空闲时）
- DMA 传输期间 GuiTask 不阻塞，可处理其他逻辑
- 中断安全：ISR 中 `osSemaphoreRelease` 是 FreeRTOS 中断安全 API

**初值设为 1 的含义**：信号量=1 表示资源空闲，acquire 后变 0（占用），ISR release 后恢复 1。

---

### 12.4 临界区（Critical Section）

**作用**：短暂屏蔽中断，保护**极短**的共享数据访问，防止被中断打断。

**本项目使用**：2 处

**① 时钟数据保护**（`ClockData.h`）：
```c
static inline void ClockData_Set(const ClockData_t *src) {
    taskENTER_CRITICAL();
    memcpy(&g_clockData, src, sizeof(g_clockData));
    taskEXIT_CRITICAL();
}
```
时钟是多字段结构体，读时可能碰到整点走秒更新（年/月/日/时/分/秒同时变），用临界区保证"原子拷贝"整个结构体。

**② WiFi 扫描结果写入**（`WifiTask.c`）：
```c
taskENTER_CRITICAL();
g_wifiScan.count = s_topN;
memcpy(g_wifiScan.aps, s_top, sizeof(WifiAp_t) * s_topN);
taskEXIT_CRITICAL();
```

**为什么用临界区而非互斥锁**：
- 临界区只屏蔽中断几 us（`memcpy` 几十字节），不影响实时性
- 不需要创建互斥锁对象，节省 RAM
- 调用点固定（`ClockData_Set/Get` 是 inline 函数），不会有嵌套/死锁问题

**注意**：临界区内**绝对不能**调用阻塞 API（`osDelay`/`osMessageQueueGet` 等），否则会导致调度器异常。

---

### 12.5 任务优先级动态调整

**作用**：运行时改变任务优先级，临时获得 CPU 独占权。

**本项目使用**：DHT11 读取期间

```c
// 读 DHT11 前：提升到 Realtime（最高优先级）
osThreadSetPriority(osThreadGetId(), osPriorityRealtime);
stEnv = DHT11_Read(&env);
// 读完后：恢复 Normal
osThreadSetPriority(osThreadGetId(), osPriorityNormal);
```

**解决的问题**：
- DHT11 单总线时序窗口 70~100us，被任务抢占会导致 T:err
- `osPriorityRealtime` 高于所有其他任务，读取期间不会被任务切换打断
- **不影响中断**（UART/DMA），只是屏蔽任务级抢占

**注意**：提升期间整个读取约 5ms，其他任务短暂等待，代价可接受；读完立即恢复。

**这不是优先级继承**：优先级继承是互斥锁的特性，这里是**显式提优先级**，更简单直接。

---

### 12.6 任务延时（osDelay）

**作用**：任务主动让出 CPU 一段时间，实现周期执行或等待。

**本项目使用**：所有周期任务

| 任务 | 延时 | 作用 |
|------|------|------|
| KeyTask | 10ms | 按键扫描周期 |
| SensorTask | 2000ms | DHT11 最小采样间隔 1s |
| WifiTask | 1000ms(连)/200ms(未连) | 走秒/快速响应配网 |
| StatsTask | 5000ms | 统计输出周期 |
| ESP8266_Cmd | 10ms | 等待 AT 应答轮询 |

**为什么用 osDelay 而非忙等**：
```c
// 坏：忙等占满 CPU
while (HAL_GetTick() - start < timeout);

// 好：osDelay 让出 CPU，其他任务可运行
osDelay(10);
```
`osDelay` 把任务置为阻塞态，调度器运行其他就绪任务，CPU 空闲率 88%+。

---

### 12.7 FreeRTOS 钩子函数（Hooks）

**作用**：FreeRTOS 在特定事件发生时调用用户回调，用于诊断和容错。

**本项目使用**：4 个钩子

**① 栈溢出钩子**（`vApplicationStackOverflowHook`）：
```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    FatalUartSend("[FATAL] stack overflow in task: ");
    FatalUartSend(pcTaskName);
    taskDISABLE_INTERRUPTS();
    for(;;);   // 停机
}
```
触发：任务栈指针越界。处理：串口打印任务名后停机（继续跑会 HardFault）。

**② 堆分配失败钩子**（`vApplicationMallocFailedHook`）：
```c
void vApplicationMallocFailedHook(void) {
    FatalUartSend("[FATAL] pvPortMalloc failed, heap exhausted");
    taskDISABLE_INTERRUPTS();
    for(;;);
}
```
触发：`pvPortMalloc` 失败（`configTOTAL_HEAP_SIZE` 不足）。队列/任务都靠堆分配。

**③ Tick 钩子**（`vApplicationTickHook`）：
```c
void vApplicationTickHook(void) {
    uint32_t now = DWT->CYCCNT;
    s_runCycles += (uint32_t)(now - s_lastCycles);
    s_lastCycles = now;
}
```
每 1ms 在 SysTick 中断中调用，用于 DWT 64 位累加（解决 CYCCNT 25.56s 回绕）。

**④ 运行时统计钩子**（`vConfigureTimerForRunTimeStats` / `vGetRunTimeCounterValue`）：
- 配置 DWT 作为统计时钟
- 供 `vTaskGetRunTimeStats` 计算各任务 CPU 占比

**为什么用轮询直发而非 DMA**：钩子已关中断/在中断上下文，不能用互斥锁+DMA（会死锁），所以用 `HAL_UART_Transmit` 轮询直发固定字符串。

---

### 12.8 本项目未使用的 FreeRTOS 原语及原因

| 原语 | 未使用原因 |
|------|-----------|
| **互斥锁 (Mutex)** | 共享数据用临界区（短拷贝），无需优先级继承；LED 控制走队列，无共享资源竞争 |
| **任务挂起/恢复 (vTaskSuspend/Resume)** | 任务用 `osDelay` + 队列阻塞实现等待，无需挂起 |
| **事件组 (Event Group)** | WiFi 用双向队列（cmdQ/evtQ）通信，比事件组更适合传递结构化数据 |
| **任务通知 (Task Notification)** | 信号量已满足 DMA 同步需求；任务通知更省 RAM 但本项目不缺 RAM |
| **软件定时器 (Software Timer)** | 周期任务用 `osDelay` 实现，无需独立定时器任务 |
| **计数信号量 (Counting Semaphore)** | 无资源池/多实例计数场景 |

**设计哲学**：够用即可，不引入不必要的复杂度。队列+二值信号量+临界区覆盖了本项目 100% 的并发需求。

---

### 12.9 FreeRTOS 配置关键项

`FreeRTOSConfig.h` 中的关键配置：

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `configTOTAL_HEAP_SIZE` | 较大 | 8 任务 + 4 队列 + 信号量 |
| `configUSE_PREEMPTION` | 1 | 抢占式调度 |
| `configUSE_TIME_SLICING` | 1 | 同优先级时间片轮转 |
| `configUSE_IDLE_HOOK` | 1 | 启用空闲钩子 |
| `configUSE_TICK_HOOK` | 1 | 启用 tick 钩子（DWT 累加） |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 栈溢出检测方法 2（最严格） |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 启用堆失败钩子 |
| `configGENERATE_RUN_TIME_STATS` | 1 | 启用运行时统计（DWT） |
| `configUSE_TRACE_FACILITY` | 1 | 启用 `vTaskList`/`vTaskGetRunTimeStats` |
| `configMINIMAL_STACK_SIZE` | 128 | 空闲任务栈 |

---

*文档基于项目实际代码生成，所有参数与实现均与源码对齐。*
