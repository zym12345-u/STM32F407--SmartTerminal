/* bsp_ESP8266.c
 * ESP8266 AT 驱动：USART2 中断收字节、环形缓冲、命令等待、SNTP 时间解析。
 * 硬件：USART2 = PA2(TX)/PA3(RX)，AF7，115200 8N1，无硬件流控。
 * 注意：只允许 WifiTask 单消费者调用；等待应答用 osDelay 让出 CPU，长命令期间需定期喂狗。
 *
 * ══════ 这一层在干什么（理解 AT 模式和环形缓冲）══════
 *
 * ── AT 指令模式：MCU 不直接控制 WiFi，而是「用文本命令指挥模块」──
 *   ESP8266 出厂自带固件，自己就能处理 WiFi 协议栈、TCP/IP。
 *   MCU 通过串口给它发【字符串命令】，它就照做并回文本结果：
 *      发 AT               → 回 OK                 （探测模块在不在）
 *      发 AT+CWLAP         → 回一串热点列表 + OK    （扫描）
 *      发 AT+CWJAP="x","y" → 回 WIFI CONNECTED / OK（连热点）
 *   好处：MCU 不用管复杂的 WiFi 协议栈，只当「下命令的老板」。
 *   代价：靠字符串判断成败很脆弱 —— 模块可能回 FAIL / ERROR / busy，
 *         所以 ESP8266_Cmd 既要等期望的关键字，也要把这几类失败一并识别成失败。
 *
 * ── 【注意】AT 命令是「一问一答」，但模块也会主动上报 ──
 *   除了应答你的命令，模块还会自己吐东西（比如 WIFI DISCONNECT、busy p...）。
 *   所以接收端不能假设「收到的一定是这次的应答」，必须按关键字去匹配。
 *
 * ── 环形缓冲：为什么不能收到就直接处理 ──
 *   串口是一个字节一个字节来的（115200 下约 87us 一个），而且不知道一句话有多长。
 *   中断里如果直接解析，既慢又容易被打断，且一条命令的应答可能横跨很多次中断。
 *   所以采用「中断只管存、任务负责读」的分工：
 *       中断：收一字节 → 放进 s_rx 环形缓冲 → head++
 *       任务：Tail 追 Head，把数据拷出来做一次完整解析
 *   环形缓冲的好处是不用搬移内存，头尾指针转圈就行。
 *   代价是要处理「满了怎么办」（这里直接丢新数据，见 ESP_RX_BUF_SIZE 的注释）。
 *   注意这两个指针都是【单写者】：head 只有中断写、tail 只有任务写，
 *   所以不需要关中断加锁 —— 这就是注释里「单生产者/单消费者」的意义。
 *
 * ── 为什么等待应答要用 osDelay 而不是死等 ──
 *   死等（while 轮询）会一直占着 CPU，同优先级任务全饿死，
 *   而且看门狗那边也得不到心跳。用 osDelay 让出 CPU，别的任务照常跑。
 *
 * ── 为什么长命令（如 CWJAP）期间要专门喂狗 ──
 *   连热点最长要 20 秒。如果这 20 秒里一次心跳都不报，
 *   MonitorTask 会判定 WifiTask 卡死 → 不喂狗 → 整机复位。
 *   所以 ESP8266_Cmd 内部每 500ms 补报一次心跳。
 */
#include "bsp_ESP8266.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "bsp_IWDG.h"
#include <stdio.h>
#include <string.h>


/*串口分配：USART1 调试 / USART2 ESP8266 / USART3 RS485
 * 句柄 huart_esp 避免与 usart.c 的 huart2 混淆*/
#define ESP_UART            USART2
#define ESP_UART_IRQN       USART2_IRQn
#define ESP_UART_PRIO       6U    /* ISR 只操作环形缓冲,不调 RTOS API,优先级 6 即可 */

/*AT+CWLAP 每条热点约 70~90 字节，2KB 可容 20 条左右，超出丢弃*/
#define ESP_RX_BUF_SIZE     2048U

static UART_HandleTypeDef huart_esp;

/* 单生产者(USART2 ISR)/单消费者(WifiTask)环形缓冲 */
static volatile uint16_t s_rxHead = 0U;   /* ISR 写位置 */
static volatile uint16_t s_rxTail = 0U;   /* 任务读位置 */
static uint8_t           s_rx[ESP_RX_BUF_SIZE];


/*内部函数*/
static void ESP_GPIO_UART_Init(void);
static void ESP_RingFlush(void);
static uint16_t ESP_RingCopy(uint8_t *out, uint16_t max);


/**
  * 功    能：ESP8266 的串口中断：把收到的字节塞进环形缓冲
  * 参    数：无
  * 返 回 值：无
  * 补    充：ISR 上下文，不调 RTOS API；stm32f4xx_it.c 不得再定义同名函数
  *            中断里不调 RTOS API；stm32f4xx_it.c 里不能再定义同名函数，否则链接冲突
  */
void USART2_IRQHandler(void)
{
    uint32_t isr = ESP_UART->SR;

    if ((isr & USART_SR_RXNE) != 0U)
    {
        uint8_t d = (uint8_t)(ESP_UART->DR & 0xFFU);
        uint16_t next = (uint16_t)((s_rxHead + 1U) % ESP_RX_BUF_SIZE);   /* 环形自增，2047 后回卷到 0 */
        /*满时丢新字节、保留已收应答：丢旧数据会让 strstr 永远匹配不上*/
        if (next != s_rxTail)
        {
            s_rx[s_rxHead] = d;
            s_rxHead = next;
        }
    }

    if ((isr & USART_SR_ORE) != 0U)
    {
        (void)ESP_UART->DR;            /* 读 SR 后读 DR 清除 ORE */
    }
}

/**
  * 功    能：配 ESP8266 用的 GPIO、时钟和 NVIC，全在本文件完成，不依赖 CubeMX。重复调用无害
  * 参    数：无
  * 返 回 值：无
  * 补    充：GPIO/时钟/NVIC 都在本文件配置，不依赖 CubeMX；重复调用幂等
  */
static void ESP_GPIO_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2=TX，PA3=RX，AF7 */
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;       /* RX 挂上拉,空闲时避免杂讯误触发 */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart_esp.Instance          = ESP_UART;
    huart_esp.Init.BaudRate     = 115200;                              /* 出厂默认 115200，改前先发 AT+UART_CUR */
    huart_esp.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_esp.Init.StopBits     = UART_STOPBITS_1;
    huart_esp.Init.Parity       = UART_PARITY_NONE;
    huart_esp.Init.Mode         = UART_MODE_TX_RX;
    huart_esp.Init.HwFlowCtl    = UART_HWCONTROL_NONE;                 /* 未接 RTS/CTS，只能靠 AT 层流控 */
    huart_esp.Init.OverSampling = UART_OVERSAMPLING_16;                /* 16 倍过采样，115200 下分频误差更小 */
    (void)HAL_UART_Init(&huart_esp);

    HAL_NVIC_SetPriority(ESP_UART_IRQN, ESP_UART_PRIO, 0U);
    HAL_NVIC_EnableIRQ(ESP_UART_IRQN);
    __HAL_UART_ENABLE_IT(&huart_esp, UART_IT_RXNE);
}

/**
  * 功    能：清空接收环形缓冲（读写指针归零），丢掉所有已收内容
  * 参    数：无
  * 返 回 值：无
  * 补    充：任务上下文调用；读写指针归零，丢弃全部已收内容
  */
static void ESP_RingFlush(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
}

/**
  * 功    能：把环形缓冲里已有数据拷出来，但【不推进读指针】——
  * 参    数：out  输出缓冲，容量 >= max
  *            max  最多拷贝字节数，通常取输出缓冲容量 - 1
  * 返 回 值：实际拷贝字节数，范围 0~max；不写 '\0'
  * 补    充：只拷贝不推进 s_rxTail，否则前半段应答找不回
  *            这样同一段应答可以被反复解析，也能兜住「半个应答先到、后半截后到」的情况
  */
static uint16_t ESP_RingCopy(uint8_t *out, uint16_t max)
{
    uint16_t n = 0U;
    uint16_t t = s_rxTail;
    uint16_t h = s_rxHead;

    while ((t != h) && (n < max))
    {
        out[n++] = s_rx[t];
        t = (uint16_t)((t + 1U) % ESP_RX_BUF_SIZE);     /* 读指针同样按 2KB 环形回卷 */
    }
    return n;
}


/*应答快照放 BSS：WifiTask 栈只有 3KB；命令串行，static 安全*/
static uint8_t s_snap[ESP_RX_BUF_SIZE];


/*对外接口*/
/**
  * 功    能：初始化 ESP8266 的串口和环形缓冲。只配本地，不探测模块在不在线
  * 参    数：无
  * 返 回 值：无
  * 补    充：调度器启动前或任务上下文调用一次；不探测模块在线
  *            调度器启动前或任务里调一次
  */
void ESP8266_Init(void)
{
    ESP_GPIO_UART_Init();
    ESP_RingFlush();
}

/**
  * 功    能：发一条 AT 命令，等应答里出现 expect 子串（如 "OK"）
  * 参    数：cmd  完整命令串，必须自带 "\r\n"，不允许 NULL
  *            expect  成功标志子串，如 "OK"，不允许 NULL
  *            timeoutMs  最长等待，单位 ms，常用 1000~20000，CWJAP 需 20s
  * 返 回 值：1 = 已匹配 expect，0 = 超时、参数非法或模块返回 FAIL/ERROR/busy
  * 补    充：仅 WifiTask 调用；等待期间每 500ms 上报一次 WDT_TASK_WIFI 心跳
  *            cmd 要自带 \r\n；CWJAP 这类慢命令 timeoutMs 得给到 20s
  *            等待期间每 500ms 报一次 WIFI 任务心跳，否则会被看门狗判成卡死
  */
uint8_t ESP8266_Cmd(const char *cmd, const char *expect, uint32_t timeoutMs)
{
    uint32_t startTick;
    uint32_t lastBeat;                  /* 心跳节流基准 */

    if ((cmd == NULL) || (expect == NULL))
    {
        return 0U;
    }

    /*发命令前清空旧应答*/
    ESP_RingFlush();
    (void)HAL_UART_Transmit(&huart_esp, (uint8_t *)cmd, (uint16_t)strlen(cmd), 2000U);   /* 超时 2s；几十字节 @115200 实际 <10ms */

    /*入口先报一次心跳：否则模块秒回时整次调用零心跳，SNTP 轮询会饿死看门狗*/
    Watchdog_Heartbeat(WDT_TASK_WIFI);

    startTick = HAL_GetTick();
    lastBeat  = startTick;
    do
    {
        uint16_t n = ESP_RingCopy(s_snap, (uint16_t)(sizeof(s_snap) - 1U));
        s_snap[n] = '\0';

        if (strstr((char *)s_snap, expect) != NULL)
        {
            return 1U;
        }
        /*模块明确报错时不必等到超时*/
        if ((strstr((char *)s_snap, "\r\nFAIL") != NULL) ||
            (strstr((char *)s_snap, "\r\nERROR") != NULL))
        {
            return 0U;
        }
        /*模块正忙立即返回，精确匹配 token 避免与热点名冲突*/
        if ((strstr((char *)s_snap, "busy p...") != NULL) ||
            (strstr((char *)s_snap, "busy s...") != NULL))
        {
            return 0U;
        }
        /*10ms 轮询：再快无意义，再慢会拖长响应，同时让出 CPU*/
        osDelay(10);
        /*AT 等待最长 20s，心跳停发超 3s 会被判卡死，每 500ms 补报一次*/
        if ((uint32_t)(HAL_GetTick() - lastBeat) >= 500U)
        {
            Watchdog_Heartbeat(WDT_TASK_WIFI);
            lastBeat = HAL_GetTick();
        }
    } while ((uint32_t)(HAL_GetTick() - startTick) < timeoutMs);

    return 0U;
}

/**
  * 功    能：解析最近一次 CIPSNTPTIME 的应答，取网络时间。已经按东八区换算好
  * 参    数：out  输出时间，不允许 NULL，进入时先整体清 0
  * 返 回 值：1 = 解析成功；0 = 有应答但无效（未同步或格式不符）；2 = 连 "OK" 都没收到
  * 补    充：仅 WifiTask 调用；读 static 快照，不推进环形读指针；时区 8，已是北京时间
  */
uint8_t ESP8266_FetchTime(ESP8266_Time_t *out)
{
    char     weekStr[4]  = {0};
    char     monthStr[4] = {0};
    char    *p;
    int      year, month, day, hour, min, sec, week, parsed;
    uint16_t n;

    if (out == NULL)
    {
        return 0U;
    }
    memset(out, 0, sizeof(*out));

    /*连 OK 都没有：模块掉线或复位*/
    if (ESP8266_Cmd("AT+CIPSNTPTIME?\r\n", "OK", 2000U) == 0U)
    {
        return 2U;
    }

    n = ESP8266_CopyResponse((char *)s_snap, (uint16_t)sizeof(s_snap));
    (void)n;

    p = strstr((char *)s_snap, "+CIPSNTPTIME:");
    if (p == NULL)
    {
        return 0U;
    }
    p += strlen("+CIPSNTPTIME:");

    /*未同步时模块返回 1970 年默认值，视为无效*/
    if (strstr(p, "1970") != NULL)
    {
        return 0U;
    }

    /*形如：Tue Oct 06 11:30:00 2026*/
    parsed = sscanf(p, "%3s %3s %d %d:%d:%d %d",
                    weekStr, monthStr, &day, &hour, &min, &sec, &year);
    if (parsed != 7)                    /* 星期/月/日/时/分/秒/年 共 7 个字段，少一个即格式异常 */
    {
        return 0U;
    }

    if      (strcmp(monthStr, "Jan") == 0) month = 1;
    else if (strcmp(monthStr, "Feb") == 0) month = 2;
    else if (strcmp(monthStr, "Mar") == 0) month = 3;
    else if (strcmp(monthStr, "Apr") == 0) month = 4;
    else if (strcmp(monthStr, "May") == 0) month = 5;
    else if (strcmp(monthStr, "Jun") == 0) month = 6;
    else if (strcmp(monthStr, "Jul") == 0) month = 7;
    else if (strcmp(monthStr, "Aug") == 0) month = 8;
    else if (strcmp(monthStr, "Sep") == 0) month = 9;
    else if (strcmp(monthStr, "Oct") == 0) month = 10;
    else if (strcmp(monthStr, "Nov") == 0) month = 11;
    else if (strcmp(monthStr, "Dec") == 0) month = 12;
    else return 0U;

    if      (strcmp(weekStr, "Mon") == 0) week = 1;
    else if (strcmp(weekStr, "Tue") == 0) week = 2;
    else if (strcmp(weekStr, "Wed") == 0) week = 3;
    else if (strcmp(weekStr, "Thu") == 0) week = 4;
    else if (strcmp(weekStr, "Fri") == 0) week = 5;
    else if (strcmp(weekStr, "Sat") == 0) week = 6;
    else if (strcmp(weekStr, "Sun") == 0) week = 7;
    else return 0U;

    out->year  = (uint16_t)year;
    out->month = (uint8_t)month;
    out->day   = (uint8_t)day;
    out->hour  = (uint8_t)hour;
    out->min   = (uint8_t)min;
    out->sec   = (uint8_t)sec;
    out->week  = (uint8_t)week;
    return 1U;
}

/**
  * 功    能：把最近的原始应答拷成字符串（调试看用），不推进读指针，可以重复调
  * 参    数：out  调用方缓冲，不允许 NULL
  *            max  缓冲容量，单位 byte，必须 > 0
  * 返 回 值：实际拷贝字符数，不含结尾 '\0'；参数非法返回 0
  * 补    充：不推进 s_rxTail，可重复调用；最多拷 max-1 字节
  *            最多拷 max-1 字节
  */
uint16_t ESP8266_CopyResponse(char *out, uint16_t max)
{
    uint16_t n;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }
    n = ESP_RingCopy((uint8_t *)out, (uint16_t)(max - 1U));
    out[n] = '\0';
    return n;
}
