/**
  ******************************************************************************
  * @file    bsp_ESP8266.c
  * @brief   板载 ESP8266 WiFi 模块 AT 指令驱动 —— 实现文件
  *
  *          对照网盘 17-3 例程重写为 FreeRTOS 友好版本:
  *            官方 bsp_ESP8266 的 delay_ms 是 CPU 空转忙等、UART 阻塞收,
  *            对时一个流程要独占 CPU 十几秒;本驱动接收走中断环形缓冲,
  *            命令等待 osDelay 让出 CPU,任务栈上生成快照后 strstr/sscanf。
  ******************************************************************************
  */
#include "bsp_ESP8266.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>


/* ================================ 串口硬件 ================================ */
#define ESP_UART            USART3
#define ESP_UART_IRQN       USART3_IRQn
#define ESP_UART_PRIO       6U    /* ISR 只操作环形缓冲,不调 RTOS API,优先级 6 即可 */

/* AT+CWLAP 扫描会返回十几个热点、每条约 70~90 字节,2KB 可容纳 20 条左右;
 * 超出部分丢弃,UI 只取信号最强的前 WIFI_AP_MAX 个,截断可接受 */
#define ESP_RX_BUF_SIZE     2048U

static UART_HandleTypeDef huart3;

/* 单生产者(USART3 ISR)/单消费者(WifiTask)环形缓冲 */
static volatile uint16_t s_rxHead = 0U;   /* ISR 写位置 */
static volatile uint16_t s_rxTail = 0U;   /* 任务读位置 */
static uint8_t           s_rx[ESP_RX_BUF_SIZE];


/* ================================ 内部函数 ================================ */
static void ESP_GPIO_UART_Init(void);
static void ESP_RingFlush(void);
static uint16_t ESP_RingCopy(uint8_t *out, uint16_t max);


/**
  * @brief  USART3 中断入口(全局函数,启动文件向量表直接指向这里;
  *         stm32f4xx_it.c 未定义 USART3_IRQHandler,无重复定义冲突)。
  *         先读 SR 再读 DR 可同时清除 RXNE 与 ORE(过载)标志。
  */
void USART3_IRQHandler(void)
{
    uint32_t isr = ESP_UART->SR;

    if ((isr & USART_SR_RXNE) != 0U)
    {
        uint8_t d = (uint8_t)(ESP_UART->DR & 0xFFU);
        uint16_t next = (uint16_t)((s_rxHead + 1U) % ESP_RX_BUF_SIZE);
        if (next != s_rxTail)          /* 缓冲满则丢弃最旧?这里直接丢弃新字节 */
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
  * @brief  USART3 + PB10/PB11 初始化(自包含,GPIO/时钟/NVIC 都在本文件配置,
  *         不依赖 CubeMX;HAL_UART_MspInit 对 USART3 无处理,不冲突)
  */
static void ESP_GPIO_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* PB10=USART3_TX, PB11=USART3_RX, AF7 */
    gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;       /* RX 挂上拉,空闲时避免杂讯误触发 */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    huart3.Instance          = ESP_UART;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&huart3);

    HAL_NVIC_SetPriority(ESP_UART_IRQN, ESP_UART_PRIO, 0U);
    HAL_NVIC_EnableIRQ(ESP_UART_IRQN);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

static void ESP_RingFlush(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
}

/* 把环形缓冲当前内容拍快照成线性字符串(不改读写位置),返回长度 */
static uint16_t ESP_RingCopy(uint8_t *out, uint16_t max)
{
    uint16_t n = 0U;
    uint16_t t = s_rxTail;
    uint16_t h = s_rxHead;

    while ((t != h) && (n < max))
    {
        out[n++] = s_rx[t];
        t = (uint16_t)((t + 1U) % ESP_RX_BUF_SIZE);
    }
    return n;
}


/* 应答快照缓冲(BSS,不放任务栈:WifiTask 栈只有 3KB)。
 * 本驱动只有 WifiTask 一个调用者,命令是"发一条等一条"的串行流程,static 安全 */
static uint8_t s_snap[ESP_RX_BUF_SIZE];


/* ================================ 对外接口 ================================ */
void ESP8266_Init(void)
{
    ESP_GPIO_UART_Init();
    ESP_RingFlush();
}

uint8_t ESP8266_Cmd(const char *cmd, const char *expect, uint32_t timeoutMs)
{
    uint32_t startTick;

    if ((cmd == NULL) || (expect == NULL))
    {
        return 0U;
    }

    /* 发命令前清空旧应答,保证只匹配本次响应 */
    ESP_RingFlush();
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)cmd, (uint16_t)strlen(cmd), 2000U);

    startTick = HAL_GetTick();
    do
    {
        uint16_t n = ESP_RingCopy(s_snap, (uint16_t)(sizeof(s_snap) - 1U));
        s_snap[n] = '\0';

        if (strstr((char *)s_snap, expect) != NULL)
        {
            return 1U;
        }
        /* 模块明确报错时不必等到超时(注意排除命令回显里不可能含 FAIL/ERROR) */
        if ((strstr((char *)s_snap, "\r\nFAIL") != NULL) ||
            (strstr((char *)s_snap, "\r\nERROR") != NULL))
        {
            return 0U;
        }
        /* 模块正忙(上一条命令未结束/正在扫描):快速失败让调用方重试,
         * 不耗满超时;匹配精确 token,避免与热点名冲突 */
        if ((strstr((char *)s_snap, "busy p...") != NULL) ||
            (strstr((char *)s_snap, "busy s...") != NULL))
        {
            return 0U;
        }
        osDelay(10);
    } while ((uint32_t)(HAL_GetTick() - startTick) < timeoutMs);

    return 0U;
}

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

    /* 连 OK 都没有:模块掉线/复位/串口不通,调用方应重新连 WiFi */
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

    /* 未同步时模块返回 1970 年默认值,视为无效 */
    if (strstr(p, "1970") != NULL)
    {
        return 0U;
    }

    /* 形如: Tue Oct 06 11:30:00 2026 */
    parsed = sscanf(p, "%3s %3s %d %d:%d:%d %d",
                    weekStr, monthStr, &day, &hour, &min, &sec, &year);
    if (parsed != 7)
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
