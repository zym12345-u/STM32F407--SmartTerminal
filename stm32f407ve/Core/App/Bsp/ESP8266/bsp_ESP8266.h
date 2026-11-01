/* bsp_ESP8266.h
 * ESP8266 AT 驱动接口：初始化、命令收发、SNTP 时间查询、应答快照读取。
 * 硬件：USART2 = PA2(TX)/PA3(RX)，AF7，115200 8N1，无硬件流控。
 * 注意：只允许 WifiTask 单消费者调用；USART2 由本驱动独占，stm32f4xx_it.c 不得再定义 USART2_IRQHandler。
 */
#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include <stdint.h>


/*SNTP 时间，已是北京时间（AT+CIPSNTPCFG 时区 8）
 * 写：ESP8266_FetchTime 全员；读：RTC 校时取 year~sec，显示/日志取 week
 * 解析失败时字段全 0，调用方先看返回值再取字段*/
typedef struct
{
    uint16_t year;     /* 年，1970~2106 */
    uint8_t  month;    /* 月，1~12 */
    uint8_t  day;      /* 日，1~31 */
    uint8_t  hour;     /* 时，0~23 */
    uint8_t  min;      /* 分，0~59 */
    uint8_t  sec;      /* 秒，0~59 */
    uint8_t  week;     /* 星期，1=周一 ~ 7=周日 */
} ESP8266_Time_t;


/* 初始化 ESP8266 的串口和环形缓冲，不探测模块是否在线。 */
void ESP8266_Init(void);

/* 发 AT 命令并等应答标志子串。返回 1 收到、0 超时或被模块拒绝。
 * 只给 WifiTask 用，等待期间会用 osDelay 让出 CPU。 */
uint8_t ESP8266_Cmd(const char *cmd, const char *expect, uint32_t timeoutMs);

/* 取网络时间（已换算成北京时间）。返回 1 成功、0 应答无效、2 连 OK 都没收到。 */
uint8_t ESP8266_FetchTime(ESP8266_Time_t *out);

/* 把最近的原始应答拷成字符串（调试用），不推进读指针，可以重复调。
 * 最多拷 max-1 字节，返回实际字符数。 */
uint16_t ESP8266_CopyResponse(char *out, uint16_t max);


#endif /* __BSP_ESP8266_H */
