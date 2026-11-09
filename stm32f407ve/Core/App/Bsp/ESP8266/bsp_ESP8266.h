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


/**
  * 功    能：初始化 ESP8266 的串口和环形缓冲，不探测模块是否在线
  * 参    数：无
  * 返 回 值：无
  */
void ESP8266_Init(void);

/**
  * 功    能：发 AT 命令并等应答标志子串。返回 1 收到、0 超时或被模块拒绝
  * 参    数：cmd  完整命令字符串，需自带 "\r\n"，如 "AT\r\n"
  *            expect  成功标志，如 "OK"/"ready"，不允许 NULL
  *            timeoutMs  超时，单位 ms，每 10ms 检查一次
  * 返 回 值：1 = 收到 expect，0 = 超时或模块返回 FAIL/ERROR
  * 补    充：仅 WifiTask 调用，等待期间 osDelay 让出 CPU
  *            只给 WifiTask 用，等待期间会用 osDelay 让出 CPU
  */
uint8_t ESP8266_Cmd(const char *cmd, const char *expect, uint32_t timeoutMs);

/**
  * 功    能：取网络时间（已换算成北京时间）。返回 1 成功、0 应答无效、2 连 OK 都没收到
  * 参    数：out  解析成功时输出时间
  * 返 回 值：1 = 成功；0 = 有应答但时间无效；2 = 连 "OK" 都没收到（模块掉线）
  * 补    充：时区已设 8，输出即北京时间
  */
uint8_t ESP8266_FetchTime(ESP8266_Time_t *out);

/**
  * 功    能：把最近的原始应答拷成字符串（调试用），不推进读指针，可以重复调
  * 参    数：out  调用方缓冲，自动补 '\0'
  *            max  缓冲容量
  * 返 回 值：实际拷贝字节数，不含结尾 '\0'
  * 补    充：不推进读位置，可重复调用；仅 WifiTask 使用
  */
uint16_t ESP8266_CopyResponse(char *out, uint16_t max);


/*══════ TCP 数据通道（OTA 下载用）══════
 * 与上面「发命令等关键字」不同，下面这组是给「收二进制数据」用的：
 * 普通模式下模块把收到的字节包成 "+IPD,<长度>:<数据>" 混在串口流里，
 * ESP8266_TcpRead() 负责把这层壳剥掉，按字节原样交给调用方。
 * 只有 WifiTask 能调；一次连接内不要穿插 ESP8266_Cmd()，否则会把数据流清掉。*/

/**
  * 功    能：清空 +IPD 解析状态（连接/断开前后都要调，避免半个包串到下一次）
  * 参    数：无
  * 返 回 值：无
  */
void ESP8266_TcpRxReset(void);

/**
  * 功    能：建立一条 TCP 连接：先关旧连接 → 普通模式 → 单连接 → CIPSTART
  * 参    数：host  服务器域名或点分 IP，不允许 NULL
  *            port  端口，1~65535
  * 返 回 值：1 = 模块回 OK（已连上），0 = 参数非法/模式设置失败/连接失败
  * 补    充：仅 WifiTask 调用；CIPSTART 最长等 10s，期间内部补报心跳
  */
uint8_t ESP8266_TcpConnect(const char *host, uint16_t port);

/**
  * 功    能：用 AT+CIPSEND 发一段数据（HTTP 请求就是走这里）
  * 参    数：data  数据首地址，不允许 NULL；len  字节数，1~2048
  * 返 回 值：1 = 模块出现 '>' 且数据已写入串口，0 = 失败
  * 补    充：只负责发出去，不等 "SEND OK"——等它会顺手清掉可能已经先到的 +IPD
  */
uint8_t ESP8266_TcpSend(const uint8_t *data, uint16_t len);

/**
  * 功    能：发一段数据并等 "SEND OK"（连续发包用；只窥探不取走，不丢 +IPD）
  * 参    数：data  数据首地址，不允许 NULL；len  字节数，1~2048
  *            ackTimeoutMs  等 SEND OK 的超时，0 按 1000ms 处理
  * 返 回 值：1 = 模块回 SEND OK；0 = 没拿到 '>' 或没等到 SEND OK
  * 补    充：上一条 CIPSEND 未完成时模块回 busy p...，用本函数可避免背靠背发送失败
  */
uint8_t ESP8266_TcpSendWait(const uint8_t *data, uint16_t len, uint32_t ackTimeoutMs);

/**
  * 功    能：关掉 TCP 连接（没连接时模块回 ERROR，属正常）
  * 参    数：无
  * 返 回 值：无
  * 补    充：会清空接收环，调用后不要再指望读之前的数据
  */
void ESP8266_TcpClose(void);

/**
  * 功    能：从 +IPD 流里取最多 max 字节的纯数据，最多等 waitMs
  * 参    数：out  输出缓冲，不允许 NULL；max  最多取多少；waitMs  本次最多等多久
  * 返 回 值：实际取到的字节数，0~max（0 表示这段时间没有数据，不代表连接关闭）
  * 补    充：自动跳过 "SEND OK"、"CLOSED" 等文本；跨多次调用能记住半个 +IPD 包；
  *            调用期间每 500ms 补报一次 WifiTask 心跳
  */
uint16_t ESP8266_TcpRead(uint8_t *out, uint16_t max, uint32_t waitMs);


#endif /* __BSP_ESP8266_H */
