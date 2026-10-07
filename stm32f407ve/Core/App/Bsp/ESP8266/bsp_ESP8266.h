/**
  ******************************************************************************
  * @file    bsp_ESP8266.h
  * @brief   板载 ESP8266 WiFi 模块 AT 指令驱动(FreeRTOS 原生实现)
  *
  *          硬件:魔女 STM32F407VET6 开发板,ESP8266 板载在板子右上角,
  *                用跳线帽把模块 TX/RX 接到排针第 2、3 行的 USART3:
  *                  PB10 = USART3_TX  -> ESP8266 RXD
  *                  PB11 = USART3_RX  <- ESP8266 TXD
  *                波特率 115200,8N1,出厂 AT 固件(需支持 AT+CIPSNTPCFG,
  *                网盘 17-3 例程目录附 AT v1.7.1 固件与烧录工具)。
  *
  *          设计:
  *            - TX:任务上下文阻塞发送(AT 命令很短,115200 下最长命令约 7ms)
  *            - RX:USART3 中断逐字节进环形缓冲,ISR 不调任何 RTOS API;
  *            - 等待应答用 osDelay 让出 CPU(不同于官方裸机例程的 delay_ms 忙等);
  *            - USART3_IRQHandler 直接读寄存器,自带独立中断入口,
  *              不依赖 usart.c 中单一的 HAL_UART_RxCpltCallback。
  ******************************************************************************
  */
#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include <stdint.h>


/* SNTP 同步到的网络日期时间(已是北京时间,AT+CIPSNTPCFG 时区=8) */
typedef struct
{
    uint16_t year;     /* 完整年份,如 2026 */
    uint8_t  month;    /* 1~12 */
    uint8_t  day;      /* 1~31 */
    uint8_t  hour;     /* 0~23 */
    uint8_t  min;      /* 0~59 */
    uint8_t  sec;      /* 0~59 */
    uint8_t  week;     /* 星期:1=周一 ... 7=周日 */
} ESP8266_Time_t;


/* 初始化 USART3(PB10/PB11)、GPIO、NVIC,清空接收环形缓冲并使能 RX 中断 */
void ESP8266_Init(void);

/**
  * @brief  发送一条 AT 命令并等待期望应答
  * @param  cmd        完整命令字符串(需自带 "\r\n"),如 "AT\r\n"
  * @param  expect     成功标志字符串,如 "OK"/"ready";NULL 表示只发送不等应答
  * @param  timeoutMs  超时(ms)。等待期间每 10ms 检查一次,osDelay 让出 CPU
  * @retval 1=收到 expect;0=超时,或模块先返回了 FAIL/ERROR
  */
uint8_t ESP8266_Cmd(const char *cmd, const char *expect, uint32_t timeoutMs);

/**
  * @brief  查询一次 SNTP 时间(AT+CIPSNTPTIME?)并解析
  * @param  out 解析成功时输出时间
  * @retval 1=成功拿到有效时间;0=模块有应答但时间无效(仍为 1970 未同步/格式错);
  *         2=连 "OK" 都没收到(模块掉线/复位,需要重新连 WiFi)
  */
uint8_t ESP8266_FetchTime(ESP8266_Time_t *out);

/**
  * @brief  把当前接收缓冲里的全部应答拍快照成线性字符串(不改读位置)
  * @param  out 调用方提供的缓冲;会自动补 '\0'
  * @param  max 缓冲容量
  * @retval 实际拷贝的字节数(不含补的结尾 0)
  * @note   供 WifiTask 解析 AT+CWLAP 的多行长应答;本驱动只有 WifiTask 一个
  *         消费者,快照缓冲由调用方(static)提供,不占任务栈
  */
uint16_t ESP8266_CopyResponse(char *out, uint16_t max);


#endif /* __BSP_ESP8266_H */
