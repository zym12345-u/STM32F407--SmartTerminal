/* bsp_LogFile.h
 * 串口日志异步落盘：UART_Printf 投递 → 低优先级任务写 TF 卡。
 *
 * 设计原则：投递绝不阻塞业务任务。SD 未就绪、队列满时直接丢弃并计数，
 * 不能因为写卡慢把 GUI / WiFi / CAN 拖住。
 */
#ifndef __BSP_LOGFILE_H
#define __BSP_LOGFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LOGFILE_OK      0U
#define LOGFILE_ERR     1U

/**
  * 功    能：创建日志队列并打开/追加日志文件
  * 参    数：无
  * 返 回 值：LOGFILE_OK / LOGFILE_ERR
  * 补    充：必须在 SDCard_Init() 成功之后调用；可重复调用
  */
uint8_t LogFile_Init(void);

/**
  * 功    能：把一段文本投递到日志队列（非阻塞）
  * 参    数：data  文本首地址；len  字节数
  * 返 回 值：无
  * 补    充：超过单条容量会自动分片；队列满则丢弃剩余部分
  */
void LogFile_Push(const char *data, uint16_t len);

/**
  * 功    能：从队列取一条并写文件，由 SdCardTask 循环调用
  * 参    数：无
  * 返 回 值：无
  */
void LogFile_TaskRun(void);

/**
  * 功    能：请求把日志文件内容回传到串口（异步执行）
  * 参    数：无
  * 返 回 值：无
  * 补    充：只置标志，真正的读文件由 SdCardTask 在 LogFile_TaskRun 里做；
  *            回传期间会暂停日志入队，避免 dump 出的内容又被写回文件形成回环
  */
void LogFile_RequestDump(void);

/**
  * 功    能：取因队列满丢弃的日志条数
  * 参    数：无
  * 返 回 值：累计丢弃条数
  */
uint32_t LogFile_GetDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LOGFILE_H */
