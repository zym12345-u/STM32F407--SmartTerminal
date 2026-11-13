/* bsp_LogFile.c
 * 串口日志异步落盘实现。
 *
 * 为什么用队列而不是在 UART_Printf 里直接 f_write：
 *   f_write 走 FatFS → SDIO 轮询，单次可能几百微秒到几毫秒；
 *   UART_Printf 持有打印互斥锁，直接写卡会把所有打印调用方一起拖慢，
 *   高优先级任务（GUI/WiFi）也会被牵连。改成"投递 + 异步消费"后，
 *   打印方只付一次内存拷贝的代价。
 *
 * 为什么每 4 条才 f_sync 一次：
 *   每条都同步会把 TF 卡写成"每次一个扇区"，既慢又伤卡；
 *   定时批量同步的代价是掉电可能丢最后几条（学习项目可接受）。
 */
#include "bsp_LogFile.h"

#include "ff.h"
#include "cmsis_os2.h"
#include "usart.h"     /*UART_Printf，用于把日志回传到串口*/
#include <string.h>

#define LOG_ITEM_SIZE     64U    /*单条最大字节数，超长自动分片*/
#define LOG_QUEUE_LEN     16U    /*约 16×68B，共 1KB 左右*/
#define LOG_SYNC_EVERY    4U     /*每写 4 条同步一次*/
#define LOG_PATH          "0:/LOG.TXT"   /*FatFS 未开长文件名，必须 8.3 格式*/
#define LOG_OLD_PATH      "0:/LOG.OLD"   /*滚动后的上一份日志*/
#define LOG_MAX_BYTES     65536UL        /*单文件上限 64KB，加备份最多占 128KB*/

typedef struct
{
    uint16_t len;                    /*本条有效字节数*/
    char     data[LOG_ITEM_SIZE];    /*文本内容，不保证以 '\0' 结尾*/
} LogItem_t;

static osMessageQueueId_t s_queue;
static FIL      s_file;
static uint8_t  s_fileOpen;
static uint8_t  s_syncPending;
static uint32_t s_dropCount;
static volatile uint8_t s_dumpReq;      /*串口请求回传日志*/
static volatile uint8_t s_dumpActive;   /*回传进行中：暂停入队，防回环*/

/**
  * 功    能：创建日志队列并打开日志文件，定位到末尾准备追加
  * 参    数：无
  * 返 回 值：LOGFILE_OK / LOGFILE_ERR
  */
uint8_t LogFile_Init(void)
{
    FRESULT fr;

    if (s_fileOpen != 0U)
    {
        return LOGFILE_OK;
    }

    if (s_queue == NULL)
    {
        s_queue = osMessageQueueNew(LOG_QUEUE_LEN, sizeof(LogItem_t), NULL);
        if (s_queue == NULL)
        {
            return LOGFILE_ERR;
        }
    }

    /*FA_OPEN_ALWAYS：文件不存在就新建，存在就打开，便于多次上电追加*/
    fr = f_open(&s_file, LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        return LOGFILE_ERR;
    }

    /*定位到文件末尾，保证是追加而不是覆盖*/
    fr = f_lseek(&s_file, f_size(&s_file));
    if (fr != FR_OK)
    {
        (void)f_close(&s_file);
        return LOGFILE_ERR;
    }

    s_fileOpen = 1U;
    s_syncPending = 0U;
    return LOGFILE_OK;
}

/**
  * 功    能：投递日志（非阻塞），超长分片，队列满则丢弃
  * 参    数：data  文本首地址；len  字节数
  * 返 回 值：无
  */
void LogFile_Push(const char *data, uint16_t len)
{
    uint16_t off = 0U;

    if ((s_queue == NULL) || (s_fileOpen == 0U) || (data == NULL) || (len == 0U))
    {
        return;
    }

    /*回传期间不能入队：dump 的内容本身就是从日志文件读出来的，
     *再写回去会越滚越大，形成回环*/
    if (s_dumpActive != 0U)
    {
        return;
    }

    while (off < len)
    {
        LogItem_t item;
        uint16_t  chunk = (uint16_t)(len - off);

        if (chunk > LOG_ITEM_SIZE)
        {
            chunk = LOG_ITEM_SIZE;
        }

        item.len = chunk;
        (void)memcpy(item.data, &data[off], chunk);

        /*超时 0：队列满立刻返回，绝不反压业务任务*/
        if (osMessageQueuePut(s_queue, &item, 0U, 0U) != osOK)
        {
            s_dropCount++;
            return;
        }

        off = (uint16_t)(off + chunk);
    }
}

/**
  * 功    能：日志文件超限后滚动：当前文件改名为 LOG.OLD，新建空 LOG.TXT
  * 参    数：无
  * 返 回 值：无
  * 补    充：只保留一份备份，占用上限约为阈值的 2 倍；
  *            滚动失败时 s_fileOpen 保持 0，调用方应停止本轮写入
  */
static void LogFile_Rollover(void)
{
    UINT bw = 0U;
    const char *mark = "[LOG] --- rotated ---\r\n";

    (void)f_close(&s_file);
    s_fileOpen = 0U;
    s_syncPending = 0U;

    /*旧备份先删：f_rename 不覆盖已存在的目标文件*/
    (void)f_unlink(LOG_OLD_PATH);
    (void)f_rename(LOG_PATH, LOG_OLD_PATH);

    if (f_open(&s_file, LOG_PATH, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
        s_fileOpen = 1U;
        /*新文件首行留个标记，回看时能区分"从哪开始是新一段"*/
        (void)f_write(&s_file, mark, (UINT)strlen(mark), &bw);
        (void)f_sync(&s_file);
    }
}

/**
  * 功    能：把日志文件内容分块读到串口
  * 参    数：无
  * 返 回 值：无
  * 补    充：只在 SdCardTask 里调用；回传前先 f_sync，避免看到旧内容；
  *            回传期间置 s_dumpActive，LogFile_Push 直接丢弃，防止内容被写回去
  */
static void LogFile_Dump(void)
{
    FIL     f;
    UINT    br = 0U;
    FRESULT fr;
    uint32_t total = 0U;
    static char buf[128];   /*静态：SdCardTask 栈只有 2KB，别放大缓冲*/

    /*先把仍留在 FatFS 缓冲里的内容刷到卡上，否则回传的是旧快照*/
    (void)f_sync(&s_file);
    s_dumpActive = 1U;

    /*FatFS 开了 _FS_LOCK：同一个文件不能同时存在两个已打开对象。
     *写对象还开着时再 f_open 读会返回 FR_LOCKED(16)，
     *所以先关写对象，读完再重新打开并定位到末尾继续追加*/
    (void)f_close(&s_file);
    s_fileOpen = 0U;

    fr = f_open(&f, LOG_PATH, FA_READ);
    if (fr != FR_OK)
    {
        /*打开失败也要把写对象恢复回来，否则日志从此不再落盘*/
        if (f_open(&s_file, LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
        {
            (void)f_lseek(&s_file, f_size(&s_file));
            s_fileOpen = 1U;
        }
        s_dumpActive = 0U;
        UART_Printf("[LOG] dump open fail fr=%u\r\n", (unsigned)fr);
        return;
    }

    UART_Printf("[LOG] dump start size=%lu\r\n", (unsigned long)f_size(&f));

    for (;;)
    {
        fr = f_read(&f, buf, sizeof(buf) - 1U, &br);
        if ((fr != FR_OK) || (br == 0U))
        {
            break;
        }
        buf[br] = '\0';
        UART_Printf("%s", buf);
        total += br;
    }

    (void)f_close(&f);
    UART_Printf("\r\n[LOG] dump end %lu bytes\r\n", (unsigned long)total);

    /*恢复写对象：重新打开并定位到末尾，后续日志继续追加*/
    if (f_open(&s_file, LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK)
    {
        (void)f_lseek(&s_file, f_size(&s_file));
        s_fileOpen = 1U;
    }

    s_dumpActive = 0U;
}

/**
  * 功    能：请求回传日志（只置标志，实际读文件在 SdCardTask 里执行）
  * 参    数：无
  * 返 回 值：无
  */
void LogFile_RequestDump(void)
{
    if (s_fileOpen != 0U)
    {
        s_dumpReq = 1U;
    }
}

/**
  * 功    能：取一条日志写入文件，累计若干条后同步一次
  * 参    数：无
  * 返 回 值：无
  */
void LogFile_TaskRun(void)
{
    LogItem_t item;
    UINT bw = 0U;

    if ((s_fileOpen == 0U) || (s_queue == NULL))
    {
        return;
    }

    /*有回传请求就先处理：先把已写内容刷盘，再读出来打印*/
    if (s_dumpReq != 0U)
    {
        s_dumpReq = 0U;
        LogFile_Dump();
        return;
    }

    /*写之前看文件是否到上限：f_size 只读 FIL 字段，不访问卡，开销可忽略*/
    if (f_size(&s_file) >= (FSIZE_t)LOG_MAX_BYTES)
    {
        LogFile_Rollover();
        if (s_fileOpen == 0U)
        {
            return;   /*滚动失败，本轮丢弃，避免往已关闭对象写*/
        }
    }

    if (osMessageQueueGet(s_queue, &item, 0U, 0U) != osOK)
    {
        /*队列已空：立刻把还没落盘的写下去。
         *批量同步是为了少写盘，但空闲时不同步，掉电会丢最后几条——
         *而"最后几条"往往正是死机/复位前最关键的现场*/
        if (s_syncPending > 0U)
        {
            (void)f_sync(&s_file);
            s_syncPending = 0U;
        }
        return;
    }

    if ((f_write(&s_file, item.data, item.len, &bw) != FR_OK) || (bw != item.len))
    {
        return;
    }

    s_syncPending++;
    if (s_syncPending >= LOG_SYNC_EVERY)
    {
        (void)f_sync(&s_file);
        s_syncPending = 0U;
    }
}

/**
  * 功    能：取累计丢弃条数
  * 参    数：无
  * 返 回 值：丢弃条数
  */
uint32_t LogFile_GetDropCount(void)
{
    return s_dropCount;
}
