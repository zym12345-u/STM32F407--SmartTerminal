/* OtaData.h
 * WiFi OTA 的跨任务共享状态：WifiTask 写，GuiTask 只读。
 * 为什么不用事件队列：下载进度是连续变化的值，GUI 用 150ms 定时器轮询比塞满队列更省资源。
 * 字段都是 volatile 单写单读；GUI 只按快照显示，不做“读一半写一半”的复合判断。
 */
#ifndef __OTA_DATA_H
#define __OTA_DATA_H

#include <stdint.h>

/*下载阶段 0~6；0 与 BSS 清零同值，上电即 IDLE*/
typedef enum
{
    OTA_ST_IDLE = 0,     /* 空闲，可发起下载 */
    OTA_ST_ERASE,        /* 正在擦除槽 B */
    OTA_ST_META,         /* 正在取 app.meta */
    OTA_ST_DOWNLOAD,     /* 正在收 app.bin 并写槽 B */
    OTA_ST_VERIFY,       /* 回读校验 + 写记录 */
    OTA_ST_DONE,         /* 下载完成，槽 B 已有可用镜像（尚未切换启动） */
    OTA_ST_FAIL          /* 失败，原因见 err */
} OtaState_t;

/*失败原因 0~11；GUI 按码翻译成提示文字，避免跨任务传字符串*/
typedef enum
{
    OTA_ERR_NONE = 0,
    OTA_ERR_WIFI,        /* WiFi 未连接，未开始下载 */
    OTA_ERR_FLASH,       /* W25Q 不在位或擦/写失败 */
    OTA_ERR_ERASE,       /* 擦槽 B 失败 */
    OTA_ERR_CONNECT,     /* TCP 连接失败 */
    OTA_ERR_HTTP,        /* HTTP 应答异常（非 200/头不全） */
    OTA_ERR_META,        /* app.meta 解析失败 */
    OTA_ERR_SIZE,        /* 大小越界或与 Content-Length 不符 */
    OTA_ERR_TIMEOUT,     /* 收数据 5s 无进展 */
    OTA_ERR_CRC,         /* 整包 CRC 或回读 CRC 不符 */
    OTA_ERR_RECORD,      /* 槽记录写入/回读校验失败 */
    OTA_ERR_QUEUE        /* WifiTask 命令队列不可用（堆分配失败等） */
} OtaErr_t;

typedef struct
{
    volatile uint8_t  state;     /* OtaState_t；WifiTask 写，GuiTask 读 */
    volatile uint8_t  err;       /* OtaErr_t，仅 FAIL 时有意义 */
    volatile uint8_t  percent;   /* 0~100，按阶段划分：擦除 0~20、元数据 20~25、下载 25~95、校验到 100 */
    volatile uint16_t newVer;    /* 目标版本 (主<<8|次)，来自 app.meta；0=未知 */
    volatile uint32_t got;       /* 已收到字节数 */
    volatile uint32_t total;     /* 镜像总字节数，来自 app.meta */
} OtaStatus_t;

extern OtaStatus_t g_otaStatus;

#endif /* __OTA_DATA_H */
