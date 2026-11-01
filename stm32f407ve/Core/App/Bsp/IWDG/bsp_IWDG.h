/* bsp_IWDG.h
 * IWDG 启动/喂狗/改窗口与任务心跳接口声明。
 * 硬件：IWDG 由内部 LSI ≈32kHz 驱动，启动后无法关闭。
 * 注意：喂狗只允许 MonitorTask；STORAGE 槽位 boot 后无常驻上报者，判死必须跳过。
 */
#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#include <stdint.h>

/*任务 ID(心跳上报用)*/
typedef enum {
    WDT_TASK_GUI      = 0,
    WDT_TASK_SENSOR   = 1,
    WDT_TASK_WIFI     = 2,
    WDT_TASK_MODBUS   = 3,
    WDT_TASK_KEY      = 4,
    WDT_TASK_LED      = 5,
    WDT_TASK_COMMAND  = 6,
    WDT_TASK_MONITOR  = 7,
    WDT_TASK_STORAGE  = 8,   /* W25Q128/LittleFS 存储槽位，仅 boot 自检时上报 */
    WDT_TASK_CAN      = 9,   /* CAN 通信任务槽，参与判死 */
    WDT_TASK_MAX      = 10   /* 槽位总数，s_lastBeat 按此长度分配，增删任务要同步改 */
} WDT_TaskId_t;

/**
  * 函    数：IWDG_Init
  * 参    数：无
  * 返 回 值：无
  */
void IWDG_Init(void);

/**
  * 函    数：IWDG_Refresh
  * 参    数：无
  * 返 回 值：无
  */
void IWDG_Refresh(void);

/* 重装载档位: T = 256 * RLR / 32000(LSI) */
#define IWDG_RELOAD_NORMAL      125U    /* ≈1s, 日常窗口 */
#define IWDG_RELOAD_FLASH_SAVE  500U    /* ≈4s, 内部 Flash 擦除期间临时窗口(>max 2s) */

/**
  * 函    数：IWDG_SetReload
  * 参    数：rlr  重装载值 1~4095；T ≈ 256*rlr/32000 s
  * 返 回 值：无
  */
void IWDG_SetReload(uint16_t rlr);

/**
  * 函    数：Watchdog_Heartbeat
  * 参    数：taskId  本任务 ID，越界忽略
  * 返 回 值：无
  */
void Watchdog_Heartbeat(WDT_TaskId_t taskId);

/**
  * 函    数：Watchdog_CheckAllAlive
  * 参    数：无
  * 返 回 值：1=全部存活，0=有任务卡死(调用方不得喂狗)
  */
uint8_t Watchdog_CheckAllAlive(void);

/**
  * 函    数：Watchdog_GetLastBeat
  * 参    数：taskId  任务 ID，越界返回 0
  * 返 回 值：最后心跳 tick(1kHz，单位 ms)
  */
uint32_t Watchdog_GetLastBeat(WDT_TaskId_t taskId);

#endif /* __BSP_IWDG_H */
