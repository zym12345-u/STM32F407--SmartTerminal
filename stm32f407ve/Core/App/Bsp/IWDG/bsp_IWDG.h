/**
  ******************************************************************************
  * @file    bsp_IWDG.h
  * @brief   独立看门狗(IWDG)驱动 + 任务心跳监控
  *
  *          用法:
  *            1. main 里调用 IWDG_Init() 启动看门狗(超时约 1s)
  *            2. 各任务主循环里调用 Watchdog_Heartbeat(WDT_TASK_xxx) 上报存活
  *            3. MonitorTask 周期检查所有任务心跳, 全部存活才喂狗
  *            4. 任一任务卡死超时 => 不喂狗 => IWDG 复位系统
  *
  *          注意: IWDG 由 LSI(约 32kHz) 驱动, 一旦启动无法关闭,
  *          必须持续喂狗否则系统复位。调试时可在 DEBUG 模式下暂停 IWDG。
  ******************************************************************************
  */
#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#include <stdint.h>

/* 任务 ID(用于心跳上报), 按位标志便于扩展 */
typedef enum {
    WDT_TASK_GUI      = 0,
    WDT_TASK_SENSOR   = 1,
    WDT_TASK_WIFI     = 2,
    WDT_TASK_MODBUS   = 3,
    WDT_TASK_KEY      = 4,
    WDT_TASK_LED      = 5,
    WDT_TASK_COMMAND  = 6,
    WDT_TASK_MONITOR  = 7,
    WDT_TASK_MAX      = 8
} WDT_TaskId_t;

/** 初始化并启动 IWDG(超时约 1000ms), 启动后无法关闭 */
void IWDG_Init(void);

/** 喂狗(刷新计数器), 仅 MonitorTask 调用 */
void IWDG_Refresh(void);

/** 任务上报心跳(在任务主循环里调用, 传入本任务 ID) */
void Watchdog_Heartbeat(WDT_TaskId_t taskId);

/** 检查所有任务心跳是否在超时窗口内有更新, 全部存活返回 1, 否则返回 0 */
uint8_t Watchdog_CheckAllAlive(void);

/** 获取最后一次心跳时间戳(用于诊断哪个任务卡死) */
uint32_t Watchdog_GetLastBeat(WDT_TaskId_t taskId);

#endif /* __BSP_IWDG_H */
