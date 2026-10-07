/**
  ******************************************************************************
  * @file    ClockData.h
  * @brief   WiFi 对时任务与显示任务之间的共享时钟数据
  *
  *          架构:
  *            WifiTask 负责连 WiFi、SNTP 首次对时、本地走秒与周期重同步,
  *            结果写入 g_clockData;GuiTask 的 LVGL 定时器每秒读出刷新状态栏。
  *
  *          数据一致性:
  *            时钟是多字段结构体,读取期间可能恰好碰到整点走秒更新,
  *            用 FreeRTOS 临界区把"发布/读取整个结构体"包起来。
  *            临界区只保护几条内存拷贝(几 us),不影响实时性。
  ******************************************************************************
  */
#ifndef __CLOCK_DATA_H
#define __CLOCK_DATA_H

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


typedef struct
{
    uint16_t year;     /* 完整年份,如 2026 */
    uint8_t  month;    /* 1~12 */
    uint8_t  day;      /* 1~31 */
    uint8_t  hour;     /* 0~23 */
    uint8_t  min;      /* 0~59 */
    uint8_t  sec;      /* 0~59 */
    uint8_t  week;     /* 1=周一 ... 7=周日 */
    uint8_t  synced;   /* 0=从未同步(显示 syncing...),1=已拿到网络时间 */
} ClockData_t;


/* 全局共享时钟数据,由 WifiTask 写入,GuiTask 读取 */
extern ClockData_t g_clockData;


/* WifiTask 发布整组时间(临界区内拷贝) */
static inline void ClockData_Set(const ClockData_t *src)
{
    taskENTER_CRITICAL();
    memcpy((void *)&g_clockData, (const void *)src, sizeof(g_clockData));
    taskEXIT_CRITICAL();
}

/* GuiTask 读取整组时间(临界区内拷贝) */
static inline void ClockData_Get(ClockData_t *dst)
{
    taskENTER_CRITICAL();
    memcpy((void *)dst, (const void *)&g_clockData, sizeof(g_clockData));
    taskEXIT_CRITICAL();
}


#endif /* __CLOCK_DATA_H */
