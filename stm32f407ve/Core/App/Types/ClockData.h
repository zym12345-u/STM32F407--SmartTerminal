/* ClockData.h
 * 共享时钟快照：WifiTask 联网对时并写 g_clockData，GuiTask 每秒读来刷新状态栏。
 */
/*多字段结构体整体拷贝，用临界区避免撕裂读；Set 只允许 WifiTask 调用*/
#ifndef __CLOCK_DATA_H
#define __CLOCK_DATA_H

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


/*时间快照：year 为完整年份，其余字段范围见右；WifiTask 写，GuiTask 读*/
typedef struct
{
    uint16_t year;     /*完整年份，如 2026*/
    uint8_t  month;    /*1~12*/
    uint8_t  day;      /*1~31*/
    uint8_t  hour;     /*0~23*/
    uint8_t  min;      /*0~59*/
    uint8_t  sec;      /*0~59*/
    uint8_t  week;     /*1=周一 … 7=周日*/
    uint8_t  synced;   /*0=从未同步(显示 syncing...)，1=已拿到网络时间*/
} ClockData_t;


/*全局时钟数据，WifiTask 写，GuiTask 读*/
extern ClockData_t g_clockData;


/*WifiTask 发布整组时间，临界区只包一次 memcpy*/
/**
  * 函    数：ClockData_Set
  * 参    数：src  待发布的时间快照，字段范围见结构体
  * 返 回 值：无
  */
static inline void ClockData_Set(const ClockData_t *src)
{
    taskENTER_CRITICAL();
    memcpy((void *)&g_clockData, (const void *)src, sizeof(g_clockData));
    taskEXIT_CRITICAL();
}

/*GuiTask 读取整组时间，临界区只包一次 memcpy*/
/**
  * 函    数：ClockData_Get
  * 参    数：dst  输出缓冲，调用方提供
  * 返 回 值：无
  */
static inline void ClockData_Get(ClockData_t *dst)
{
    taskENTER_CRITICAL();
    memcpy((void *)dst, (const void *)&g_clockData, sizeof(g_clockData));
    taskEXIT_CRITICAL();
}


#endif /* __CLOCK_DATA_H */
