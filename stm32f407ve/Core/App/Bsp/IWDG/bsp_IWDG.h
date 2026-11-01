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

/* 启动独立看门狗，超时约 1s。一旦启动就关不掉，只能在系统已经能正常喂狗之后调。 */
void IWDG_Init(void);

/* 喂一次狗，把计数重装。喂晚了就复位，所以只允许 MonitorTask 在确认所有任务存活后调。 */
void IWDG_Refresh(void);

/* 重装载档位: T = 256 * RLR / 32000(LSI) */
#define IWDG_RELOAD_NORMAL      125U    /* ≈1s, 日常窗口 */
#define IWDG_RELOAD_FLASH_SAVE  500U    /* ≈4s, 内部 Flash 擦除期间临时窗口(>max 2s) */

/* 改看门狗的超时窗口，T ≈ 256 × rlr / 32000 秒（LSI 约 32kHz）。
 * 擦 Flash 这类长耗时的操作，靠调大窗口而不是靠中途喂狗来覆盖。 */
void IWDG_SetReload(uint16_t rlr);

/* 登记「我还活着」：记下当前 tick。各任务在自己主循环里调用，被卡住就没人替你报。
 * taskId 越界直接忽略，不会写坏数组。 */
void Watchdog_Heartbeat(WDT_TaskId_t taskId);

/* 检查所有任务的心跳是否都在 3s 内更新过（MONITOR 自己除外，它卡死由 IWDG 兜底）。
 * 返回 1 全部存活、可以喂狗；返回 0 说明有任务卡死，绝不能喂。 */
uint8_t Watchdog_CheckAllAlive(void);

/* 查某个任务最后一次心跳的时间戳（tick，1kHz 折算成 ms）。
 * 卡死告警时用它打印出到底是哪几个任务没动静。 */
uint32_t Watchdog_GetLastBeat(WDT_TaskId_t taskId);

#endif /* __BSP_IWDG_H */
