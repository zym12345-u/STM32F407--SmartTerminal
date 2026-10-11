/**
  ******************************************************************************
  * @file    bsp_IWDG.c
  * @brief   独立看门狗(IWDG)驱动 + 任务心跳监控实现
  *         直接操作 IWDG 寄存器, 不依赖 HAL_IWDG 模块(避免引入额外源文件)
  ******************************************************************************
  */
#include "bsp_IWDG.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* IWDG 超时配置: LSI ≈ 32kHz, 预分频 256, 重装载值 125
 * T = (256 * 125) / 32000 ≈ 1.0s */
#define IWDG_PRESCALER_256  6U
#define IWDG_RELOAD         125U
#define WDT_HEARTBEAT_TIMEOUT_MS   3000U   /* 任务心跳超时: 3s 未更新视为卡死 */

/* 各任务最后心跳时间戳(tick) */
static volatile uint32_t s_lastBeat[WDT_TASK_MAX] = {0U};

void IWDG_Init(void)
{
    /* 使能 LSI(若尚未启动) */
    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) { }

    /* 启动 IWDG: 向 KR 写 0xCCCC */
    IWDG->KR = 0xCCCCU;

    /* 解锁 PR 和 RLR: 向 KR 写 0x5555 */
    IWDG->KR = 0x5555U;

    /* 设置预分频 = 256 */
    IWDG->PR = IWDG_PRESCALER_256;

    /* 设置重装载值 */
    IWDG->RLR = IWDG_RELOAD;

    /* 等待寄存器更新完成 */
    while (IWDG->SR != 0U) { }

    /* 首次喂狗 */
    IWDG_Refresh();
}

void IWDG_Refresh(void)
{
    /* 喂狗: 向 KR 写 0xAAAA */
    IWDG->KR = 0xAAAAU;
}

void Watchdog_Heartbeat(WDT_TaskId_t taskId)
{
    if (taskId < WDT_TASK_MAX)
    {
        s_lastBeat[taskId] = xTaskGetTickCount();
    }
}

uint8_t Watchdog_CheckAllAlive(void)
{
    uint32_t now = xTaskGetTickCount();
    uint8_t  i;

    /* 检查除 MonitorTask 外的所有任务心跳 */
    for (i = 0U; i < WDT_TASK_MAX; i++)
    {
        if (i == WDT_TASK_MONITOR)
        {
            continue;
        }
        /* 心跳为 0 说明该任务从未上报(可能尚未启动), 暂不判死 */
        if (s_lastBeat[i] != 0U &&
            (now - s_lastBeat[i]) > pdMS_TO_TICKS(WDT_HEARTBEAT_TIMEOUT_MS))
        {
            return 0U;                          /* 任务 i 卡死 */
        }
    }
    return 1U;
}

uint32_t Watchdog_GetLastBeat(WDT_TaskId_t taskId)
{
    if (taskId < WDT_TASK_MAX)
    {
        return s_lastBeat[taskId];
    }
    return 0U;
}
