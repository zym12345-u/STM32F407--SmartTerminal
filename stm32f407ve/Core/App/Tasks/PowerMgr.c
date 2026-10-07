/**
  * @file    PowerMgr.c
  * @brief   低功耗-自动息屏管理模块实现。
  *          核心思路:维护 s_lastActiveTick(最后一次用户活动的 FreeRTOS tick),
  *          GUITask 每轮调用 PowerMgr_Process() 检查是否超过 POWERMGR_SLEEP_TIMEOUT_MS,
  *          超时则 enter_sleep() 关背光;息屏期间触摸/按键触发 PowerMgr_Wakeup()
  *          调用 exit_sleep() 开背光。
  *          enter_sleep/exit_sleep 成对出现,只操作背光(PA15),不碰 ILI9341 寄存器,
  *          唤醒后 LVGL 无需重绘。
  */

#include "PowerMgr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_LCD_ILI9341.h"
#include "usart.h"

/* 最后一次用户活动的 tick 值(uint32,减法天然处理溢出) */
static volatile TickType_t s_lastActiveTick;

/* 息屏状态标志:1=背光已关,0=正常显示 */
static volatile uint8_t    s_sleeping;

/**
  * @brief  进入息屏:关闭背光。与 exit_sleep 成对。
  * @note   只关背光(PA15 低电平),不发送 ILI9341 睡眠命令,
  *         显存/FSMC 保持,唤醒后无需重绘。
  */
static void enter_sleep(void)
{
    if (s_sleeping != 0U)
    {
        return;  /* 已息屏,避免重复操作 */
    }
    LCD_DisplayOff();
    s_sleeping = 1U;
    UART_Printf("[PWR] enter sleep (backlight off)\r\n");
}

/**
  * @brief  退出息屏:开启背光 + 刷新活动计时。与 enter_sleep 成对。
  */
static void exit_sleep(void)
{
    if (s_sleeping == 0U)
    {
        return;  /* 未息屏 */
    }
    s_sleeping = 0U;
    s_lastActiveTick = xTaskGetTickCount();
    LCD_DisplayOn();
    UART_Printf("[PWR] exit sleep (backlight on)\r\n");
}

void PowerMgr_Init(void)
{
    s_lastActiveTick = xTaskGetTickCount();
    s_sleeping = 0U;
}

void PowerMgr_Feed(void)
{
    s_lastActiveTick = xTaskGetTickCount();
}

void PowerMgr_Wakeup(void)
{
    if (s_sleeping != 0U)
    {
        exit_sleep();
    }
    else
    {
        /* 未息屏时等价于刷新活动计时 */
        s_lastActiveTick = xTaskGetTickCount();
    }
}

void PowerMgr_Process(void)
{
    /* 仅在未息屏时检查超时;息屏后由触摸/按键主动唤醒,此处不再检测 */
    if (s_sleeping == 0U)
    {
        TickType_t now = xTaskGetTickCount();
        if ((now - s_lastActiveTick) >= (TickType_t)POWERMGR_SLEEP_TIMEOUT_MS)
        {
            enter_sleep();
        }
    }
}

uint8_t PowerMgr_IsSleeping(void)
{
    return s_sleeping;
}
