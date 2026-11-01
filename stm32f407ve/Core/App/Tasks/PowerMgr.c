/* PowerMgr.c
 * 自动息屏：30s 无操作关背光，触摸/按键唤醒，只动 PA15，不碰 ILI9341 显存。
 * 周期：GuiTask 正常分支每 5ms 调一次 PowerMgr_Process()，息屏后 50ms 轮询触摸。
 * 注意：tick 用无符号减法比较，uint32 溢出天然安全，不要改成绝对大小判断。
 */

#include "PowerMgr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_LCD_ILI9341.h"
#include "usart.h"

/*最后一次用户活动的 tick，uint32 减法天然处理溢出*/
static volatile TickType_t s_lastActiveTick;

/*息屏标志：1=背光已关，0=正常显示*/
static volatile uint8_t    s_sleeping;

/*息屏状态机：s_sleeping 0 正常显示 / 1 息屏
 * 状态          含义         进入条件                          迁移                     超时
 * 0 正常显示    背光开       上电 Init 或刚唤醒                无操作满 30000ms 进状态 1  无
 * 1 息屏        背光关       enter_sleep 关背光                触摸唤醒进状态 0          无，只能人为唤醒
 * 切换只发生在 GuiTask 上下文，故 s_sleeping 无需加锁。
 */

/* 进息屏：关背光并记下时间。省电主要靠外设自己，这里只管显示。 */
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

/* 退出息屏：重新点亮背光。 */
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

/* 初始化自动息屏模块：清状态、记下起始时刻。 */
void PowerMgr_Init(void)
{
    /*必须在 GuiTask 进主循环前调用一次，否则 s_lastActiveTick=0 会立刻息屏*/
    s_lastActiveTick = xTaskGetTickCount();
    s_sleeping = 0U;
}

/* 喂一次「有人在操作」的信号：按键、触摸、命令都该调它，把息屏倒计时重置。 */
void PowerMgr_Feed(void)
{
    s_lastActiveTick = xTaskGetTickCount();
}

/* 强制唤醒：正在息屏时点亮屏幕（比如息屏中收到触摸或远程命令）。 */
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

/* 周期处理：判断是不是该息屏、该不该唤醒。各任务主循环里调一下即可。 */
void PowerMgr_Process(void)
{
    /*只在未息屏时查超时：息屏后靠触摸/按键唤醒*/
    if (s_sleeping == 0U)
    {
        TickType_t now = xTaskGetTickCount();
        if ((now - s_lastActiveTick) >= (TickType_t)POWERMGR_SLEEP_TIMEOUT_MS)   /* 30s 无操作 */
        {
            enter_sleep();
        }
    }
}

/* 问一下当前是不是息屏中。返回 1 息屏、0 正常显示。 */
uint8_t PowerMgr_IsSleeping(void)
{
    return s_sleeping;
}
