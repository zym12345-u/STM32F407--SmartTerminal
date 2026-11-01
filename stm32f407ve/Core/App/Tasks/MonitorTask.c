/* MonitorTask.c
 * 系统监控：每 500ms 检查全部任务心跳，全部在 3s 内更新才喂 IWDG，
 * 有任务卡死则打印最后心跳并停止喂狗，约 1s 后复位。
 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdio.h>

#include "bsp_IWDG.h"
#include "usart.h"   /* UART_Printf */

#define MONITOR_INTERVAL_MS     500U    /*监控周期，单位 ms，本任务读*/

/**
  * 函    数：StartMonitorTask
  * 参    数：argument  未使用，任务参数
  * 返 回 值：无
  */
void StartMonitorTask(void *argument)
{
    (void)argument;

    /*IWDG 超时约 1s，启动后无法关闭*/
    IWDG_Init();
    UART_Printf("[MON] IWDG started (timeout ~1s)\r\n");

    for (;;)
    {
        /*本任务不检查自己，自身卡死时没人喂狗，由 IWDG 复位*/
        Watchdog_Heartbeat(WDT_TASK_MONITOR);

        if (Watchdog_CheckAllAlive())
        {
            IWDG_Refresh();                /*全部存活才喂狗*/
        }
        else
        {
            /*有任务卡死：打印诊断，不喂狗，等 IWDG 复位*/
            /*报告先拼进缓冲再一次发出，避免多任务日志穿插打散*/
            uint8_t i;
            char    beats[320] = {0};
            int     p = 0;
            /*snprintf 返回应写长度，p 可能超出缓冲；循环须卡剩余空间，否则无符号下溢写爆栈*/
            for (i = 0U; (i < WDT_TASK_MAX) && (p < (int)sizeof(beats) - 28); i++)
            {
                p += snprintf(&beats[p], sizeof(beats) - (size_t)p,
                              "  task[%u]=%lu\r\n", (unsigned)i,
                              (unsigned long)Watchdog_GetLastBeat((WDT_TaskId_t)i));
            }
            UART_Printf("[MON] task stuck! last beats:\r\n%s", beats);
            /*不喂狗，约 1s 后复位*/
        }

        (void)osDelay(MONITOR_INTERVAL_MS);
    }
}

/**
  * 函    数：MonitorTask_Create
  * 参    数：无
  * 返 回 值：无，句柄不保存
  */
void MonitorTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name       = "MonitorTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t)osPriorityHigh,   /*高优先级，保证监控及时*/
    };
    (void)osThreadNew(StartMonitorTask, NULL, &attr);
}
