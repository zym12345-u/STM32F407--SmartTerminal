/**
  ******************************************************************************
  * @file    MonitorTask.c
  * @brief   系统监控任务: 任务心跳检测 + IWDG 看门狗喂狗
  *
  *          工作流程:
  *            1. 启动 IWDG(超时 1s, 启动后无法关闭)
  *            2. 每 500ms 检查所有任务心跳是否在 3s 内更新
  *            3. 全部存活 -> 喂狗; 有任务卡死 -> 不喂狗 -> IWDG 复位
  *            4. 复位后可通过检查 RCC_CSR 判断是否为 IWDG 复位(诊断用)
  *
  *          各任务需在主循环里调用 Watchdog_Heartbeat(WDT_TASK_xxx) 上报存活
  ******************************************************************************
  */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdio.h>   /* snprintf: 诊断报告先在局部缓冲拼好再发出 */

#include "bsp_IWDG.h"
#include "usart.h"   /* UART_Printf */

#define MONITOR_INTERVAL_MS     500U    /* 监控周期 */

void StartMonitorTask(void *argument)
{
    (void)argument;

    /* 启动 IWDG(此时调度器已运行, MonitorTask 自身会持续喂狗) */
    IWDG_Init();
    UART_Printf("[MON] IWDG started (timeout ~1s)\r\n");

    for (;;)
    {
        /* 报自己的心跳。注意 CheckAllAlive() 会跳过 MONITOR(不监控自己),
         * 所以本任务自己卡死时不会被"判死"逻辑发现 ——
         * 但那时也没人喂狗了,IWDG 照样会复位,这是最后的兜底 */
        Watchdog_Heartbeat(WDT_TASK_MONITOR);

        if (Watchdog_CheckAllAlive())
        {
            IWDG_Refresh();                /* 所有任务存活 -> 喂狗 */
        }
        else
        {
            /* 有任务卡死: 打印诊断信息, 不喂狗, 等待 IWDG 复位 */
            /* 整个报告先拼进局部缓冲, 再一次 UART_Printf 发出。
             * 与 CanTask 的 rx 行同理: 这里是"多行拼一份报告", 若逐行调用,
             * 其它任务的日志会插在 task[3] 与 task[4] 之间, 这份诊断就被
             * 打散得没法读。真正卡死的现场恰恰最需要它可读。
             * 容量: WDT_TASK_MAX(10) * "  task[NN]=NNNNNNNNN\r\n"(约 22B)
             *      ≈ 220B, 给 320B 留足余量; 用 snprintf 逐段拼并卡边界。 */
            uint8_t i;
            char    beats[320];
            int     p = 0;
            for (i = 0U; i < WDT_TASK_MAX; i++)
            {
                p += snprintf(&beats[p], sizeof(beats) - (size_t)p,
                              "  task[%u]=%lu\r\n", (unsigned)i,
                              (unsigned long)Watchdog_GetLastBeat((WDT_TaskId_t)i));
            }
            UART_Printf("[MON] task stuck! last beats:\r\n%s", beats);
            /* 不喂狗, IWDG 约 1s 后复位 */
        }

        (void)osDelay(MONITOR_INTERVAL_MS);
    }
}

void MonitorTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name       = "MonitorTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t)osPriorityHigh,   /* 高优先级: 保证监控及时 */
    };
    (void)osThreadNew(StartMonitorTask, NULL, &attr);
}
