/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* LED 队列元素类型 LEDMessage 定义在 App/Types(供下方生成的 sizeof(LEDMessage*) 使用);
 * 任务实现在 App/Tasks:KeyTask.c(强定义覆盖本文件弱骨架)、LEDTask.c(external) */
#include "LEDType.h"
/* LVGL GUI 任务创建函数(Core/App/Tasks/GuiTask.c) */
#include "usart.h"
extern void GuiTask_Create(void);
extern void SensorTask_Create(void);
extern void WifiTask_Create(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 类型定义已迁移至 Core/App/Types/LEDType.h */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务(KEYTask/LEDTask)与队列(LEDQueue)的句柄、属性均由 CubeMX 生成,
 * 见本文件下方 Definitions for ... 区域,此处不再重复定义 */
/* USER CODE END Variables */
/* Definitions for KEYTask */
osThreadId_t KEYTaskHandle;
const osThreadAttr_t KEYTask_attributes = {
  .name = "KEYTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for LEDQueue */
osMessageQueueId_t LEDQueueHandle;
const osMessageQueueAttr_t LEDQueue_attributes = {
  .name = "LEDQueue"
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* StartKeyTask / StartLEDTask 的 extern 声明由 CubeMX 在下方生成区输出;
 * 任务实现在 Core/App/Tasks/KeyTask.c、LEDTask.c */
static void StartStatsTask(void *argument);   /* 本文件 USER CODE 区的统计任务 */
/* USER CODE END FunctionPrototypes */

void StartKeyTask(void *argument);
extern void StartLEDTask(void *argument);
extern void StartCommandTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 开机版本标记:烧录新固件后串口第一行应能看到本行(调度器启动前,轮询直发)。
   * 若看不到此行/时间戳是旧的,说明烧录的不是最新固件 */
  UART_Printf("\r\n[SYS] boot " __DATE__ " " __TIME__ "\r\n");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LEDQueue */
  LEDQueueHandle = osMessageQueueNew (16, sizeof(LEDMessage*), &LEDQueue_attributes);

  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* LEDQueue 已由 CubeMX 在上方创建(深度 16,元素为 LEDMessage 指针) */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of KEYTask */
  KEYTaskHandle = osThreadNew(StartKeyTask, NULL, &KEYTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* KEYTask、LEDTask 已由 CubeMX 在上方 osThreadNew 创建,此处无需重复创建 */
  /* LVGL GUI 任务:实现在 Core/App/Tasks/GuiTask.c,不占用 CubeMX 任务表 */
  GuiTask_Create();
  /* 传感器采集任务:实现在 Core/App/Tasks/SensorTask.c,采集 BH1750+DHT11 */
  SensorTask_Create();
  /* WiFi 对时任务:实现在 Core/App/Tasks/WifiTask.c,ESP8266(USART3) SNTP 取网络时间 */
  WifiTask_Create();
  /* 系统统计任务:5s 周期打印任务栈水位 + 各任务 CPU 占比(DWT 计数) */
  {
    static const osThreadAttr_t statsTaskAttr = {
      .name = "StatsTask",
      .stack_size = 1024U,       /* 字节;buf[640] 为 static,栈只放 printf 调用帧 */
      .priority = (osPriority_t)osPriorityLow,
    };
    osThreadNew(StartStatsTask, NULL, &statsTaskAttr);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartKeyTask */
/**
  * @brief  Function implementing the KEYTask thread.
  * @note   真正实现位于 Core/App/Tasks/KeyTask.c(同名强定义在链接时覆盖本弱骨架,
  *         与 P3_QueueDemo 组织方式一致)
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartKeyTask */
__weak void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartKeyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 任务实现已迁移:
 *   Core/App/Tasks/KeyTask.c - StartKeyTask(强定义,覆盖上方 __weak 骨架)
 *   Core/App/Tasks/LEDTask.c - StartLEDTask(CubeMX external 入口)
 * 消息类型见 Core/App/Types/LEDType.h */

/* ====================== FreeRTOS 运行时统计(DWT 周期计数器) ====================== */
/* CYCCNT @168MHz 是 32 位,25.56s 就回绕,直接当统计时钟几分钟后百分比全部失真。
 * 做法:每 1ms 在 tick 钩子里用无符号差分把增量累加进 64 位变量(无符号减法天然
 * 处理 25.56s 回绕),读出时再加上本 ms 内的增量并右移 10 位(约 164kHz,
 * 32 位结果 7.3 小时才回绕) */
static volatile uint64_t s_runCycles = 0U;
static volatile uint32_t s_lastCycles = 0U;

/* vTaskStartScheduler() 在任何任务运行前调用一次本函数;不能依赖 DHT11_Init 里的 DWT_Init */
void vConfigureTimerForRunTimeStats(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  s_lastCycles = 0U;
  s_runCycles  = 0U;
}

/* 1ms tick 中断:累加本 tick 内的 cycle 增量(回绕也正确:0 - 大数 = 回绕后的真实差值) */
void vApplicationTickHook(void)
{
  uint32_t now = DWT->CYCCNT;
  s_runCycles += (uint32_t)(now - s_lastCycles);
  s_lastCycles = now;
}

/* 调用点均在临界区内(vTaskSwitchContext / uxTaskGetSystemState),SysTick 已被
 * BASEPRI 屏蔽,64 位累加值不会在读取中途被改写 */
uint32_t vGetRunTimeCounterValue(void)
{
  uint32_t now = DWT->CYCCNT;
  uint64_t total = s_runCycles + (uint32_t)(now - s_lastCycles);
  return (uint32_t)(total >> 10);
}

/* ====================== 系统诊断钩子 ====================== */
/* 栈溢出/堆耗尽后已关中断,不能再走互斥锁+DMA(会死锁),用轮询直发固定字符串 */
static void FatalUartSend(const char *s)
{
  uint16_t n = 0U;
  while (s[n] != '\0' && n < 200U) { n++; }
  (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, n, 0xFFFFU);
}

/* 栈溢出:打印任务名后停机(继续跑会发生无规律 HardFault,停机更易定位) */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  FatalUartSend("\r\n[FATAL] stack overflow in task: ");
  FatalUartSend(pcTaskName);
  FatalUartSend("\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;) { ; }
}

/* 堆分配失败:队列消息全靠 pvPortMalloc,失败说明 configTOTAL_HEAP_SIZE 不足 */
void vApplicationMallocFailedHook(void)
{
  FatalUartSend("\r\n[FATAL] pvPortMalloc failed, heap exhausted\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;) { ; }
}

/* ====================== 统计任务:每 5s 打印任务状态/栈水位/CPU 占比 ====================== */
static void StartStatsTask(void *argument)
{
  (void)argument;
  /* 8 个任务(含 Idle/Tmr Svc)每行约 45 字节,640 字节足够 */
  static char buf[640];

  osDelay(5000U);   /* 等系统稳定(避开 LVGL 首屏渲染)再开始统计 */
  for(;;)
  {
    /* 任务表: Name State Prio StackMin(剩余字) Num */
    vTaskList(buf);
    UART_Printf("\r\n================ Task List ================\r\n%s", buf);

    /* CPU 占比: Name AbsTime(统计单位) Percent%%(上电以来累计平均) */
    vTaskGetRunTimeStats(buf);
    UART_Printf("================ CPU Usage =================\r\n%s", buf);
    UART_Printf("============================================\r\n");

    osDelay(5000U);
  }
}
/* USER CODE END Application */

