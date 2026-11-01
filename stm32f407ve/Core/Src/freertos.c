/* USER CODE BEGIN Header */
/* freertos.c
 * CubeMX 生成的 FreeRTOS 骨架：创建任务与队列，并挂运行时统计和故障钩子。
 * 注意：栈溢出/堆耗尽钩子里已关中断，只能轮询直发。
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LEDType.h"          /*队列元素类型 LEDMessage*/
#include "usart.h"
extern void GuiTask_Create(void);      /*LVGL GUI 任务*/
extern void SensorTask_Create(void);   /*BH1750 + DHT11 采集*/
extern void WifiTask_Create(void);     /*ESP8266 SNTP 对时*/
extern void ModbusTask_Create(void);    /*RS485 Modbus RTU 从站*/
extern void CanTask_Create(void);       /*CAN1 500Kbps 收发*/
extern void MonitorTask_Create(void);   /*心跳监控 + IWDG 喂狗*/
#include "bsp_W25Q128.h"               /*外部 Flash 自检*/
#include "bsp_lfs_port.h"              /*LittleFS 挂载*/
#include "bsp_font_gbk.h"              /*外挂 GBK 字库自检*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
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
static void StartStatsTask(void *argument);   /*本文件的统计任务*/
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
  UART_Printf("\r\n[SYS] boot " __DATE__ " " __TIME__ "\r\n");   /*开机版本标记，能看到这行才算烧对固件*/

  /* 读复位标志后立即清除；NRST 引脚总会置位，所以看门狗/软件复位优先判断 */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  {
    UART_Printf("[SYS] reset cause: IWDG watchdog!\r\n");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
  {
    UART_Printf("[SYS] reset cause: WWDG watchdog\r\n");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  {
    UART_Printf("[SYS] reset cause: software (Keil flash / NVIC_SystemReset)\r\n");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
  {
    UART_Printf("[SYS] reset cause: low-power management\r\n");
  }
  else
  {
    UART_Printf("[SYS] reset cause: power-on / NRST pin\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LEDQueue */
  LEDQueueHandle = osMessageQueueNew (16, sizeof(LEDMessage*), &LEDQueue_attributes);

  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of KEYTask */
  KEYTaskHandle = osThreadNew(StartKeyTask, NULL, &KEYTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  GuiTask_Create();       /*LVGL GUI*/
  SensorTask_Create();   /*BH1750 + DHT11*/
  WifiTask_Create();      /*ESP8266 网络对时*/
  ModbusTask_Create();    /*RS485 Modbus 从站*/
  CanTask_Create();       /*CAN1 报文收发*/
  MonitorTask_Create();   /*心跳检测 + IWDG 喂狗*/
  /* 每 5s 打印任务栈水位和 CPU 占比 */
  {
    static const osThreadAttr_t statsTaskAttr = {
      .name = "StatsTask",
      .stack_size = 1024U,       /*字节；打印缓冲为 static*/
      .priority = (osPriority_t)osPriorityLow,
    };
    osThreadNew(StartStatsTask, NULL, &statsTaskAttr);
  }
  /* 所有任务建完后的稳态堆余量，调度器启动还会再占约 1536B */
  UART_Printf("[SYS] free heap after all app tasks: %u bytes\r\n",
              (unsigned)xPortGetFreeHeapSize());
  /* Flash 容量寄存器，VET6 应为 512KB */
  UART_Printf("[SYS] flash size register: %u KB\r\n",
              (unsigned)(*(__IO uint16_t *)0x1FFF7A22UL));
  /* 地址别名甄别：读 0x080E0000 与 0x08060000 首字对比，只读安全 */
  UART_Printf("[SYS] magic@0x080E0000=0x%08lX word@0x08060000=0x%08lX\r\n",
              (unsigned long)(*(__IO uint32_t *)0x080E0000UL),
              (unsigned long)(*(__IO uint32_t *)0x08060000UL));
  /* 调度器启动前做 Flash 自检：JEDEC ID + 尾扇区擦写回读 */
  if (W25Q128_Init() != 0U)
  {
    W25Q128_SelfTest();
    W25Q128_CheckGBKFont();   /*检测 0xA00000 起的 GBK 字库*/
    /* 挂载 LittleFS，空白 Flash 会自动格式化 */
    (void)LFS_Port_Mount();
    /* 读“中”的 16px 点阵验证字库 */
    (void)BSP_Font_SelfTest();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartKeyTask */
/**
  * 函    数：StartKeyTask
  * 参    数：argument  未使用
  * 返 回 值：无
  */
/* USER CODE END Header_StartKeyTask */
__weak void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartKeyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 运行时统计：CYCCNT 32 位 25.56s 就回绕，每 1ms 把增量累进 64 位变量 */
static volatile uint64_t s_runCycles = 0U;
static volatile uint32_t s_lastCycles = 0U;

/**
  * 函    数：vConfigureTimerForRunTimeStats
  * 参    数：无
  * 返 回 值：无
  */
void vConfigureTimerForRunTimeStats(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  s_lastCycles = 0U;
  s_runCycles  = 0U;
}

/**
  * 函    数：vApplicationTickHook
  * 参    数：无
  * 返 回 值：无
  */
/* 在 SysTick 中断上下文里执行，只做加法 */
void vApplicationTickHook(void)
{
  uint32_t now = DWT->CYCCNT;
  s_runCycles += (uint32_t)(now - s_lastCycles);
  s_lastCycles = now;
}

/**
  * 函    数：vGetRunTimeCounterValue
  * 参    数：无
  * 返 回 值：uint32_t  累计周期右移 10 位（约 164kHz，7.3 小时回绕）
  */
uint32_t vGetRunTimeCounterValue(void)
{
  uint32_t now = DWT->CYCCNT;
  uint64_t total = s_runCycles + (uint32_t)(now - s_lastCycles);
  return (uint32_t)(total >> 10);
}

/* 故障钩子：已关中断，只能轮询直发，不能再取锁 */
/**
  * 函    数：FatalUartSend
  * 参    数：s  以 '\0' 结尾的字符串，最多发 200 字节
  * 返 回 值：无
  */
static void FatalUartSend(const char *s)
{
  uint16_t n = 0U;
  while (s[n] != '\0' && n < 200U) { n++; }
  (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, n, 0xFFFFU);
}

/**
  * 函    数：vApplicationStackOverflowHook
  * 参    数：xTask  溢出任务句柄（未使用）
  * 参    数：pcTaskName  任务名，最多打印 200 字节
  * 返 回 值：无
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  FatalUartSend("\r\n[FATAL] stack overflow in task: ");
  FatalUartSend(pcTaskName);
  FatalUartSend("\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;) { ; }
}

/**
  * 函    数：vApplicationMallocFailedHook
  * 参    数：无
  * 返 回 值：无
  */
void vApplicationMallocFailedHook(void)
{
  FatalUartSend("\r\n[FATAL] pvPortMalloc failed, heap exhausted\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;) { ; }
}

/**
  * 函    数：StartStatsTask
  * 参    数：argument  未使用
  * 返 回 值：无
  */
static void StartStatsTask(void *argument)
{
  (void)argument;
  /* 8 个任务每行约 45 字节，640 字节够用 */
  static char buf[640];

  osDelay(5000U);   /* 避开 LVGL 首屏渲染再开始统计 */
  for(;;)
  {
    /* 任务表：名称/状态/优先级/最小剩余栈 */
    vTaskList(buf);
    UART_Printf("\r\n================ Task List ================\r\n%s", buf);

    /* CPU 占比，上电以来累计平均 */
    vTaskGetRunTimeStats(buf);
    /* 分隔线并进同一次调用，否则别的任务日志会插进来 */
    UART_Printf("================ CPU Usage =================\r\n%s"
                "============================================\r\n", buf);

    osDelay(5000U);
  }
}
/* USER CODE END Application */

