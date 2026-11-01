/* USER CODE BEGIN Header */
/* freertos.c
 * CubeMX 生成的 FreeRTOS 骨架：创建任务与队列，并挂运行时统计和故障钩子。
 * 注意：栈溢出/堆耗尽钩子里已关中断，只能轮询直发。
 *
 * ══════ FreeRTOS 的启动流程（顺序错了就跑不起来）══════
 *
 * ── 标准三步 ──
 *   HAL_Init / 时钟 / 外设初始化
 *      ↓
 *   MX_FREERTOS_Init()   ← 本文件，创建所有任务和队列（此时调度器还没跑）
 *      ↓
 *   osKernelStart()      ← 启动调度器，从此再也不返回
 *
 * ── 为什么任务必须在 osKernelStart 之前创建 ──
 *   调度器启动前，系统里【没有任务】这个概念，只有 main 这一条执行流。
 *   创建任务本质是「往调度器的就绪表里登记」，等调度器一启动，它才知道该轮流跑谁。
 *   启动后再创建也行（任何任务里都能建），但那样就失去了「上电即全部就绪」的确定性，
 *   所以通常都放在这里的初始化阶段。
 *
 * ── 关于「调度器启动前不能用的 API」──
 *   很多 RTOS 调用（osDelay、osMutexAcquire、等信号量）都依赖「当前有任务」。
 *   调度器没跑时没有当前任务，调了会崩或行为未定义。
 *   所以初始化阶段只能做「创建对象」，不能做「等待」。
 *   （这也是 W25Q128 驱动里那套 DWT 忙等延时的由来 —— boot 阶段不能用 osDelay。）
 *
 * ── 任务栈大小怎么定 ──
 *   数字是「字（word）」，不是字节。1024 表示 1024×4 = 4096 字节。
 *   定小了不一定立刻崩：栈溢出会踩到相邻内存，表现为「跑一会儿莫名重启」。
 *   所以本工程开了栈溢出检测（configCHECK_FOR_STACK_OVERFLOW），
 *   并且 MonitorTask 会定期打印各任务栈的剩余水位，跑一段时间观察才敢定稿。
 *
 * ── 两个故障钩子为什么只能「轮询直发 + 死循环」──
 *   vApplicationStackOverflowHook / vApplicationMallocFailedHook 被调用时，
 *   系统已经处于不可信状态（栈已经爆了 / 堆已经没了）。
 *   此时任何依赖内存分配的调用都可能再次崩溃，所以：
 *     1) FatalUartSend 直接轮询串口寄存器发字符，不过 RTOS、不加锁、不 malloc
 *     2) 发完就死循环，等独立看门狗把整机复位
 *   这是「现场留证据」和「尽快恢复」之间的标准取舍。
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
/* CubeMX 生成的默认空实现（__weak）。真正的按键任务在 KeyTask.c 里，
 * 同名覆盖掉了这个弱符号，所以这里永远轮不到执行。 */
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
  * 功    能：FreeRTOS 统计 CPU 占用率用的钩子：这里使能 DWT 周期计数器当时间基准
  * 参    数：无
  * 返 回 值：无
  * 补    充：只有开了 configGENERATE_RUN_TIME_STATS 才会被调到
  */
void vConfigureTimerForRunTimeStats(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  s_lastCycles = 0U;
  s_runCycles  = 0U;
}

/* 每个系统节拍（1ms）被调用一次。这里留空，没做任何事。 */
/* 在 SysTick 中断上下文里执行，只做加法 */
void vApplicationTickHook(void)
{
  uint32_t now = DWT->CYCCNT;
  s_runCycles += (uint32_t)(now - s_lastCycles);
  s_lastCycles = now;
}

/**
  * 功    能：给 FreeRTOS 统计用的时间基准：返回 DWT 周期数右移 10 位
  * 参    数：无
  * 返 回 值：uint32_t  累计周期右移 10 位（约 164kHz，7.3 小时回绕）
  * 补    充：折算后约 164kHz，约 7.3 小时回绕一次，够统计一次运行周期用
  */
uint32_t vGetRunTimeCounterValue(void)
{
  uint32_t now = DWT->CYCCNT;
  uint64_t total = s_runCycles + (uint32_t)(now - s_lastCycles);
  return (uint32_t)(total >> 10);
}

/* 故障钩子：已关中断，只能轮询直发，不能再取锁 */
/**
  * 功    能：系统快挂时用的最后一条打印：不走互斥锁、不依赖 RTOS，直接轮询串口往外发
  * 参    数：s  以 '\0' 结尾的字符串，最多发 200 字节
  * 返 回 值：无
  * 补    充：因为走到这里时堆或栈已经出问题了，套接字/信号量这些都不能再信。最多发 200 字节
  */
static void FatalUartSend(const char *s)
{
  uint16_t n = 0U;
  while (s[n] != '\0' && n < 200U) { n++; }
  (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, n, 0xFFFFU);
}

/**
  * 功    能：有任务把栈用爆时会进这里：打印任务名（最多 200 字节）然后死循环，
  * 参    数：xTask  溢出任务句柄（未使用）
  *            pcTaskName  任务名，最多打印 200 字节
  * 返 回 值：无
  * 补    充：等独立看门狗把整机复位。xTask 句柄用不上
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
  * 功    能：FreeRTOS 堆分配失败时会进这里：打印一条然后死循环等复位
  * 参    数：无
  * 返 回 值：无
  * 补    充：出现它就说明 configTOTAL_HEAP_SIZE 开小了，要去调大
  */
void vApplicationMallocFailedHook(void)
{
  FatalUartSend("\r\n[FATAL] pvPortMalloc failed, heap exhausted\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;) { ; }
}

/**
  * 功    能：统计任务：每 5s 打一次任务列表和 CPU 占用率，纯调试用，不参与业务
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

