/**
  ******************************************************************************
  * @file    SensorTask.c
  * @brief   传感器采集任务:周期读取 BH1750(光照) 与 DHT11(温湿度),
  *          结果写入全局共享结构体 g_sensorData,供 GuiTask 显示。
  *
  *          硬件:
  *            BH1750 -> 硬件 I2C1 (PB6=SCL, PB7=SDA)
  *            DHT11  -> 单总线 PC1
  *
  *          采样策略:
  *            DHT11 要求采样间隔 ≥ 1s,本任务周期 2s;
  *            BH1750 连续模式下可随时读取。
  ******************************************************************************
  */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_BH1750.h"
#include "bsp_DHT11.h"
#include "SensorData.h"
#include "usart.h"


/* 全局共享传感器数据 */
SensorData_t g_sensorData = {0};


/* CMSIS-RTOS2 的 stack_size 单位是字节(cmsis_os2.c 中 /sizeof(StackType_t) 转字):
 * 1024 字节 = 256 字。printf 浮点格式化栈开销较大,是否够用由 StatsTask 的
 * StackMin 列监控,水位过低时再加大 */
#define SENSOR_TASK_STACK_SIZE   1024U


/**
  * @brief  传感器采集任务入口
  */
void StartSensorTask(void *argument)
{
  (void)argument;

  BH1750_Data_t light;
  DHT11_Data_t  env;
  BH1750_Status_t stLight;
  DHT11_Status_t  stEnv;
  int tInt, tDec, hInt, hDec;   /* printf 整数/小数拆分(避免 %f) */

  /* 1. 初始化两个传感器 */
  if (BH1750_Init() == BH1750_OK)
    UART_Printf("[SENSOR] BH1750 init ok\r\n");
  else
    UART_Printf("[SENSOR] BH1750 init FAILED\r\n");

  DHT11_Init();
  UART_Printf("[SENSOR] DHT11 init ok\r\n");

  /* 2. 周期采集 */
  for (;;)
  {
    /* ---- 光照 BH1750 ---- */
    stLight = BH1750_Read(&light);
    if (stLight == BH1750_OK)
    {
      g_sensorData.light       = light.lux;
      g_sensorData.light_valid = 1;
    }
    else
    {
      g_sensorData.light_valid = 0;
    }

    /* ---- 温湿度 DHT11 ----
     * 单总线读 40 位是 us 级时序敏感操作(超时窗口 70~100us),读取期间临时把
     * 本任务提到 Realtime(高于 CommandTask High1/KEYTask High),避免被任务级
     * 抢占导致 T:err;只挡任务切换,不影响 UART/DMA 等中断响应。整个读取约 5ms */
    osThreadSetPriority(osThreadGetId(), osPriorityRealtime);
    stEnv = DHT11_Read(&env);
    osThreadSetPriority(osThreadGetId(), osPriorityNormal);
    if (stEnv == DHT11_OK)
    {
      g_sensorData.temperature = env.temperature;
      g_sensorData.humidity    = env.humidity;
      g_sensorData.temp_valid  = 1;
    }
    else
    {
      g_sensorData.temp_valid  = 0;
    }

    /* 调试:串口打印一次(避免 %f 浮点格式化,拆成整数+小数,省栈省 CPU) */
    tInt = (int)g_sensorData.temperature;
    tDec = (int)((g_sensorData.temperature - tInt) * 10.0f);
    if (tDec < 0) tDec = -tDec;
    hInt = (int)g_sensorData.humidity;
    hDec = (int)((g_sensorData.humidity - hInt) * 10.0f);
    if (hDec < 0) hDec = -hDec;
    UART_Printf("[SENSOR] T=%d.%dC H=%d.%d%% L=%dlux  (T:%s L:%s)\r\n",
           tInt, tDec,
           hInt, hDec,
           (int)g_sensorData.light,
           g_sensorData.temp_valid  ? "ok" : "err",
           g_sensorData.light_valid ? "ok" : "err");

    osDelay(2000);   /* DHT11 最小间隔 1s,留余量取 2s */
  }
}


/**
  * @brief  创建传感器采集任务(供 freertos.c 调用)
  */
void SensorTask_Create(void)
{
  static const osThreadAttr_t sensorTaskAttr = {
    .name = "SensorTask",
    .stack_size = SENSOR_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
  };
  osThreadNew(StartSensorTask, NULL, &sensorTaskAttr);
}
