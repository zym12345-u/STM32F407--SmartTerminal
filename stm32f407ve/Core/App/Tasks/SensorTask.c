/* SensorTask.c
 * 传感器采集：每 2s 读 BH1750(光照, I2C1 PB6/PB7) 与 DHT11(温湿度, 单总线 PC1)，
 * 结果写入 g_sensorData，供 GUI/Modbus/CAN 读取。
 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_BH1750.h"
#include "bsp_DHT11.h"
#include "bsp_IWDG.h"
#include "SensorData.h"
#include "usart.h"


/*全局共享传感器数据，SensorTask 写，其余读*/
SensorData_t g_sensorData = {0};


/*stack_size 单位是字节：1024 字节＝256 字，StackMin 水位过低时再加*/
#define SENSOR_TASK_STACK_SIZE   1024U


/**
  * 功    能：传感器任务：周期读 DHT11 温湿度和 BH1750 光照，结果写进全局 g_sensorData
  * 参    数：argument  未使用，任务参数
  * 返 回 值：无
  * 补    充：读 DHT11 期间会临时把自己提到最高优先级（时序要求严格），读完立刻降回来
  */
void StartSensorTask(void *argument)
{
  (void)argument;

  BH1750_Data_t light;
  DHT11_Data_t  env;
  BH1750_Status_t stLight;
  DHT11_Status_t  stEnv;
  int tInt, tDec, hInt, hDec;   /*整数/小数拆分，避免 %f*/
  TickType_t lastWakeTime;      /*绝对周期基准*/

  /*初始化传感器*/
  if (BH1750_Init() == BH1750_OK)
    UART_Printf("[SENSOR] BH1750 init ok\r\n");
  else
    UART_Printf("[SENSOR] BH1750 init FAILED\r\n");

  DHT11_Init();
  UART_Printf("[SENSOR] DHT11 init ok\r\n");

  /*vTaskDelayUntil 以绝对时刻为基准，周期恒为 2s 不漂移*/
  lastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    Watchdog_Heartbeat(WDT_TASK_SENSOR);
    /*光照 BH1750*/
    stLight = BH1750_Read(&light);
    if (stLight == BH1750_OK)
    {
      g_sensorData.light       = light.lux;
      g_sensorData.light_valid = 1;
    }
    else
    {
      g_sensorData.light_valid = 0;   /*置 0，Modbus/CAN 据此回 0xFFFF*/
    }

    /*温湿度 DHT11*/
    /*读 40 位是 us 级时序，读期间提到 Realtime 防任务抢占，约 5ms，不挡中断*/
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
      g_sensorData.temp_valid  = 0;   /*温湿度共用同一有效位*/
    }

    /*拆整数+小数打印，避免 %f 的栈开销*/
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

    /*阻塞到下一个绝对时刻，采样间隔严格 2000ms，满足 DHT11 ≥1s*/
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000U));
  }
}


/**
  * 功    能：创建传感器任务，由 freertos.c 启动时调一次。句柄不保存
  * 参    数：无
  * 返 回 值：无，句柄不保存
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
