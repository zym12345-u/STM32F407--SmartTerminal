/* USER CODE BEGIN Header */
/* LEDTask.c
 * LED 控制：阻塞等待 LEDQueue，按消息颜色驱动绿灯 PC5 / 蓝灯 PB2，
 * 处理完释放消息内存。
 */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
#include "bsp_IWDG.h"
#include "LEDType.h"

/*队列句柄由 freertos.c 创建*/
extern osMessageQueueId_t LEDQueueHandle;

/* LED 任务主循环：阻塞等 LEDQueue 的消息，收到就去点灯，点完把消息释放掉。
 * 队列里传的是指针，所以释放这一步必须做，否则每次点灯都漏一块内存。 */
void StartLEDTask(void *argument)
{
  (void)argument;

  for(;;)
  {
    LEDMessage *msg = NULL;

    Watchdog_Heartbeat(WDT_TASK_LED);

    /*最多等 1s 就回来喂心跳，不能等 portMAX_DELAY*/
    if(osMessageQueueGet(LEDQueueHandle, &msg, NULL, 1000U) == osOK)
    {
      /*板载 LED 低电平点亮，On 输出 RESET，Off 输出 SET*/
      GPIO_PinState pinLevel = (msg->state == LEDState_On) ? GPIO_PIN_RESET : GPIO_PIN_SET;

      switch(msg->color)
      {
        case LEDColor_Green:
          HAL_GPIO_WritePin(led_green_GPIO_Port, led_green_Pin, pinLevel);
          break;

        case LEDColor_Blue:
          HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, pinLevel);
          break;

        default:
          break;
      }
      vPortFree(msg);   /*消费后释放生产者分配的内存*/
    }
  }
}
