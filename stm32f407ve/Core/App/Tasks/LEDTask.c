/* USER CODE BEGIN Header */
/**
  * @file    LEDTask.c
  * @brief   LED 控制任务(消费者):阻塞等待 LEDQueue 消息,按消息中的颜色
  *          驱动对应引脚(绿灯 PC5 / 蓝灯 PB2),处理完释放消息内存。
  *          (参考 P3_QueueDemo/Core/App/Tasks/LEDTask.c)
  */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
#include "LEDType.h"

/* LEDQueue 句柄由 freertos.c 中 CubeMX 生成的代码定义/创建,此处外部引用 */
extern osMessageQueueId_t LEDQueueHandle;

/**
  * @brief  LED 控制任务入口(由 CubeMX 的 LEDTask 创建,As external)
  * @note   板载 LED 低电平点亮(灌电流,官方流水灯例程证实):
  *          亮 -> GPIO_PIN_RESET,灭 -> GPIO_PIN_SET
  */
void StartLEDTask(void *argument)
{
  (void)argument;

  for(;;)
  {
    LEDMessage *msg = NULL;

    /* 队列空时永久阻塞,不占用 CPU;收到消息后立即点亮/熄灭对应小灯 */
    if(osMessageQueueGet(LEDQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      /* 低电平点亮:On 输出 RESET,Off 输出 SET */
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
      vPortFree(msg);   /* 消费完毕,释放生产者分配的内存 */
    }
  }
}
