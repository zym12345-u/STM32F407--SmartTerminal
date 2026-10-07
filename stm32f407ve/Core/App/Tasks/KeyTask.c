/* USER CODE BEGIN Header */
/**
  * @file    KeyTask.c
  * @brief   按键检测任务(生产者):10ms 周期轮询两个板载按键,积分式消抖
  *          (连续 3 次采样同电平才确认),仅在稳定按下沿翻转对应小灯一次。
  *          控制消息投递 LEDQueue,由 LEDTask 统一执行点灯。
  *          翻转前读 LED 引脚电平为唯一真相,与触摸按钮/串口三路控制不冲突。
  *          按键 -> 小灯映射(上下拉/方向相反,以原厂 03-1 例程为准):
  *            KEY_1 = PA0,内部下拉,按下高电平,稳定上升沿 -> 绿灯 PC5
  *            KEY_2 = PA1,内部上拉,按下低电平,稳定下降沿 -> 蓝灯 PB2
  * @note    freertos.c 中保留同名 __weak 空骨架,本文件的强定义在链接时
  *          覆盖弱符号(与 P3_QueueDemo 的组织方式一致)。
  */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
#include "LEDType.h"
#include "PowerMgr.h"

/* 队列句柄由 freertos.c 中 CubeMX 生成的代码定义/创建,此处外部引用 */
extern osMessageQueueId_t LEDQueueHandle;

/* 扫描周期 10ms;连续 3 次(30ms)同电平才确认状态稳定,劣质按键可调大到 5 */
#define KEY_SCAN_PERIOD_MS   10U
#define KEY_DEBOUNCE_CNT     3U

/**
  * @brief  向 LEDQueue 投递一条 LED 控制消息(参考 P3_QueueDemo 生产者):
  *         分配 LEDMessage → 填颜色/状态 → 传指针;非阻塞发送,
  *         队列满导致发送失败时立即释放内存,避免内存泄漏
  */
static void LED_PostMessage(LEDColor color, LEDState state)
{
  LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));
  if(msg != NULL)
  {
    msg->color = color;
    msg->state = state;
    if(osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
      vPortFree(msg);
    }
  }
}

/**
  * @brief  按键检测任务入口(由 CubeMX 的 KEYTask 创建)
  */
void StartKeyTask(void *argument)
{
  (void)argument;

  /* 显式把两键配为普通输入(轮询不用 EXTI),上下拉按原厂 03-1 例程:
   *   KEY_1/PA0 = 内部下拉,按下接高电平
   *   KEY_2/PA1 = 内部上拉,按下接地低电平
   * 注意两键方向相反!CubeMX 生成的是 NOPULL 浮空,PA1 空闲电平不确定,
   * 必须在 App 层重配,否则蓝灯键失灵(实测踩坑)。 */
  GPIO_InitTypeDef keyIn = {0};
  keyIn.Mode  = GPIO_MODE_INPUT;
  keyIn.Speed = GPIO_SPEED_FREQ_LOW;
  keyIn.Pin   = KEY_1_Pin;
  keyIn.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_1_GPIO_Port, &keyIn);
  keyIn.Pin   = KEY_2_Pin;
  keyIn.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_2_GPIO_Port, &keyIn);

  /* 两键各自维护"已确认稳定电平"和消抖剩余计数,互不影响 */
  GPIO_PinState stable1 = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
  GPIO_PinState stable2 = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
  uint8_t cnt1 = KEY_DEBOUNCE_CNT;
  uint8_t cnt2 = KEY_DEBOUNCE_CNT;

  for(;;)
  {
    GPIO_PinState raw;

    /* KEY_1 = PA0,板载下拉,按下为高:稳定到高电平(上升沿)翻转绿灯 */
    raw = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
    if(raw != stable1)
    {
      if(--cnt1 == 0U)
      {
        stable1 = raw;
        cnt1 = KEY_DEBOUNCE_CNT;
        if(stable1 == GPIO_PIN_SET)
        {
          /* 按键活动:息屏则唤醒,否则刷新活动计时 */
          PowerMgr_Wakeup();
          /* 小灯低电平点亮:读到 RESET(亮)→目标 Off,SET(灭)→目标 On */
          GPIO_PinState greenLvl = HAL_GPIO_ReadPin(led_green_GPIO_Port, led_green_Pin);
          LED_PostMessage(LEDColor_Green,
                          (greenLvl == GPIO_PIN_RESET) ? LEDState_Off : LEDState_On);
        }
      }
    }
    else
    {
      cnt1 = KEY_DEBOUNCE_CNT;
    }

    /* KEY_2 = PA1,内部上拉,按下为低:稳定到低电平(下降沿)翻转蓝灯 */
    raw = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
    if(raw != stable2)
    {
      if(--cnt2 == 0U)
      {
        stable2 = raw;
        cnt2 = KEY_DEBOUNCE_CNT;
        if(stable2 == GPIO_PIN_RESET)
        {
          /* 按键活动:息屏则唤醒,否则刷新活动计时 */
          PowerMgr_Wakeup();
          GPIO_PinState blueLvl = HAL_GPIO_ReadPin(led_blue_GPIO_Port, led_blue_Pin);
          LED_PostMessage(LEDColor_Blue,
                          (blueLvl == GPIO_PIN_RESET) ? LEDState_Off : LEDState_On);
        }
      }
    }
    else
    {
      cnt2 = KEY_DEBOUNCE_CNT;
    }

    osDelay(KEY_SCAN_PERIOD_MS);
  }
}
