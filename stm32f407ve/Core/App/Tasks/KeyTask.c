/* USER CODE BEGIN Header */
/* KeyTask.c
 * 按键检测：10ms 轮询 KEY_1(PA0)/KEY_2(PA1)，连续 3 次同电平才确认，
 * 稳定按下沿翻转一次小灯，状态经 LEDQueue 交给 LEDTask。
 */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS.h"
#include "bsp_IWDG.h"
#include "LEDType.h"
#include "PowerMgr.h"

/*队列句柄由 freertos.c 创建*/
extern osMessageQueueId_t LEDQueueHandle;

/*扫描周期，单位 ms，任务读*/
#define KEY_SCAN_PERIOD_MS   10U
/*连续同电平确认次数，3 次约 30ms 消抖*/
#define KEY_DEBOUNCE_CNT     3U

/* 往 LEDQueue 投一条点灯指令：malloc 一条消息 → 填色和状态 → 投指针。
 * 队列满说明消息没送出去，要立刻 free，否则泄漏。 */
static void LED_PostMessage(LEDColor color, LEDState state)
{
  LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));   /*生产者分配，LEDTask 消费后释放*/
  if(msg != NULL)
  {
    msg->color = color;
    msg->state = state;
    if(osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
      vPortFree(msg);   /*队列满立即释放，不漏内存*/
    }
  }
}

/* 按键扫描任务：10ms 扫一次，连续 3 次同电平才认（消抖），
 * 确认按下后往 LEDQueue 投一条点灯指令。 */
void StartKeyTask(void *argument)
{
  (void)argument;

  /*CubeMX 生成的 PA0/PA1 是 NOPULL，PA1 浮空会让蓝灯键失灵，须在此重配*/
  GPIO_InitTypeDef keyIn = {0};
  keyIn.Mode  = GPIO_MODE_INPUT;
  keyIn.Speed = GPIO_SPEED_FREQ_LOW;
  keyIn.Pin   = KEY_1_Pin;
  keyIn.Pull  = GPIO_PULLDOWN;      /*PA0 内部下拉，按下为高*/
  HAL_GPIO_Init(KEY_1_GPIO_Port, &keyIn);
  keyIn.Pin   = KEY_2_Pin;
  keyIn.Pull  = GPIO_PULLUP;        /*PA1 内部上拉，按下为低*/
  HAL_GPIO_Init(KEY_2_GPIO_Port, &keyIn);

  /*两键各自维护稳定电平与消抖计数*/
  GPIO_PinState stable1 = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
  GPIO_PinState stable2 = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
  uint8_t cnt1 = KEY_DEBOUNCE_CNT;
  uint8_t cnt2 = KEY_DEBOUNCE_CNT;

  for(;;)
  {
    GPIO_PinState raw;

    Watchdog_Heartbeat(WDT_TASK_KEY);

    /*KEY_1 按下为高，稳定到高电平翻绿灯*/
    raw = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
    if(raw != stable1)
    {
      if(--cnt1 == 0U)
      {
        stable1 = raw;
        cnt1 = KEY_DEBOUNCE_CNT;
        if(stable1 == GPIO_PIN_SET)
        {
          /*按键活动，唤醒屏幕或刷新息屏计时*/
          PowerMgr_Wakeup();
          /*小灯低电平点亮，读到 RESET 表示当前亮，翻转目标为灭*/
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

    /*KEY_2 按下为低，稳定到低电平翻蓝灯*/
    raw = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
    if(raw != stable2)
    {
      if(--cnt2 == 0U)
      {
        stable2 = raw;
        cnt2 = KEY_DEBOUNCE_CNT;
        if(stable2 == GPIO_PIN_RESET)
        {
          /*按键活动，唤醒屏幕或刷新息屏计时*/
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

    osDelay(KEY_SCAN_PERIOD_MS);   /*10ms × 3 次 ≈ 30ms 消抖*/
  }
}
