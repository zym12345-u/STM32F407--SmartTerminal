/* USER CODE BEGIN Header */
/* CommandTask.c
 * 串口命令解析：从 CommandQueue 取 USART1 字节，组帧校验后把每对 {颜色,状态}
 * 投递 LEDQueue，由 LEDTask 点灯。
 */
/* 协议帧 [0xAA][LEN][COLOR1][STATE1]...[CHECKSUM]
 * 字段        长度  含义              取值                 谁写谁读
 * HEAD        1B    包头              0xAA                 上位机写，本任务读
 * LEN         1B    整帧长度(含头尾)  4~30                 上位机写，本任务读
 * COLOR/STATE 2B    颜色与状态成对    COLOR 1~3，STATE 0/1 上位机写，本任务读
 * CHECKSUM    1B    前 LEN-1 字节累加 0~255(uint8 截断)    上位机写，本任务读
 * COLOR 1=红 2=绿 3=蓝，本板只有绿/蓝，红被忽略；STATE 0=灭 1=亮
 */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "bsp_IWDG.h"
#include "LEDType.h"
#include "PowerMgr.h"

/*队列句柄由 freertos.c 创建*/
extern osMessageQueueId_t CommandQueueHandle;
extern osMessageQueueId_t LEDQueueHandle;

/*协议常量*/
#define CMD_FRAME_HEAD      0xAAU   /*包头*/
#define CMD_FRAME_MIN_LEN   4U      /*整帧长度下限 4 字节*/
#define CMD_FRAME_BUF_SIZE  30U     /*接收缓冲大小，也是整帧最大长度*/

/*协议颜色码到 LEDColor 映射，不支持的颜色返回 0xFF*/
#define CMD_COLOR_RED       1U
#define CMD_COLOR_GREEN     2U
#define CMD_COLOR_BLUE      3U
#define CMD_COLOR_INVALID   0xFFU

/**
  * 功    能：把串口协议里的颜色码（1~3）翻成 LED 枚举
  * 参    数：code  协议颜色码，取值 1~3
  * 返 回 值：绿/蓝对应枚举，红及其它值返回 (LEDColor)0xFF
  */
static LEDColor Command_MapColor(uint8_t code)
{
  LEDColor color;

  switch(code)
  {
    case CMD_COLOR_GREEN:
      color = LEDColor_Green;
      break;
    case CMD_COLOR_BLUE:
      color = LEDColor_Blue;
      break;
    default:
      color = (LEDColor)CMD_COLOR_INVALID;   /*红色等不支持的颜色，忽略*/
      break;
  }
  return color;
}

/**
  * 功    能：串口命令任务：从 CommandQueue 取字节拼成一条命令，解析后执行（点灯等）
  * 参    数：argument  未使用，任务参数
  * 返 回 值：无
  * 补    充：取消息带 1s 超时，所以即使没命令进来也会定期报心跳
  */
void StartCommandTask(void *argument)
{
  (void)argument;

  uint8_t receive = 0U;
  uint8_t command[CMD_FRAME_BUF_SIZE];
  uint8_t commandIndex = 0U;    /*已存入字节数*/
  uint8_t commandLength = 0U;   /*本帧总长度*/

  /*打开 USART1 中断接收*/
  UART1_Receive_Start();

  for(;;)
  {
    Watchdog_Heartbeat(WDT_TASK_COMMAND);

    /*最多等 1s 就回来喂心跳，不能改 portMAX_DELAY，否则无命令时被判卡死*/
    if(osMessageQueueGet(CommandQueueHandle, &receive, NULL, 1000U) != osOK)
    {
      continue;
    }

    if(commandIndex == 0U)
    {
      /*找包头*/
      if(receive == CMD_FRAME_HEAD)
      {
        command[commandIndex++] = receive;
      }
    }
    else if(commandIndex == 1U)
    {
      /*第二字节为整帧长度，越界丢弃*/
      if((receive < CMD_FRAME_MIN_LEN) || (receive > (uint8_t)sizeof(command)))
      {
        commandIndex = 0U;   /*长度越界，重新找包头*/
        continue;
      }
      commandLength = receive;
      command[commandIndex++] = receive;
    }
    else
    {
      /*依次收负载与校验字节*/
      command[commandIndex++] = receive;

      if(commandIndex == commandLength)
      {
        /*整帧收完，累加前 LEN-1 字节*/
        uint8_t checksum = 0U;
        for(uint8_t i = 0U; i < (uint8_t)(commandLength - 1U); i++)
        {
          checksum += command[i];
        }

        if(checksum == command[commandLength - 1U])
        {
          /*合法命令算用户活动，刷新息屏计时*/
          PowerMgr_Feed();
          for(uint8_t i = 2U; i < (uint8_t)(commandLength - 2U); i += 2U)   /*步长 2，{颜色,状态} 成对*/
          {
            LEDColor color = Command_MapColor(command[i]);
            if(color == (LEDColor)CMD_COLOR_INVALID)
            {
              continue;   /*不支持的颜色，跳过这一对*/
            }

            LEDMessage *message = pvPortMalloc(sizeof(LEDMessage));
            if(message != NULL)
            {
              message->color = color;
              message->state = (command[i + 1U] != 0U) ? LEDState_On : LEDState_Off;
              /*0 tick 投递，队列满就释放，不漏内存*/
              if(osMessageQueuePut(LEDQueueHandle, &message, 0U, 0U) != osOK)
              {
                vPortFree(message);
              }
            }
          }
        }

        /*帧尾不论校验结果都复位状态机*/
        commandIndex = 0U;
        commandLength = 0U;
      }
    }
  }
}
