/* USER CODE BEGIN Header */
/**
  * @file    CommandTask.c
  * @brief   串口命令解析任务:从 CommandQueue 逐字节取出命令串口收到的数据,
  *          按协议组帧、校验,校验通过后把每对 {颜色, 状态} 打包成 LEDMessage
  *          指针投递到 LEDQueue,由 LEDTask 点灯。
  *          (协议参考 P3_QueueDemo/Core/App/Tasks/CommandTask.c)
  *
  * 帧格式:  [0xAA][LEN][COLOR1][STATE1]...[CHECKSUM]
  *            包头   整帧长度(含包头和校验,4~30)   负载(成对出现)  校验和
  *          校验和 = 前面 LEN-1 个字节的累加和(uint8 溢出截断)
  *          COLOR: 1=红 2=绿 3=蓝(案例枚举+1);本板只有绿灯/蓝灯,
  *                 2 -> LEDColor_Green,3 -> LEDColor_Blue,1(红)忽略
  *          STATE: 0=灭 1=亮
  */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "LEDType.h"
#include "PowerMgr.h"

/* 队列句柄由 freertos.c 中 CubeMX 生成的代码定义/创建 */
extern osMessageQueueId_t CommandQueueHandle;
extern osMessageQueueId_t LEDQueueHandle;

/* 协议常量 */
#define CMD_FRAME_HEAD      0xAAU   /* 包头 */
#define CMD_FRAME_MIN_LEN   4U      /* 最小整帧长度:包头+长度+校验 = 3,带1对负载为 5,案例下限取 4 */
#define CMD_FRAME_BUF_SIZE  30U     /* 接收缓冲区/整帧最大长度 */

/* 协议颜色码 -> 本工程 LEDColor 映射;不支持的颜色返回 0xFF */
#define CMD_COLOR_RED       1U
#define CMD_COLOR_GREEN     2U
#define CMD_COLOR_BLUE      3U
#define CMD_COLOR_INVALID   0xFFU

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
      color = (LEDColor)CMD_COLOR_INVALID;   /* 红色等本板不支持的颜色:忽略 */
      break;
  }
  return color;
}

void StartCommandTask(void *argument)
{
  (void)argument;

  uint8_t receive = 0U;
  uint8_t command[CMD_FRAME_BUF_SIZE];
  uint8_t commandIndex = 0U;    /* 已存入缓冲区的字节数 */
  uint8_t commandLength = 0U;   /* 本帧总长度 */

  /* 任务一启动就挂起命令串口中断接收(队列已由 MX_FREERTOS_Init 先行创建)。
   * 当前命令口 = USART1(板载 USB-TTL,一根 USB 线直连即可测试);
   * 若以后改用外接模块的 USART2,把下面一行换成 UART2_Receive_Start() 即可。 */
  UART1_Receive_Start();

  for(;;)
  {
    /* 没有命令字节时永久阻塞,不占用 CPU */
    if(osMessageQueueGet(CommandQueueHandle, &receive, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    if(commandIndex == 0U)
    {
      /* 状态 0:寻找包头 */
      if(receive == CMD_FRAME_HEAD)
      {
        command[commandIndex++] = receive;
      }
    }
    else if(commandIndex == 1U)
    {
      /* 状态 1:第二字节是整帧长度,合法性检查 */
      if((receive < CMD_FRAME_MIN_LEN) || (receive > (uint8_t)sizeof(command)))
      {
        commandIndex = 0U;   /* 长度不合法,丢弃本帧,重新找包头 */
        continue;
      }
      commandLength = receive;
      command[commandIndex++] = receive;
    }
    else
    {
      /* 状态 2+:依次收集负载和校验字节 */
      command[commandIndex++] = receive;

      if(commandIndex == commandLength)
      {
        /* 整帧接收完成:计算校验和 = 前 LEN-1 字节之和 */
        uint8_t checksum = 0U;
        for(uint8_t i = 0U; i < (uint8_t)(commandLength - 1U); i++)
        {
          checksum += command[i];
        }

        if(checksum == command[commandLength - 1U])
        {
          /* 收到合法串口命令:视为用户活动,刷新息屏计时 */
          PowerMgr_Feed();
          /* 校验通过:负载从下标 2 开始,到校验位前结束,每 2 字节一对 */
          for(uint8_t i = 2U; i < (uint8_t)(commandLength - 2U); i += 2U)
          {
            LEDColor color = Command_MapColor(command[i]);
            if(color == (LEDColor)CMD_COLOR_INVALID)
            {
              continue;   /* 不支持的颜色(如红色),跳过这一对 */
            }

            LEDMessage *message = pvPortMalloc(sizeof(LEDMessage));
            if(message != NULL)
            {
              message->color = color;
              message->state = (command[i + 1U] != 0U) ? LEDState_On : LEDState_Off;
              /* 非阻塞投递;队列满时释放内存,避免泄漏 */
              if(osMessageQueuePut(LEDQueueHandle, &message, 0U, 0U) != osOK)
              {
                vPortFree(message);
              }
            }
          }
        }

        /* 无论校验是否通过,帧结束后复位状态机等待下一帧 */
        commandIndex = 0U;
        commandLength = 0U;
      }
    }
  }
}
