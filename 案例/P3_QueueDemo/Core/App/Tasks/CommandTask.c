#include "cmsis_os2.h"
#include "usart.h"
#include "Types/LEDType.h"
#include "FreeRTOS.h"

void StartCommandTask(void *argument) {
  UART2_Receive_Start();
  uint8_t receive;
  uint8_t command[30];
  uint8_t commandIndex = 0;  // 命令索引
  uint8_t commandLength = 0; // 命令长度
  for (;;) {
    osMessageQueueGet(CommandQueueHandle, &receive, 0, osWaitForever);
    if (commandIndex == 0) {               // 如果命令索引为0，则寻找包头
      if (receive == 0xAA) {               // 找到包头
        command[commandIndex++] = receive; // 命令索引加1,代表已经找到包头
      }
    } else if (commandIndex == 1) { // 如果命令索引为1，则receive为命令长度
      if (receive < 4 || receive > sizeof(command)) {
        commandIndex = 0; // 重置命令索引
        continue;         // 命令长度不合法，重置命令索引，继续寻找包头
      }
      commandLength = receive;
      command[commandIndex++] = receive; // 命令索引加1,代表已经找到命令长度
    } else {
      command[commandIndex++] = receive;
      if (commandIndex == commandLength) { // 如果命令索引等于命令长度，则命令接收完成
        uint8_t checksum = 0;              // 校验和
        for (uint8_t i = 0; i < commandLength - 1; i++) {
          checksum += command[i];
        }
        if (checksum == command[commandLength - 1]) { // 如果校验和等于命令长度，则命令接收完成
          for (uint8_t i = 2; i < commandLength - 2; i+= 2) {
            LEDMessage* message = pvPortMalloc(sizeof(LEDMessage));
            message->color = command[i] - 1;
            message->state = command[i + 1];
            osMessageQueuePut(LEDQueueHandle, &message, 0, osWaitForever);
          }
        }
        commandIndex = 0; // 重置命令索引
        commandLength = 0; // 重置命令长度
      }

    }
  }
}