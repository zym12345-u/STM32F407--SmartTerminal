/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
/* USART1 TX DMA 句柄(在 usart.c 中定义,stm32f4xx_it.c 的 DMA2_Stream7_IRQHandler 需要引用) */
extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/* 启动单字节中断接收(字节由回调投递到 CommandQueue) */
void UART1_Receive_Start(void);  /* USART1 = 板载 USB-TTL(PA9/PA10),当前命令口 */
void UART2_Receive_Start(void);  /* USART2 = PA2/PA3,需外接 USB-TTL 模块 */

/* 整消息原子打印(多任务日志不会互相穿插);任务上下文使用,
 * 内部完成 DMA 发送后才返回。中断/临界区内禁止调用 */
void UART_Printf(const char *fmt, ...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

