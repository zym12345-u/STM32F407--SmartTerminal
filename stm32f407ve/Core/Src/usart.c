/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "cmsis_os.h"

/* CommandQueue 由 freertos.c 中 CubeMX 生成的代码定义/创建(ISR 生产者向其投递字节) */
extern osMessageQueueId_t CommandQueueHandle;

/* ===== USART1 TX DMA 相关 =====
 * DMA2 Stream7 Channel4 = USART1_TX(STM32F407 参考手册 Table 43)。
 * 任务上下文日志统一走 UART_Printf(static 缓冲 + DMA,整帧原子);
 * fputc 仅供 BSP 初始化遗留 printf 使用(互斥锁 + 轮询单字节,不碰 DMA)。 */
DMA_HandleTypeDef hdma_usart1_tx;

/* 发送路径:
 *   - UART_Printf(任务上下文热路径): static 缓冲 + DMA,持锁等整帧发完
 *   - fputc(仅 BSP 初始化等遗留 printf): 互斥锁 + 轮询单字节
 * 两路共用一把互斥锁保证不交错;只有 UART_Printf 用 DMA 和完成信号量 */
static volatile uint8_t  uart1DmaBusy = 0U;          /* DMA 是否正在发送 */

static osSemaphoreId_t uart1TxDoneSem;   /* DMA 发送完成信号量 */
static osMutexId_t     uart1PrintfMutex; /* 多任务 printf 串行化互斥锁 */

/* DMA 初始化(在 MX_USART1_UART_Init 末尾 USER CODE 区调用) */
static void UART1_DMA_Init(void);
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  UART1_DMA_Init();
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
  /* .ioc 未勾选 USART1 全局中断,这里手写使能(命令接收走板载 USB-TTL)。
   * 优先级 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,允许在中断里调队列 API */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* printf 重定向到 USART1(板载 USB-TTL,串口助手 115200 可见)。
 * 工程用 AC5 标准库(未开 MicroLIB),必须关闭半主机:
 * 否则标准库 printf 会执行 BKPT 0xAB 半主机指令,真机上直接 HardFault。
 *
 * 发送路径(任务上下文热路径请用 UART_Printf):
 *   RTOS 运行时 fputc = 互斥锁 + 轮询单字节,仅供 BSP 初始化等遗留 printf;
 *   不使用 DMA,避免其完成回调污染 UART_Printf 的完成信号量。 */
#include <stdio.h>
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { (void)x; }

/* DMA 初始化: USART1_TX -> DMA2 Stream7 Channel4 */
static void UART1_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_usart1_tx.Instance                 = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel             = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  (void)HAL_DMA_Init(&hdma_usart1_tx);

  /* 把 DMA 句柄挂到 huart1, HAL_UART_Transmit_DMA 会用到 */
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  /* DMA 中断优先级 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
   * 允许在 ISR 里调用 osSemaphoreRelease */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

  /* 创建信号量(初值 0,DMA 完成时 release)和互斥锁 */
  if (uart1TxDoneSem == NULL)
    uart1TxDoneSem = osSemaphoreNew(1U, 0U, NULL);
  if (uart1PrintfMutex == NULL)
    uart1PrintfMutex = osMutexNew(NULL);
}

void _ttywrch(int ch)
{
  /* 转发到 fputc 的 DMA 发送路径 */
  (void)fputc(ch, &__stdout);
}

int fputc(int ch, FILE *f)
{
  (void)f;
  uint8_t c = (uint8_t)ch;

  /* RTOS 未启动(main 初始化阶段):单线程,直接轮询发送 */
  if (osKernelGetState() != osKernelRunning ||
      uart1PrintfMutex == NULL || uart1TxDoneSem == NULL)
  {
    HAL_UART_Transmit(&huart1, &c, 1U, 0xFFFFU);
    return ch;
  }

  /* RTOS 已运行:运行时热路径统一走 UART_Printf(DMA)。fputc 仅为 BSP 初始化
   * 等遗留 printf 服务,这里用互斥锁 + 轮询单字节发送,刻意不碰 DMA/busy/信号量:
   * 否则 fputc 的 DMA 完成回调会往完成信号量里多放一个计数,导致后续
   * UART_Printf 的"等本次 DMA 完成"提前返回,在 DMA 还在读缓冲时就重写了
   * 共享发送缓冲,线上出现整块重复/字节撕裂 */
  (void)osMutexAcquire(uart1PrintfMutex, osWaitForever);
  (void)HAL_UART_Transmit(&huart1, &c, 1U, 0xFFFFU);
  (void)osMutexRelease(uart1PrintfMutex);
  return ch;
}

/* DMA 发送完成回调: 清标志 + 释放信号量唤醒等待整帧发完的 UART_Printf */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1DmaBusy = 0U;
    if (uart1TxDoneSem != NULL)
      (void)osSemaphoreRelease(uart1TxDoneSem);
  }
}

/* ===== 整消息原子 printf(任务上下文使用) =====
 * fputc 路径按字节加锁、每个 '\n' 放锁,一次含多行的 printf(如 vTaskList 报告)
 * 会被其他任务的 printf 从字节/缓冲级别穿插。UART_Printf 把"格式化→启动DMA→
 * 等DMA发完"全程放在同一把互斥锁内,保证整条消息在线上连续不被打断。
 * 格式化缓冲为 static(同一时刻只有锁持有者使用),不占调用者栈。 */
#include <stdarg.h>
#include <string.h>
#define UART1_PRINTF_BUF_SIZE  640U
static char uart1PrintfBuf[UART1_PRINTF_BUF_SIZE];

/* ===== 诊断成帧(临时):每条消息加 <B序号>...<E校验> 头尾 =====
 * 曾用序号+XOR 给线上每一帧盖戳,实测 110 帧序号连续、校验全对,
 * 证明串口助手中的"交错/重复"是 PC 录制侧重排,单片机发送无损。
 * 默认关闭;需要再次取证时改为 1。 */
#define UART_DIAG_FRAME         0
#if UART_DIAG_FRAME
#define UART1_WIRE_BUF_SIZE     (UART1_PRINTF_BUF_SIZE + 16U)
static uint8_t  uart1WireBuf[UART1_WIRE_BUF_SIZE];
static uint32_t uart1MsgSeq = 0U;
#endif

void UART_Printf(const char *fmt, ...)
{
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = vsnprintf(uart1PrintfBuf, sizeof(uart1PrintfBuf), fmt, ap);
  va_end(ap);

  if (len <= 0)
    return;
  if (len >= (int)sizeof(uart1PrintfBuf))
    len = (int)sizeof(uart1PrintfBuf) - 1;

  /* RTOS 未启动(main 初始化阶段):单线程,直接轮询发送 */
  if (osKernelGetState() != osKernelRunning ||
      uart1PrintfMutex == NULL || uart1TxDoneSem == NULL)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart1PrintfBuf, (uint16_t)len, 0xFFFFU);
    return;
  }

  /* 默认发裸载荷;诊断成帧在互斥锁内组装(uart1WireBuf 也是静态共享资源,
   * 绝不能在锁外组装,否则会被高优先级任务在取锁前重写) */
  uint8_t *txData = (uint8_t *)uart1PrintfBuf;
  uint16_t txLen  = (uint16_t)len;

  (void)osMutexAcquire(uart1PrintfMutex, osWaitForever);

#if UART_DIAG_FRAME
  {
    uint8_t xor = 0U;
    int i;
    for (i = 0; i < len; i++)
      xor ^= (uint8_t)uart1PrintfBuf[i];
    int h = snprintf((char *)uart1WireBuf, 8U, "<B%04lu>",
                     (unsigned long)(uart1MsgSeq & 0xFFFFU));
    int t = snprintf((char *)uart1WireBuf + h + len, 8U, "<E%02X>",
                     (unsigned)xor);
    memcpy(&uart1WireBuf[h], uart1PrintfBuf, (size_t)len);
    txData = uart1WireBuf;
    txLen  = (uint16_t)(h + len + t);
    uart1MsgSeq++;
  }
#endif

  /* 防御:排空可能遗留的完成信号量计数(历史 fputc DMA 路径曾留下多余计数,
   * 会让下面的等待提前返回),确保随后的 acquire 等到的一定是本次 DMA 的 TC */
  while (osSemaphoreAcquire(uart1TxDoneSem, 0U) == osOK) { ; }

  /* 等上一条 DMA 真正发完(正常情况下 UART_Printf 返回时已等完,busy 恒为 0) */
  if (uart1DmaBusy)
    (void)osSemaphoreAcquire(uart1TxDoneSem, osWaitForever);

  uart1DmaBusy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1, txData, txLen) != HAL_OK)
  {
    uart1DmaBusy = 0U;
    (void)osMutexRelease(uart1PrintfMutex);
    return;
  }

  /* 等发送真正结束。信号量只作"完成唤醒",最终以硬件为准:DMA 剩余计数为 0
   * 且 UART 移位完成(TC)才放行——即使信号量存在任何多余计数,也绝不可能在
   * DMA 还读 uart1PrintfBuf 时提前返回去复用该缓冲(此前线上撕裂的根因) */
  for (;;)
  {
    (void)osSemaphoreAcquire(uart1TxDoneSem, osWaitForever);
    if ((__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0U) &&
        (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != RESET))
    {
      break;
    }
  }
  uart1DmaBusy = 0U;

  (void)osMutexRelease(uart1PrintfMutex);
}

/* 单字节中断接收(参考 P3_QueueDemo/usart.c):
 * 每收到 1 字节, 在接收完成回调里分发, 然后立即重新挂起下一次接收。
 * USART1 = 板载 USB-TTL(PA9/PA10): 字节投递 CommandQueue(命令任务消费);
 * USART2 = PA2/PA3: 已改作 RS485 物理层, 字节交给 RS485 环形缓冲
 *          (bsp_RS485.c), 不再进 CommandQueue; 命令任务只启动 USART1。 */
static uint8_t rxData1;
static uint8_t rxData2;

extern void RS485_RxByteISR(uint8_t byte);   /* bsp_RS485.c: RS485 接收环形缓冲 */

void UART1_Receive_Start(void)
{
  HAL_UART_Receive_IT(&huart1, &rxData1, 1);
}

void UART2_Receive_Start(void)
{
  HAL_UART_Receive_IT(&huart2, &rxData2, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    (void)osMessageQueuePut(CommandQueueHandle, &rxData1, 0U, 0U);
    HAL_UART_Receive_IT(&huart1, &rxData1, 1);
  }
  else if(huart->Instance == USART2)
  {
    /* USART2 = RS485 物理层: 字节进 RS485 环形缓冲(重新挂接收在 ISR 内完成) */
    RS485_RxByteISR(rxData2);
  }
}
/* USER CODE END 1 */

