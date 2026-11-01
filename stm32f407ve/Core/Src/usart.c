/* USER CODE BEGIN Header */
/* usart.c
 * USART1/USART2/USART3 的 HAL 初始化，以及 USART1 的 DMA 发送和 printf 重定向。
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "bsp_RS485.h"   /*s_rxByte 与 RS485_RxByteISR*/

extern osMessageQueueId_t CommandQueueHandle;   /*ISR 向队列投递命令字节*/

DMA_HandleTypeDef hdma_usart1_tx;   /*USART1_TX 固定走 DMA2 Stream7 Channel4*/

static volatile uint8_t  uart1DmaBusy = 0U;   /*DMA 正在发送*/

/* DMA 完成信号量 + 两路发送共用的锁 */
static osSemaphoreId_t uart1TxDoneSem;
static osMutexId_t     uart1PrintfMutex;

static void UART1_DMA_Init(void);
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

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
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);   /*优先级 5 才能在 ISR 里调队列 API*/
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
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
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
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* AC5 不关半主机，printf 里的 BKPT 0xAB 在真机上直接 HardFault */
#include <stdio.h>
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { (void)x; }

/* 把 USART1 的发送改成 DMA 方式（DMA2 的 Stream7），并建好「发送完成」信号量。
 * printf 走这条路才不用 CPU 一个字节一个字节地等。 */
static void UART1_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_usart1_tx.Instance                 = DMA2_Stream7;   /*Channel4 = USART1_TX*/
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

  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5U, 0U);   /*5 = 最大 syscall 优先级，ISR 里才能 release 信号量*/
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

  /* 信号量初值 0，DMA 完成时 release */
  if (uart1TxDoneSem == NULL)
    uart1TxDoneSem = osSemaphoreNew(1U, 0U, NULL);
  if (uart1PrintfMutex == NULL)
    uart1PrintfMutex = osMutexNew(NULL);
}

/* ARM 编译器半主机（semihosting）模式要求的回调，用来吞掉一个字符。
 * 没有它会一直等调试器，脱机运行就卡死。 */
void _ttywrch(int ch)
{
  (void)fputc(ch, &__stdout);
}

/* printf 的重定向入口：标准库每输出一个字符就调它一次，这里直接丢给 UART1。
 * 相当于把 printf 接到串口上。f 参数用不上，原样把 ch 返回去。 */
int fputc(int ch, FILE *f)
{
  (void)f;
  uint8_t c = (uint8_t)ch;

  /* 内核未启动时无并发，直接轮询 */
  if (osKernelGetState() != osKernelRunning ||
      uart1PrintfMutex == NULL || uart1TxDoneSem == NULL)
  {
    HAL_UART_Transmit(&huart1, &c, 1U, 0xFFFFU);
    return ch;
  }

  /* fputc 不能走 DMA：多放一个完成计数会让 UART_Printf 提前复用缓冲，线上出现重复字节 */
  (void)osMutexAcquire(uart1PrintfMutex, osWaitForever);
  (void)HAL_UART_Transmit(&huart1, &c, 1U, 0xFFFFU);
  (void)osMutexRelease(uart1PrintfMutex);
  return ch;
}

/* 一次 DMA 发送真正结束的回调：释放一个信号量，
 * 告诉还在等的任务「发完了，缓冲可以复用了」。只处理 USART1。 */
/* 在 DMA2_Stream7 中断里执行，属中断上下文 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1DmaBusy = 0U;
    if (uart1TxDoneSem != NULL)
      (void)osSemaphoreRelease(uart1TxDoneSem);
  }
}

/* 整条消息原子输出：格式化、启动 DMA、等发完都持有同一把锁，避免多任务日志穿插 */
#include <stdarg.h>
#include <string.h>
#define UART1_PRINTF_BUF_SIZE  640U
static char uart1PrintfBuf[UART1_PRINTF_BUF_SIZE];

/* 曾用它确认串口线上无交错，PC 录制侧重排所致，正常保持 0 */
#define UART_DIAG_FRAME         0
#if UART_DIAG_FRAME
#define UART1_WIRE_BUF_SIZE     (UART1_PRINTF_BUF_SIZE + 16U)
static uint8_t  uart1WireBuf[UART1_WIRE_BUF_SIZE];
static uint32_t uart1MsgSeq = 0U;
#endif

/* 线程安全的串口打印：把一行格式化好 → 取互斥锁 → DMA 发出去 → 等真正发完再放锁。
 * 整行不会被别的任务插进来切断；代价是调用期间会阻塞到发送完成。
 * RTOS 还没启动时（main 初始化阶段）走轮询发送，那条路径不取锁。 */
void UART_Printf(const char *fmt, ...)
{
  va_list ap;
  int len;

  /* 内核未启动时不能取锁，句柄为空会直接挂死 */
  if (osKernelGetState() != osKernelRunning ||
      uart1PrintfMutex == NULL || uart1TxDoneSem == NULL)
  {
    va_start(ap, fmt);
    len = vsnprintf(uart1PrintfBuf, sizeof(uart1PrintfBuf), fmt, ap);
    va_end(ap);

    if (len <= 0)
      return;
    if (len >= (int)sizeof(uart1PrintfBuf))
      len = (int)sizeof(uart1PrintfBuf) - 1;

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)uart1PrintfBuf, (uint16_t)len, 0xFFFFU);
    return;
  }

  /* 必须先取锁再格式化：静态缓冲在锁外会被高优先级任务覆盖 */
  (void)osMutexAcquire(uart1PrintfMutex, osWaitForever);

  uint8_t *txData = (uint8_t *)uart1PrintfBuf;
  uint16_t txLen  = 0U;

  va_start(ap, fmt);
  len = vsnprintf(uart1PrintfBuf, sizeof(uart1PrintfBuf), fmt, ap);
  va_end(ap);

  /* 锁内不提前 return，漏放锁会让其它任务永久阻塞 */
  if (len > 0)
  {
    if (len >= (int)sizeof(uart1PrintfBuf))
      len = (int)sizeof(uart1PrintfBuf) - 1;
    txLen = (uint16_t)len;

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

    /* 排空遗留计数，确保等到的一定是本次 DMA 的 TC */
    while (osSemaphoreAcquire(uart1TxDoneSem, 0U) == osOK) { ; }

    /* 等上一条 DMA 真正发完 */
    if (uart1DmaBusy)
      (void)osSemaphoreAcquire(uart1TxDoneSem, osWaitForever);

    uart1DmaBusy = 1U;
    if (HAL_UART_Transmit_DMA(&huart1, txData, txLen) == HAL_OK)
    {
      /* 以 DMA 计数清零 + TC 为准，信号量多计数也不会提前复用缓冲 */
      for (;;)
      {
        (void)osSemaphoreAcquire(uart1TxDoneSem, osWaitForever);
        if ((__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0U) &&
            (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != RESET))
        {
          break;
        }
      }
    }
    uart1DmaBusy = 0U;
  }

  (void)osMutexRelease(uart1PrintfMutex);
}

/* 单字节中断接收：USART1 字节进 CommandQueue，USART3 字节进 RS485 环形缓冲 */
static uint8_t rxData1;   /*USART1 接收缓冲*/
/* USART3 必须用 bsp_RS485.c 的 s_rxByte，另建变量会导致收到的全是 0 */

extern void RS485_RxByteISR(uint8_t byte);   /*RS485 接收环形缓冲*/

/* 开始收 USART1 的数据：挂上接收中断，之后每收到一字节由回调再重新挂上。 */
void UART1_Receive_Start(void)
{
  HAL_UART_Receive_IT(&huart1, &rxData1, 1);
}

/* 开始收 USART3（RS485）的数据，机制同 USART1。 */
void UART3_Receive_Start(void)
{
  HAL_UART_Receive_IT(&huart3, &s_rxByte, 1);
}

/* 收到一字节的回调：把数据投进对应的消息队列，然后立刻重新挂接收。
 * 忘了重挂的话就只收得到这一字节，后面全丢。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    (void)osMessageQueuePut(CommandQueueHandle, &rxData1, 0U, 0U);
    HAL_UART_Receive_IT(&huart1, &rxData1, 1);
  }
  else if(huart->Instance == USART3)
  {
    RS485_RxByteISR(s_rxByte);   /*必须读 s_rxByte，那才是 HAL 写入的缓冲*/
  }
}
/* USER CODE END 1 */

