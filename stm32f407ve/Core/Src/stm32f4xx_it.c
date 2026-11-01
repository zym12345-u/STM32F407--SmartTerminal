/* USER CODE BEGIN Header */
/* stm32f4xx_it.c
 * 异常与中断入口：内核异常，以及 EXTI/TIM4/USART3/CAN1_RX0 等外设中断。
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"        /*hcan1*/
#include "usart.h"      /*huart1、hdma_usart1_tx*/
#include "bsp_RS485.h"  /*RS485_IdleISR*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart1;   /*手写 USART1 中断用*/
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* 故障留痕：轮询直发，不依赖 tick/DMA/RTOS */
  {
    static const char msg[] = "\r\n[FATAL] HardFault!\r\n";
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 0xFFFFU);
  }
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  {
    static const char msg[] = "\r\n[FATAL] MemManage!\r\n";
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 0xFFFFU);
  }
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  {
    static const char msg[] = "\r\n[FATAL] BusFault!\r\n";
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 0xFFFFU);
  }
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  {
    static const char msg[] = "\r\n[FATAL] UsageFault!\r\n";
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 0xFFFFU);
  }
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  * @note  USART3 = RS485 物理层,走 HAL 回调(HAL_UART_RxCpltCallback -> RS485_RxByteISR)。
  *        USART2 已让给 ESP8266,其中断入口由 bsp_ESP8266.c 的 USART2_IRQHandler 接管。
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  /* HAL 不处理 IDLE 标志，必须在这里先处理 */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
  {
    RS485_IdleISR();
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* USART1 中断入口（串口1 的收发），转给 HAL 统一处理。 */
/* .ioc 未勾选 USART1 NVIC，此入口手写 */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * 功    能：DMA2 Stream7 中断入口 —— 这条流是 USART1 的发送 DMA，转给 HAL 处理
  * 参    数：无
  * 返 回 值：无
  */
void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);   /*Stream7 = USART1_TX，中断优先级 5*/
}

/**
  * 功    能：CAN1 接收 FIFO0 中断入口，转给 HAL —— HAL 里再回调到 bsp_CAN 的收帧函数
  * 参    数：无
  * 返 回 值：无
  */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);   /*优先级必须 5：回调里要调 FromISR，0/0 会触发断言*/
}
/* USER CODE END 1 */
