/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for BtnTask */
osThreadId_t BtnTaskHandle;
const osThreadAttr_t BtnTask_attributes = {
  .name = "BtnTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LaunchTask */
osThreadId_t LaunchTaskHandle;
const osThreadAttr_t LaunchTask_attributes = {
  .name = "LaunchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialSem */
osSemaphoreId_t SerialSemHandle;
const osSemaphoreAttr_t SerialSem_attributes = {
  .name = "SerialSem"
};
/* Definitions for LaunchSem */
osSemaphoreId_t LaunchSemHandle;
const osSemaphoreAttr_t LaunchSem_attributes = {
  .name = "LaunchSem"
};
/* Definitions for CryptoICSem */
osSemaphoreId_t CryptoICSemHandle;
const osSemaphoreAttr_t CryptoICSem_attributes = {
  .name = "CryptoICSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBtnTask(void *argument);
void StartLaunchTask(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SerialSem */
  SerialSemHandle = osSemaphoreNew(1, 1, &SerialSem_attributes);

  /* creation of LaunchSem */
  LaunchSemHandle = osSemaphoreNew(10, 0, &LaunchSem_attributes);

  /* creation of CryptoICSem */
  CryptoICSemHandle = osSemaphoreNew(3, 3, &CryptoICSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BtnTask */
  BtnTaskHandle = osThreadNew(StartBtnTask, NULL, &BtnTask_attributes);

  /* creation of LaunchTask */
  LaunchTaskHandle = osThreadNew(StartLaunchTask, NULL, &LaunchTask_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBtnTask */
/**
  * @brief  Function implementing the BtnTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBtnTask */
void StartBtnTask(void *argument)
{
  /* USER CODE BEGIN StartBtnTask */
  int count = 0;
  char msg[50];
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET) {
      osDelay(10);
      if (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET) {
        count++;
        sprintf(msg, "按键 %d 次", count);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

        osSemaphoreRelease(LaunchSemHandle);

        while (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET) {}
      }
    }
    osDelay(10);
  }
  /* USER CODE END StartBtnTask */
}

/* USER CODE BEGIN Header_StartLaunchTask */
/**
* @brief Function implementing the LaunchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLaunchTask */
void StartLaunchTask(void *argument)
{
  /* USER CODE BEGIN StartLaunchTask */
  int count = 0;
  char msg[50];
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(LaunchSemHandle, osWaitForever);

    count++;
    sprintf(msg, "开始发射第 %d 枚导弹", count);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    osSemaphoreAcquire(CryptoICSemHandle, osWaitForever);

    sprintf(msg, "导弹发射任务获取到加密IC");
    osSemaphoreAcquire(SerialSemHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    osSemaphoreRelease(SerialSemHandle);

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    osDelay(2000);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

    osSemaphoreRelease(CryptoICSemHandle);
  }
  /* USER CODE END StartLaunchTask */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint32_t count = 0;
  char msg[50];
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    osSemaphoreAcquire(CryptoICSemHandle, osWaitForever);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务03获取到加密IC ( %ld -> %ld )",count + 1, count);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    osDelay(10000);

    osSemaphoreRelease(CryptoICSemHandle);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务03释放到加密IC ( %ld -> %ld )",count - 1, count);
    osSemaphoreAcquire(SerialSemHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    osSemaphoreRelease(SerialSemHandle);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint32_t count = 0;
  char msg[50];
  /* Infinite loop */
  for(;;)
  {
    osDelay(1500);
    osSemaphoreAcquire(CryptoICSemHandle, osWaitForever);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务04获取到加密IC ( %ld -> %ld )",count + 1, count);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    osDelay(10000);

    osSemaphoreRelease(CryptoICSemHandle);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务04释放到加密IC ( %ld -> %ld )",count - 1, count);
    osSemaphoreAcquire(SerialSemHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    osSemaphoreRelease(SerialSemHandle);

  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  uint32_t count = 0;
  char msg[50];
  /* Infinite loop */
  for(;;)
  {
    osDelay(2000);
    osSemaphoreAcquire(CryptoICSemHandle, osWaitForever);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务05获取到加密IC ( %ld -> %ld )",count + 1, count);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    osDelay(10000);

    osSemaphoreRelease(CryptoICSemHandle);

    count = osSemaphoreGetCount(CryptoICSemHandle);
    sprintf(msg, "任务05释放到加密IC ( %ld -> %ld )",count - 1, count);
    osSemaphoreAcquire(SerialSemHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    osSemaphoreRelease(SerialSemHandle);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

