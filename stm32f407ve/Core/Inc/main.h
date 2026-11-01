/* USER CODE BEGIN Header */
/* main.h
 * main.c 的公共声明：引脚宏、Error_Handler 原型与 HAL 头文件。
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_1_EXTI_IRQn EXTI0_IRQn
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOA
#define KEY_2_EXTI_IRQn EXTI1_IRQn
#define led_green_Pin GPIO_PIN_5
#define led_green_GPIO_Port GPIOC
#define led_blue_Pin GPIO_PIN_2
#define led_blue_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
