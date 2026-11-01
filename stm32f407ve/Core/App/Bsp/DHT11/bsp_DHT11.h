/* bsp_DHT11.h
 * DHT11 引脚、状态码与数据结构声明。
 * 硬件：DATA=PC1，单总线，需外接 4.7k 上拉至 3.3V。
 * 注意：两次读取间隔 ≥1s，当前 SensorTask 周期 2000ms。
 */
#ifndef __BSP_DHT11_H
#define __BSP_DHT11_H

#include "stm32f4xx_hal.h"


/*引脚定义*/
#define DHT11_GPIO_PORT       GPIOC
#define DHT11_GPIO_PIN        GPIO_PIN_1
#define DHT11_GPIO_CLK_EN()   __HAL_RCC_GPIOC_CLK_ENABLE()


/*状态码*/
/* 0=成功; 1=data 为空; 2=等待电平超时; 3=校验和不匹配 */
typedef enum
{
    DHT11_OK            = 0,
    DHT11_ERROR         = 1,
    DHT11_TIMEOUT       = 2,
    DHT11_CHECKSUM_ERR  = 3
} DHT11_Status_t;


/*数据结构*/
typedef struct
{
    float     temperature;   /*温度，单位 °C，本驱动算/调用方读*/
    float     humidity;      /*湿度，单位 %RH，本驱动算/调用方读*/
    uint8_t   is_valid;      /*数据有效标志，0-无效 1-有效，本驱动写/调用方读*/
} DHT11_Data_t;


/*对外接口*/

/**
  * 函    数：DHT11_Init
  * 参    数：无
  * 返 回 值：恒为 DHT11_OK
  */
DHT11_Status_t DHT11_Init(void);

/**
  * 函    数：DHT11_Read
  * 参    数：data  输出，temperature 单位 °C(0~50)，humidity 单位 %RH(20~90)
  * 返 回 值：DHT11_OK=成功，DHT11_ERROR=data 为空，DHT11_TIMEOUT=超时，DHT11_CHECKSUM_ERR=校验错
  */
DHT11_Status_t DHT11_Read(DHT11_Data_t *data);


#endif /* __BSP_DHT11_H */
