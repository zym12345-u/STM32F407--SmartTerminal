/**
  ******************************************************************************
  * @file    bsp_DHT11.h
  * @brief   DHT11 温湿度传感器驱动(单总线 GPIO 接口)
  *
  *          硬件连接:
  *            VCC  -> 3.3V
  *            GND  -> GND
  *            DATA -> PC1 (需外接 4.7k 上拉电阻至 3.3V)
  *
  *          说明:
  *            1. 单总线协议,时序要求微秒级精度,使用 DWT 周期计数器实现延时
  *               与超时检测,不依赖 SysTick/FreeRTOS 时基。
  *            2. DHT11 采样间隔建议 ≥ 1 秒,过快读取会导致数据异常。
  *            3. 温湿度数据:温度/湿度各 8 位整数 + 8 位小数(DHT11 小数部分恒为 0),
  *               最后 1 字节校验 = 湿度整数+湿度小数+温度整数+温度小数。
  ******************************************************************************
  */
#ifndef __BSP_DHT11_H
#define __BSP_DHT11_H

#include "stm32f4xx_hal.h"


/* ====================== 引脚定义 ====================== */
#define DHT11_GPIO_PORT       GPIOC
#define DHT11_GPIO_PIN        GPIO_PIN_1
#define DHT11_GPIO_CLK_EN()   __HAL_RCC_GPIOC_CLK_ENABLE()


/* ====================== 状态码 ====================== */
typedef enum
{
    DHT11_OK            = 0,
    DHT11_ERROR         = 1,
    DHT11_TIMEOUT       = 2,
    DHT11_CHECKSUM_ERR  = 3
} DHT11_Status_t;


/* ====================== 数据结构 ====================== */
typedef struct
{
    float     temperature;   /* 温度,单位 °C  */
    float     humidity;      /* 湿度,单位 %RH */
    uint8_t   is_valid;      /* 数据是否有效:0-无效 1-有效 */
} DHT11_Data_t;


/* ====================== 对外接口 ====================== */

/**
  * @brief  初始化 DHT11(配置 DATA 引脚为输入上拉)
  * @retval DHT11_OK-成功
  */
DHT11_Status_t DHT11_Init(void);

/**
  * @brief  读取一次温湿度数据
  * @param  data  输出参数,存放温湿度数据
  * @retval DHT11_OK-成功;其它-失败
  * @note   调用间隔建议 ≥ 1 秒
  */
DHT11_Status_t DHT11_Read(DHT11_Data_t *data);


#endif /* __BSP_DHT11_H */
