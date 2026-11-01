#ifndef  __BSP_XPT2046_H
#define  __BSP_XPT2046_H
/* bsp_XPT2046.h
 * XPT2046 触摸驱动头文件：引脚宏与对外函数声明。
 * 硬件：CS=PD13 CLK=PE0 MOSI=PE2 MISO=PE3 IRQ=PE4；引脚宏分 HAL 库与标准库两套写法。
 */
#include "bsp_lcd_ili9341.h"
#include "stdio.h"


#ifdef USE_HAL_DRIVER                                        // HAL 库版本
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // 标准库版本
#include "stm32f4xx.h"
#endif




/*引脚定义：GPIO 模拟 SPI，IRQ 仅查询电平，不使用外部中断*/
#ifdef USE_HAL_DRIVER                                        // HAL 库版本
#define    XPT2046_IRQ_GPIO         GPIOE                    // 触摸手指按下指示信号(不使用外部中断)
#define    XPT2046_IRQ_PIN          GPIO_PIN_4
#define    XPT2046_IRQ_PORT_CLK     RCC_AHB1Periph_GPIOE

#define    XPT2046_CS_GPIO          GPIOD                    // 模拟 SPI_CS
#define    XPT2046_CS_PIN           GPIO_PIN_13
#define    XPT2046_CS_PORT_CLK      RCC_AHB1Periph_GPIOD

#define    XPT2046_CLK_GPIO         GPIOE                    // 模拟 SPI_CLK
#define    XPT2046_CLK_PIN          GPIO_PIN_0
#define    XPT2046_CLK_PORT_CLK     RCC_AHB1Periph_GPIOE

#define    XPT2046_MOSI_GPIO        GPIOE                    // 模拟 SPI_MOSI
#define    XPT2046_MOSI_PIN         GPIO_PIN_2
#define    XPT2046_MOSI_PORT_CLK    RCC_AHB1Periph_GPIOE

#define    XPT2046_MISO_GPIO        GPIOE                    // 模拟 SPI_MISO
#define    XPT2046_MISO_PIN         GPIO_PIN_3
#define    XPT2046_MISO_PORT_CLK    RCC_AHB1Periph_GPIOE
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // 标准库版本
#define    XPT2046_IRQ_GPIO         GPIOE                    // 触摸手指按下指示信号(不使用外部中断)
#define    XPT2046_IRQ_PIN          GPIO_Pin_4
#define    XPT2046_IRQ_PORT_CLK     RCC_AHB1Periph_GPIOE

#define    XPT2046_CS_GPIO          GPIOD                    // 模拟 SPI_CS
#define    XPT2046_CS_PIN           GPIO_Pin_13
#define    XPT2046_CS_PORT_CLK      RCC_AHB1Periph_GPIOD

#define    XPT2046_CLK_GPIO         GPIOE                    // 模拟 SPI_CLK
#define    XPT2046_CLK_PIN          GPIO_Pin_0
#define    XPT2046_CLK_PORT_CLK     RCC_AHB1Periph_GPIOE

#define    XPT2046_MOSI_GPIO        GPIOE                    // 模拟 SPI_MOSI
#define    XPT2046_MOSI_PIN         GPIO_Pin_2
#define    XPT2046_MOSI_PORT_CLK    RCC_AHB1Periph_GPIOE

#define    XPT2046_MISO_GPIO        GPIOE                    // 模拟 SPI_MISO
#define    XPT2046_MISO_PIN         GPIO_Pin_3
#define    XPT2046_MISO_PORT_CLK    RCC_AHB1Periph_GPIOE
#endif



/*对外函数声明*/
void      XPT2046_Init (uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir ); // 初始化引脚与校准系数，dir：0=竖屏 1=横屏
uint8_t   XPT2046_ReCalibration(void);                                        // 四点十字校准，返回 0=成功
uint8_t   XPT2046_IsPressed(void);                                            // 0=松开 1=按下
uint16_t  XPT2046_GetX(void);                                                 // 当前触摸点 X 坐标
uint16_t  XPT2046_GetY(void);                                                 // 当前触摸点 Y 坐标
void      XPT2046_Cmd(uint8_t status);                                        // 触摸开关，初始化后默认开启
void      XPT2046_TouchDown(void);                                            // 按下回调，原示例中使用
void      XPT2046_TouchUp(void);                                              // 松开回调，原示例中使用




#endif
