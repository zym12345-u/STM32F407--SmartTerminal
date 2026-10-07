#ifndef  __BSP_XPT2046_H
#define  __BSP_XPT2046_H
/***********************************************************************************************************************************
 *【出品】  魔女开发板团队    https://demoboard.taobao.com
 ***********************************************************************************************************************************
 *【文件名称】 bsp_XPT2046.h
 *
 *【文件功能】 XPT2046 电阻触摸芯片驱动的头文件:包含引脚定义、相关全局结构体、
 *             全部对外函数声明,以及 XPT2046 的初始化说明、移植说明、使用说明。
 *
 *【硬件平台】 STM32F407 + Keil5
 *
 *【移植说明】 1- 本驱动由 LCD_ILI9341 + W25Q128 + XPT2046 综合示例工程中裁剪而来;
 *             2- 在 Keil > Option > C/C++ > Include Paths 中添加本文件所在目录;
 *             3- 在 Keil 工程中把 bsp_LCD_ILI9341.c、bsp_XPT2046.c 加入编译
 *               (原例程的 bsp_W25Q128.c 已在本工程裁剪掉,校准系数改为固定值);
 *             4- 使用方法: 在需要调用触摸函数的文件中 #include "bsp_XPT2046.h"。
 *
 *【CubeMX】   本驱动不需要在 CubeMX 中配置任何外设,所需 GPIO 均在驱动内部初始化,
 *             也不要在 CubeMX 中把这些引脚改作他用。
 *
 *【使用说明】 1- 电阻触摸屏用手指直接按动时,普通 IO 口通过 XPT2046 的 INT 引脚
 *               即可检测到按下(本工程采用查询方式读取 INT/IRQ 电平);
 *             2- 普通按动即可可靠响应,无需指甲或触摸笔;若戴手套或触摸屏表面有
 *               水/污渍可能导致检测异常,属电阻屏正常现象;
 *             3- 电阻屏的缺点是不支持多点触控,优点是成本低、驱动简单、资源占用少,
 *               很适合 LVGL 这类按钮/滑块控件;
 *             4- 裸机或 RTOS(LVGL)下的调用示例:
 *                  初始化: XPT2046_Init(xLCD.width, xLCD.height, xLCD.dir);  // 宽、高、显示方向
 *                  查按下: XPT2046_IsPressed();   // 读 IRQ 电平判断有无触摸,0-未按 1-按下(带消抖)
 *                  读坐标: XPT2046_GetX();        // 读触摸点横坐标 X,返回值 uint16_t
 *                         XPT2046_GetY();        // 读触摸点纵坐标 Y,返回值 uint16_t
 *             5- 如果触摸位置与画面有偏差,调用 XPT2046_ReCalibration(),
 *               按屏幕提示依次点击十字光标即可重新校准;
 *             6- 原例程依赖 bsp_W25Q128.h 用于掉电保存校准数据;本工程已裁剪外部
 *               Flash,校准系数使用固定默认值(见 bsp_XPT2046.c)。
 *
 *【版权声明】 版权归魔女开发板所有,仅供学习使用。
 *             https://demoboard.taobao.com

************************************************************************************************************************************/
#include "bsp_lcd_ili9341.h"
#include "stdio.h"


#ifdef USE_HAL_DRIVER                                        // HAL 库版本
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // 标准库版本
#include "stm32f4xx.h"
#endif




/*****************************************************************************
 ** 引脚定义
 ** 注意:触摸采用 GPIO 模拟 SPI,IRQ 仅用作状态查询,不使用外部中断。
*****************************************************************************/
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



/*****************************************************************************
 ** 全局函数声明
 *****************************************************************************/
// 初始化
void      XPT2046_Init (uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir ); // 初始化触摸(含引脚、校准系数);参数:显示屏宽、高、显示方向(1,2,3,4)
// 触摸校准
uint8_t   XPT2046_ReCalibration(void);                                        // 屏幕十字光标四点校准;触摸坐标不准时运行一次,点完四个十字即完成,可在任何时候调用
// 触摸状态与坐标
uint8_t   XPT2046_IsPressed(void);                                            // 判断触摸屏按下状态;返回:0-松开 1-按下;移植 LVGL 时在 lv_port_indev.c 的 touchpad_is_pressed() 中调用
uint16_t  XPT2046_GetX(void);                                                 // 读取触摸点 X 坐标值;移植 LVGL 时在 lv_port_indev.c 的 touchpad_get_xy() 中赋值给 *x
uint16_t  XPT2046_GetY(void);                                                 // 读取触摸点 Y 坐标值;移植 LVGL 时在 lv_port_indev.c 的 touchpad_get_xy() 中赋值给 *y
// 原例程配套的扩展函数:LVGL 移植中未使用,保留不影响编译
void      XPT2046_Cmd(uint8_t status);                                        // 触摸开关控制(需要时调用);初始化后默认开启;可用于临时关闭触摸、停止响应等场景
void      XPT2046_TouchDown(void);                                            // 按下回调,原综合示例中使用
void      XPT2046_TouchUp(void);                                              // 松开回调,原综合示例中使用




#endif
