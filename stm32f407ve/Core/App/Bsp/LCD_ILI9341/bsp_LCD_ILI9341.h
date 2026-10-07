#ifndef __LCD_ILI9341_H
#define __LCD_ILI9341_H
/**==================================================================================================================
 *【文件名称】 bsp_LCD_ILI9341.h
 *【功能描述】 2.8寸显示屏驱动
 *==================================================================================================================
 *【硬件平台】 STM32F407 + KEIL5.27 + 2.8寸显示屏_ILI9341
 *
 *【说    明】 1- 显示驱动,本程序配套外部 2.8寸 ILI9341 屏,34PIN 排针对插;
 *             2- 显示内容可以是文本(显示方式:使用自带转模的英文字模数据、使用 Flash 中的字模数据);
 *             3- 若要使用 Flash 中的字模数据,需要加入 bsp_W25Q128.c 参与编译,并在本文件中放开其头文件包含。
 *
 *【更新记录】 2025-12-01  使用新方式读取 LCD 的 ID,读取高度、读取宽度
 *             2024-07-15  优化模板程序,完善代码注释
 *             2024-01-25  从标准库工程移植到 HAL 库工程,使用寄存器方式实现,两种库均可使用
 *             2023-01-12  添加程序中转模文本显示、相关代码及注释
 *             2022-12-30  新建程序,实现显示屏的图片显示等功能
 *
 *【版权说明】 版权归魔女开发板所有,仅供学习,谢谢
 *             https://demoboard.taobao.com
====================================================================================================================*/
#include "stdlib.h"

#ifdef USE_HAL_DRIVER                         // HAL 库版本
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_STDPERIPH_DRIVER                   // 标准库版本
#include "stm32f4xx.h"
#endif


/* W25Q128外部Flash字库已裁掉(LVGL使用自带字体,不需要外部中文字库) */
//#include "bsp_W25Q128.h"                      // 外部闪存
//#include "bsp_24C02.h"                        // 24C02


/*****************************************************************************
 ** 背光引脚
 **
*****************************************************************************/
#ifdef USE_HAL_DRIVER                         // HAL 库版本
#define LCD_BL_GPIO   GPIOA                   // 背光控制引脚
#define LCD_BL_PIN    GPIO_PIN_15
#endif

#ifdef USE_STDPERIPH_DRIVER                   // 标准库版本
#define LCD_BL_GPIO   GPIOA                   // 背光控制引脚
#define LCD_BL_PIN    GPIO_Pin_15
#endif


#define LCD_WIDTH     240                     // 屏幕宽度(竖屏默认)
#define LCD_HEIGHT    320                     // 屏幕高度(竖屏默认)







/******************************* 常用 RGB565 颜色值 *****************************/
#define      WHITE               0xFFFF       // 白色
#define      BLACK               0x0000       // 黑色
#define      GREY                0xF7DE       // 灰色
#define      GRAY                0X8430       // 灰色
#define      RED                 0xF800       // 红
#define      MAGENTA             0xF81F       // 品红色
#define      GRED                0xFFE0       // 橙红色
#define      BROWN               0XBC40       // 棕色
#define      BRRED               0XFC07       // 棕红色
#define      GREEN               0x07E0       // 绿
#define      CYAN                0x7FFF       // 青色
#define      YELLOW              0xFFE0       // 黄色
#define      LIGHTGREEN          0X841F       // 浅绿色
#define      BLUE                0x001F       // 蓝
#define      GBLUE               0x07FF       // 浅蓝 1
#define      LIGHTBLUE           0X7D7C       // 浅蓝 2
#define      BLUE2               0x051F       // 浅蓝 3
#define      GRAYBLUE            0X5458       // 灰蓝
#define      DARKBLUE            0X01CF       // 深蓝
#define      LGRAY               0XC618       // 浅灰色,偏白灰色
#define      LGRAYBLUE           0XA651       // 浅灰蓝色(中间色调)
#define      LBBLUE              0X2B12       // 浅棕蓝色(选择条目的反色)






/*****************************************************************************
 ** 全局函数声明
*****************************************************************************/
// 初始化
void LCD_Init(void);                                                                                       // 显示屏初始化
void LCD_SetDir(uint8_t dir);                                                                              // 设置显示方向; 0-竖屏 3-横屏 5-反向竖屏 6-反向横屏;注意:使用触摸时每次改方向后都要重新校准

// 主要的基础绘图函数(显示线条、图片、颜色填充等),其他函数都通过对它们的调用来实现
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);                                                // 画点
void LCD_Fill(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY, uint16_t color);             // 对指定区域填色
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor);      // 显示英文字符串(支持英文、数字)

// 扩展1:控制
void LCD_DisplayOn(void);                                                                                  // 开启显示
void LCD_DisplayOff(void);                                                                                 // 关闭显示
uint16_t LCD_GetWidth(void);                                                                               // 获取屏幕宽度(单位:像素)
uint16_t LCD_GetHeight(void);                                                                              // 获取屏幕高度(单位:像素)
uint8_t  LCD_GetDir(void);                                                                                 // 获取屏幕当前显示方向

// 扩展2:线条、绘图
void LCD_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                         // 画线
void LCD_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t _color);                           // 画圆
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);             // 显示图片; 图片需要预先转换成数组数据

// 扩展3:触摸用
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor);                                     // 用于触摸屏校准中画十字光标
uint16_t  LCD_ReadPoint(uint16_t  x, uint16_t  y);

// 扩展4:LVGL 移植
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData);        // 把指定区域的数据(整块图像)输出到屏,供 LVGL 刷新(DMA 搬运)
void LCD_SetFlushDoneCallback(void (*cb)(void));                                                            // 注册 DMA 传输完成回调(内部调用 lv_disp_flush_ready)



#endif
