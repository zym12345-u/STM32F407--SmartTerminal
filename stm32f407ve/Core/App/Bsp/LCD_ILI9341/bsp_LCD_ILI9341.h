#ifndef __LCD_ILI9341_H
#define __LCD_ILI9341_H
/* bsp_LCD_ILI9341.h
 * ILI9341 2.8 寸屏驱动头文件：背光引脚、屏尺寸、RGB565 颜色宏与绘图函数声明。
 * 硬件：FSMC Bank1/NE1，CS=PD7、RS=PD11(FSMC_A16)、D0~D15 接 PD/PE、WR=PD5、RD=PD4，背光 BL=PA15 高电平点亮。
 * 注意：GRAM 原生竖屏 240×320，横屏由 0x36 的 MV 位旋转；改方向后触摸必须重新校准。
 */
#include "stdlib.h"

#ifdef USE_HAL_DRIVER                         // HAL 库版本
#include "stm32f4xx_hal.h"
#endif

#ifdef USE_STDPERIPH_DRIVER                   // 标准库版本
#include "stm32f4xx.h"
#endif


/* W25Q128 外部字库已裁剪，LVGL 用自带字体 */
//#include "bsp_W25Q128.h"                      // 外部闪存
//#include "bsp_24C02.h"                        // 24C02


/*背光引脚*/
#ifdef USE_HAL_DRIVER                         // HAL 库版本
#define LCD_BL_GPIO   GPIOA                   // 背光控制引脚
#define LCD_BL_PIN    GPIO_PIN_15
#endif

#ifdef USE_STDPERIPH_DRIVER                   // 标准库版本
#define LCD_BL_GPIO   GPIOA                   // 背光控制引脚
#define LCD_BL_PIN    GPIO_Pin_15
#endif


#define LCD_WIDTH     240                     // 竖屏宽度，面板原生列数不可改
#define LCD_HEIGHT    320                     // 竖屏高度，横屏时与宽度由 LCD_SetDir 互换







/*常用 RGB565 颜色值*/
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






/*全局函数声明*/
// 初始化
void LCD_Init(void);                                                                                       // 显示屏初始化
void LCD_SetDir(uint8_t dir);                                                                              // 0=竖屏 1=横屏；改方向后触摸需重新校准

// 基础绘图：其余函数都由这几个拼出来
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);                                                // 画点
void LCD_Fill(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY, uint16_t color);             // 对指定区域填色
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor);      // 显示 ASCII 字符串，size=12/16/24/32

// 控制类
void LCD_DisplayOn(void);                                                                                  // 开显示 + 开背光
void LCD_DisplayOff(void);                                                                                 // 关显示 + 关背光
uint16_t LCD_GetWidth(void);                                                                               // 当前方向下的宽度(像素)
uint16_t LCD_GetHeight(void);                                                                              // 当前方向下的高度(像素)
uint8_t  LCD_GetDir(void);                                                                                 // 0=竖屏 1=横屏

// 线条与图形
void LCD_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                         // 画线
void LCD_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t _color);                           // 画圆
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);             // 显示图片，数据须预先转成 RGB565 数组

// 触摸校准用
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor);                                     // 画十字光标
uint16_t  LCD_ReadPoint(uint16_t  x, uint16_t  y);                                                         // 读回一个像素的 RGB565 值

// LVGL 刷新
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData);        // DMA 整块刷屏，width/height 是右下角坐标
void LCD_SetFlushDoneCallback(void (*cb)(void));                                                            // 注册 DMA 完成回调，内部触发 lv_disp_flush_ready



#endif
