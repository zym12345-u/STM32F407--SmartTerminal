/* bsp_LcdInternal.h
 * ILI9341 驱动拆分后的内部接口：仅 Bsp/LCD_ILI9341 下的 bsp_Lcd*.c 使用。
 * 对外接口仍在 bsp_LCD_ILI9341.h。
 */
#ifndef __LCD_INTERNAL_H
#define __LCD_INTERNAL_H

#include <stdint.h>
#include "bsp_LCD_ILI9341.h"

/* Bank1/NE1 + FSMC_A16 接 RS：A16=0 访问 0x6001FFFE 写命令，A16=1 访问 0x60020000 写数据 */
typedef volatile struct
{
    uint16_t LCD_REG;
    uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD       ((LCD_TypeDef *) 0x6001FFFE)              /* (0x60000000 | 0x0001FFFE) */

/* 主文件提供：初始化是否完成（LCD_String 用） */
uint8_t Lcd_IsReady(void);

/* DMA 刷屏文件提供：LCD_Init 末尾调用 */
void LcdDma_Init(void);


#endif /* __LCD_INTERNAL_H */
