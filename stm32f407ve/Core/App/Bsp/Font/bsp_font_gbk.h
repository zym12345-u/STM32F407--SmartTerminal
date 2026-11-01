/* bsp_font_gbk.h
 * LVGL 外挂 GBK 16px 字体对象与开机自检接口声明。
 * 硬件：SPI1 = PA5/PA6/PA7 + CS=PC13，字库位于 W25Q128 0x00A00000 起。
 * 注意：映射表 6877 项按 Unicode 升序；16px 段每字 32 字节。
 */
#ifndef __BSP_FONT_GBK_H
#define __BSP_FONT_GBK_H

#include <stdint.h>
#include "lvgl.h"

/* Unicode -> GB2312 映射表(自动生成, 见 bsp_gbk_map.c) */
extern const uint32_t g_bspGbkMap[];      /* 6877 项, 高 16 位=Unicode, 低 16 位=GBK 码, 升序 */
extern const uint16_t g_bspGbkMapNum;     /* 表项数, 二分查找的右边界(左闭右开) */

/**
  * 函    数：BSP_Font_Gbk16
  * 参    数：无
  * 返 回 值：外挂 GBK 16px 字体对象指针(ASCII 回落 montserrat_16)
  */
const lv_font_t *BSP_Font_Gbk16(void);

/**
  * 函    数：BSP_Font_SelfTest
  * 参    数：无
  * 返 回 值：1=读取成功且点阵不是全 0/全 1，0=读取失败或内容异常
  */
uint8_t BSP_Font_SelfTest(void);

#endif /* __BSP_FONT_GBK_H */
