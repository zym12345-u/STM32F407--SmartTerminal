/**
  ******************************************************************************
  * @file    bsp_font_gbk.h
  * @brief   LVGL 外挂 GBK 中文字库适配器(板载 W25Q128, 0x00A00000 起)
  *
  *          背景: 内部 Flash 里的 lv_font_gb2312_16 约 262KB, 只为拿到一个
  *                16px 中文字库。V1.2 改成"按需从外部 Flash 读点阵",
  *                内部 Flash 只留 Unicode->GB2312 映射表(约 27KB), 换回 235KB。
  *
  *          字库布局(厂商例程反推 + 偏移算术交叉验证):
  *            基址 0x00A00000, 按字号分段:
  *              12px +0x000000 (24B/字)   16px +0x08C460 (32B/字)
  *              24px +0x1474E0 (72B/字)   32px +0x2EC200 (128B/字)
  *            每段 190 x 126 = 23940 个码位, 段长 = 23940 x 单字字节数
  *            16px 段实测: 0x1474E0 - 0x08C460 = 0xBB080 = 23940 x 32 (吻合)
  *
  *          码位公式(与厂商 W25Q128_ReadFontData 一致):
  *            idx = 190*(hi-0x81) + (lo<0x7F ? lo-0x40 : lo-0x41)
  *
  *          点阵格式: 1bpp, 16x16, 每行 2 字节, 高位在左(MSB first),
  *                    与 LVGL 的 1bpp 行对齐格式天然一致, 不需要转换。
  *
  *          ASCII 不走外挂字库(厂商库只提供汉字矩阵), 由 fallback 字体
  *          lv_font_montserrat_16 渲染, 两字体 line_height 都是 19, 排版一致。
  *
  *          线程约束: 与 bsp_W25Q128 相同, 只能在 boot 阶段或单任务串行访问
  *                    SPI1 的上下文调用(当前只有 GUITask 在渲染时调用)。
  ******************************************************************************
  */
#ifndef __BSP_FONT_GBK_H
#define __BSP_FONT_GBK_H

#include <stdint.h>
#include "lvgl.h"

/* Unicode -> GB2312 映射表(自动生成, 见 bsp_gbk_map.c) */
extern const uint32_t g_bspGbkMap[];
extern const uint16_t g_bspGbkMapNum;

/** 取得"外挂 GBK 16px"字体对象(ASCII 自动回落 montserrat_16) */
const lv_font_t *BSP_Font_Gbk16(void);

/** 开机自检: 读 "中"(GB2312 0xD6D0) 点阵, 打印地址/前 8 字节/1 比特占比。
 *  返回 1 = 读取成功且内容不是全 0/全 1(结构可信), 0 = 读取失败或内容异常 */
uint8_t BSP_Font_SelfTest(void);

#endif /* __BSP_FONT_GBK_H */
