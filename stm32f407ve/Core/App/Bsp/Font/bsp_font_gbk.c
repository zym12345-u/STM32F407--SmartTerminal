/**
  ******************************************************************************
  * @file    bsp_font_gbk.c
  * @brief   LVGL 外挂 GBK 字库适配层(详述见 bsp_font_gbk.h)
  *
  *          实现要点:
  *            1. LVGL 只认 Unicode, 板载字库按 GB2312 码位索引,
  *               所以先用 g_bspGbkMap 做 Unicode -> GB2312 二分查找。
  *            2. 点阵 1bpp/16x16/32B, 直接读进 static 缓冲交给 LVGL 使用。
  *               LVGL 是"取一个字形 -> 立刻画 -> 再取下一个"的串行流程,
  *               所以单个静态缓冲就够, 不需要按字形分配内存。
  *            3. **这块字库是列式(纵向)存储**: 32 字节 = 16 列 x 每列 2 字节,
  *               列内高位在上。LVGL 要的是行式(横向), 所以取到后必须转置一次。
  *               2026-10-23 真机现象: 汉字显示出来但左右镜像 + 逆时针 90 度,
  *               正是"把转置后的图当原图"的特征; 实测 "中" 前 8 字节
  *               00 00 00 00 3F E0 10 40, 按列读 = 第 3 列长竖 + 第 4 列上下两点
  *               (口字左边框 + 内部上下横), 按行读则不成立 → 列式坐实。
  *            4. 查不到(ASCII/emoji/未收录汉字)返回 false, 由 fallback
  *               字体 lv_font_montserrat_16 兜底。
  *            5. 缓冲用 static, 不占堆, 也不占 boot 阶段仅 1KB 的 MSP 栈。
  ******************************************************************************
  */
#include "bsp_font_gbk.h"
#include "bsp_W25Q128.h"
#include "usart.h"
#include <string.h>

#define GBK_FONT_BASE       0x00A00000UL   /* 外挂字库基址 */
#define GBK16_SECTION_OFS   0x0008C460UL   /* 16px 段在字库内的偏移 */
#define GBK16_GLYPH_BYTES   32U            /* 16x16 1bpp = 32B */
#define GBK16_CELL          16             /* 字身宽高 = 16px */
#define GBK16_LINE_HEIGHT   19             /* 与内置 gb2312_16 / montserrat_16 对齐 */
#define GBK16_BASE_LINE     3

static uint8_t s_glyphBuf[GBK16_GLYPH_BYTES];

/** Unicode -> GB2312 双字节码; 表里没有返回 0 */
static uint16_t gbk16_uni_to_code(uint32_t uni)
{
    uint16_t lo;
    uint16_t hi;

    if (uni > 0xFFFFU)
    {
        return 0U;                       /* 映射表只覆盖 BMP */
    }

    lo = 0U;
    hi = g_bspGbkMapNum;                 /* 左闭右开 */
    while (lo < hi)
    {
        uint16_t mid = (uint16_t)((lo + hi) >> 1);
        uint32_t u   = g_bspGbkMap[mid] >> 16;

        if (u == uni)
        {
            return (uint16_t)(g_bspGbkMap[mid] & 0xFFFFU);
        }
        if (u < uni)
        {
            lo = (uint16_t)(mid + 1U);
        }
        else
        {
            hi = mid;
        }
    }
    return 0U;
}

/** GB2312 码位 -> 字库内字节地址; 越界返回 0xFFFFFFFF */
static uint32_t gbk16_glyph_addr(uint16_t gbk)
{
    uint8_t  hi = (uint8_t)(gbk >> 8);
    uint8_t  lo = (uint8_t)(gbk & 0xFFU);
    uint32_t idx;

    if (hi < 0x81U || hi > 0xFEU || lo < 0x40U || lo == 0x7FU || lo == 0xFFU)
    {
        return 0xFFFFFFFFUL;
    }

    idx = 190U * (uint32_t)(hi - 0x81U)
        + ((lo < 0x7FU) ? (uint32_t)(lo - 0x40U) : (uint32_t)(lo - 0x41U));

    return GBK_FONT_BASE + GBK16_SECTION_OFS + idx * GBK16_GLYPH_BYTES;
}

/** 列式 -> 行式: 16x16 1bpp 转置(两次转置等于还原, 可逆) */
static void gbk16_transpose(uint8_t *buf)
{
    uint8_t  src[GBK16_GLYPH_BYTES];
    uint8_t  oy;
    uint8_t  ox;

    memcpy(src, buf, GBK16_GLYPH_BYTES);
    memset(buf, 0, GBK16_GLYPH_BYTES);

    for (oy = 0U; oy < 16U; oy++)                 /* oy: 行式缓冲的行 */
    {
        for (ox = 0U; ox < 16U; ox++)             /* ox: 行式缓冲的列 */
        {
            /* out(oy,ox) = in(ox,oy): 第 ox 行、第 oy 位 */
            uint8_t b = src[ox * 2U + (oy >> 3)];

            if ((b & (uint8_t)(0x80U >> (oy & 7U))) != 0U)
            {
                buf[oy * 2U + (ox >> 3)] |= (uint8_t)(0x80U >> (ox & 7U));
            }
        }
    }
}

/** LVGL 取字形描述 */
static bool gbk16_get_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc,
                          uint32_t letter, uint32_t letter_next)
{
    (void)font;
    (void)letter_next;

    if (gbk16_uni_to_code(letter) == 0U)
    {
        return false;                    /* 交给 fallback 字体 */
    }

    dsc->adv_w  = GBK16_CELL;
    dsc->box_w  = GBK16_CELL;
    dsc->box_h  = GBK16_CELL;
    dsc->ofs_x  = 0;
    /* 16px 字身整体坐在基线上方: 顶部 = (19-3) - 16 - 0 = 0 */
    dsc->ofs_y  = 0;
    dsc->bpp    = 1;
    dsc->is_placeholder = 0;
    return true;
}

/** LVGL 取字形点阵指针(指向 static 缓冲, 下一次调用会被覆盖) */
static const uint8_t *gbk16_get_bitmap(const lv_font_t *font, uint32_t letter)
{
    uint16_t gbk;
    uint32_t addr;

    (void)font;

    gbk = gbk16_uni_to_code(letter);
    if (gbk == 0U)
    {
        return NULL;
    }

    addr = gbk16_glyph_addr(gbk);
    if (addr == 0xFFFFFFFFUL)
    {
        return NULL;
    }

    if (W25Q128_ReadData(addr, s_glyphBuf, GBK16_GLYPH_BYTES) != HAL_OK)
    {
        return NULL;
    }

    gbk16_transpose(s_glyphBuf);         /* 列式字库 -> LVGL 要的行式 */
    return s_glyphBuf;
}

static const lv_font_t s_gbk16 = {
    .get_glyph_dsc    = gbk16_get_dsc,
    .get_glyph_bitmap = gbk16_get_bitmap,
    .line_height      = GBK16_LINE_HEIGHT,
    .base_line        = GBK16_BASE_LINE,
    .subpx            = LV_FONT_SUBPX_NONE,
    .underline_position  = -1,
    .underline_thickness = 1,
    .dsc              = NULL,
    .fallback         = &lv_font_montserrat_16,
#if LV_USE_USER_DATA
    .user_data        = NULL,
#endif
};

const lv_font_t *BSP_Font_Gbk16(void)
{
    return &s_gbk16;
}

uint8_t BSP_Font_SelfTest(void)
{
    const uint16_t gbk  = 0xD6D0U;        /* "中" */
    const uint32_t addr = gbk16_glyph_addr(gbk);
    uint16_t bits = 0U;
    uint8_t  i;
    uint8_t  j;

    if (addr == 0xFFFFFFFFUL)
    {
        UART_Printf("[FONT] self-test FAIL: bad glyph code\r\n");
        return 0U;
    }

    if (W25Q128_ReadData(addr, s_glyphBuf, GBK16_GLYPH_BYTES) != HAL_OK)
    {
        UART_Printf("[FONT] self-test FAIL: read @0x%08lX\r\n", (unsigned long)addr);
        return 0U;
    }

    gbk16_transpose(s_glyphBuf);          /* 与渲染路径完全一致 */

    for (i = 0U; i < (uint8_t)GBK16_GLYPH_BYTES; i++)
    {
        for (j = 0U; j < 8U; j++)
        {
            if ((s_glyphBuf[i] & (uint8_t)(1U << j)) != 0U)
            {
                bits++;
            }
        }
    }

    UART_Printf("[FONT] glyph(0xD6D0) @0x%08lX, first8: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)addr,
                s_glyphBuf[0], s_glyphBuf[1], s_glyphBuf[2], s_glyphBuf[3],
                s_glyphBuf[4], s_glyphBuf[5], s_glyphBuf[6], s_glyphBuf[7]);

    /* "中" 是空心 + 一竖, 点亮比例不会接近 0% 也不会接近 100% */
    if (bits == 0U || bits >= 256U)
    {
        UART_Printf("[FONT] self-test FAIL: bitmap all %s (%u/256 bits)\r\n",
                    (bits == 0U) ? "0x00" : "0xFF", (unsigned int)bits);
        return 0U;
    }

    UART_Printf("[FONT] self-test PASS: %u/256 bits lit\r\n", (unsigned int)bits);
    return 1U;
}
