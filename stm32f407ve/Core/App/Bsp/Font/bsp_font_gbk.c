/* bsp_font_gbk.c
 * LVGL 外挂 GBK 字库适配：Unicode 查表得 GB2312 码，按码位算地址，读 32 字节点阵。
 * 硬件：SPI1 = PA5/PA6/PA7 + CS=PC13，字库位于 W25Q128 0x00A00000，16px 段 +0x0008C460。
 * 注意：点阵是列式存储，必须转置成行式；static 缓冲下一次取字形即被覆盖。
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

/* 把 Unicode 码点翻成 GB2312 双字节码，查表用。
 * 表里没有（或码点超过 0xFFFF）返回 0 表示不认。 */
static uint16_t gbk16_uni_to_code(uint32_t uni)
{
    uint16_t lo;
    uint16_t hi;

    if (uni > 0xFFFFU)
    {
        return 0U;                       /* 映射表只覆盖 BMP */
    }

    lo = 0U;
    hi = g_bspGbkMapNum;                 /* 左闭右开 [0, g_bspGbkMapNum) */
    while (lo < hi)
    {
        uint16_t mid = (uint16_t)((lo + hi) >> 1);
        uint32_t u   = g_bspGbkMap[mid] >> 16;    /* 比高 16 位 Unicode */

        if (u == uni)
        {
            return (uint16_t)(g_bspGbkMap[mid] & 0xFFFFU);    /* 命中取低 16 位 GBK 码 */
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

/* 由 GB2312 码算出这个字的点阵在外部 Flash 字库里的地址：
 * 基址 + 偏移 + 序号×32 字节。码不合法返回 0xFFFFFFFF。 */
static uint32_t gbk16_glyph_addr(uint16_t gbk)
{
    uint8_t  hi = (uint8_t)(gbk >> 8);
    uint8_t  lo = (uint8_t)(gbk & 0xFFU);
    uint32_t idx;

    if (hi < 0x81U || hi > 0xFEU || lo < 0x40U || lo == 0x7FU || lo == 0xFFU)
    {
        return 0xFFFFFFFFUL;
    }

    idx = 190U * (uint32_t)(hi - 0x81U)        /* 190 = 每个区去掉 0x7F 后的低位个数 */
        + ((lo < 0x7FU) ? (uint32_t)(lo - 0x40U) : (uint32_t)(lo - 0x41U));   /* 跨过 0x7F 空洞 */

    return GBK_FONT_BASE + GBK16_SECTION_OFS + idx * GBK16_GLYPH_BYTES;        /* 0xA00000+0x8C460+idx*32 */
}

/* 把 32 字节点阵从「列式」转成「行式」，原地转。
 * 字库里存的是列扫描格式，LVGL 要的是按行排的。 */
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
            /*列式取第 ox 列第 oy 位，写到行式第 oy 行第 ox 列*/
            uint8_t b = src[ox * 2U + (oy >> 3)];

            if ((b & (uint8_t)(0x80U >> (oy & 7U))) != 0U)
            {
                buf[oy * 2U + (ox >> 3)] |= (uint8_t)(0x80U >> (ox & 7U));
            }
        }
    }
}

/* LVGL 字体回调之一：告诉它这个字有没有字形、字宽多少。
 * 返回 true 用外挂 GBK 字体，false 交给 fallback（ASCII 走 montserrat_16）。 */
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
    /*16px 字身坐在基线上方，顶部偏移为 0*/
    dsc->ofs_y  = 0;
    dsc->bpp    = 1;
    dsc->is_placeholder = 0;
    return true;
}

/* LVGL 字体回调之一：取这个字的点阵数据。走 SPI 从外部 Flash 读，再转成行式。
 * 查表失败、地址越界或 Flash 读失败都返回 NULL，LVGL 会自动跳过这个字。 */
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

/* 取外挂 GBK 16px 字体对象，交给 LVGL 设字体用。返回的是文件内的静态对象。 */
const lv_font_t *BSP_Font_Gbk16(void)
{
    return &s_gbk16;
}

/* 字库自检：从 Flash 读一个已知字的点阵，看点亮位数是否在 1~255 之间。
 * 全 0 说明没烧字库、全 1 说明 Flash 是空片（0xFF）。返回 1 正常、0 异常。 */
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
