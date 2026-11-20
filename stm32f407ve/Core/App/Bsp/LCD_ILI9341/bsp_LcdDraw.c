/* bsp_LcdDraw.c
 * ILI9341 字符与图形绘制：ASCII 字符、字符串、线、圆、矩形、位图、十字标记。
 * 全部基于 LCD_DrawPoint / LCD_Fill，不直接操作 FSMC 寄存器。
 */
#include "bsp_LCD_ILI9341.h"
#include "bsp_LcdInternal.h"
#include "font.h"


/**
  * 功    能：在指定位置画一个 ASCII 字符（内部函数）。字号支持 12/16/24/32
  * 参    数：x,y 起始坐标；num 字符 0x20~0x7E；size 字号 12/16/24/32；bColor 背景色；fColor 前景色
  * 返 回 值：无
  */
static void drawAscii(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint32_t bColor, uint32_t fColor)
{
    static uint8_t temp = 0;
    static uint8_t csize = 0;
    static uint16_t y0 = 0;
    uint16_t w = LCD_GetWidth();
    uint16_t h = LCD_GetHeight();

    y0 = y;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);   // 每列字节数(size/8 向上取整)*列数(size/2)；12→12, 16→16, 24→36, 32→64 字节
    num = num - ' ';                                          // 字库只收 0x20~0x7E，下标 = ASCII-0x20
    for (uint8_t t = 0; t < csize; t++)
    {
        if (size == 12)         temp = aFontASCII12[num][t];  // 调用1206字体
        else if (size == 16)    temp = aFontASCII16[num][t];  // 调用1608字体
        else if (size == 24)    temp = aFontASCII24[num][t];  // 调用2412字体
        else if (size == 32)    temp = aFontASCII32[num][t];  // 调用3216字体
        else                    return;                       // 没有字模

        for (uint8_t t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80) LCD_DrawPoint(x, y, fColor);     // 点亮该点
            else             LCD_DrawPoint(x, y, bColor);     // 阴码：1=显示点，0=背景点

            temp <<= 1;                                       // 逐列式取模: 每次取最高位(MSB 对应本列最上面的像素)
            y++;
            if (y >= h)      return;                          // 超出屏幕高度(下)
            if ((y - y0) == size)                             // 一列画满 size 个像素(本字的字高)
            {
                y = y0;                                       // 回到本列顶端
                x++;                                          // 换到右边一列
                if (x >= w) return;                           // 超出屏幕宽度(右)
                break;
            }
        }
    }
}


/**
  * 功    能：画一串 ASCII 字符。只支持 ASCII —— 中文要走 LVGL 那边的 GBK 字体
  * 参    数：x,y 起始坐标；pFont 字符串，仅 ASCII；size 12/16/24/32；fColor 前景色；bColor 背景色
  * 返 回 值：无
  */
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor)
{
    uint16_t w;
    uint16_t h;

    if (Lcd_IsReady() == 0U) return;

    w = LCD_GetWidth();
    h = LCD_GetHeight();

    uint16_t xStart = x;

    if (size != 12 && size != 16 && size != 24 && size != 32)      // 如果字号不支持
        size = 24;

    while (*pFont != 0)                                            // 循环读取字符，直到遇到'\0'时停止
    {
        if (x > (w - size))                                        // 坐标判断，超出宽度则换行（回到行首并下移一行）
        {
            x = xStart;
            y = y + size;
        }
        if (y > (h - size))                                        // 坐标判断，超出高度则直接退出（已从屏幕底部溢出）
            return;

        if (*pFont < 128)                                          // 仅支持 ASCII 字符
        {
            drawAscii(x, y, *pFont, size, bColor, fColor);
        }
        pFont++;
        x += size / 2;                                              // 推进一个字符宽: 字模是半角, 宽度只有字号的一半(12号字宽6)
    }
}


/**
  * 功    能：画一条直线，起止点都包含在内
  * 参    数：xStart,yStart 起点；xEnd,yEnd 终点(含端点)；_color RGB565 颜色
  * 返 回 值：无
  */
void LCD_Line(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t _color)
{
    uint16_t  t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = xEnd - xStart;                             // 坐标增量
    delta_y = yEnd - yStart;
    uRow = xStart;
    uCol = yStart;
    if (delta_x > 0)incx = 1;                            // 设置单步方向
    else if (delta_x == 0)incx = 0;                      // 垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0) incy = 1;
    else if (delta_y == 0)incy = 0;                      // 水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)distance = delta_x;            // 选取基本增量坐标轴  ; 取较长的轴作为主轴, 保证线上不留空隙
    else distance = delta_y;
    for (t = 0; t <= distance + 1; t++)                  // 画线计算          ; distance+1 次循环, 保证终点像素也被画到
    {
        LCD_DrawPoint(uRow, uCol, _color);               // 画点
        xerr += delta_x ;
        yerr += delta_y ;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}


/**
  * 功    能：画一个圆（圆心 + 半径），只画轮廓
  * 参    数：Xpos,Ypos 圆心坐标；Radius 半径(像素)；_color 颜色
  * 返 回 值：无
  */
void LCD_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t _color)
{
    int16_t mx = Xpos, my = Ypos, x = 0, y = Radius;
    int16_t d = 1 - Radius;         // 中点画圆法初始判别式; 用整数迭代避免开方/三角函数
    while (y > x)                   // 只算 1/8 圆弧, 下面 8 个点靠对称性补全
    {
        LCD_DrawPoint(x + mx, y + my, _color);
        LCD_DrawPoint(-x + mx, y + my, _color);
        LCD_DrawPoint(-x + mx, -y + my, _color);
        LCD_DrawPoint(x + mx, -y + my, _color);
        LCD_DrawPoint(y + mx, x + my, _color);
        LCD_DrawPoint(-y + mx, x + my, _color);
        LCD_DrawPoint(y + mx, -x + my, _color);
        LCD_DrawPoint(-y + mx, -x + my, _color);
        if (d < 0)                  // 中点在圆内: 只走 x, y 不动
        {
            d += 2 * x + 3;
        }
        else                        // 中点在圆外: x、y 同时走一格
        {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}


/**
  * 功    能：画矩形：filled 传 0 画空心框、传 1 填实心
  * 参    数：x,y 左上角；width/height 宽高(像素)；color 颜色；filled 0=空心 1=实心
  * 返 回 值：无
  */
void LCD_Rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t filled)
{
    if (filled)                                          // 0=空心 1=实心
    {
        LCD_Fill(x, y, width, height, color);            // width/height 直接当右下角坐标，比"宽高"语义多画 1 像素
    }
    else
    {
        LCD_Line(x, y, x + width - 1, y, color);
        LCD_Line(x, y + height - 1, x + width - 1, y + height - 1, color);
        LCD_Line(x, y, x, y + height - 1, color);
        LCD_Line(x + width - 1, y, x + width - 1, y + height - 1, color);
    }
}


/**
  * 功    能：画一张 RGB565 位图，每个像素占 2 字节。左上角 + 宽高 + 数据指针
  * 参    数：x,y 左上角；width/height 像素宽高；image RGB565 数组，每像素 2 字节
  * 返 回 值：无
  */
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image)
{
    for (uint16_t i = 0; i < height; i++)           // 逐行写入
    {
        LCD->LCD_REG = 0X2A;                        // 0x2A 设 x 起点
        LCD->LCD_RAM = x >> 8;                      // 高 8 位
        LCD->LCD_RAM = x ;                          // 低 8 位，写 16 位也只有低 8 位有效
        LCD->LCD_REG = 0X2B;                        // 0x2B 设 y 起点
        LCD->LCD_RAM = (y + i) >> 8;                // 高 8 位
        LCD->LCD_RAM = y + i;                       // 低 8 位
        LCD->LCD_REG = 0X2C;                        // 0x2C 开始写 GRAM
        for (uint16_t j = 0; j < width; j++)        // 每像素 2 字节
        {
            LCD->LCD_RAM = image[1] << 8 | *image;  // image[1] 当高字节：数组首字节反而是颜色的低 8 位
            image += 2;                             // 指向下一个像素
        }
    }
}


/**
  * 功    能：画一个十字标记 —— 触摸校准时的靶标就是它
  * 参    数：x,y 中心坐标；len 十字总长(像素)；fColor 颜色
  * 返 回 值：无
  */
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor)
{
    uint16_t temp = len / 2;                        // 半臂长

    LCD_Line(x - temp, y, x + temp, y, fColor);
    LCD_Line(x, y - temp, x, y + temp, fColor);
}
