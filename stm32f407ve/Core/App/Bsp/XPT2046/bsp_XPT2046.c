/* bsp_XPT2046.c
 * XPT2046 四线电阻触摸驱动：GPIO 模拟 SPI 发控制字、读 12 位 ADC，按校准系数换算成屏幕坐标。
 * 硬件：CS=PD13 CLK=PE0 MOSI=PE2 MISO=PE3 IRQ=PE4；全部软件翻转，不走硬件 SPI。
 * 注意：校准系数与显示方向强绑定，竖屏、横屏各一套，改方向必须重新校准。
 */
#include "bsp_XPT2046.h"
/* #include "bsp_W25Q128.h" -- 外部 Flash 已裁剪，校准值改为固定系数 */
#include "string.h"                 // memset
#include "stdio.h"                  // printf




typedef    struct                   // 触摸数据结构体
{
    uint8_t EN;                     // 触摸开关，0=关闭后不响应上层调用，1=开启

    uint16_t lcdX;                  // 换算后的 LCD 坐标 X
    uint16_t lcdY;                  // 换算后的 LCD 坐标 Y

    uint16_t adcX;                  // X 通道原始 ADC 值(4 次采样去极值后的平均)
    uint16_t adcY;                  // Y 通道原始 ADC 值

    uint16_t lcdWidth;              // 屏宽，由 XPT2046_Init 传入
    uint16_t lcdHeight;             // 屏高，由 XPT2046_Init 传入

    float xfac;                     // 换算系数，xfac=(像素差)/(ADC 差)，单位 像素/计数，可为负
    float yfac;
    short xoff;                     // 零点偏移，xoff=(屏宽-xfac*(adcX1+adcX2))/2
    short yoff;

    uint8_t  dir;                   // 显示方向，0=竖屏，1=横屏
    uint32_t dataAddr;              // 校准数据原存放地址(外部 Flash 已裁剪，现未使用)
} xXPT2046_TypeDey;

static xXPT2046_TypeDey xXPT2046;   // 本文件私有的触摸状态




/*宏定义*/
#define XPT2046_CHANNEL_X   0x90         // 测 X+ 通道(Y 方向)，控制字 bit7~5=001
#define XPT2046_CHANNEL_Y   0xD0         // 测 Y+ 通道(X 方向)，控制字 bit7~5=101

/* 12 位 ADC 有效区间：正常按压落在 200~3900 内，超出 50~4050 视为断线或无回应，整次采样丢弃 */
#define XPT2046_ADC_MIN     50           // 下限，MISO 断线时读数恒 0
#define XPT2046_ADC_MAX     4050         // 上限，从机无响应时可能读到 4095

#define  IRQ_READ           ( XPT2046_IRQ_GPIO -> IDR & XPT2046_IRQ_PIN )                  // 触摸信号读脚,空闲高电平,按下低电平
#define  CS_HIGH            ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN )
#define  CS_LOW             ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN << 16 )
#define  CLK_HIGH           ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN)
#define  CLK_LOW            ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN << 16 )
#define  MOSI_1             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN )
#define  MOSI_0             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN << 16 )
#define  MISO               ( (( XPT2046_MISO_GPIO -> IDR) & XPT2046_MISO_PIN ) ? 1 : 0 )




/*底层函数*/
static void      delayUS(uint32_t ulCount);       // 软件空循环微秒延时
static void      sendCMD(uint8_t cmd);            // 发 8 位控制字
static uint16_t  receiveData(void);               // 读 12 位 ADC
static int16_t   readADC_X(void);                 // 读屏幕 X 方向 ADC
static int16_t   readADC_Y(void);                 // 读屏幕 Y 方向 ADC
static uint8_t   readAdcXY(void);                 // 采样并滤波，结果存入 xXPT2046




/* 微秒级软件延时（忙等）。触摸时序用得很死：位间隔 1us、转换等待 5us。 */
static void delayUS(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        uint8_t uc = 12;     // 12 次自减在 168MHz 下约 1us，换主频或优化等级需重新标定
        while (uc --);
    }
}



/* 发一个控制字选通道：0x90 选 Y+、0xD0 选 X+（都是 12 位、单端、内部参考）。 */
static void sendCMD(uint8_t cmd)
{
    /* 本函数不碰 CS：发命令和收数据必须在同一次片选内完成，中途拉高 CS 本次转换作废 */
    for (uint8_t i = 0; i < 8; i++)
    {
        ((cmd >> (7 - i)) & 0x01) ? MOSI_1 : MOSI_0;  // 在 CLK 低电平期间摆好数据
        delayUS(1);
        CLK_HIGH  ;                                   // 上升沿由芯片锁存
        delayUS(1);
        CLK_LOW   ;
    }
}



/* 读回 12 位 ADC 原始值（通道由上一次 sendCMD 选好）。没触摸时读数没有意义。 */
static uint16_t receiveData(void)
{
    uint16_t usBuf = 0;

    /* 空打一个时钟并等待，芯片内部完成一次转换约需 6us */
    CLK_HIGH;
    delayUS(5);
    CLK_LOW ;
    delayUS(5);

    for (uint8_t i = 0; i < 12; i++)
    {
        usBuf = usBuf << 1;
        CLK_HIGH;
        delayUS(1);
        usBuf |= MISO ;    // 逐位拼装，先收到的为最高位
        CLK_LOW;
        delayUS(1);
    }

    return usBuf;
}



/* 读屏幕 X 方向的 12 位 ADC 原始值（0~4095），还没换算成像素。 */
static int16_t readADC_X(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_Y);  // 竖屏 X 方向走 Y+ 通道，交叉由电阻屏走线决定
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_X);
    return  receiveData();
}



/* 读屏幕 Y 方向的 12 位 ADC 原始值（0~4095）。 */
static int16_t readADC_Y(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_X);  // 竖屏 Y 方向走 X+ 通道
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_Y);
    return  receiveData();
}



/* 一次把 X/Y 两个 ADC 值都读出来，顺便判断是不是真按下了。
 * 返回 1 成功（结果写进 adcX/adcY）；0 表示没按压或读数越界。 */
static uint8_t readAdcXY()
{
    static uint8_t  cnt = 0;
    static uint16_t xSum = 0, ySum = 0;
    static int16_t  xyArray [2] [10] = {{0}, {0}};    // X/Y 两组采样缓存，按 10 组预留，实际只用前 4 组
    int32_t  xMin, xMax, yMin, yMax;                  // 极值，用于去掉最大最小后求平均

    cnt = 0;
    xSum = 0;
    ySum = 0;
    memset(xyArray, 0, 20);                           // 只清前 20 字节，后面只用前 cnt 组，不影响结果
    xMin = 0;
    xMax = 0;
    yMin = 0;
    yMax = 0;

    /*采样：IRQ 低电平表示有按压，最多采 4 组*/
    while ((IRQ_READ == 0) && (cnt < 4))              // 改采样上限时后面 >>1 的除数要同步改
    {
        xyArray[0] [cnt] = readADC_X();
        xyArray[1] [cnt] = readADC_Y();
        cnt ++;
    }

    /*求极值*/
    xMax = xMin = xyArray [0] [0];
    yMax = yMin = xyArray [1] [0];
    for (uint8_t i = 1; i < cnt; i++)
    {
        if (xyArray[0] [i] < xMin)    xMin = xyArray[0] [i];   // X 最小值
        if (xyArray[0] [i] > xMax)    xMax = xyArray[0] [i];   // X 最大值

        if (xyArray[1] [i] < yMin)    yMin = xyArray[1] [i];   // Y 最小值
        if (xyArray[1] [i] > yMax)    yMax = xyArray[1] [i];   // Y 最大值
    }
    /*去极值求平均：4 次采样去掉 2 个极值后除 2*/
    for (uint8_t i = 0; i < cnt; i++)
    {
        xSum = xSum + xyArray[0][i];
        ySum = ySum + xyArray[1][i];
    }
    xXPT2046.adcX = (xSum - xMin - xMax) >> 1;  // 除数固定为 2，采样不足 4 次时结果会偏小
    xXPT2046.adcY = (ySum - yMin - yMax) >> 1;

    /*范围校验：断线或从机无响应时读数贴 0 或 4095，直接判为无效触摸*/
    if (cnt == 0 ||
        xXPT2046.adcX < XPT2046_ADC_MIN || xXPT2046.adcX > XPT2046_ADC_MAX ||
        xXPT2046.adcY < XPT2046_ADC_MIN || xXPT2046.adcY > XPT2046_ADC_MAX)
    {
        return 0;
    }

    return 1;
}



/* 按校准系数把 ADC 原始值换算成 LCD 像素坐标（写进 lcdX/lcdY）。
 * 原始值到像素不是简单线性，所以必须带零偏和比例系数一起算。 */
static void adcXYToLcdXY(void)
{
    static int16_t lcdX = 0;
    static int16_t lcdY = 0;
    /*线性换算：lcd = adc * fac + off*/
    lcdX = xXPT2046.adcX * xXPT2046.xfac + xXPT2046.xoff ;   // ADC 计数 -> 像素
    lcdY = xXPT2046.adcY * xXPT2046.yfac + xXPT2046.yoff ;
    /*限幅*/
    if (lcdX < 0)  lcdX = 0;                                  // 靠屏幕左/上边缘时算出的负值夹到 0
    if (lcdX > xXPT2046.lcdWidth)  lcdX = xXPT2046.lcdWidth;
    if (lcdY < 0)  lcdY = 0;
    if (lcdY > xXPT2046.lcdHeight)  lcdY = xXPT2046.lcdHeight;  // 上限是宽/高本身，边界会多出 1 像素
    xXPT2046.lcdX = lcdX;
    xXPT2046.lcdY = lcdY;
}



/* 把校准算出来的换算系数（xfac/yfac）和零偏（xoff/yoff）记下来，后续坐标换算都用它。
 * 固定返回 0。 */
static uint8_t writeCorrectingData(float xfac, float yfac, short xoff, short yoff)
{
    xXPT2046.xfac = xfac; xXPT2046.yfac = yfac;   // 只写 RAM，掉电丢失，重启回到 Init 里的固定系数
    xXPT2046.xoff = xoff; xXPT2046.yoff = yoff;
    return 0;
}


/* 初始化触摸：配 SPI、GPIO 和 PENIRQ 中断，并把上次的校准系数读回来。
 * lcdWidth/lcdHeight 是屏幕像素尺寸；dir 0 = 竖屏、1 = 横屏。 */
void XPT2046_Init(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    xXPT2046.dataAddr  = 0x00A00000 - 100 ;                      // 校准数据原存放地址(外部 Flash 已裁剪，现未使用)

    xXPT2046.lcdWidth  = lcdWidth;
    xXPT2046.lcdHeight = lcdHeight;
    xXPT2046.dir = dir;

#ifdef USE_HAL_DRIVER                                           // HAL库版本
    /*时钟*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN ;

    /*CS = PD13，推挽输出，低电平选中*/
    GPIO_InitStruct.Pin   = XPT2046_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);

    CS_HIGH;                                                     // 空闲拉高，停止通信

    GPIO_InitStruct.Pin   = XPT2046_CLK_PIN;                     // CLK = PE0
    HAL_GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MOSI_PIN;                    // MOSI = PE2
    HAL_GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MISO_PIN;                    // MISO = PE3，单独设成输入
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = XPT2046_IRQ_PIN;                      // IRQ = PE4
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                      // 只查询电平，不开外部中断
    HAL_GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

#ifdef USE_STDPERIPH_DRIVER                                      // 标准库版本
    /*时钟*/
    RCC_AHB1PeriphClockCmd(XPT2046_IRQ_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CS_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CLK_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MOSI_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MISO_PORT_CLK, ENABLE);
    /*CS = PD13*/
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CS_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);
    CS_HIGH;                                                     // 空闲拉高
    // CLK
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CLK_PIN;
    GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);
    // MOSI
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MOSI_PIN;
    GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);
    // MISO
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);
    // IRQ，只查询电平，不开中断
    GPIO_InitStruct.GPIO_Pin  = XPT2046_IRQ_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

    CLK_LOW ;                                                    // 空闲低电平，数据在低电平期间切换
    MOSI_0;                                                      // MOSI 空闲为低
    CS_LOW;                                                      // 拉低片选，开始通信

    /*校准系数：外部 Flash 已裁剪，两种方向各用一组固定值*/
    if (xXPT2046.dir == 0)
    {
        xXPT2046.xfac = 0.0675f; xXPT2046.yfac = 0.091f;       // 屏宽 240/有效行程 3555 计数，屏高 320/3516 计数
        xXPT2046.xoff = -18;      xXPT2046.yoff = -19;         // 有效行程略超 0~4095，往负方向拉回
    }
    else
    {
        xXPT2046.xfac = -0.09184f; xXPT2046.yfac = 0.077956f;  // 横屏 X 轴 ADC 递增方向与像素相反，斜率必须为负
        xXPT2046.xoff = 338;       xXPT2046.yoff = -45;        // 旋转后有效行程起点变了，靠偏移对齐
    }

    XPT2046_Cmd(ENABLE);                                         // 打开触摸屏
}



/* 重新校准触摸：屏幕上依次点几个十字，据此算出比例系数和零偏。
 * 返回 0 表示校准完成，随后会软复位让新系数生效。 */
uint8_t  XPT2046_ReCalibration(void)
{
    uint16_t pixelOff = 30;   // 十字离边框 30 像素，手指按上去不被边框挡住
    uint16_t adcX1, adcX2, adcX3, adcX4, adcY1, adcY2, adcY3, adcY4; // 记录校准过程中的测量值
    float xfac = 0;
    float yfac = 0;
    short xoff = 0;
    short yoff = 0;
    uint16_t crossX = 0;      // 用于画十字坐标
    uint16_t crossY = 0;      // 用于画十字坐标
    char strTemp[30];
    uint16_t lcdWidth  = xXPT2046.lcdWidth;
    uint16_t lcdHeight = xXPT2046.lcdHeight;

    printf("\r\rTouch panel re-calibration....\r");
    printf("Please use the pen and click the reticle!\r\r");
    LCD_Fill(0, 0, lcdWidth, lcdHeight, BLACK);
    LCD_String(20, 90,  "Please use the pen , ", 16, WHITE, BLACK);
    LCD_String(20, 115, "and click on the reticle !", 16, WHITE, BLACK);

    // 左上角
    crossX = pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏
    adcX1 = xXPT2046 .adcX;                                           // 记录 ADC 值
    adcY1 = xXPT2046.adcY;
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX1);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示读数
    sprintf(strTemp, "Y:%d", adcY1);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);
    delayUS(800000);                                                  // 停留 0.8s，看清读数再移向下一点

    // 右上角
    crossX = lcdWidth - pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏
    adcX2 = xXPT2046 .adcX;                                           // 记录 ADC 值
    adcY2 = xXPT2046.adcY;
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX2);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示读数
    sprintf(strTemp, "Y:%d", adcY2);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);
    delayUS(800000);                                                  // 停留 0.8s

    // 左下角
    crossX = pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏
    adcX3 = xXPT2046 .adcX;                                           // 记录 ADC 值
    adcY3 = xXPT2046.adcY;
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX3);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示读数
    sprintf(strTemp, "Y:%d", adcY3);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);
    delayUS(800000);                                                  // 停留 0.8s

    // 右下角
    crossX = lcdWidth - pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏
    adcX4 = xXPT2046 .adcX;                                           // 记录 ADC 值
    adcY4 = xXPT2046.adcY;
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX4);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示读数
    sprintf(strTemp, "Y:%d", adcY4);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);
    delayUS(400000);                                                  // 最后一点只停 0.4s，随后算系数并复位

    /*对角两点取平均，抵消单点误差*/
    adcX1 = (adcX1 + adcX3) / 2;
    adcX2 = (adcX2 + adcX4) / 2;

    adcY1 = (adcY1 + adcY2) / 2;
    adcY2 = (adcY3 + adcY4) / 2;

    /*系数 = 像素差/ADC 差，偏移让两端对齐*/
    xfac = (float)(pixelOff - (lcdWidth - pixelOff)) / (adcX1 - adcX2);
    yfac = (float)(pixelOff - (lcdHeight - pixelOff)) / (adcY1 - adcY2);
    xoff = (lcdWidth - xfac * (adcX1 + adcX2)) / 2;
    yoff = (lcdHeight - yfac * (adcY1 + adcY2)) / 2;

    /*写回 RAM(外部 Flash 已裁剪)*/
    writeCorrectingData(xfac, yfac, xoff, yoff);

    xXPT2046.xfac = xfac;
    xXPT2046.xoff = xoff;
    xXPT2046.yfac = yfac;
    xXPT2046.yoff = yoff;

    printf(">>>Calibration done! Coefficients kept in RAM only.\r\r");

    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;  // 系统复位,软复位
    return 0;
}



/* 开或关触摸检测：status 为 0 关闭、非 0 开启。 */
void XPT2046_Cmd(uint8_t status)
{
    if (status != 0)
    {
        xXPT2046 .EN = 1;
    }
    else
    {
        xXPT2046.EN = 0;
    }
}



/* 查当前有没有按下。返回 1 时坐标已经同步更新到 lcdX/lcdY，可以直接取。 */
uint8_t XPT2046_IsPressed(void)
{
    static uint8_t status = 0;

    if (xXPT2046.EN == 0)             // 开关关闭时直接当未按下
        return 0;

    status = IRQ_READ ? 0 : 1 ;       // 空闲高电平，按下低电平
    if (status == 1)
    {
        if (readAdcXY() == 0)         // 断线或越界，丢弃本次
            return 0;
        adcXYToLcdXY();               // 换算成屏幕坐标
        uint16_t x1 = xXPT2046.lcdX;
        uint16_t y1 = xXPT2046.lcdY;

        if (readAdcXY() == 0)
            return 0;
        adcXYToLcdXY();
        uint16_t x2 = xXPT2046.lcdX;
        uint16_t y2 = xXPT2046.lcdY;

        // 两次采样差超过 10 像素判为野点；阈值取 3 会把轻按抖动(4~8px)误判成松手
        uint8_t x, y;
        if (x1 > x2)
            x = x1 - x2;
        else
            x = x2 - x1;

        if (y1 > y2)
            y = y1 - y2;
        else
            y = y2 - y1;

        if (x > 10 || y > 10)
            return 0;

        // 两次测量取平均值
        xXPT2046.lcdX = (x1 + x2) >> 1;
        xXPT2046.lcdY = (y1 + y2) >> 1;
    }

    return  status ;      // 0=未按下，1=按下
}



/* 取上次触摸点的横坐标，单位像素。 */
uint16_t  XPT2046_GetX(void)
{
    return xXPT2046.lcdX;
}



/* 取上次触摸点的纵坐标，单位像素。 */
uint16_t  XPT2046_GetY(void)
{
    return xXPT2046.lcdY;
}



/* 通知「按下了」：置按下标志，方便上层做按下沿的判断。 */
void XPT2046_TouchDown(void)
{
//    // 示例:在触摸处画一个黄色点
//    LCD_DrawPoint(xXPT2046.lcdX, xXPT2046.lcdY, YELLOW);
//    // 示例:把触摸坐标显示到 LCD
//    static char strTem[20];
//    sprintf(strTem, "%3d  %3d", xXPT2046.lcdX, xXPT2046.lcdY);
//    LCD_String(180, 290, strTem, 12, WHITE, BLACK);
}



/* 通知「抬起了」：清按下标志。 */
void XPT2046_TouchUp(void)
{


}
