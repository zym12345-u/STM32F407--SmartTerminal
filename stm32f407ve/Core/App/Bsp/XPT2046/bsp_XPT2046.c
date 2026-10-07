#include "bsp_XPT2046.h"            // 头文件
/* #include "bsp_W25Q128.h" -- W25Q128 removed, fixed touch calibration used */
#include "string.h"                 // C语言的标准字符串库,用于字符串、内存处理
#include "stdio.h"                  // C语言的标准输入输出库,用于 printf 等函数




/******************************* 触摸相关的静态数据 ***************************/
typedef    struct                   // 触摸数据结构体
{
    uint8_t EN;                     // 触摸开关, 0_关闭, 1_开; 关闭时停止响应上层的触摸调用,需要时可关闭,以节省芯片资源

    uint16_t lcdX;                  // 当前读到的 LCD 坐标值(
    uint16_t lcdY;

    uint16_t adcX;                  // 触摸屏读取的原始 X 通道 ADC 值(滤波后的平均值)
    uint16_t adcY;

    uint16_t lcdWidth;              // 用于记录 LCD 实际的屏幕宽度; 由 XPT2046_Init 传入
    uint16_t lcdHeight;             // 用于记录 LCD 实际的屏幕高度; 由 XPT2046_Init 传入

    float xfac;                     // ADC 与 LCD 坐标的换算系数,  xfac=(float)(20-320)/(t1x-t2x);
    float yfac;
    short xoff;                     // 坐标的偏移值, xoff=(320-xfac*(t1x+t2x))/2;
    short yoff;

    uint8_t  dir;                   // 显示方向, 0-竖屏, 1_横屏
    uint32_t dataAddr;              // 校准数据在外部 FLASH 中的存放地址
} xXPT2046_TypeDey;

static xXPT2046_TypeDey xXPT2046;   // 用于存放 XPT2046 的各类信息,全局变量只在本文件使用(不需 extern)




/******************************* 宏定义 ***************************/
#define XPT2046_CHANNEL_X   0x90         // X通道,测量通道Y+的电压值
#define XPT2046_CHANNEL_Y   0xD0         // Y通道,测量通道X+的电压值

/* ADC 有效值范围: XPT2046 为 12 位 ADC(0~4095),
 * 正常触摸按下时 X/Y 通常落在 200~3900 之间;
 * 超出此范围视为通信异常(MISO 断线/从机无响应),丢弃该次采样 */
#define XPT2046_ADC_MIN     50
#define XPT2046_ADC_MAX     4050

#define  IRQ_READ           ( XPT2046_IRQ_GPIO -> IDR & XPT2046_IRQ_PIN )                  // 触摸信号读脚,空闲高电平,按下低电平
#define  CS_HIGH            ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN )
#define  CS_LOW             ( XPT2046_CS_GPIO  -> BSRR  = XPT2046_CS_PIN << 16 )
#define  CLK_HIGH           ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN)
#define  CLK_LOW            ( XPT2046_CLK_GPIO -> BSRR  = XPT2046_CLK_PIN << 16 )
#define  MOSI_1             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN )
#define  MOSI_0             ( XPT2046_MOSI_GPIO-> BSRR  = XPT2046_MOSI_PIN << 16 )
#define  MISO               ( (( XPT2046_MISO_GPIO -> IDR) & XPT2046_MISO_PIN ) ? 1 : 0 )




/******************************* 触摸相关底层函数 ***************************/
static void      delayUS(uint32_t ulCount);       // 微秒延时函数, 为保证移植性, 不使用外部定时器
static void      sendCMD(uint8_t cmd);            // 向触摸发送命令
static uint16_t  receiveData(void);               // 获取测量值
static int16_t   readADC_X(void);                 // 获取X轴ADC值
static int16_t   readADC_Y(void);                 // 获取Y轴ADC值
static uint8_t   readAdcXY(void);                 // 获取X与Y的ADC值并滤波,得到的数据放到全局结构体变量 xXPT2046 中




// 粗略US级延时,纯软件延时,不依赖外部定时器和中断
static void delayUS(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        uint8_t uc = 12;     //参数设为12,大约延时1微秒
        while (uc --);       //约1微秒
    }
}



// 写入控制命令
// Cmd 为 0x90_通道Y+的选择控制字, 0xd0_通道X+的选择控制字
// XPT2046 时钟要求 CLK 在时钟低电平时发送串行数据,见芯片手册时序介绍
// 注意:发送完控制字后,接着读取数据,需要先片选,在同一次片选内完成(CS 一直保持低电平)
static void sendCMD(uint8_t cmd)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        ((cmd >> (7 - i)) & 0x01) ? MOSI_1 : MOSI_0; // 逐位发送
        delayUS(1);
        CLK_HIGH  ;
        delayUS(1);
        CLK_LOW   ;
    }
}



// 获取测量数据(在执行 sendCMD 命令之后执行);
// 本函数用于获取测量值
static uint16_t receiveData(void)
{
    uint16_t usBuf = 0;

    CLK_HIGH;              // 给一个时钟脉冲,等待 BUSY 拉低,这个时间是芯片在发控制字后内部 AD 转换,需要约 6US
    delayUS(5);
    CLK_LOW ;
    delayUS(5);

    for (uint8_t i = 0; i < 12; i++)
    {
        usBuf = usBuf << 1;
        CLK_HIGH;
        delayUS(1);
        usBuf |= MISO ;    // 逐位接收
        CLK_LOW;
        delayUS(1);
    }

    return usBuf;
}



// 选择一个模拟通道,启动 ADC 转换并返回 ADC 测量值
// 0x90 :通道Y+的选择控制字
// 0xd0 :通道X+的选择控制字
static int16_t readADC_X(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_Y);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_X);
    return  receiveData();
}



static int16_t readADC_Y(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_X);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_Y);
    return  receiveData();
}



/******************************************************************************
 * 函数名称 readAdcXY
 * 功  能: 获取触摸按下时 X、Y 的 ADC 值并滤波处理
 * 入  参: 无
 * 返 回: 1  获取成功,已把数据放到全局结构体中
 *          0  失败
 ******************************************************************************/
static uint8_t readAdcXY()
{
    static uint8_t  cnt = 0;
    static uint16_t xSum = 0, ySum = 0;
    static int16_t  xyArray [2] [10] = {{0}, {0}};    // 临时多维数组,用于存放 X、Y 的 10 次测量
    int32_t  xMin, xMax, yMin, yMax;                  // 存储测量中的最大值、最小值;用于去掉最大值去掉最小值求平均值

    cnt = 0;
    xSum = 0;
    ySum = 0;
    memset(xyArray, 0, 20);
    xMin = 0;
    xMax = 0;
    yMin = 0;
    yMax = 0;

    while ((IRQ_READ == 0) && (cnt < 4))              // 多笔采样; 当触摸 TP_INT_IN 信号为低(屏幕有按下), 且 cnt<10
    {
        xyArray[0] [cnt] = readADC_X();
        xyArray[1] [cnt] = readADC_Y();
        cnt ++;
    }

    // 求平均值滤波
    xMax = xMin = xyArray [0] [0];                              // 首先取出最大值、最小值
    yMax = yMin = xyArray [1] [0];
    for (uint8_t i = 1; i < cnt; i++)
    {
        if (xyArray[0] [i] < xMin)    xMin = xyArray[0] [i];   // 求 x 的 10 次测量最小 ADC 值
        if (xyArray[0] [i] > xMax)    xMax = xyArray[0] [i];   // 求 x 的 10 次测量最大 ADC 值

        if (xyArray[1] [i] < yMin)    yMin = xyArray[1] [i];   // 求 y 的 10 次测量最小 ADC 值
        if (xyArray[1] [i] > yMax)    yMax = xyArray[1] [i];   // 求 y 的 10 次测量最大 ADC 值
    }
    // 去掉最大值、最小值之后求平均值
    for (uint8_t i = 0; i < cnt; i++)
    {
        xSum = xSum + xyArray[0][i];
        ySum = ySum + xyArray[1][i];
    }
    xXPT2046.adcX = (xSum - xMin - xMax) >> 1;  // 去掉最大值、最小值之后,除 2
    xXPT2046.adcY = (ySum - yMin - yMax) >> 1;  // 去掉最大值、最小值之后,除 2

    /* ADC 范围校验: MISO 断线时读到 0,从机无响应时可能为 0 或 4095,
     * 这些情况都视为无效触摸,避免返回错误坐标 */
    if (cnt == 0 ||
        xXPT2046.adcX < XPT2046_ADC_MIN || xXPT2046.adcX > XPT2046_ADC_MAX ||
        xXPT2046.adcY < XPT2046_ADC_MIN || xXPT2046.adcY > XPT2046_ADC_MAX)
    {
        return 0;
    }

    return 1;
}



// 把已读的电压值换算成对应的 LCD 坐标值
static void adcXYToLcdXY(void)
{
    static int16_t lcdX = 0;
    static int16_t lcdY = 0;
    // 线性换算关系
    lcdX = xXPT2046.adcX * xXPT2046.xfac + xXPT2046.xoff ;
    lcdY = xXPT2046.adcY * xXPT2046.yfac + xXPT2046.yoff ;
    // 限制取值范围
    if (lcdX < 0)  lcdX = 0;
    if (lcdX > xXPT2046.lcdWidth)  lcdX = xXPT2046.lcdWidth;
    if (lcdY < 0)  lcdY = 0;
    if (lcdY > xXPT2046.lcdHeight)  lcdY = xXPT2046.lcdHeight;
    // 转换完成, 把临时值转存到结构体中, 供随时调用
    xXPT2046.lcdX = lcdX;
    xXPT2046.lcdY = lcdY;
}



/******************************************************************************
 * 函数名称 writeDcorrectingData
 * 功  能: 写入校准参数
 * 入  参 float xfac   x轴换算系数
 *          float yfac   y轴换算系数
 *          short xoff   x轴偏移参数
 *          short yoff   y轴偏移参数
 * 返 回: 0_写入成功, 非0_写入失败
 ******************************************************************************/



/******************************************************************************
 * 函数名称 XPT2046_Init
 * 功  能: 初始化
 * 入  参 uint16_t lcdWidth     LCD屏幕宽度
 *          uint16_t lcdHeight    LCD屏幕高度
 *          uint8_t dir           显示方向    0-竖屏 3-横屏 5-反向竖屏, 6-反向横屏
 * 返 回:
 ******************************************************************************/
/* W25Q128 removed: calibration now kept in RAM only */
static uint8_t writeCorrectingData(float xfac, float yfac, short xoff, short yoff)
{
    xXPT2046.xfac = xfac; xXPT2046.yfac = yfac;
    xXPT2046.xoff = xoff; xXPT2046.yoff = yoff;
    return 0;
}


void XPT2046_Init(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    xXPT2046.dataAddr  = 0x00A00000 - 100 ;                      // 校准数据的存放 W25Q128 地址; 0x00A00000 为字库开始地址,校准数据存放在其前面几页

    xXPT2046.lcdWidth  = lcdWidth;
    xXPT2046.lcdHeight = lcdHeight;
    xXPT2046.dir = dir;

#ifdef USE_HAL_DRIVER                                           // HAL库版本
    // 使能 GPIO 端口时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN ;  // 使能 GPIOA、B、C、D、E 时钟

    /// 初始化 CS 引脚
    GPIO_InitStruct.Pin   = XPT2046_CS_PIN;                      // 设置引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;                 // 推挽输出模式,不带上拉
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                         // 内部上拉电阻使能
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           // 引脚翻转速率,高速
    HAL_GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);            // 初始化引脚

    CS_HIGH;                                                     // 拉高片选信号,停止通信

    GPIO_InitStruct.Pin   = XPT2046_CLK_PIN;                     // CLK
    HAL_GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MOSI_PIN;                    // MOSI
    HAL_GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = XPT2046_MISO_PIN;                    // MISO
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                     // 输入模式
    HAL_GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = XPT2046_IRQ_PIN;                      // IRQ 信号引脚
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                      // 输入模式,该信号只作指示用,不使用中断
    HAL_GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

#ifdef USE_STDPERIPH_DRIVER                                      // 标准库版本
    // 使能时钟
    RCC_AHB1PeriphClockCmd(XPT2046_IRQ_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CS_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_CLK_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MOSI_PORT_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(XPT2046_MISO_PORT_CLK, ENABLE);
    // 配置CS为推挽输出模式
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CS_PIN;                 // 选择要控制的GPIO引脚
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;                  // 输出模式,普通模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                  // 推挽输出,不带上拉
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上拉电阻,输出模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;                // 翻转速率,2MHz
    GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStruct);                // 调用库函数,使用以上参数初始化GPIO
    CS_HIGH;                                                     // 拉高片选信号,停止通信
    // CLK
    GPIO_InitStruct.GPIO_Pin   = XPT2046_CLK_PIN;                // CLK
    GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStruct);
    // MOSI
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MOSI_PIN;               // MOSI
    GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStruct);
    // MISO, 输入模式
    GPIO_InitStruct.GPIO_Pin   = XPT2046_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;                   // MISO, 输入模式
    GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStruct);
    // IRQ
    GPIO_InitStruct.GPIO_Pin  = XPT2046_IRQ_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;                    // 输入模式,该信号只作指示用,不使用中断
    GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStruct);
#endif

    CLK_LOW ;                                                    // XPT2046 时钟要求 CLK 在时钟低电平时发送串行数据,见芯片手册时序介绍
    MOSI_0;                                                      // XPT2046 要求 MOSI 时钟低电平
    CS_LOW;                                                      // 拉低片选信号,使 XTP2046 开始通信

    // 通过读取标志位, 判断是否已经校准
    /* W25Q128 removed: fixed calibration coefficients kept in RAM.
       portrait(dir=0): 0.0675/0.091/-18/-19;
       landscape(dir=1): X轴ADC反向,斜率必须为负(实测2026-10): -0.09184/0.077956/338/-45 */
    if (xXPT2046.dir == 0)
    {
        xXPT2046.xfac = 0.0675f; xXPT2046.yfac = 0.091f;
        xXPT2046.xoff = -18;      xXPT2046.yoff = -19;
    }
    else
    {
        xXPT2046.xfac = -0.09184f; xXPT2046.yfac = 0.077956f;
        xXPT2046.xoff = 338;       xXPT2046.yoff = -45;
    }

    XPT2046_Cmd(ENABLE);                                         // 打开触摸屏
}



/******************************************************************************
 * 函数名称 XPT2046_ReCalibration
 * 功  能: 重新校准触摸屏,
 *          原例程校准完成后数据存到外部 FLASH, 掉电下次有效
 * 入  参 无
 * 返 回: 0_校准成功
 *          1_校准失败
 ******************************************************************************/
uint8_t  XPT2046_ReCalibration(void)
{
    uint16_t pixelOff = 30;   // 偏移像素,用于画十字
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
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏(开始)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏(停止)
    adcX1 = xXPT2046 .adcX;                                           // 记录读取到的 adc 值
    adcY1 = xXPT2046.adcY;                                            // 记录读取到的 adc 值
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX1);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示
    sprintf(strTemp, "Y:%d", adcY1);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // 显示
    delayUS(800000);

    // 右上角
    crossX = lcdWidth - pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏(开始)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏(停止)
    adcX2 = xXPT2046 .adcX;                                           // 记录读取到的 adc 值
    adcY2 = xXPT2046.adcY;                                            // 记录读取到的 adc 值
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX2);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示
    sprintf(strTemp, "Y:%d", adcY2);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // 显示
    delayUS(800000);

    // 左下角
    crossX = pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏(开始)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏(停止)
    adcX3 = xXPT2046 .adcX;                                           // 记录读取到的 adc 值
    adcY3 = xXPT2046.adcY;                                            // 记录读取到的 adc 值
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX3);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示
    sprintf(strTemp, "Y:%d", adcY3);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // 显示
    delayUS(800000);

    // 右下角
    crossX = lcdWidth - pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // 打开触摸屏(开始)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // 画十字
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // 等待按下
    XPT2046_Cmd(DISABLE);                                             // 关闭触摸屏(停止)
    adcX4 = xXPT2046 .adcX;                                           // 记录读取到的 adc 值
    adcY4 = xXPT2046.adcY;                                            // 记录读取到的 adc 值
    LCD_Cross(crossX, crossY, 20, BLACK);                             // 抹去十字
    sprintf(strTemp, "X:%d", adcX4);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // 显示
    sprintf(strTemp, "Y:%d", adcY4);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // 显示
    delayUS(400000);

    //timeCNT=0;
    // 取 adcX、adcY 的平均值; 两组测量平均值, 用于对角线上的十字坐标计算
    adcX1 = (adcX1 + adcX3) / 2;
    adcX2 = (adcX2 + adcX4) / 2;

    adcY1 = (adcY1 + adcY2) / 2;
    adcY2 = (adcY3 + adcY4) / 2;

    xfac = (float)(pixelOff - (lcdWidth - pixelOff)) / (adcX1 - adcX2);   // 计算屏幕与 LCD 坐标的换算系数,  xfac=(float)(20-320)/(t1x-t2x);
    yfac = (float)(pixelOff - (lcdHeight - pixelOff)) / (adcY1 - adcY2);
    xoff = (lcdWidth - xfac * (adcX1 + adcX2)) / 2;                       // 计算坐标的偏移值, xoff=(320-xfac*(t1x+t2x))/2;
    yoff = (lcdHeight - yfac * (adcY1 + adcY2)) / 2;

    // 写入参数(W25Q 已裁剪:只保存到 RAM)
    writeCorrectingData(xfac, yfac, xoff, yoff);

    xXPT2046.xfac = xfac;
    xXPT2046.xoff = xoff;
    xXPT2046.yfac = yfac;
    xXPT2046.yoff = yoff;

    printf(">>>Calibration done! Coefficients kept in RAM only.\r\r");

    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;  // 系统复位,软复位
    return 0;
}



/******************************************************************************
 * 函数名称 XPT2046_Cmd
 * 功  能: 触摸屏开关
 *          用于单片机比较忙时, 内部使用触摸屏的状态函数, 人为关闭以节省触摸芯片资源
 *          退出该状态, 只需再次调用 XPT2046_TouchHandler();
 * 入  参 0_关闭触摸屏检测,以节省资源
 *          1_打开触摸屏
 * 返 回:
 ******************************************************************************/
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



/******************************************************************************
 * 函数名称 XPT2046_IsPressed
 * 功  能: 判断触摸屏是否被按下
 * 入  参 无
 * 返 回: 0-未按下、1-已按下
 ******************************************************************************/
uint8_t XPT2046_IsPressed(void)
{
    static uint8_t status = 0;

    if (xXPT2046.EN == 0)             // 触摸开关关闭; 初始化后默认是开启的; 若手动关闭过,则出现某些情况下触摸没反应是正常的
        return 0;

    status = IRQ_READ ? 0 : 1 ;       // 空闲时为高电平,按下时为低电平
    if (status == 1)                  // 有触摸按下
    {
        if (readAdcXY() == 0)         // 获取 XPT2046 的触摸位置(电压值)
            return 0;                 // ADC 范围校验失败(MISO 断线等),丢弃
        adcXYToLcdXY();               // 换算成显示坐标的值; 换算后的 XY 值可以通过函数 XPT2046_GetX()、XPT2046_GetY() 获取;
        uint16_t x1 = xXPT2046.lcdX;
        uint16_t y1 = xXPT2046.lcdY;

        if (readAdcXY() == 0)         // 第二次采样校验失败也丢弃
            return 0;
        adcXYToLcdXY();               // 换算成显示坐标的值; 换算后的 XY 值可以通过函数 XPT2046_GetX()、XPT2046_GetY() 获取;
        uint16_t x2 = xXPT2046.lcdX;
        uint16_t y2 = xXPT2046.lcdY;

        // 两次测量差值(像素)
        // 阈值取 10:电阻屏轻按/按下沿抖动通常 4~8px,阈值 3 会把正常按压
        // 误判为"未按下",导致上层 LVGL 看到 press/release 闪烁、点击丢失;
        // 10 仍可滤掉真正的野点(跳变几十~上百像素)
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

    return  status ;      // 返回:0-未按下、1-已按下
}



/******************************************************************************
 * 函数名称 XPT2046_GetX
 * 功  能: 获取触摸位置的横坐标值 (X)
 * 入  参 无
 * 返 回: 横坐标值 (X)
 ******************************************************************************/
uint16_t  XPT2046_GetX(void)
{
    return xXPT2046.lcdX;
}



/******************************************************************************
 * 函数名称 XPT2046_GetY
 * 功  能: 获取触摸位置的纵坐标值 (Y)
 * 入  参 无
 * 返 回: 纵坐标值 (Y)
 ******************************************************************************/
uint16_t  XPT2046_GetY(void)
{
    return xXPT2046.lcdY;
}



/******************************************************************************
 * 函数名称 XPT2046_TouchDown
 * 功  能: 触摸屏按下时的处理
 *          空回调函数, 用户可以自行编写代码
 * 入  参
 * 返 回:
 ******************************************************************************/
void XPT2046_TouchDown(void)
{
//    // 示例:在触摸处画一个黄色点
//    LCD_DrawPoint(xXPT2046.lcdX, xXPT2046.lcdY, YELLOW);
//    // 示例:把触摸坐标显示到 LCD
//    static char strTem[20];
//    sprintf(strTem, "%3d  %3d", xXPT2046.lcdX, xXPT2046.lcdY);
//    LCD_String(180, 290, strTem, 12, WHITE, BLACK);
}



/******************************************************************************
 * 函数名称 XPT2046_TouchUp
 * 功  能: 触摸屏松开时的处理
 *          空回调函数, 用户可以自行编写代码
 * 入  参
 * 返 回:
 ******************************************************************************/
void XPT2046_TouchUp(void)
{


}
