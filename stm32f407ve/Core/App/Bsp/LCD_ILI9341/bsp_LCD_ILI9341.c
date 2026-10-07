#include "bsp_LCD_ILI9341.h"
#include "font.h"
#include <stdio.h>                  /* printf in LCD init/demo functions */
#include "FreeRTOS.h"
#include "cmsis_os.h"               /* 信号量: DMA 完成时通知 GuiTask */
#include "stm32f4xx_hal.h"          /* DMA_HandleTypeDef */


/* ===== LCD flush DMA 相关 =====
 * LVGL 调用 LCD_DispFlush 时,先整区设置 ILI9341 窗口(0x2A/0x2B/0x2C),
 * 再用 DMA2 memory-to-memory 把像素缓冲批量搬到 FSMC 的 LCD_RAM 地址(0x60020000),
 * 省掉原来 CPU 逐像素写 LCD->LCD_RAM 的循环。DMA 完成后通过回调通知 LVGL。 */
DMA_HandleTypeDef hdma_lcd;

static osSemaphoreId_t lcdDmaDoneSem;   /* DMA 完成信号量(flush 冲突时等待) */
static volatile uint8_t lcdDmaBusy = 0U;
static void (*lcdFlushDoneCb)(void) = NULL;  /* DMA 完成回调(由 lv_port_disp 注册) */

/* DMA2 Stream5 全局中断(LCD flush 传输完成) */
void DMA2_Stream5_IRQHandler(void);

/* DMA 初始化(在 LCD_Init 中调用,前向声明) */
static void LCD_DMA_Init(void);







/*****************************************************************************
 ** 背光控制
****************************************************************************/
#define LCD_BL_ON    LCD_BL_GPIO-> BSRR = LCD_BL_PIN;       // 点亮背光，引脚输出高电平
#define LCD_BL_OFF   LCD_BL_GPIO-> BSRR = LCD_BL_PIN << 16; // 关闭背光，引脚输出低电平


static void setCursor(uint16_t Xpos, uint16_t Ypos);        // 设置光标位置


typedef struct                                              // LCD重要参数集
{
    uint16_t width;                                         // LCD 宽度
    uint16_t height;                                        // LCD 高度
    uint16_t id;                                            // LCD ID
    uint8_t  dir;                                           // 屏幕显示方向: 0-竖屏, 1-横屏
    uint8_t  FlagInit;                                      // 初始化完成标志
} xLCD_TypeDef;
static xLCD_TypeDef xLCD = {0};                             // 定义LCD重要参数

// 使用NOR/SRAM的 Bank1.sector1，地址位HADDR[27,26]=11，A6作为数据/命令区分线
// 注意：访问时STM32字节地址会右移一位！ 111 1110=0X7E
volatile typedef struct                                     // LCD地址结构体
{
    uint16_t LCD_REG;
    uint16_t LCD_RAM;
} LCD_TypeDef;
#define LCD       ((LCD_TypeDef *) 0x6001FFFE)              // (0x60000000 | 0x0001FFFE)



// us级延时
static void delay_us(volatile uint32_t times)   // 微秒级延时函数，传入值约等于延时的微秒数，内部以空循环实现，相对精准
{
    times = times * 20;
    while (--times)
        __nop();
}



/******************************************************************************
 * 函数名称: delay_ms
 * 功能描述: 毫秒级延时函数
 * 备    注: 1.系统时钟168MHz
 *          2.需勾选 Options/C++/One ELF Section per Function
 *          3.代码优化等级设为 Level 3(-O3)
 * 入    参: uint32_t ms  延时数值
 * 返回值  : 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;         // 使用volatile修饰，防止编译器把此变量优化掉
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                        // 函数内部延时，防止循环被编译器优化掉
}



// 读寄存器数据
uint16_t  readReg(uint16_t  LCD_Reg)
{
    LCD->LCD_REG = LCD_Reg;                 // 写入要读的寄存器号
    delay_us(5);
    return LCD->LCD_RAM;                    // 返回读到的数据值
}



// BGR转换成RGB值; 返回RGB格式的颜色值
uint16_t  LCD_BGR2RGB(uint16_t  c)
{
    uint16_t   r, g, b, rgb;
    b = (c >> 0) & 0x1f;
    g = (c >> 5) & 0x3f;
    r = (c >> 11) & 0x1f;
    rgb = (b << 11) + (g << 5) + (r << 0);
    return (rgb);
}



/******************************************************************
 * 函数名称: LCD_Init
 * 功能描述: 初始化LCD（液晶显示芯片ILI9341）
 * 入    参: 无
 * 备    注:
 *****************************************************************/
void LCD_Init(void)
{
    xLCD.FlagInit = 0;                                       // LCD初始化完成标志; 0-未初始化完成, 1-初始化完成

    /** 1.初始化GPIO **/
    GPIO_InitTypeDef  GPIO_InitStruct = {0};                 // GPIO引脚初始化结构体

#ifdef USE_HAL_DRIVER                                        // HAL库相关配置
    // 开启GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();                            // 使能GPIOA时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();                            // 使能GPIOB时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();                            // 使能GPIOC时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();                            // 使能GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();                            // 使能GPIOE时钟
    /// 初始化引脚-背光
    GPIO_InitStruct.Pin   = LCD_BL_PIN;                      // 背光引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;             // 引脚模式：推挽输出
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                     // 带上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // 引脚翻转速率：高速
    HAL_GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);            // 初始化
    // 初始化 GPIOD引脚
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                  // 引脚模式：推挽复用
    GPIO_InitStruct.Pull = GPIO_PULLUP;                      // 带上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // 引脚翻转速率：高速
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;              // 引脚复用为FSMC
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);                  // 初始化
    // 初始化 GPIOE引脚
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                  // 引脚模式：推挽复用
    GPIO_InitStruct.Pull = GPIO_PULLUP;                      // 带上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // 引脚翻转速率：高速
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;              // 引脚复用为FSMC
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);                  // 初始化
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // 标准库
    // 使能GPIO时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN ;  // 使能GPIOA、B、C、D、E时钟
    // 初始化引脚-背光
    GPIO_InitStruct.GPIO_Pin = LCD_BL_PIN;                   // 引脚
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;               // 普通输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // 推挽
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;           // 50MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                // 上拉
    GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);                // 初始化，该引脚为背光使能脚
    // 初始化引脚-GPIOD
    GPIO_InitStruct.GPIO_Pin   = (3 << 0) | (3 << 4) | (7 << 8) | (3 << 14); // PD0,1,4,5,8,9,10,14,15 AF OUT
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;               // 复用功能
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // 推挽
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;          // 100MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;               // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);                      // 初始化
    // 初始化引脚-GPIOE
    GPIO_InitStruct.GPIO_Pin = (0X1FF << 7);                 // PE7~15,AF OUT
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                // 复用功能
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // 推挽
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;          // 100MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                // 上拉
    GPIO_Init(GPIOE, &GPIO_InitStruct);                      // 初始化
    // 初始化引脚-RS_PD11
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;                  // RS
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                // 复用功能
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // 推挽
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;          // 100MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);                      // 初始化
    // 初始化引脚-NE1_PD7
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;                   // PD7, FSMC_NE1
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                // 复用功能
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // 推挽
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;          // 100MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);                      // 初始化
    // 引脚复用配置
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);  // D2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);  // D3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);  // NOE_RD
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);  // NWE_WE
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);  // D13
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);  // D14
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC); // D15
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC); // D0
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC); // D1
    // 引脚复用配置
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC); //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC); //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC); //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC); //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC); //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC); //
    // 引脚复用配置
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); // RS
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);  // CS
#endif

    /** 2.FSMC 初始化，使用FSMC模拟8080接口 **/

    // 使能FSMC时钟
    RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;                      // 使能FSMC时钟

    // 每个BANK分为寄存器1~4，这里使用的是第1个寄存器
    // BANK1:BTCR[0]和[1],BWTR[0];
    // BANK2:BTCR[2]和[3],BWTR[1];
    // BANK3:BTCR[4]和[5],BWTR[2];
    // BANK4:BTCR[6]和[7],BWTR[3];
    FSMC_Bank1->BTCR[0]   = 0X00000000;
    FSMC_Bank1->BTCR[0 + 1] = 0X00000000;
    FSMC_Bank1E->BWTR[0]  = 0X00000000;

    //配置BCR寄存器    使用异步模式
    FSMC_Bank1->BTCR[0] |= 0x01 << 12;        // 存储器写使能
    FSMC_Bank1->BTCR[0] |= 0x01 << 14;        // 读写使用相同的时序
    FSMC_Bank1->BTCR[0] |= 0x01 << 4;         // 存储器数据总线宽度为16bit

    //读时序参数配置
    FSMC_Bank1->BTCR[0 + 1] |= 0x00 << 28;    // 模式A
    FSMC_Bank1->BTCR[0 + 1] |= 0X0F << 0;     // 地址建立时间(ADDSET)为15个HCLK 1/168M=6ns*15=90ns
    FSMC_Bank1->BTCR[0 + 1] |= 0x3C << 8;     // 数据建立时间(DATAST)为60个HCLK =6*60=360ns
    //写时序参数配置
    FSMC_Bank1E->BWTR[0] |= 0x00 << 28;       // 模式A
    FSMC_Bank1E->BWTR[0] |= 0x09 << 0;        // 地址建立时间(ADDSET)为9个HCLK=54ns
    FSMC_Bank1E->BWTR[0] |= 0x08 << 8;        // 数据建立时间(DATAST)为6ns*9个HCLK=54ns

    //使能BANK1中的第1个存储区
    FSMC_Bank1->BTCR[0] |= 0x01;              // 使能BANK1中的第1个存储区

    delay_ms(50);                             // delay 50 ms
    LCD->LCD_REG = 0x0000;                    // 写入要写的寄存器号
    LCD->LCD_RAM = 0x0000;                    // 写数据
    delay_ms(50);                             // delay 50 ms
    xLCD.id = readReg(0x0000);

    LCD->LCD_REG = 0XD3;                      // 读取9341 ID的命令
    xLCD.id = LCD->LCD_RAM;                   // dummy read
    xLCD.id = LCD->LCD_RAM;                   // 读到0X00
    xLCD.id = LCD->LCD_RAM;                   // 读取93
    xLCD.id <<= 8;
    xLCD.id |= LCD->LCD_RAM;                  // 读取41
    printf("LCD driver IC ID: %x\r\n", xLCD.id);  /* print LCD controller ID */

    //优化写时序参数配置，加快刷屏速度
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 0);      // 地址建立时间(ADDSET)清零
    FSMC_Bank1E->BWTR[0] |= 2 << 8;           // 数据建立时间(DATAST)为6ns*3个HCLK=18ns
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 8);      // 数据建立时间(DATAST)清零
    FSMC_Bank1E->BWTR[0] |= 3 << 0;           // 地址建立时间(ADDSET)为3个HCLK =18ns

    /** 3.写入ILI9341 初始化寄存器 **/

    // 电源配置，厂商推荐配置，一般不需要修改
    LCD->LCD_REG = 0xCF;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0xC1;
    LCD->LCD_RAM = 0X30;
    LCD->LCD_REG = 0xED;
    LCD->LCD_RAM = 0x64;
    LCD->LCD_RAM = 0x03;
    LCD->LCD_RAM = 0X12;
    LCD->LCD_RAM = 0X81;
    LCD->LCD_REG = 0xE8;
    LCD->LCD_RAM = 0x85;
    LCD->LCD_RAM = 0x10;
    LCD->LCD_RAM = 0x7A;
    LCD->LCD_REG = 0xCB;
    LCD->LCD_RAM = 0x39;
    LCD->LCD_RAM = 0x2C;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x34;
    LCD->LCD_RAM = 0x02;
    LCD->LCD_REG = 0xF7;
    LCD->LCD_RAM = 0x20;
    LCD->LCD_REG = 0xEA;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0xC0;  // Power control
    LCD->LCD_RAM = 0x1B;  // VRH[5:0]
    LCD->LCD_REG = 0xC1;  // Power control
    LCD->LCD_RAM = 0x01;  // SAP[2:0];BT[3:0]
    LCD->LCD_REG = 0xC5;  // VCM control
    LCD->LCD_RAM = 0x30;  // 3F
    LCD->LCD_RAM = 0x30;  // 3C
    LCD->LCD_REG = 0xC7;  // VCM control2
    LCD->LCD_RAM = 0XB7;
    LCD->LCD_REG = 0x36;  // Memory Access Control
    LCD->LCD_RAM = 0x48;
    LCD->LCD_REG = 0x3A;
    LCD->LCD_RAM = 0x55;
    LCD->LCD_REG = 0xB1;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x1A;
    LCD->LCD_REG = 0xB6;  // Display Function Control
    LCD->LCD_RAM = 0x0A;
    LCD->LCD_RAM = 0xA2;
    LCD->LCD_REG = 0xF2;  // 3Gamma Function Disable
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0x26;  // Gamma curve selected
    LCD->LCD_RAM = 0x01;
    LCD->LCD_REG = 0xE0;  // Set Gamma
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x2A;
    LCD->LCD_RAM = 0x28;
    LCD->LCD_RAM = 0x08;
    LCD->LCD_RAM = 0x0E;
    LCD->LCD_RAM = 0x08;
    LCD->LCD_RAM = 0x54;
    LCD->LCD_RAM = 0XA9;
    LCD->LCD_RAM = 0x43;
    LCD->LCD_RAM = 0x0A;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0XE1;   // Set Gamma
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x15;
    LCD->LCD_RAM = 0x17;
    LCD->LCD_RAM = 0x07;
    LCD->LCD_RAM = 0x11;
    LCD->LCD_RAM = 0x06;
    LCD->LCD_RAM = 0x2B;
    LCD->LCD_RAM = 0x56;
    LCD->LCD_RAM = 0x3C;
    LCD->LCD_RAM = 0x05;
    LCD->LCD_RAM = 0x10;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x3F;
    LCD->LCD_RAM = 0x3F;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_REG = 0x2B;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x01;
    LCD->LCD_RAM = 0x3f;
    LCD->LCD_REG = 0x2A;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0xef;
    LCD->LCD_REG = 0x11;  // 退出睡眠模式

    delay_ms(120);
    LCD->LCD_REG = 0x29;  // 开启显示

    /** 4.收尾工作 **/

    LCD_SetDir(0);        // 默认竖屏显示方向
    LCD_Fill(0, 0, xLCD.width, xLCD.height, BLACK);
    LCD_BL_ON;            // 点亮LCD背光
    xLCD.FlagInit = 1;

    /* 配置 DMA2 Stream5 用于 LCD flush(整区像素批量搬运) */
    LCD_DMA_Init();
}



/******************************************************************
 * 函数名称: LCD_SetDir
 * 功能描述: 设置屏幕显示方向
 * 入    参: uint8_t dir     0-竖屏 1-横屏
 * 备    注: 无论使用什么显示方向，触摸都要重新校准
 *          寄存器参数取值: 0-竖屏, 3-竖屏反向, 5-横屏, 6-横屏反向; 注意：无论什么方向显示，触摸都要重新校准
 * 返回值 : 无
 *****************************************************************/
void LCD_SetDir(uint8_t dir)
{
    uint16_t  regval = 0;
    uint16_t  temp = 0;

    if (dir == 1)
        dir = 6;

    if (dir == 0 || dir == 3)         // 竖屏
    {
        xLCD.dir = 0;
        xLCD.width = LCD_WIDTH;
        xLCD.height = LCD_HEIGHT;
    }
    else                              // 横屏
    {
        xLCD.dir = 1;
        xLCD.width = LCD_HEIGHT;
        xLCD.height = LCD_WIDTH;
    }

    if (dir == 0) regval |= (0 << 7) | (0 << 6) | (0 << 5); // 竖屏，从上到下、从左到右
    if (dir == 3) regval |= (1 << 7) | (1 << 6) | (0 << 5); // 竖屏，从下到上、从右到左
    if (dir == 5) regval |= (0 << 7) | (1 << 6) | (1 << 5); // 横屏，从上到下、从右到左
    if (dir == 6) regval |= (1 << 7) | (0 << 6) | (1 << 5); // 横屏，从下到上、从左到右

    regval |= 0X08;
    LCD->LCD_REG = 0X36;               // 写入要设置的寄存器号
    LCD->LCD_RAM = regval;             // 写数据

    // 确保xLCD.width代表X轴方向像素数，xLCD.height代表Y轴方向像素数，与实际硬件扫描方向一致
    if (regval & 0X20)                 // 当bit5置位为1
    {
        if (xLCD.width < xLCD.height)  // 交换X,Y
        {
            temp = xLCD.width;
            xLCD.width = xLCD.height;
            xLCD.height = temp;
        }
    }
    else
    {
        if (xLCD.width > xLCD.height)  // 交换X,Y
        {
            temp = xLCD.width;
            xLCD.width = xLCD.height;
            xLCD.height = temp;
        }
    }

    //设置X轴坐标（水平方向显示范围）
    LCD->LCD_REG = 0X2A;                      // 列地址设置命令
    LCD->LCD_RAM = 0;                         // 起始列地址高8位
    LCD->LCD_RAM = 0;                         // 起始列地址低8位，从0开始
    LCD->LCD_RAM = (xLCD.width - 1) >> 8;     // 结束列地址高8位
    LCD->LCD_RAM = (xLCD.width - 1) & 0XFF;   // 结束列地址低8位
    //设置Y轴坐标（垂直方向显示范围）
    LCD->LCD_REG = 0X2B;                      // 页(行)地址设置命令
    LCD->LCD_RAM = 0;                         // 起始页地址高8位
    LCD->LCD_RAM = 0;                         // 起始页地址低8位，从0开始
    LCD->LCD_RAM = (xLCD.height - 1) >> 8;    // 结束页地址高8位
    LCD->LCD_RAM = (xLCD.height - 1) & 0XFF;  // 结束页地址低8位
}



//设置光标位置  Xpos:横坐标、Ypos:纵坐标
static void setCursor(uint16_t  Xpos, uint16_t  Ypos)
{
    LCD->LCD_REG = 0X2A;           // 发送指令：设置x坐标
    LCD->LCD_RAM = Xpos >> 8;
    LCD->LCD_RAM = Xpos & 0XFF;
    LCD->LCD_REG = 0X2B;           // 发送指令：设置y坐标
    LCD->LCD_RAM = Ypos >> 8;
    LCD->LCD_RAM = Ypos & 0XFF;
}



/******************************************************************
 * 函数名称: LCD_DrawPoint
 * 功能描述: 画点函数
 * 入    参: x,y:    坐标
 *          _color: 点的颜色
 * 备    注:
 *****************************************************************/
void LCD_DrawPoint(uint16_t  x, uint16_t  y, uint16_t _color)
{
    LCD->LCD_REG = 0X2A;        // 指令：设置x坐标
    LCD->LCD_RAM = x >> 8;      // x坐标 高8位
    LCD->LCD_RAM = x & 0XFF;    // x坐标 低8位

    LCD->LCD_REG = 0X2B;        // 指令：设置y坐标
    LCD->LCD_RAM = y >> 8;      // y坐标 高8位
    LCD->LCD_RAM = y & 0XFF;    // y坐标 低8位

    LCD->LCD_REG = 0X2C;        // 指令：开始写GRAM
    LCD->LCD_RAM = _color;      // 颜色值
}



/******************************************************************
 * 函数名称: LCD_Fill
 * 功能描述: 在指定区域内填充单个颜色
 * 入    参: uint16_t startX     起始X坐标
 *          uint16_t startY     起始Y坐标
 *          uint16_t endX       结束X坐标
 *          uint16_t endY       结束Y坐标
 *          uint16_t color      颜色值
 * 返回值 : 无
 *****************************************************************/
void LCD_Fill(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY, uint16_t color)
{
    uint16_t  xlen = 0;
    xlen = endX - startX + 1;
    for (uint16_t i = startY; i <= endY; i++)
    {
        setCursor(startX, i);                  // 设置光标位置
        LCD->LCD_REG = 0X2C;                   // 开始写GRAM
        for (uint16_t j = 0; j < xlen; j++)
            LCD->LCD_RAM = color;              // 颜色值
    }
}



/******************************************************************
 * 函数名称: drawAscii
 * 功能描述: 在指定位置显示一个字符
 * 入    参: uint16_t x,y     起始坐标
 *          uint8_t  num     要显示的字符:" "--->"~"
 *          uint8_t  size    字号大小 12/16/24/32
 *          uint32_t bColor  背景颜色
 *          uint32_t fColor  前景颜色
 * 备    注: 字符采用从左到右、从上到下扫描的方式取模
 *****************************************************************/
static void drawAscii(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint32_t bColor, uint32_t fColor)
{
    static uint8_t temp = 0;
    static uint8_t csize = 0;
    static uint16_t y0 = 0;

    y0 = y;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);   // 得到一个字符应占用的字节数
    num = num - ' ';                                          // 得到偏移后字符的值（ASCII字库从空格开始取模，减去' '得到对应字符在字库中的序号）
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
            else             LCD_DrawPoint(x, y, bColor);     // 不点亮该点

            temp <<= 1;
            y++;
            if (y >= xLCD.height)    return;                  // 超出屏幕高度(下)
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                if (x >= xLCD.width) return;                  // 超出屏幕宽度(右)
                break;
            }
        }
    }
}




/******************************************************************************
 * 函数名称: LCD_String
 * 功能描述: 在LCD上显示字符串(仅支持英文/数字/ASCII)
 * 备    注: 英文字模数据放在头文件font.h中，统一编译进芯片内部Flash
 *          中文字模数据原从外部Flash字库芯片W25Q128读取，
 *          本工程已移除W25Q128且LVGL使用自带字体，中文显示功能已裁剪。
 * 入    参: uint16_t   x       字符串起始X坐标
 *          uint16_t   y       字符串起始y坐标
 *          char*      pFont   要显示的字符串起始地址
 *          uint8_t    size    字号大小:12 16 24 32
 *          uint32_t   fColor  前景颜色
 *          uint32_t   bColor  背景颜色
 * 返回值:  无
 * 备    注: 最后更新于2020年05月1日
 ******************************************************************************/
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor)
{
    if (xLCD .FlagInit == 0) return;

    uint16_t xStart = x;

    if (size != 12 && size != 16 && size != 24 && size != 32)      // 如果字号不支持
        size = 24;

    while (*pFont != 0)                                            // 循环读取字符，直到遇到'\0'时停止
    {
        if (x > (xLCD.width - size))                               // 坐标判断，超出宽度则换行（回到行首并下移一行）
        {
            x = xStart;
            y = y + size;
        }
        if (y > (xLCD.height - size))                              // 坐标判断，超出高度则直接退出（已从屏幕底部溢出）
            return;

        if (*pFont < 128)                                          // 仅支持 ASCII 字符
        {
            drawAscii(x, y, *pFont, size, bColor, fColor);
        }
        pFont++;
        x += size / 2;
    }
}



/*****************************************************************
 * 函数名称:LCD_DisplayOn
 * 功能描述:开启显示
 * 入    参:
 * 返回值:
*****************************************************************/
void LCD_DisplayOn(void)
{
    LCD->LCD_REG = 0X29;                // 开启显示
    LCD_BL_GPIO->BSRR |= LCD_BL_PIN;    // 打开背光
}



/*****************************************************************
 * 函数名称:LCD_DisplayOff
 * 功能描述:关闭显示, 熄灭背光
 * 入    参:
 *
 * 返回值:
*****************************************************************/
void LCD_DisplayOff(void)
{
    LCD->LCD_REG = 0X28;                                // 关闭显示
    LCD_BL_GPIO->BSRR |= (uint32_t)LCD_BL_PIN << 16;    // 关闭背光
}



/******************************************************************
 * 函数名称: LCD_GetWidth
 * 功能描述: 获取屏幕的宽度，单位:像素
 * 备    注: 无
 * 入    参: 无
 * 返回值 : uint16_t
 *****************************************************************/
uint16_t LCD_GetWidth(void)
{
    return xLCD.width;

}



/******************************************************************
 * 函数名称: LCD_GetHeight
 * 功能描述: 获取屏幕的高度，单位:像素
 * 备    注: 无
 * 入    参: 无
 * 返回值 : uint16_t
 *****************************************************************/
uint16_t LCD_GetHeight(void)
{
    return xLCD.height;
}



/******************************************************************
 * 函数名称: LCD_GetDir
 * 功能描述: 获取屏幕显示方向
 * 备    注: 无
 * 入    参: 无
 * 返回值 : 0-竖屏,1-横屏
 *****************************************************************/
uint8_t  LCD_GetDir(void)
{
    return xLCD.dir;
}



/*****************************************************************
 * 函数名称:LCD_Line
 * 功能描述:画线段
 * 入    参:xStart      起始x坐标
 *         yStart      起始y坐标
 *         xEnd        结束x坐标
 *         yEnd        结束y坐标
 *         color       颜色
 * 返回值:无
******************************************************************/
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
    if (delta_x > delta_y)distance = delta_x;            // 选取基本增量坐标轴
    else distance = delta_y;
    for (t = 0; t <= distance + 1; t++)                  // 画线计算
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




/******************************************************************
 * 函数名称: LCD_Circle
 * 功能描述: 在指定位置画圆
 * 入    参: uint16_t Xpos     X方向坐标
 *          uint16_t Ypos     Y方向坐标
 *          uint16_t Radius   圆半径
 *          uint16_t _color   颜色
 * 备    注:
 *****************************************************************/
void LCD_Circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t _color)
{
    int16_t mx = Xpos, my = Ypos, x = 0, y = Radius;
    int16_t d = 1 - Radius;
    while (y > x)
    {
        LCD_DrawPoint(x + mx, y + my, _color);
        LCD_DrawPoint(-x + mx, y + my, _color);
        LCD_DrawPoint(-x + mx, -y + my, _color);
        LCD_DrawPoint(x + mx, -y + my, _color);
        LCD_DrawPoint(y + mx, x + my, _color);
        LCD_DrawPoint(-y + mx, x + my, _color);
        LCD_DrawPoint(y + mx, -x + my, _color);
        LCD_DrawPoint(-y + mx, -x + my, _color);
        if (d < 0)
        {
            d += 2 * x + 3;
        }
        else
        {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}



/*****************************************************************
 * 函数名称:LCD_Rectangle
 * 功能描述:画一个矩形 (方框或实心)
 * 入    参:x       起始x坐标
 *         y       起始y坐标
 *         width   矩形的宽度
 *         height  矩形的高度
 *         color   颜色
 *         filled  是否实心填充
 * 返回值:无
*****************************************************************/
void LCD_Rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t filled)
{
    if (filled)
    {
        LCD_Fill(x, y, width, height, color);
    }
    else
    {
        LCD_Line(x, y, x + width - 1, y, color);
        LCD_Line(x, y + height - 1, x + width - 1, y + height - 1, color);
        LCD_Line(x, y, x, y + height - 1, color);
        LCD_Line(x + width - 1, y, x + width - 1, y + height - 1, color);
    }
}



/******************************************************************
 * 函数名称: LCD_Image
 * 功能描述: 在指定位置显示一张图片
 * 备    注: 图片数据放在font.h中，只能显示小于屏幕的图片
 *          Image2Lcd输出水平扫描，16位真彩色，高位在前(即高位字节对应颜色的高位，颜色高低位相反)
 * 入    参: uint16_t x,y     左上角起始坐标
 *          uint16_t width   图片宽度
 *          uint16_t height  图片高度
 *          uint8_t* image   数据地址
 * 返回值 : 无
 *****************************************************************/
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image)
{
    for (uint16_t i = 0; i < height; i++)           // 一行一行地显示
    {
        LCD->LCD_REG = 0X2A;                        // 设置x坐标
        LCD->LCD_RAM = x >> 8;                      // X坐标高8位
        LCD->LCD_RAM = x ;                          // X坐标低8位（因为指令只对低8位有效，所以等效于 x & 0XFF）
        LCD->LCD_REG = 0X2B;                        // 设置y坐标
        LCD->LCD_RAM = (y + i) >> 8;                // Y坐标高8位
        LCD->LCD_RAM = y + i;                       // Y坐标低8位（因为指令只对低8位有效，所以等效于 Y & 0XFF）
        LCD->LCD_REG = 0X2C;                        // 开始写GRAM
        for (uint16_t j = 0; j < width; j++)        // 一行的像素循环，每个像素占两个字节
        {
            LCD->LCD_RAM = image[1] << 8 | *image;  // 写入16位真彩色数据
            image += 2;                             // 数据指针指向下一个字节
        }
    }
}



/******************************************************************
 * 函数名称: LCD_Cross
 * 功能描述: 在指定位置画一个十字符号，用于触摸屏校准
 * 入    参: uint16_t x  十字符号中心点的坐标x
 *          uint16_t y  十字符号中心点的坐标y
 *          uint16_t len     十字符号的边长
 *          uint32_t fColor  颜色
 * 返回值 : 无
 * 备    注:
 *****************************************************************/
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor)
{
    uint16_t temp = len / 2;

    LCD_Line(x - temp, y, x + temp, y, fColor);
    LCD_Line(x, y - temp, x, y + temp, fColor);
}



/*****************************************************************
 * 函数名称:LCD_ReadPoint
 * 功能描述:读取某一个点的颜色（该点的像素值）
 * 入    参:x      x坐标
 *         y      y坐标
 * 返回值:16位真彩色颜色值
*****************************************************************/
uint16_t  LCD_ReadPoint(uint16_t  x, uint16_t  y)
{
    uint16_t  r = 0, g = 0, b = 0;
    if (x >= xLCD.width || y >= xLCD.height)return 0; // 超出屏幕范围，直接返回
    setCursor(x, y);
    LCD->LCD_REG = 0X2E;                              // 发送读GRAM指令

    r = LCD->LCD_RAM;                                 // dummy Read

    delay_us(20);
    r = LCD->LCD_RAM;                                 // 实际RGB颜色

    delay_us(20);
    b = LCD->LCD_RAM;
    g = r & 0XFF;                                     // 第一次读到的是RG值，R在前、G在后，各占8位
    g <<= 8;

    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));
}




/******************************************************************
 * 函数名称: LCD_DMA_Init
 * 功能描述: 配置 DMA2 Stream5 为 memory-to-memory 模式,用于把 LVGL 像素缓冲
 *          批量搬到 FSMC 的 LCD_RAM 地址(0x60020000)。
 * 备    注: 源地址 = 像素缓冲(自增), 目的地址 = LCD_RAM(不增), 数据宽度 16bit
 *****************************************************************/
static void LCD_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_lcd.Instance                 = DMA2_Stream5;
    hdma_lcd.Init.Channel             = DMA_CHANNEL_0;        /* M2M 模式通道无关 */
    hdma_lcd.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma_lcd.Init.PeriphInc           = DMA_PINC_ENABLE;      /* 源(像素缓冲)自增 */
    hdma_lcd.Init.MemInc              = DMA_MINC_DISABLE;     /* 目的(LCD_RAM)不增 */
    hdma_lcd.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_lcd.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_lcd.Init.Mode                = DMA_NORMAL;
    hdma_lcd.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_lcd.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    (void)HAL_DMA_Init(&hdma_lcd);

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 6U, 0U);   /* 优先级 6(允许 FreeRTOS API) */
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    if (lcdDmaDoneSem == NULL)
        lcdDmaDoneSem = osSemaphoreNew(1U, 1U, NULL);  /* 初值 1 = DMA 空闲; flush 时 acquire, 完成中断 release */
}

/* 注册 DMA 完成回调(LVGL 的 lv_disp_flush_ready 由该回调触发) */
void LCD_SetFlushDoneCallback(void (*cb)(void))
{
    lcdFlushDoneCb = cb;
}

/******************************************************************
 * 函数名称: LCD_DispFlush
 * 功能描述: 在指定区域显示一张图片(LVGL 刷新回调)
 * 备    注: 先整区设置 ILI9341 窗口,再用 DMA 批量搬运像素到 GRAM。
 *          DMA 传输完成后触发回调调用 lv_disp_flush_ready,GuiTask 不阻塞。
 * 入    参: uint16_t   x        左上角起始X坐标
 *          uint16_t   y        左上角起始Y坐标
 *          uint16_t   width    右下角X坐标(x2)
 *          uint16_t   height   右下角Y坐标(y2)
 *          uint16_t  *pData    数据地址
 * 返回值 : 无
 *****************************************************************/
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData)
{
    /* 1.等待上一次 DMA 完成(窗口设置必须在 DMA 空闲后进行,
     *    否则 ILI9341 正接收像素数据时收到命令会错乱) */
    (void)osSemaphoreAcquire(lcdDmaDoneSem, osWaitForever);

    /* 2.整区设置列地址(0x2A)和页地址(0x2B),ILI9341 收到 0x2C 后自动递增 GRAM 地址 */
    LCD->LCD_REG = 0x2A;
    LCD->LCD_RAM = x >> 8;
    LCD->LCD_RAM = x & 0xFF;
    LCD->LCD_RAM = width >> 8;
    LCD->LCD_RAM = width & 0xFF;

    LCD->LCD_REG = 0x2B;
    LCD->LCD_RAM = y >> 8;
    LCD->LCD_RAM = y & 0xFF;
    LCD->LCD_RAM = height >> 8;
    LCD->LCD_RAM = height & 0xFF;

    LCD->LCD_REG = 0x2C;                              /* 开始写 GRAM */

    /* 3.用寄存器直接启动 DMA(避开 HAL State 管理导致第二次启动失败的问题) */
    uint32_t pixelCount = (uint32_t)(width - x + 1U) * (uint32_t)(height - y + 1U);
    lcdDmaBusy = 1U;

    DMA_Stream_TypeDef *s = DMA2_Stream5;
    s->CR  &= ~DMA_SxCR_EN;                          /* 禁用流 */
    while (s->CR & DMA_SxCR_EN) { ; }                /* 等待禁用完成 */
    s->PAR  = (uint32_t)pData;                       /* 源: 像素缓冲(自增) */
    s->M0AR = (uint32_t)&LCD->LCD_RAM;               /* 目的: LCD_RAM(不增) */
    s->NDTR = pixelCount;                            /* 数据长度 */
    s->CR  |= DMA_SxCR_TCIE;                         /* 使能传输完成中断 */
    s->CR  |= DMA_SxCR_EN;                           /* 启动 DMA 流 */
}

/* DMA2 Stream5 传输完成中断: 直接操作寄存器,绕过 HAL 回调机制,确保可靠触发 */
void DMA2_Stream5_IRQHandler(void)
{
    /* DMA2 Stream5 的传输完成标志在 HISR 的 bit11(TCIF5) */
    if (DMA2->HISR & DMA_HISR_TCIF5)
    {
        /* 清除传输完成标志(写 HIFCR 的 CTCIF5 位) */
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;

        /* 清忙标志 + 释放信号量 + 通知 LVGL flush 完成 */
        lcdDmaBusy = 0U;
        if (lcdDmaDoneSem != NULL)
            (void)osSemaphoreRelease(lcdDmaDoneSem);
        if (lcdFlushDoneCb != NULL)
            lcdFlushDoneCb();
    }
}

