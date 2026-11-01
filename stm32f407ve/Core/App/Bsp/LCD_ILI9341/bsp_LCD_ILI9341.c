/* bsp_LCD_ILI9341.c
 * ILI9341 2.8 寸屏驱动：FSMC 8080 并口初始化、命令与像素写入，点/线/圆/矩形/字符/图片绘制，DMA 整块刷屏。
 * 硬件：CS=PD7、RS=PD11(FSMC_A16)、D0~D15 接 PD/PE、WR=PD5、RD=PD4，背光 BL=PA15；屏原生 240×320。
 * 注意：0x6001FFFE 写命令、0x60020000 写数据；MADCTL 的 MV 位决定横竖屏，改方向后触摸必须重新校准。
 */
#include "bsp_LCD_ILI9341.h"
#include "font.h"
#include <stdio.h>                  // printf 打印屏 ID
#include "FreeRTOS.h"
#include "cmsis_os.h"               // 刷屏信号量
#include "stm32f4xx_hal.h"          // DMA_HandleTypeDef


/* DMA 刷屏：先设 0x2A/0x2B 窗口，再用 DMA2 把像素搬到 LCD_RAM(0x60020000)，完成后回调通知 LVGL */
DMA_HandleTypeDef hdma_lcd;

static osSemaphoreId_t lcdDmaDoneSem;   /* DMA 完成信号量(flush 冲突时等待) */
static volatile uint8_t lcdDmaBusy = 0U;
static void (*lcdFlushDoneCb)(void) = NULL;  /* DMA 完成回调(由 lv_port_disp 注册) */

/* DMA2 Stream5 全局中断(LCD flush 传输完成) */
void DMA2_Stream5_IRQHandler(void);

/* DMA 初始化(在 LCD_Init 中调用,前向声明) */
static void LCD_DMA_Init(void);







/*背光控制*/
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

// Bank1/NE1 + FSMC_A16 接 RS：A16=0 访问 0x6001FFFE 写命令，A16=1 访问 0x60020000 写数据
volatile typedef struct                                     // LCD地址结构体
{
    uint16_t LCD_REG;
    uint16_t LCD_RAM;
} LCD_TypeDef;
#define LCD       ((LCD_TypeDef *) 0x6001FFFE)              // (0x60000000 | 0x0001FFFE)



/**
  * 功    能：微秒级忙等延时。单次别超过 1000us，长了占用 CPU 太狠
  * 参    数：times  延时的微秒数，取值 1~几百，单次不超过 1000
  * 返 回 值：无
  */
static void delay_us(volatile uint32_t times)
{
    times = times * 20;                         // 1us≈20 次空循环(168MHz 实测)，换主频需重新标定
    while (--times)
        __nop();
}



/* 毫秒级延时。 */
static volatile uint32_t ulTimesMS;         // volatile 防止 -O3 把延时循环优化掉
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                        // 递减到 0 即延时结束
}



/**
  * 功    能：读 LCD 的某个寄存器：先发命令码再读回数据。比如 0xD3 读 ID4，可用来确认芯片型号
  * 参    数：LCD_Reg  寄存器命令码 0x00~0xFF，例如 0xD3 读 ID4
  * 返 回 值：寄存器数据(按 16 位总线读，高字节通常无意义)
  */
uint16_t  readReg(uint16_t  LCD_Reg)
{
    LCD->LCD_REG = LCD_Reg;                 // 写入要读的寄存器号
    delay_us(5);                            // 命令发出后留 5us 等芯片把数据准备好
    return LCD->LCD_RAM;                    // 返回读到的数据值
}



/**
  * 功    能：把 BGR565 的颜色值转成 RGB565。两种排布的位域正好相反，屏幕发色不对时先查这里
  * 参    数：c  BGR565 颜色值，位域 b[4:0]、g[10:5]、r[15:11]
  * 返 回 值：RGB565 颜色值，位域 r[15:11]、g[10:5]、b[4:0]
  */
uint16_t  LCD_BGR2RGB(uint16_t  c)
{
    uint16_t   r, g, b, rgb;
    b = (c >> 0) & 0x1f;                // 蓝色取低 5 位  bit4~0
    g = (c >> 5) & 0x3f;                // 绿色取中间 6 位 bit10~5
    r = (c >> 11) & 0x1f;               // 红色取高 5 位  bit15~11
    rgb = (b << 11) + (g << 5) + (r << 0);   // 重心排布: B 挪到最高 5 位, R 落到最低 5 位
    return (rgb);
}



/**
  * 功    能：初始化 ILI9341：硬复位 → 读 ID 确认 → 配像素格式和时序 → 开显示
  * 参    数：无
  * 返 回 值：无
  */
void LCD_Init(void)
{
    xLCD.FlagInit = 0;                                       // LCD初始化完成标志; 0-未初始化完成, 1-初始化完成

    /*初始化 GPIO*/
    GPIO_InitTypeDef  GPIO_InitStruct = {0};                 // GPIO引脚初始化结构体

#ifdef USE_HAL_DRIVER                                        // HAL库相关配置
    /*GPIO 时钟*/
    __HAL_RCC_GPIOA_CLK_ENABLE();                            // 背光 PA15
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /*背光：推挽输出，默认低电平(灭)*/
    GPIO_InitStruct.Pin   = LCD_BL_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;             // 推挽输出
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                     // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       // 高速
    HAL_GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);
    /*GPIOD：D0~D3、D13~D15 与 WR/RD 走 FSMC*/
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;              // AF12 = FSMC
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    /*GPIOE：D4~D12 走 FSMC*/
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;              // AF12 = FSMC
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
#endif

#ifdef USE_STDPERIPH_DRIVER                                  // 标准库
    /*标准库写法，本工程走 HAL 分支，此段不参与编译*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN ;  // GPIOA~E 时钟
    /*背光：普通推挽输出*/
    GPIO_InitStruct.GPIO_Pin = LCD_BL_PIN;                   // PA15
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);                // 背光使能脚
    /*GPIOD：D0~D3、D13~D15 与 WR/RD*/
    GPIO_InitStruct.GPIO_Pin   = (3 << 0) | (3 << 4) | (7 << 8) | (3 << 14); // PD0,1,4,5,8,9,10,14,15
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    /*GPIOE：D4~D12*/
    GPIO_InitStruct.GPIO_Pin = (0X1FF << 7);                 // PE7~15
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
    /*RS = PD11(FSMC_A16)，A16 决定访问命令还是数据*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;                  // RS
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    /*CS = PD7(FSMC_NE1)*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;                   // PD7
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    /*引脚复用：AF_FSMC*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);  // D2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);  // D3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);  // NOE_RD
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);  // NWE_WE
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);  // D13
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);  // D14
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC); // D15
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC); // D0
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC); // D1
    /*GPIOE 全部复用为 AF_FSMC*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);
    /*RS 与 CS 复用为 AF_FSMC*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); // RS
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);  // CS
#endif

    /*FSMC 初始化：模式 A 模拟 8080 并口*/

    RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;                      // 使能 FSMC 时钟

    // 只用 Bank1 第 1 组寄存器：BTCR[0]/BTCR[1] 管读写时序，BWTR[0] 管写时序
    FSMC_Bank1->BTCR[0]   = 0X00000000;
    FSMC_Bank1->BTCR[0 + 1] = 0X00000000;
    FSMC_Bank1E->BWTR[0]  = 0X00000000;

    /*BCR：异步模式 A，16 位总线*/
    FSMC_Bank1->BTCR[0] |= 0x01 << 12;        // 存储器写使能
    FSMC_Bank1->BTCR[0] |= 0x01 << 14;        // 读写使用相同的时序
    FSMC_Bank1->BTCR[0] |= 0x01 << 4;         // 数据总线宽度 16bit

    /*读时序：ADDSET=15 个 HCLK(90ns)、DATAST=60 个 HCLK(360ns)，6ns 为 168MHz 下的 HCLK 周期*/
    FSMC_Bank1->BTCR[0 + 1] |= 0x00 << 28;    // 模式 A
    FSMC_Bank1->BTCR[0 + 1] |= 0X0F << 0;     // 地址建立时间(ADDSET)=15
    FSMC_Bank1->BTCR[0 + 1] |= 0x3C << 8;     // 数据建立时间(DATAST)=60
    /*写时序：先用宽松值(ADDSET=9、DATAST=8)，初始化完成后再压缩*/
    FSMC_Bank1E->BWTR[0] |= 0x00 << 28;       // 模式 A
    FSMC_Bank1E->BWTR[0] |= 0x09 << 0;        // ADDSET=9
    FSMC_Bank1E->BWTR[0] |= 0x08 << 8;        // DATAST=8

    FSMC_Bank1->BTCR[0] |= 0x01;              // 使能 Bank1 第 1 存储区(NE1)

    delay_ms(50);                             // 上电后等屏内部复位完成
    LCD->LCD_REG = 0x0000;                    // 对寄存器 0 写读一遍，试探总线是否通
    LCD->LCD_RAM = 0x0000;
    delay_ms(50);
    xLCD.id = readReg(0x0000);

    LCD->LCD_REG = 0XD3;                      // 0xD3 读 ID4
    xLCD.id = LCD->LCD_RAM;                   // 第 1 次空读
    xLCD.id = LCD->LCD_RAM;                   // 厂商码 0x00
    xLCD.id = LCD->LCD_RAM;                   // 0x93
    xLCD.id <<= 8;
    xLCD.id |= LCD->LCD_RAM;                  // 0x41，拼成 0x9341
    printf("LCD driver IC ID: %x\r\n", xLCD.id);  // 正常应打印 9341

    /*初始化完成后压缩写时序，提高刷屏速度*/
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 0);      // 先清 ADDSET
    FSMC_Bank1E->BWTR[0] |= 2 << 8;           // DATAST 先写 2
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 8);      // 再清 DATAST
    FSMC_Bank1E->BWTR[0] |= 3 << 0;           // 最终 ADDSET=3 个 HCLK，约 18ns

    /*写入 ILI9341 初始化寄存器*/

    LCD->LCD_REG = 0xCF;          // 0xCF 电源控制 B(PWPB)，3 字节参数
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0xC1;
    LCD->LCD_RAM = 0X30;
    LCD->LCD_REG = 0xED;          // 0xED 上电时序控制，4 字节参数
    LCD->LCD_RAM = 0x64;
    LCD->LCD_RAM = 0x03;
    LCD->LCD_RAM = 0X12;
    LCD->LCD_RAM = 0X81;
    LCD->LCD_REG = 0xE8;          // 0xE8 驱动时序 A，3 字节参数
    LCD->LCD_RAM = 0x85;
    LCD->LCD_RAM = 0x10;
    LCD->LCD_RAM = 0x7A;
    LCD->LCD_REG = 0xCB;          // 0xCB 电源控制 A，5 字节参数
    LCD->LCD_RAM = 0x39;
    LCD->LCD_RAM = 0x2C;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x34;
    LCD->LCD_RAM = 0x02;
    LCD->LCD_REG = 0xF7;          // 0xF7 Pump Ratio Control: 电荷泵倍率, 固定 0x20
    LCD->LCD_RAM = 0x20;
    LCD->LCD_REG = 0xEA;          // 0xEA 驱动时序 B，2 字节参数
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0xC0;  // 0xC0 电源控制 1
    LCD->LCD_RAM = 0x1B;  // VRH[5:0]
    LCD->LCD_REG = 0xC1;  // 0xC1 电源控制 2
    LCD->LCD_RAM = 0x01;  // SAP[2:0];BT[3:0]
    LCD->LCD_REG = 0xC5;  // 0xC5 VCOM 控制 1
    LCD->LCD_RAM = 0x30;  // VCOMH 偏移
    LCD->LCD_RAM = 0x30;  // VCOML 偏移
    LCD->LCD_REG = 0xC7;  // 0xC7 VCOM 控制 2
    LCD->LCD_RAM = 0XB7;
    LCD->LCD_REG = 0x36;  // 0x36 MADCTL：bit7=MY 行方向, bit6=MX 列方向, bit5=MV 行列交换
    LCD->LCD_RAM = 0x48;                 // 0x48 = bit6(MX=1) + bit3(BGR=1): 竖屏 + BGR 顺序
    LCD->LCD_REG = 0x3A;                 // 0x3A 像素格式(Pixel Format)设置
    LCD->LCD_RAM = 0x55;                 // 0x55 = 16bit/pixel, RGB565(低 3 位 101)
    LCD->LCD_REG = 0xB1;                 // 0xB1 帧率控制，2 字节参数
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x1A;                 // 0x1A 分频: 约 70Hz, 越低越闪、越高越易花屏
    LCD->LCD_REG = 0xB6;  // 0xB6 显示功能控制
    LCD->LCD_RAM = 0x0A;
    LCD->LCD_RAM = 0xA2;                 // 0xA2 的低 5 位 = 320 行, 决定面板垂直扫描行数
    LCD->LCD_REG = 0xF2;  // 0xF2 关闭 3-Gamma
    LCD->LCD_RAM = 0x00;                 // 0x00 = 关闭 3-Gamma 功能
    LCD->LCD_REG = 0x26;  // 0x26 选择 Gamma 曲线
    LCD->LCD_RAM = 0x01;                 // 0x01 = 选 Gamma 曲线 1(与下面 E0/E1 各 15 字节配套)
    LCD->LCD_REG = 0xE0;  // 0xE0 正极性 Gamma 校正，15 字节
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
    LCD->LCD_REG = 0XE1;   // 0xE1 负极性 Gamma 校正，15 字节
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
    LCD->LCD_REG = 0x2B;                 // 0x2B 页地址范围：0~0x013F(319)
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x01;                 // 结束页 高 8 位
    LCD->LCD_RAM = 0x3f;                 // 结束页 低 8 位，0x013F=319
    LCD->LCD_REG = 0x2A;                 // 0x2A 列地址范围：0~0x00EF(239)
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;                 // 结束列 高 8 位
    LCD->LCD_RAM = 0xef;                 // 结束列 低 8 位，0x00EF=239
    LCD->LCD_REG = 0x11;  // 0x11 退出睡眠

    delay_ms(120);                       // 数据手册: 发 0x11 后至少等 120ms 才能发 0x29, 否则显示异常
    LCD->LCD_REG = 0x29;  // 0x29 开显示，此时背光未开，屏上全黑

    /*收尾：设方向、清屏、开背光*/

    LCD_SetDir(0);        // 默认竖屏，内部按 0x36 重设扫描方向并交换宽高
    LCD_Fill(0, 0, xLCD.width, xLCD.height, BLACK);   // 清屏；结束坐标传 width/height，比实际多覆盖 1 行 1 列
    LCD_BL_ON;            // 先清屏再开背光，避免上电看到随机 GRAM 内容
    xLCD.FlagInit = 1;    // 置初始化完成标志，供 LCD_String 检查

    /*配置 DMA2 Stream5，用于整区刷屏*/
    LCD_DMA_Init();
}



/**
  * 功    能：切横竖屏（dir 0 竖屏、1 横屏，内部会换算成寄存器值 6）
  * 参    数：dir  0=竖屏 1=横屏(内部换算成 6)；改方向后触摸必须重新校准
  * 返 回 值：无
  * 补    充：【改完方向必须重新校准触摸】—— 触摸的坐标换算和屏幕方向是绑死的，不重校就会点哪都不对
  */
void LCD_SetDir(uint8_t dir)
{
    uint16_t  regval = 0;
    uint16_t  temp = 0;

    if (dir == 1)
        dir = 6;                      // 1 映射到 6：横屏、从上到下、从左到右

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

    regval |= 0X08;                    // bit3 = BGR=1: 像素按 BGR 顺序解析(与 LCD_BGR2RGB 对应)
    LCD->LCD_REG = 0X36;               // 0x36 MADCTL：bit7=MY 行方向, bit6=MX 列方向, bit5=MV 行列交换
    LCD->LCD_RAM = regval;             // 竖屏 0x48；横屏必须置 bit5(MV) 才能做 90 度旋转

    // 保证 width 对应 X 轴像素数，height 对应 Y 轴像素数
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

    //设定 X 轴范围(0x2A)
    LCD->LCD_REG = 0X2A;                      // 列地址设置命令
    LCD->LCD_RAM = 0;                         // 起始列地址高8位
    LCD->LCD_RAM = 0;                         // 起始列地址低8位，从0开始
    LCD->LCD_RAM = (xLCD.width - 1) >> 8;     // 结束列地址高8位
    LCD->LCD_RAM = (xLCD.width - 1) & 0XFF;   // 结束列地址低8位
    //设定 Y 轴范围(0x2B)
    LCD->LCD_REG = 0X2B;                      // 页(行)地址设置命令
    LCD->LCD_RAM = 0;                         // 起始页地址高8位
    LCD->LCD_RAM = 0;                         // 起始页地址低8位，从0开始
    LCD->LCD_RAM = (xLCD.height - 1) >> 8;    // 结束页地址高8位
    LCD->LCD_RAM = (xLCD.height - 1) & 0XFF;  // 结束页地址低8位
}



/**
  * 功    能：把显存写入指针移到指定坐标：之后连续写像素就从这一点开始向右、向下铺
  * 参    数：Xpos  列坐标，竖屏 0~239、横屏 0~319；Ypos  页坐标，竖屏 0~319、横屏 0~239
  * 返 回 值：无
  * 补    充：坐标范围随方向变（竖屏 240 宽 / 横屏 320 宽），传错会写飞
  */
static void setCursor(uint16_t  Xpos, uint16_t  Ypos)
{
    LCD->LCD_REG = 0X2A;           // 0x2A 列地址，只发起始=结束，窗口压成 1 像素
    LCD->LCD_RAM = Xpos >> 8;      // 起始列 高 8 位
    LCD->LCD_RAM = Xpos & 0XFF;    // 起始列 低 8 位
    LCD->LCD_REG = 0X2B;           // 0x2B 页地址，同样只发起始=结束
    LCD->LCD_RAM = Ypos >> 8;      // 起始页 高 8 位
    LCD->LCD_RAM = Ypos & 0XFF;    // 起始页 低 8 位
}



/**
  * 功    能：在 (x,y) 画一个像素点
  * 参    数：x,y  坐标；_color  RGB565 颜色
  * 返 回 值：无
  */
void LCD_DrawPoint(uint16_t  x, uint16_t  y, uint16_t _color)
{
    LCD->LCD_REG = 0X2A;        // 0x2A 设置 x 坐标
    LCD->LCD_RAM = x >> 8;      // x 坐标 高 8 位
    LCD->LCD_RAM = x & 0XFF;    // x 坐标 低 8 位

    LCD->LCD_REG = 0X2B;        // 0x2B 设置 y 坐标
    LCD->LCD_RAM = y >> 8;      // y 坐标 高 8 位
    LCD->LCD_RAM = y & 0XFF;    // y 坐标 低 8 位

    LCD->LCD_REG = 0X2C;        // 0x2C 写 GRAM，地址自动 +1
    LCD->LCD_RAM = _color;      // 16 位 RGB565(高 5 位 R / 中 6 位 G / 低 5 位 B)，一次写一个像素
}



/**
  * 功    能：用纯色填一块矩形区域，起止坐标都包含在内
  * 参    数：startX/startY 起点；endX/endY 终点(含端点)；color 填充色
  * 返 回 值：无
  */
void LCD_Fill(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY, uint16_t color)
{
    uint16_t  xlen = 0;
    xlen = endX - startX + 1;                  // 含两端点: 0x2A 参数是"结束坐标"而不是"宽度", 与长度语义差 1
    for (uint16_t i = startY; i <= endY; i++)   // 逐行填充, 每行重新定光标, 因为 setCursor 把窗口压成 1x1
    {
        setCursor(startX, i);                  // 定位到本行起点
        LCD->LCD_REG = 0X2C;                   // 0x2C 连续写 GRAM
        for (uint16_t j = 0; j < xlen; j++)
            LCD->LCD_RAM = color;              // 地址自动递增, 无需再发命令
    }
}



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
            if (y >= xLCD.height)    return;                  // 超出屏幕高度(下)
            if ((y - y0) == size)                             // 一列画满 size 个像素(本字的字高)
            {
                y = y0;                                       // 回到本列顶端
                x++;                                          // 换到右边一列
                if (x >= xLCD.width) return;                  // 超出屏幕宽度(右)
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
        x += size / 2;                                              // 推进一个字符宽: 字模是半角, 宽度只有字号的一半(12号字宽6)
    }
}



/**
  * 功    能：打开显示输出
  * 参    数：无
  * 返 回 值：无
  */
void LCD_DisplayOn(void)
{
    LCD->LCD_REG = 0X29;                // 0x29 开显示
    LCD_BL_GPIO->BSRR |= LCD_BL_PIN;    // 打开背光
}



/**
  * 功    能：关闭显示输出。画面内容还在，只是不再刷新出来
  * 参    数：无
  * 返 回 值：无
  */
void LCD_DisplayOff(void)
{
    LCD->LCD_REG = 0X28;                                // 0x28 关显示
    LCD_BL_GPIO->BSRR |= (uint32_t)LCD_BL_PIN << 16;    // 关闭背光
}



/**
  * 功    能：当前方向下的屏幕宽度：竖屏 240，横屏 320
  * 参    数：无
  * 返 回 值：当前方向下的宽度(像素)，竖屏 240、横屏 320
  */
uint16_t LCD_GetWidth(void)
{
    return xLCD.width;

}



/**
  * 功    能：当前方向下的屏幕高度：竖屏 320，横屏 240
  * 参    数：无
  * 返 回 值：当前方向下的高度(像素)，竖屏 320、横屏 240
  */
uint16_t LCD_GetHeight(void)
{
    return xLCD.height;
}



/**
  * 功    能：当前是竖屏还是横屏：0 竖屏、1 横屏
  * 参    数：无
  * 返 回 值：0=竖屏 1=横屏
  */
uint8_t  LCD_GetDir(void)
{
    return xLCD.dir;
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



/**
  * 功    能：读某个像素的颜色（RGB565）。坐标超出屏幕直接返回 0
  * 参    数：x,y 坐标，超出屏幕直接返回 0
  * 返 回 值：RGB565 颜色值
  */
uint16_t  LCD_ReadPoint(uint16_t  x, uint16_t  y)
{
    uint16_t  r = 0, g = 0, b = 0;
    if (x >= xLCD.width || y >= xLCD.height)return 0; // 超出屏幕范围，直接返回
    setCursor(x, y);
    LCD->LCD_REG = 0X2E;                              // 0x2E 读 GRAM，每读一次地址自动 +1

    r = LCD->LCD_RAM;                                 // 第 1 次是空读，数据还没推到总线

    delay_us(20);
    r = LCD->LCD_RAM;                                 // 这次拿到 R(高 5 位)+G(低 6 位) 共 16 位

    delay_us(20);
    b = LCD->LCD_RAM;
    g = r & 0XFF;                                     // 先取 RG 的低 8 位(即 G)，再挪到高位供下面右移
    g <<= 8;

    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));   // 裁掉 GRAM 里 18 位颜色的多余低位, 压回 RGB565
}




/**
  * 功    能：配好刷屏用的 DMA（DMA2 Stream5），这样整块填色不用 CPU 一个像素一个像素搬
  * 参    数：无
  * 返 回 值：无
  */
static void LCD_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();                              // 开 DMA2 时钟

    /*M2M：源=像素缓冲(自增)，目的=LCD_RAM(固定)，16bit*/
    hdma_lcd.Instance                 = DMA2_Stream5;
    hdma_lcd.Init.Channel             = DMA_CHANNEL_0;        /* M2M 不看通道号，只有外设请求模式才用 */
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

/**
  * 功    能：注册「DMA 刷完了」的回调，LVGL 靠它知道什么时候能复用那块缓冲
  * 参    数：cb  DMA 完成回调，传 NULL 表示取消；回调在中断上下文执行
  * 返 回 值：无
  * 补    充：传 NULL 取消注册。注意回调是在中断上下文里跑的，里头不能调阻塞 API
  */
void LCD_SetFlushDoneCallback(void (*cb)(void))
{
    lcdFlushDoneCb = cb;                      // 回调里不能调阻塞型 API，只能置标志或释放信号量
}

/**
  * 功    能：把一块 RGB565 数据刷到屏幕上（LVGL 的 flush 走这里）
  * 参    数：x,y 左上角；width/height 右下角坐标 x2/y2(不是宽高)；pData RGB565 像素数组
  * 返 回 值：无
  * 补    充：【参数名有坑】width/height 实际是右下角坐标 x2/y2，不是宽和高 —— 按宽高传会画歪
  */
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData)
{
    /*等待上次 DMA 搬完，再发命令*/
    (void)osSemaphoreAcquire(lcdDmaDoneSem, osWaitForever);   // 窗口设置必须在 DMA 空闲时做，否则 ILI9341 解析错乱

    /*设窗口：0x2A 列范围、0x2B 页范围*/
    LCD->LCD_REG = 0x2A;                              // 列地址设置
    LCD->LCD_RAM = x >> 8;                             // 起始列 高 8 位
    LCD->LCD_RAM = x & 0xFF;                           // 起始列 低 8 位
    LCD->LCD_RAM = width >> 8;                         // 结束列 高 8 位(形参 width 实为 x2)
    LCD->LCD_RAM = width & 0xFF;                       // 结束列 低 8 位

    LCD->LCD_REG = 0x2B;                              // 页地址设置
    LCD->LCD_RAM = y >> 8;                             // 起始页 高 8 位
    LCD->LCD_RAM = y & 0xFF;                           // 起始页 低 8 位
    LCD->LCD_RAM = height >> 8;                        // 结束页 高 8 位(形参 height 实为 y2)
    LCD->LCD_RAM = height & 0xFF;                      // 结束页 低 8 位

    LCD->LCD_REG = 0x2C;                              // 0x2C 写 GRAM，地址自动递增

    /*直接写寄存器启动 DMA，绕开 HAL 状态机(否则第二次启动被判"忙"而失败)*/
    uint32_t pixelCount = (uint32_t)(width - x + 1U) * (uint32_t)(height - y + 1U);   // 像素数(含端点)，每像素 1 个 NDTR 计数
    lcdDmaBusy = 1U;

    DMA_Stream_TypeDef *s = DMA2_Stream5;
    s->CR  &= ~DMA_SxCR_EN;                          // 先停流
    while (s->CR & DMA_SxCR_EN) { ; }                // 等硬件真正停下
    s->PAR  = (uint32_t)pData;                       // 源：像素缓冲，自增
    s->M0AR = (uint32_t)&LCD->LCD_RAM;               // 目的：LCD_RAM，地址固定
    s->NDTR = pixelCount;                            // 传输个数
    s->CR  |= DMA_SxCR_TCIE;                         // 传完开中断
    s->CR  |= DMA_SxCR_EN;                           // 启动
}

/**
  * 功    能：DMA2 Stream5 中断入口 —— 这条流是 LCD 刷屏用的，转给 HAL 处理
  * 参    数：无
  * 返 回 值：无
  */
void DMA2_Stream5_IRQHandler(void)
{
    /*传输完成标志在 HISR 的 bit11(TCIF5)*/
    if (DMA2->HISR & DMA_HISR_TCIF5)
    {
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;               // 写 1 清标志

        lcdDmaBusy = 0U;                              // 置空闲，允许下次 flush
        if (lcdDmaDoneSem != NULL)
            (void)osSemaphoreRelease(lcdDmaDoneSem);  // 中断里只能用非阻塞版本
        if (lcdFlushDoneCb != NULL)
            lcdFlushDoneCb();                         // 通知 LVGL 这一帧已刷完
    }
}

