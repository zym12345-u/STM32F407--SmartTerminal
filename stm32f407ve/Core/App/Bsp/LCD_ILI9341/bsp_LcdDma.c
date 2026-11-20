/* bsp_LcdDma.c
 * ILI9341 DMA 整块刷屏：DMA2 Stream5 初始化、完成回调、中断入口。
 * 只由 LVGL 的 flush 路径调用；不碰 LCD 方向与绘图状态。
 */
#include "bsp_LCD_ILI9341.h"
#include "bsp_LcdInternal.h"
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

/**
  * 功    能：配好刷屏用的 DMA（DMA2 Stream5），这样整块填色不用 CPU 一个像素一个像素搬
  * 参    数：无
  * 返 回 值：无
  */
void LcdDma_Init(void)
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
