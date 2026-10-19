/**
  ******************************************************************************
  * @file    bsp_W25Q128.c
  * @brief   W25Q128 串行 NOR Flash(16MB, SPI1)驱动实现 —— V1.2 存储阶段
  *
  *          里程碑 M1/M1.5 验收口径:
  *            M1   : 0x9F 读到 0xEF4018, 且 0x90 读到 0xEF17(交叉验证)
  *            M1.5 : 自测扇区 擦除->页编程->回读校验 通过, 并打印实测耗时
  *                   (本地无 W25Q128 数据手册, 用实测值回填巡检草案时序表)
  ******************************************************************************
  */
#include "bsp_W25Q128.h"
#include "bsp_IWDG.h"       /* Watchdog_Heartbeat */
#include "usart.h"          /* UART_Printf */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"      /* osDelay / osKernelGetState */

/* ---- 命令字 ---- */
#define W25Q_CMD_WRITE_ENABLE   0x06U
#define W25Q_CMD_READ_SR1       0x05U
#define W25Q_CMD_PAGE_PROGRAM   0x02U
#define W25Q_CMD_READ_DATA      0x03U
#define W25Q_CMD_SECTOR_ERASE   0x20U   /* 4KB */
#define W25Q_CMD_DEVICE_ID      0x90U
#define W25Q_CMD_JEDEC_ID       0x9FU

/* ---- 超时(无本地手册, 先按巡检草案估算 max x2 取值, 实测后收紧) ---- */
#define W25Q_ERASE_TIMEOUT_MS   1000U   /* 4KB 擦除: 唯一有出处的数字是 >150ms */
#define W25Q_PROG_TIMEOUT_MS    100U    /* 页编程估算 max ~3ms */
#define W25Q_SPI_BYTE_TIMEOUT   100U    /* 单字节 HAL 超时(10.5MHz 下 1 字节 <1us) */

/* ---- CS(PC13) ---- */
#define W25Q_CS_LOW()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

static SPI_HandleTypeDef s_hspi1;
static uint8_t           s_initOk = 0U;
static uint32_t          s_lastBusyMs = 0U;   /* 最近一次 WaitReady 实测耗时 */

/* 单字节收发; HAL 出错返回 0xFF(等效 MISO 悬空的全 1, 让上层走超时/失配路径,
 * 而不是把 0x00 误认为"BUSY 已清零") */
static uint8_t W25Q_SpiByte(uint8_t tx)
{
    uint8_t rx = 0xFFU;
    if (HAL_SPI_TransmitReceive(&s_hspi1, &tx, &rx, 1U, W25Q_SPI_BYTE_TIMEOUT) != HAL_OK)
    {
        return 0xFFU;
    }
    return rx;
}

/* 等待 1ms: boot 阶段调度器未跑, osDelay 无当前 TCB 会崩, 退化为 HAL_Delay */
static void W25Q_Delay1ms(void)
{
    if (osKernelGetState() == osKernelRunning)
    {
        osDelay(1U);
    }
    else
    {
        HAL_Delay(1U);
    }
}

/* boot 阶段专用延时: DWT 周期计数忙等, 不依赖 HAL tick/任何中断/RTOS。
 * 背景(V1.2 黑屏定位): 本工程此前从未在调度器启动前调过 HAL_Delay,
 * "boot 阶段 HAL tick 是否可靠"是无出处的假设; DWT 开了 TRCENA 就自由计数,
 * 是 boot 阶段零依赖的延时手段。168MHz 下 32 位 CYCCNT 约 25.5s 才回绕 */
static void W25Q_DelayMsBoot(uint32_t ms)
{
    uint32_t start;
    uint32_t cycles = ms * (SystemCoreClock / 1000U);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT      = 0U;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;

    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

uint8_t W25Q128_ReadStatus1(void)
{
    uint8_t st;
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_SR1);
    st = W25Q_SpiByte(0xFFU);
    W25Q_CS_HIGH();
    return st;
}

HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs)
{
    uint8_t  st;
    uint32_t t0 = HAL_GetTick();

    /* ① 入口必报(D3 教训: 快响应路径若只在循环里报, 整次调用零心跳) */
    Watchdog_Heartbeat(WDT_TASK_STORAGE);

    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_SR1);
    do
    {
        st = W25Q_SpiByte(0xFFU);
        if ((st & 0x01U) == 0U)                 /* BUSY 清零 */
        {
            W25Q_CS_HIGH();
            s_lastBusyMs = HAL_GetTick() - t0;
            return HAL_OK;
        }
        Watchdog_Heartbeat(WDT_TASK_STORAGE);   /* ② 长等待节流补报 */
        W25Q_Delay1ms();
    } while ((HAL_GetTick() - t0) < timeoutMs);

    W25Q_CS_HIGH();
    s_lastBusyMs = HAL_GetTick() - t0;
    return HAL_TIMEOUT;                         /* ③ 必须有超时, 不死等 */
}

uint32_t W25Q128_GetLastBusyMs(void)
{
    return s_lastBusyMs;
}

static void W25Q_WriteEnable(void)
{
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

uint32_t W25Q128_ReadJedecId(void)
{
    uint32_t id;
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_JEDEC_ID);
    id  = ((uint32_t)W25Q_SpiByte(0xFFU)) << 16;   /* 厂商 ID, 期望 0xEF */
    id |= ((uint32_t)W25Q_SpiByte(0xFFU)) << 8;    /* 存储类型, 期望 0x40 */
    id |=  (uint32_t)W25Q_SpiByte(0xFFU);          /* 容量, 期望 0x18 */
    W25Q_CS_HIGH();
    return id;
}

uint16_t W25Q128_ReadDeviceId(void)
{
    uint16_t id;
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_DEVICE_ID);
    (void)W25Q_SpiByte(0x00U);                     /* 24 位假地址 */
    (void)W25Q_SpiByte(0x00U);
    (void)W25Q_SpiByte(0x00U);
    id  = ((uint16_t)W25Q_SpiByte(0xFFU)) << 8;    /* 厂商, 期望 0xEF */
    id |=  (uint16_t)W25Q_SpiByte(0xFFU);          /* 设备, 期望 0x17 */
    W25Q_CS_HIGH();
    return id;
}

HAL_StatusTypeDef W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint32_t num)
{
    uint32_t i;

    if (s_initOk == 0U)
    {
        return HAL_ERROR;
    }
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_DATA);
    (void)W25Q_SpiByte((uint8_t)(addr >> 16));
    (void)W25Q_SpiByte((uint8_t)(addr >> 8));
    (void)W25Q_SpiByte((uint8_t)addr);
    for (i = 0U; i < num; i++)
    {
        pData[i] = W25Q_SpiByte(0xFFU);
    }
    W25Q_CS_HIGH();
    return HAL_OK;
}

HAL_StatusTypeDef W25Q128_SectorErase4K(uint32_t addr)
{
    HAL_StatusTypeDef st;

    if (s_initOk == 0U)
    {
        return HAL_ERROR;
    }
    if (addr >= W25Q128_GBK_BASE)      /* 字库写保护(M2 证实前一律拒绝) */
    {
        UART_Printf("[W25Q] erase 0x%08lX rejected: GBK font region\r\n",
                    (unsigned long)addr);
        return HAL_ERROR;
    }

    W25Q_WriteEnable();
    st = W25Q128_WaitReady(W25Q_ERASE_TIMEOUT_MS);   /* 等前一次操作完成 */
    if (st != HAL_OK)
    {
        return st;
    }

    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_SECTOR_ERASE);
    (void)W25Q_SpiByte((uint8_t)(addr >> 16));
    (void)W25Q_SpiByte((uint8_t)(addr >> 8));
    (void)W25Q_SpiByte((uint8_t)addr);
    W25Q_CS_HIGH();

    return W25Q128_WaitReady(W25Q_ERASE_TIMEOUT_MS); /* 等擦除完成, 耗时入 s_lastBusyMs */
}

HAL_StatusTypeDef W25Q128_PageProgram(uint32_t addr, const uint8_t *pData, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t          i;

    if (s_initOk == 0U)
    {
        return HAL_ERROR;
    }
    /* 页编程单次 <=256B 且不可跨页(跨页会回卷到页首, 静默写错位置) */
    if ((len == 0U) || (len > W25Q128_PAGE_SIZE) ||
        ((addr & (W25Q128_PAGE_SIZE - 1U)) + (uint32_t)len) > W25Q128_PAGE_SIZE)
    {
        return HAL_ERROR;
    }
    if ((addr + (uint32_t)len) > W25Q128_GBK_BASE)   /* 字库写保护 */
    {
        UART_Printf("[W25Q] program 0x%08lX+%u rejected: GBK font region\r\n",
                    (unsigned long)addr, (unsigned)len);
        return HAL_ERROR;
    }

    W25Q_WriteEnable();
    st = W25Q128_WaitReady(W25Q_PROG_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }

    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_PAGE_PROGRAM);
    (void)W25Q_SpiByte((uint8_t)(addr >> 16));
    (void)W25Q_SpiByte((uint8_t)(addr >> 8));
    (void)W25Q_SpiByte((uint8_t)addr);
    for (i = 0U; i < len; i++)
    {
        (void)W25Q_SpiByte(pData[i]);
    }
    W25Q_CS_HIGH();

    return W25Q128_WaitReady(W25Q_PROG_TIMEOUT_MS);
}

uint8_t W25Q128_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* CS = PC13 推挽输出上拉, 先置高(不选中) */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* SCK=PA5 / MISO=PA6 / MOSI=PA7, AF5 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);
    UART_Printf("[W25Q] init: gpio ok\r\n");      /* 分段定位留痕(V1.2 黑屏) */

    /* SPI1: 与厂商一致的 Mode 3(CPOL=1/CPHA=2edge), 8bit MSB 软 NSS;
     * 里程碑先跑 10.5MHz(84MHz/8), 链路通后再逐级试 21M/42M */
    __HAL_RCC_SPI1_CLK_ENABLE();
    s_hspi1.Instance               = SPI1;
    s_hspi1.Init.Mode              = SPI_MODE_MASTER;
    s_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    s_hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    s_hspi1.Init.NSS               = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    s_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&s_hspi1) != HAL_OK)
    {
        UART_Printf("[W25Q] HAL_SPI_Init failed\r\n");
        s_initOk = 0U;
        return 0U;
    }
    UART_Printf("[W25Q] init: spi ok\r\n");       /* 分段定位留痕(V1.2 黑屏) */

    /* 上电稳定延时(厂商注释: 避免烧录时多次复位导致擦写过程错误);
     * 用 DWT 忙等而非 HAL_Delay: 消除 boot 阶段对 HAL tick 的依赖(见函数注释) */
    W25Q_DelayMsBoot(300U);
    UART_Printf("[W25Q] init: delay ok\r\n");     /* 分段定位留痕(V1.2 黑屏) */

    /* M1 验收: 两条命令交叉验证, 全对才算链路通 */
    {
        uint32_t jedec = W25Q128_ReadJedecId();
        uint16_t dev   = W25Q128_ReadDeviceId();

        UART_Printf("[W25Q] JEDEC ID(0x9F) = 0x%06lX (expect 0x%06lX)\r\n",
                    (unsigned long)jedec, (unsigned long)W25Q128_JEDEC_ID_EXPECT);
        UART_Printf("[W25Q] device ID(0x90) = 0x%04X (expect 0x%04X)\r\n",
                    (unsigned)dev, (unsigned)W25Q128_DEVICE_ID_EXPECT);

        s_initOk = (jedec == W25Q128_JEDEC_ID_EXPECT &&
                    dev   == W25Q128_DEVICE_ID_EXPECT) ? 1U : 0U;
    }

    if (s_initOk != 0U)
    {
        UART_Printf("[W25Q] M1 PASS: link OK (Mode3, 10.5MHz)\r\n");
    }
    else
    {
        /* 全 1: MISO 悬空/CS 没拉上; 全 0: MISO 短路/时序错 —— 按此方向查线 */
        UART_Printf("[W25Q] M1 FAIL: check CS=PC13 / PA5-7 wiring & Mode3\r\n");
    }
    return s_initOk;
}

void W25Q128_SelfTest(void)
{
    /* 测试扇区: 用户区最后一个扇区(0x009FF000), 避开 0x00A00000 起的字库区 */
    const uint32_t  testAddr = W25Q128_GBK_BASE - W25Q128_SECTOR_SIZE;
    static uint8_t  s_wr[W25Q128_PAGE_SIZE];   /* static: boot 阶段 MSP 栈只有 1KB */
    static uint8_t  s_rd[W25Q128_PAGE_SIZE];
    uint16_t        i;
    uint8_t         eraseFF = 1U;
    HAL_StatusTypeDef st;

    if (s_initOk == 0U)
    {
        UART_Printf("[W25Q] self-test skipped (init failed)\r\n");
        return;
    }

    for (i = 0U; i < W25Q128_PAGE_SIZE; i++)
    {
        s_wr[i] = (uint8_t)i;
    }

    /* 1. 擦除 4KB, 打印实测耗时 */
    st = W25Q128_SectorErase4K(testAddr);
    UART_Printf("[W25Q] self-test: erase 4KB @0x%08lX -> %s, busy %lu ms\r\n",
                (unsigned long)testAddr, (st == HAL_OK) ? "OK" : "TIMEOUT",
                (unsigned long)W25Q128_GetLastBusyMs());
    if (st != HAL_OK)
    {
        return;
    }

    /* 2. 擦除态校验(应全 0xFF) */
    (void)W25Q128_ReadData(testAddr, s_rd, W25Q128_PAGE_SIZE);
    for (i = 0U; i < W25Q128_PAGE_SIZE; i++)
    {
        if (s_rd[i] != 0xFFU)
        {
            eraseFF = 0U;
            break;
        }
    }
    UART_Printf("[W25Q] self-test: erased page all-0xFF? %s\r\n",
                (eraseFF != 0U) ? "YES" : "NO");

    /* 3. 页编程 256B, 打印实测耗时 */
    st = W25Q128_PageProgram(testAddr, s_wr, W25Q128_PAGE_SIZE);
    UART_Printf("[W25Q] self-test: program 256B -> %s, busy %lu ms\r\n",
                (st == HAL_OK) ? "OK" : "TIMEOUT",
                (unsigned long)W25Q128_GetLastBusyMs());
    if (st != HAL_OK)
    {
        return;
    }

    /* 4. 回读比对 */
    (void)W25Q128_ReadData(testAddr, s_rd, W25Q128_PAGE_SIZE);
    for (i = 0U; i < W25Q128_PAGE_SIZE; i++)
    {
        if (s_rd[i] != s_wr[i])
        {
            break;
        }
    }
    UART_Printf("[W25Q] M1.5 %s: readback %s\r\n",
                (i == W25Q128_PAGE_SIZE) ? "PASS" : "FAIL",
                (i == W25Q128_PAGE_SIZE) ? "matched" : "MISMATCH");
}
