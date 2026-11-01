/* bsp_W25Q128.c
 * W25Q128 SPI Flash 驱动：初始化与 ID 校验、读数据、4KB 扇区擦除、256B 页编程、BUSY 等待、自测。
 * 硬件：SPI1 = PA5/PA6/PA7，CS = PC13；容量 16MB，最小擦除单位 4KB。
 * 注意：一次最多写 256 字节且不能跨页；擦除和写入前必须先发写使能；>= 0x00A00000 字库区一律拒绝擦写。
 */
#include "bsp_W25Q128.h"
#include "bsp_IWDG.h"       /* Watchdog_Heartbeat */
#include "usart.h"          /* UART_Printf */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"      /* osDelay / osKernelGetState */

/*命令字*/
#define W25Q_CMD_WRITE_ENABLE   0x06U   /* 写使能 06h，把 SR 的 WEL 置 1，擦/写前必发 */
#define W25Q_CMD_READ_SR1       0x05U   /* 读状态寄存器 1，bit0=BUSY，bit1=WEL */
#define W25Q_CMD_PAGE_PROGRAM   0x02U   /* 页编程 02h，一次最多 256 字节且不可跨页 */
#define W25Q_CMD_READ_DATA      0x03U   /* 读数据 03h，后跟 24 位地址，内部地址自动 +1 */
#define W25Q_CMD_SECTOR_ERASE   0x20U   /* 4KB */
#define W25Q_CMD_DEVICE_ID      0x90U   /* 读厂商/设备 ID 90h，需先发 3 字节假地址 */
#define W25Q_CMD_JEDEC_ID       0x9FU   /* 读 JEDEC ID 9Fh，返回 EF/40/18 三字节 */

/*超时：按实测值留约 10 倍余量*/
#define W25Q_ERASE_TIMEOUT_MS   400U    /* 实测 31~35ms, 取 400ms (~11x) */
#define W25Q_PROG_TIMEOUT_MS    10U     /* 实测 2ms, 取 10ms (5x) */
#define W25Q_SPI_BYTE_TIMEOUT   100U    /* 单字节 HAL 超时(10.5MHz 下 1 字节 <1us) */

/*CS = PC13，软件 NSS*/
#define W25Q_CS_LOW()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

static SPI_HandleTypeDef s_hspi1;
static uint8_t           s_initOk = 0U;
static uint32_t          s_lastBusyMs = 0U;   /* 最近一次 WaitReady 实测耗时 */

/* SPI 收发一个字节，读命令时发 0xFF 当哑元。调用前 CS 必须已经拉低。
 * HAL 出错时返回 0xFF 而不是 0x00 —— 返回 0 会被上层误当成「BUSY 已清零」。 */
static uint8_t W25Q_SpiByte(uint8_t tx)
{
    uint8_t rx = 0xFFU;
    if (HAL_SPI_TransmitReceive(&s_hspi1, &tx, &rx, 1U, W25Q_SPI_BYTE_TIMEOUT) != HAL_OK)
    {
        return 0xFFU;
    }
    return rx;
}

/* 用 DWT 周期数算毫秒。不能用 HAL_GetTick：boot 阶段调度器还没起，tick 根本不前进。
 * 32 位约 25.5s 回绕一次，调用方要用无符号差分算间隔。 */
static uint32_t W25Q_Millis(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL      |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return DWT->CYCCNT / (SystemCoreClock / 1000U);   /* 每毫秒 = SystemCoreClock/1000 个周期 */
}

/* boot 阶段可用的毫秒延时，靠 DWT 忙等，不清零计数器（和 WaitReady 共用同一时基）。 */
static void W25Q_DelayMsBoot(uint32_t ms)
{
    uint32_t start = W25Q_Millis();
    while ((W25Q_Millis() - start) < ms) { }
}

/* 固定 1ms 延时。boot 阶段既不能 osDelay（没有当前 TCB 会崩），
 * 也不能 HAL_Delay（tick 不走会死等），只能忙等。 */
static void W25Q_Delay1ms(void)
{
    if (osKernelGetState() == osKernelRunning)
    {
        osDelay(1U);
    }
    else
    {
        W25Q_DelayMsBoot(1U);
    }
}

/* 读状态寄存器 1（0x05 + 回读 1 字节），CS 全程保持拉低。本函数不等待 BUSY。 */
uint8_t W25Q128_ReadStatus1(void)
{
    uint8_t st;
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_SR1);
    st = W25Q_SpiByte(0xFFU);
    W25Q_CS_HIGH();
    return st;
}

/* 轮询 BUSY 位直到清零，实测耗时记进 s_lastBusyMs。
 * 【入口和每轮循环都补报一次 STORAGE 心跳】—— 4KB 擦除最长要 400ms，
 * 不补报的话这个槽位会被 MonitorTask 判成卡死，整机被复位。 */
HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs)
{
    uint8_t  st;
    uint32_t t0 = W25Q_Millis();

    /*入口必报：只在循环里报时，快响应路径全程零心跳*/
    Watchdog_Heartbeat(WDT_TASK_STORAGE);

    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_SR1);
    do
    {
        st = W25Q_SpiByte(0xFFU);
        if ((st & 0x01U) == 0U)                 /* BUSY 清零 */
        {
            W25Q_CS_HIGH();
            s_lastBusyMs = W25Q_Millis() - t0;
            return HAL_OK;
        }
        Watchdog_Heartbeat(WDT_TASK_STORAGE);   /* 长等待期间补报心跳 */
        W25Q_Delay1ms();
    } while ((W25Q_Millis() - t0) < timeoutMs);

    W25Q_CS_HIGH();
    s_lastBusyMs = W25Q_Millis() - t0;
    return HAL_TIMEOUT;                         /* 超时返回，不死等 */
}

/* 取上次等待时长。只有 WaitReady 会写它，32 位读本身原子，不用加锁。 */
uint32_t W25Q128_GetLastBusyMs(void)
{
    return s_lastBusyMs;
}

/* 发写使能命令（0x06）。【必须在 BUSY=0 时发】—— 忙的时候芯片会默默忽略且不报错，
 * 之后写/擦就会静默失败。所以调用前一定要先 WaitReady。 */
static void W25Q_WriteEnable(void)
{
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

/* 读 JEDEC ID（0x9F）。链路有问题时会读到 0x00FFFF 或 0xFFFFFF，可据此判断。
 * 读命令不会置 BUSY，所以不用写使能、也不用等待。 */
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

/* 读 Device ID（0x90，要先发 3 字节假地址，本芯片会忽略它）。
 * 异常时读到 0x0000 或 0xFFFF。读命令同样不置 BUSY。 */
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

/* 读一段数据（0x03 + 24 位地址 + 连续读）。地址由芯片自己自增。
 * addr+num 不得越过 16MB，由调用方保证。返回 HAL_ERROR 表示驱动还没初始化。 */
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

/* 擦除一个 4KB 扇区：写使能 → 发 0x20 → 等 BUSY 清零。
 * 落在字库区内的地址会被拒绝，保护字库不被误擦。 */
HAL_StatusTypeDef W25Q128_SectorErase4K(uint32_t addr)
{
    HAL_StatusTypeDef st;

    if (s_initOk == 0U)
    {
        return HAL_ERROR;
    }
    /* >= 字库区的擦写一律拒绝：字模是外部烧的，擦掉就没了*/
    if (addr >= W25Q128_GBK_BASE)
    {
        UART_Printf("[W25Q] erase 0x%08lX rejected: GBK font region\r\n",
                    (unsigned long)addr);
        return HAL_ERROR;
    }

    /*先等空闲再发 06h：BUSY 期间 06h 被忽略，擦除因 WEL=0 被丢弃，表现为静默失败*/
    st = W25Q128_WaitReady(W25Q_ERASE_TIMEOUT_MS);   /* 先等前一次操作完成 */
    if (st != HAL_OK)
    {
        return st;
    }
    W25Q_WriteEnable();

    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_SECTOR_ERASE);
    (void)W25Q_SpiByte((uint8_t)(addr >> 16));
    (void)W25Q_SpiByte((uint8_t)(addr >> 8));
    (void)W25Q_SpiByte((uint8_t)addr);
    W25Q_CS_HIGH();

    return W25Q128_WaitReady(W25Q_ERASE_TIMEOUT_MS); /* 等擦除完成, 耗时入 s_lastBusyMs */
}

/* 写一页：写使能 → 发 0x02 → 发数据 → 等 BUSY。
 * 整段必须同一页内 —— 跨页时地址会在页内回绕，把页首已经写好的数据盖掉。 */
HAL_StatusTypeDef W25Q128_PageProgram(uint32_t addr, const uint8_t *pData, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t          i;

    if (s_initOk == 0U)
    {
        return HAL_ERROR;
    }
    /*单次 <=256B 且不跨页，跨页会回卷到页首静默写错*/
    if ((len == 0U) || (len > W25Q128_PAGE_SIZE) ||
        ((addr & (W25Q128_PAGE_SIZE - 1U)) + (uint32_t)len) > W25Q128_PAGE_SIZE)   /* 页内偏移 + 长度不得越过 256B 页边界 */
    {
        return HAL_ERROR;
    }
    if ((addr + (uint32_t)len) > W25Q128_GBK_BASE)   /* 字库写保护 */
    {
        UART_Printf("[W25Q] program 0x%08lX+%u rejected: GBK font region\r\n",
                    (unsigned long)addr, (unsigned)len);
        return HAL_ERROR;
    }

    /*同擦除：先等空闲再发 06h，忙时 06h 被忽略则静默失败*/
    st = W25Q128_WaitReady(W25Q_PROG_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        return st;
    }
    W25Q_WriteEnable();

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

/* 初始化 SPI 并读 ID 校验芯片。返回 1 通过，0 失败（会打印实际读到的字节）。 */
uint8_t W25Q128_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /*CS = PC13，先置高不选中*/
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    /*SCK=PA5 / MISO=PA6 / MOSI=PA7，AF5*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);
    UART_Printf("[W25Q] init: gpio ok\r\n");      /* 分段留痕：定位卡在哪一步 */

    /*Mode 3、8bit、MSB、软件 NSS；APB2 84MHz / 8 = 10.5MHz*/
    __HAL_RCC_SPI1_CLK_ENABLE();
    s_hspi1.Instance               = SPI1;
    s_hspi1.Init.Mode              = SPI_MODE_MASTER;
    s_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    s_hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    s_hspi1.Init.NSS               = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   /* APB2 84MHz / 8 = 10.5MHz，链路通后再提速 */
    s_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&s_hspi1) != HAL_OK)
    {
        UART_Printf("[W25Q] HAL_SPI_Init failed\r\n");
        s_initOk = 0U;
        return 0U;
    }
    UART_Printf("[W25Q] init: spi ok\r\n");       /* 分段留痕：定位卡在哪一步 */

    /*上电稳定延时 300ms，DWT 忙等不依赖 HAL tick*/
    W25Q_DelayMsBoot(300U);
    UART_Printf("[W25Q] init: delay ok\r\n");     /* 分段留痕：定位卡在哪一步 */

    /*两条 ID 命令交叉验证，全对才算链路通*/
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
        /*全 1 = MISO 悬空或 CS 没拉高；全 0 = MISO 短路或时序错*/
        UART_Printf("[W25Q] M1 FAIL: check CS=PC13 / PA5-7 wiring & Mode3\r\n");
    }
    return s_initOk;
}

/* 自检：挑用户区尾部一块扇区，擦 → 写测试图案 → 读回逐个比对。不碰字库区。 */
void W25Q128_SelfTest(void)
{
    /*测试扇区：用户区最后一个扇区，避开 0x00A00000 字库区*/
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
        s_wr[i] = (uint8_t)i;       /* 0x00~0xFF 递增图案：回读错位/翻转都能一眼看出 */
    }

    /*擦除 4KB*/
    st = W25Q128_SectorErase4K(testAddr);
    UART_Printf("[W25Q] self-test: erase 4KB @0x%08lX -> %s, busy %lu ms\r\n",
                (unsigned long)testAddr, (st == HAL_OK) ? "OK" : "TIMEOUT",
                (unsigned long)W25Q128_GetLastBusyMs());
    if (st != HAL_OK)
    {
        return;
    }

    /*擦除态应全 0xFF*/
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

    /*页编程 256B*/
    st = W25Q128_PageProgram(testAddr, s_wr, W25Q128_PAGE_SIZE);
    UART_Printf("[W25Q] self-test: program 256B -> %s, busy %lu ms\r\n",
                (st == HAL_OK) ? "OK" : "TIMEOUT",
                (unsigned long)W25Q128_GetLastBusyMs());
    if (st != HAL_OK)
    {
        return;
    }

    /*回读比对*/
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

/* 字库存在性检查：多点采样看 0xFF 占比，全 0xFF 说明根本没烧字库。 */
uint8_t W25Q128_CheckGBKFont(void)
{
    static uint8_t s_buf[256U];     /* static: boot 阶段 MSP 栈仅 1KB */
    uint8_t  head16[16];
    uint8_t  boundary;
    uint8_t  nonFFcnt = 0U;
    uint8_t  ffRatio;
    uint32_t off;
    uint16_t i;
    uint8_t  pass;

    if (s_initOk == 0U)
    {
        UART_Printf("[W25Q] GBK check skipped (init failed)\r\n");
        return 0U;
    }

    /*首 16 字节全 0xFF = 擦除态 = 无字库*/
    (void)W25Q128_ReadData(W25Q128_GBK_BASE, head16, 16U);
    UART_Printf("[W25Q] GBK first 16B @0x%08lX:",
                (unsigned long)W25Q128_GBK_BASE);
    for (i = 0U; i < 16U; i++)
    {
        UART_Printf(" %02X", head16[i]);
        if (head16[i] != 0xFFU) { nonFFcnt++; }
    }
    UART_Printf("\r\n");
    if (nonFFcnt == 0U)
    {
        UART_Printf("[W25Q] M2 FAIL: font area all 0xFF (erased, no font)\r\n");
        return 0U;
    }

    /*分区边界：用户区最后一字节应为 0xFF*/
    (void)W25Q128_ReadData(W25Q128_GBK_BASE - 1U, &boundary, 1U);
    UART_Printf("[W25Q] GBK boundary byte @0x%08lX = 0x%02X (expect 0xFF)\r\n",
                (unsigned long)(W25Q128_GBK_BASE - 1U), boundary);

    /*6MB 内每 1MB 采一点，确认数据覆盖全区*/
    nonFFcnt = 0U;
    for (off = 0U; off < 0x00600000UL; off += 0x00100000UL)  /* 0,1,2,3,4,5 MB */
    {
        uint8_t b;
        (void)W25Q128_ReadData(W25Q128_GBK_BASE + off, &b, 1U);
        UART_Printf("[W25Q] GBK sample @+%luMB = 0x%02X%s\r\n",
                    (unsigned long)(off / 0x00100000UL), b,
                    (b != 0xFFU) ? "" : " (FF)");
        if (b != 0xFFU) { nonFFcnt++; }
    }

    /*首 256 字节 0xFF 占比 < 80% 才像真实字模*/
    (void)W25Q128_ReadData(W25Q128_GBK_BASE, s_buf, 256U);
    {
        uint16_t ff = 0U;
        for (i = 0U; i < 256U; i++)
        {
            if (s_buf[i] == 0xFFU) { ff++; }
        }
        ffRatio = (uint8_t)((uint32_t)ff * 100U / 256U);
    }
    UART_Printf("[W25Q] GBK first 256B: 0xFF ratio = %u%% (expect <80%%)\r\n",
                (unsigned)ffRatio);

    pass = (boundary == 0xFFU) && (nonFFcnt >= 5U) && (ffRatio < 80U);   /* 6 点采样至少 5 点非 FF，首 256B 的 FF 占比 < 80% */
    UART_Printf("[W25Q] M2 %s: GBK font %s at 0x%08lX\r\n",
                (pass != 0U) ? "PASS" : "FAIL",
                (pass != 0U) ? "detected" : "NOT found",
                (unsigned long)W25Q128_GBK_BASE);
    return pass;
}
