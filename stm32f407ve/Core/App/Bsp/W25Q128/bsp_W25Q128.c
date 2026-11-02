/* bsp_W25Q128.c
 * W25Q128 SPI Flash 驱动：初始化与 ID 校验、读数据、4KB 扇区擦除、256B 页编程、BUSY 等待、自测。
 * 硬件：SPI1 = PA5/PA6/PA7，CS = PC13；容量 16MB，最小擦除单位 4KB。
 * 注意：一次最多写 256 字节且不能跨页；擦除和写入前必须先发写使能；>= 0x00A00000 字库区一律拒绝擦写。
 *
 * ══════ SPI 的基础（理解后面每条命令怎么发都靠这段）══════
 *
 * ── 四根线，各管一件事 ──
 *   SCK   时钟，【永远由主机产生】—— 这点决定了后面那个「哑元」的写法
 *   MOSI  主机发、从机收
 *   MISO  主机收、从机发
 *   CS    片选，低电平有效，拉低表示「我要跟这个从机说话了」
 *
 * ── 全双工：为什么「读」的时候还得「发」 ──
 *   SPI 每来一个时钟，主机和从机【同时】各移出一位、各收一位。
 *   时钟只有主机能给，所以主机如果只想着收、不发东西，就【没有时钟】，从机也不会吐数据。
 *   于是读的时候要发一个占位字节（本文件用 0xFF，叫「哑元/dummy」），
 *   目的不是传数据，而是把时钟「摇」出来。这就是 W25Q_SpiByte 那个 tx 参数的含义。
 *
 * ── CPOL / CPHA：两个参数决定四种时序 ──
 *   CPOL 决定【空闲时时钟是高还是低】：0 = 空闲低，1 = 空闲高
 *   CPHA 决定【在哪个边沿采样】：0 = 第一个边沿采，1 = 第二个边沿采
 *   两者组合就是常说的 Mode 0~3。W25Q128 支持 Mode 0 和 Mode 3
 *   （大多数 SPI Flash 都这样，因为它们在两个边沿都能稳定工作）。
 *   主从两端的模式必须一致，不一致的表现是「读回来全是 0xFF 或全 0」，很像没接线。
 *
 * ── CS 为什么必须「一帧拉一次」 ──
 *   W25Q 的每条命令都要求 CS 用「高 → 低 → ... → 高」完整包住命令码 + 地址 + 数据。
 *   如果中途不拉高，从机会一直以为还在同一条命令里，后面的数据全被当成上一条的续传。
 *   本文件里用 W25Q_CS_LOW()/HIGH() 一对一对地包，就是为了这个。
 *
 * ── Flash 的擦写规则（和 RAM/EEPROM 完全不同，必须记住）──
 *   1) Flash 只能把 1 写成 0，不能把 0 写回 1 —— 所以【写之前必须先擦】，
 *      擦除是把整片区域恢复成全 0xFF。
 *   2) 最小擦除单位是【扇区 4KB】，不是字节。想改一个字节，也得先擦整个 4KB 扇区。
 *   3) 单次写入最多【一页 256 字节】且不能跨页，跨页会在页内回绕、覆盖掉页首已写的数据。
 *   4) 擦写期间芯片 BUSY，必须等它自己完成，这个等待就是 W25Q128_WaitReady。
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

/**
  * 功    能：SPI 收发一个字节，读命令时发 0xFF 当哑元。调用前 CS 必须已经拉低
  * 参    数：tx  要发出的字节，范围 0x00~0xFF；读命令时填 0xFF
  * 返 回 值：实际收到的字节；HAL 失败返回 0xFF
  * 补    充：调用前必须已拉低 CS；失败返回 0xFF，避免把 0x00 当成 BUSY 已清零
  */
static uint8_t W25Q_SpiByte(uint8_t tx)
{
    uint8_t rx = 0xFFU;
    if (HAL_SPI_TransmitReceive(&s_hspi1, &tx, &rx, 1U, W25Q_SPI_BYTE_TIMEOUT) != HAL_OK)
    {
        return 0xFFU;
    }
    return rx;
}

/*每毫秒对应的 DWT 周期数，168MHz 下是 168000*/
#define W25Q_CYCLES_PER_MS   (SystemCoreClock / 1000U)

/**
  * 功    能：取 DWT 的周期计数，顺带确保 CYCCNT 已经开起来
  * 参    数：无
  * 返 回 值：CPU 周期数，32 位，@168MHz 约 25.5 秒回绕一次
  * 补    充：等待超时一律比较【周期差】，不要先除成毫秒 ——
  *            CYCCNT/168000 得到的是一个 0~25565 的短计数器，每 25.5 秒回绕；
  *            若起点采在回绕前、终点采在回绕后，毫秒差会算成天文数字，
  *            把还在忙的擦除误判成超时（2026-11-01 真机踩到：误判后紧跟的读全是 0xFF，
  *            LittleFS 被判空白自动格式化，WiFi 凭据被擦）
  *            周期差在无符号 32 位下能跨一次回绕，所以等待时长必须远小于 25.5 秒
  */
static uint32_t W25Q_NowCycles(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL      |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return DWT->CYCCNT;
}

/**
  * 功    能：boot 阶段可用的毫秒延时，靠 DWT 忙等，不清零计数器（和 WaitReady 共用同一时基）
  * 参    数：ms  延时长度，单位 ms，本工程只用 300
  * 返 回 值：无
  * 补    充：DWT 忙等，不清零 CYCCNT，与 WaitReady 共用同一时基
  */
static void W25Q_DelayMsBoot(uint32_t ms)
{
    uint32_t t0     = W25Q_NowCycles();
    uint32_t cycles = ms * W25Q_CYCLES_PER_MS;
    while ((W25Q_NowCycles() - t0) < cycles) { }
}

/**
  * 功    能：固定 1ms 延时。boot 阶段既不能 osDelay（没有当前 TCB 会崩），
  * 参    数：无，固定 1ms
  * 返 回 值：无
  * 补    充：boot 阶段不能走 osDelay（无当前 TCB 会崩）或 HAL_Delay（tick 不走会死等）
  *            也不能 HAL_Delay（tick 不走会死等），只能忙等
  */
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

/**
  * 功    能：读状态寄存器 1（0x05 + 回读 1 字节），CS 全程保持拉低。本函数不等待 BUSY
  * 参    数：无
  * 返 回 值：状态字节；bit0 = BUSY，bit1 = WEL
  * 补    充：0x05 + 1 字节回读，期间 CS 保持拉低；本函数不等待 BUSY
  */
uint8_t W25Q128_ReadStatus1(void)
{
    uint8_t st;
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_READ_SR1);
    st = W25Q_SpiByte(0xFFU);
    W25Q_CS_HIGH();
    return st;
}

/**
  * 功    能：轮询 BUSY 位直到清零，实测耗时记进 s_lastBusyMs
  * 参    数：timeoutMs  最长等待，单位 ms；擦除传 400，编程传 10
  * 返 回 值：HAL_OK = BUSY 已清零，HAL_TIMEOUT = 超时仍未空闲
  * 补    充：入口必报一次 WDT_TASK_STORAGE 心跳，循环内每轮补报；实测耗时写入 s_lastBusyMs
  *            【入口和每轮循环都补报一次 STORAGE 心跳】—— 4KB 擦除最长要 400ms，
  *            不补报的话这个槽位会被 MonitorTask 判成卡死，整机被复位
  */
HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs)
{
    uint8_t  st;
    uint32_t t0      = W25Q_NowCycles();
    uint32_t timeout = timeoutMs * W25Q_CYCLES_PER_MS;   /*超时换算成周期数，全程比较周期差*/

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
            s_lastBusyMs = (W25Q_NowCycles() - t0) / W25Q_CYCLES_PER_MS;
            return HAL_OK;
        }
        Watchdog_Heartbeat(WDT_TASK_STORAGE);   /* 长等待期间补报心跳 */
        W25Q_Delay1ms();
    } while ((W25Q_NowCycles() - t0) < timeout);

    W25Q_CS_HIGH();
    s_lastBusyMs = (W25Q_NowCycles() - t0) / W25Q_CYCLES_PER_MS;
    return HAL_TIMEOUT;                         /* 超时返回，不死等 */
}

/**
  * 功    能：取上次等待时长。只有 WaitReady 会写它，32 位读本身原子，不用加锁
  * 参    数：无
  * 返 回 值：最近一次 WaitReady 的等待时长，单位 ms
  * 补    充：只由 WaitReady 写，32 位读天然原子，不必加锁
  */
uint32_t W25Q128_GetLastBusyMs(void)
{
    return s_lastBusyMs;
}

/**
  * 功    能：发写使能命令（0x06）。【必须在 BUSY=0 时发】—— 忙的时候芯片会默默忽略且不报错，
  * 参    数：无
  * 返 回 值：无
  * 补    充：必须在 BUSY=0 时发 06h，忙时会被忽略且不报错；调用前先 WaitReady
  *            之后写/擦就会静默失败。所以调用前一定要先 WaitReady
  */
static void W25Q_WriteEnable(void)
{
    W25Q_CS_LOW();
    (void)W25Q_SpiByte(W25Q_CMD_WRITE_ENABLE);
    W25Q_CS_HIGH();
}

/**
  * 功    能：读 JEDEC ID（0x9F）。链路有问题时会读到 0x00FFFF 或 0xFFFFFF，可据此判断
  * 参    数：无
  * 返 回 值：0x00EF4018 形式；链路异常时可能读到 0x00FFFF 或 0xFFFFFF
  * 补    充：读命令不置 BUSY，无需写使能和 WaitReady
  *            读命令不会置 BUSY，所以不用写使能、也不用等待
  */
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

/**
  * 功    能：读 Device ID（0x90，要先发 3 字节假地址，本芯片会忽略它）
  * 参    数：无
  * 返 回 值：0xEF17 形式；异常时可能为 0x0000 或 0xFFFF
  * 补    充：0x90 需先发 3 字节假地址，本芯片忽略地址；读命令不置 BUSY
  *            异常时读到 0x0000 或 0xFFFF。读命令同样不置 BUSY
  */
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

/**
  * 功    能：读一段数据（0x03 + 24 位地址 + 连续读）。地址由芯片自己自增
  * 参    数：addr  起始字节地址，24 位，范围 0x000000~0xFFFFFF
  *            pData  输出缓冲首地址，不允许 NULL
  *            num  读取字节数，范围 1~到芯片末尾的剩余字节
  * 返 回 值：HAL_OK = 读完，HAL_ERROR = 驱动未初始化
  * 补    充：芯片内部地址自增，可一口气读完；addr+num 不得越过 16MB，由调用方保证
  */
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

/**
  * 功    能：擦除一个 4KB 扇区：写使能 → 发 0x20 → 等 BUSY 清零
  * 参    数：addr  字节地址，落点所在 4KB 扇区被整体擦成 0xFF
  * 返 回 值：HAL_OK = 擦完，HAL_TIMEOUT = 等待超时，HAL_ERROR = 未初始化或落在 GBK 字库区
  * 补    充：固定顺序 WaitReady -> 06h -> 20h -> WaitReady
  *            落在字库区内的地址会被拒绝，保护字库不被误擦
  */
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

/**
  * 功    能：写一页：写使能 → 发 0x02 → 发数据 → 等 BUSY
  * 参    数：addr  起始地址，整段必须在同一 256B 页内
  *            pData  数据首地址，不允许 NULL
  *            len  字节数，范围 1~256，且页内偏移 + len <= 256
  * 返 回 值：HAL_OK = 编程完成，HAL_TIMEOUT = 等待超时，HAL_ERROR = 未初始化或参数越界
  * 补    充：NOR 只能把 1 写成 0，写前该区域必须已擦除
  *            整段必须同一页内 —— 跨页时地址会在页内回绕，把页首已经写好的数据盖掉
  */
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

/**
  * 功    能：初始化 SPI 并读 ID 校验芯片。返回 1 通过，0 失败（会打印实际读到的字节）
  * 参    数：无
  * 返 回 值：1 = 初始化且两项 ID 校验通过，0 = 初始化或 ID 校验失败
  * 补    充：boot 阶段调用，上电延时走 DWT 300ms；只应调用一次，失败会打印实读 ID
  */
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

    /* 下面每一条的选择理由都写清楚了，看完就知道 Mode 3 是怎么拼出来的 */
    __HAL_RCC_SPI1_CLK_ENABLE();
    s_hspi1.Instance               = SPI1;
    s_hspi1.Init.Mode              = SPI_MODE_MASTER;       /* 主机：时钟由我们产生 */
    s_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;  /* 全双工，MOSI/MISO 各占一根 */

    /* 8 位一帧。Flash 的命令码、地址、数据都按字节组织，用 8 位最自然。
     * 设成 16 位的话发命令码时会把两个字节捆成一帧，时序就对不上了。 */
    s_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;

    /* ↓ 这两行合起来 = Mode 3（W25Q128 支持 Mode 0 和 Mode 3）
     *   CPOL = HIGH  → 空闲时 SCK 保持高电平
     *   CPHA = 2EDGE → 在第二个边沿（上升沿）采样
     * 主从两端必须一致，不一致的典型症状是「读回来全 0xFF 或全 0」，很像没接线。 */
    s_hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    s_hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;

    /* 软件 NSS：不用外设自动生成片选，改由代码手动拉 PC13。
     * 因为一条 Flash 命令是「命令码 + 地址 + 数据」多段组成的，
     * 必须用一次 CS 低电平把它们全包住；硬件自动 NSS 每发一帧就抬一次，做不到。 */
    s_hspi1.Init.NSS               = SPI_NSS_SOFT;

    /* APB2 84MHz / 8 = 10.5MHz。W25Q128 最高能到 104MHz，
     * 这里刻意压慢，是为了先把链路调通；确认稳定后再逐档提速排查。 */
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    /* MSB 先行。SPI 协议本身没规定顺序，由器件决定：
     * Flash 的命令码是按高位在前定义的，发反了整条命令都认不出来。 */
    s_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;

    s_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;    /* TI 的另一种帧格式，这里不用 */
    s_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;  /* CRC 由 Flash 命令层自己做，链路层不用 */
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

/**
  * 功    能：自检：挑用户区尾部一块扇区，擦 → 写测试图案 → 读回逐个比对。不碰字库区
  * 参    数：无
  * 返 回 值：无，结果通过串口打印 PASS/FAIL
  * 补    充：测试地址 0x009FF000，会破坏该扇区原数据；缓存必须 static（boot 阶段栈只有 1KB）
  */
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

/**
  * 功    能：字库存在性检查：多点采样看 0xFF 占比，全 0xFF 说明根本没烧字库
  * 参    数：无
  * 返 回 值：1 = 判定有字库，0 = 未初始化或判定无字库
  * 补    充：全程只读；s_buf 为 static，避免占用 boot 阶段 1KB 栈
  */
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
