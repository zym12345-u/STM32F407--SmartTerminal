/* bsp_I2CBus.c
 * I2C1 总线共享层：硬件初始化、原始读写、ACK 探测、总线恢复、跨任务互斥。
 * 硬件：I2C1 = PB6(SCL) / PB7(SDA)，开漏上拉，100kHz。
 * 挂在总线上的器件：BH1750(7位地址 0x23) / 24C02(7位地址 0x50)。
 *
 * ══════ I2C 的基础（理解后面每一个函数都靠这一段）══════
 *
 * ── 为什么必须"开漏输出 + 上拉电阻" ──
 *   I2C 只有两根线却要挂多个设备，而且任何设备都可能随时开口。
 *   如果用推挽输出：一个设备输出高、另一个输出低，就是【直接短路】，会烧器件。
 *   开漏的特点很关键 ——「只能主动拉低，不能主动拉高」：
 *       想输出 0 → 自己把线拉低
 *       想输出 1 → 松手不管，由上拉电阻把线拉回高
 *   这样多个设备同时驱动也不会短路，谁拉低谁说了算。
 *   （这个套路和 CAN 的「显性覆盖隐性」是同一个思想：能主动拉低的一方赢。）
 *
 * ── 起始 / 停止条件 ──
 *   SCL 为高期间，SDA 由高变低 → 【起始 START】
 *   SCL 为高期间，SDA 由低变高 → 【停止 STOP】
 *   规律：普通数据位的电平变化必须发生在 SCL【低】电平期间，
 *         只有起始和停止才允许在 SCL 高时改变 SDA。
 *   「重复起始 RESTART」= 不先 STOP、在总线上直接再发一次 START：
 *   时序上合法，作用是【不释放总线】地切换读/写方向。
 *
 * ── ACK 应答 ──
 *   主机每发完 8 位（1 字节）就释放 SDA，从机若收到就拉低 SDA 一个时钟 → 这叫 ACK。
 *   主机收到 ACK 才知道「从机在、地址没写错」；没收到（SR1 的 AF 位置 1）
 *   就说明地址错了或器件不应答（EEPROM 写周期内就是这样）。
 *   读方向反过来：主机收完一个字节要回 ACK 表示「继续发」，
 *   收到最后那个字节时回 【NACK】（不拉低）告诉从机「够了，别发了」。
 *
 * ── 为什么会出现「总线死锁」 ──
 *   如果从机在传输中途掉电或复位，它可能正好卡在「把 SDA 拉低」这个状态松不开手。
 *   此时主机要发起始条件（需要 SDA 由高变低），可 SDA 一直被从机按着不放，永远等不到。
 *   解法：主机手动打 9 个 SCL 脉冲，把从机内部那个没发完的字节「推」完，
 *   它就有机会释放 SDA 了 —— 这就是 I2CBus_Recovery 干的事。
 */

#include "bsp_I2CBus.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define I2CBUS_TIMEOUT   0xFFFFU   /* 内部轮询计数：100kHz 下远超一个字节的传输时间 */


/* ── 总线互斥锁 ──────────────────────────────────────────
 * 保护对象是 I2C1 外设状态机(CR1/DR/SR1/SR2)这个共享资源。
 * 创建在 I2CBus_Init 里；内核未跑时 Lock/Unlock 直通(无并发)。 */
static osMutexId_t s_busMutex = NULL;


/**
  * 功    能：配 I2C1 的引脚(PB6/PB7 复用开漏 AF4)和时序寄存器(100kHz)
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅供本文件使用。I2CBus_Init 和 I2CBus_Recovery 都要调它 ——
  *            Recovery 里外设被强制复位过，CCR/TRISE/ACK 全回默认值，
  *            不重配的话这条总线就废了(后续所有访问超时)。
  */
static void i2c_hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*开 GPIOB 与 I2C1 时钟*/
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /*PB6/PB7 复用为 I2C1，开漏上拉*/
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 配 CCR/TRISE 之前必须先清 PE（外设使能位）—— STM32 的硬性要求：
     * I2C 处于关闭状态时这些时序寄存器才允许写，边跑边改会被忽略，且不报错。 */
    I2C1->CR1  &= ~I2C_CR1_PE;                          /* 关闭 I2C1 */

    /* FREQ：告诉外设「我的 APB1 时钟是多少」，它内部按这个值去推算时序 */
    I2C1->CR2   = 42U;                                  /* FREQ = APB1 = 42MHz */

    /* CCR：决定 SCL 的频率。
     *   标准模式(100kHz)：CCR = fPCLK1 / (2 × fSCL) = 42M / 200k = 210
     *   为什么要除 2？因为一个 SCL 周期里高、低电平各占一半，
     *   CCR 配的是「半个周期」对应的计数值。 */
    I2C1->CCR   = 210U;

    /* TRISE：SCL 上升沿允许的最长时间。标准模式规范给的是 1000ns。
     *   APB1 一个周期 = 1/42MHz ≈ 23.8ns，所以 TRISE = 1000/23.8 + 1 ≈ 43。
     *   配小了会时序违规（上升沿被误判成异常），配大了会白白降低速率。 */
    I2C1->TRISE = 43U;

    I2C1->OAR1  = 0;                                    /* 自身地址(作主机时用不到) */
    I2C1->CR1  |= I2C_CR1_PE;                           /* 使能 I2C1 */
    I2C1->CR1  |= I2C_CR1_ACK;                          /* 使能应答：收到字节硬件自动回 ACK */
}


/* ══════════════════ 总线恢复 ══════════════════ */
/**
  * 功    能：总线死锁恢复——手动打 9 个 SCL 脉冲再补一个 STOP，把卡死的 SDA 顶出来，最后重配总线
  * 参    数：无
  * 返 回 值：无
  * 补    充：只在「等标志超时」这类异常路径调用。做法(照搬原 BH1750 驱动里
  *            真机验证过的实现)：SCL/SDA 临时切 GPIO 开漏，打时钟时【边打边看
  *            SDA】，SDA 一被从机释放就提前收手(最多 9 个)；最后摆一个 STOP
  *            形态，切回复用，并强制复位外设 + 完整重配时序
  *            (不重配的话 CCR/TRISE 全是默认值，总线就废了)。
  */
static void I2CBus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t i;

    /*关外设*/
    I2C1->CR1 &= ~I2C_CR1_PE;

    /*SCL/SDA 切回 GPIO 开漏输出*/
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 先释放 SDA/SCL 为高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    /*最多 9 个 SCL 脉冲：8 位数据 + 1 个应答位*/
    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   /* SCL 低 */
        for (volatile uint32_t d = 0; d < 100; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL 高 */
        for (volatile uint32_t d = 0; d < 100; d++) { }
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
            break;   /* SDA 已被从机释放 */
    }

    /*SCL 高期间 SDA 低→高，构成 STOP*/
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   /* SDA 低 */
    for (volatile uint32_t d = 0; d < 100; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL 高 */
    for (volatile uint32_t d = 0; d < 100; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);     /* SDA 高 = STOP */
    for (volatile uint32_t d = 0; d < 100; d++) { }

    /*外设彻底复位(状态机/时序寄存器全部回默认)，再完整重配*/
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    i2c_hw_init();
}


/* ══════════════════ 初始化 ══════════════════ */
uint8_t I2CBus_Init(void)
{
    i2c_hw_init();

    /* 互斥锁只建一次(幂等) */
    if (s_busMutex == NULL)
    {
        s_busMutex = osMutexNew(NULL);
    }
    return 0U;
}


/* ══════════════════ 锁 ══════════════════ */
void I2CBus_Lock(void)
{
    if ((osKernelGetState() == osKernelRunning) && (s_busMutex != NULL))
    {
        (void)osMutexAcquire(s_busMutex, osWaitForever);
    }
}

void I2CBus_Unlock(void)
{
    if ((osKernelGetState() == osKernelRunning) && (s_busMutex != NULL))
    {
        (void)osMutexRelease(s_busMutex);
    }
}


/* ══════════════════ 原始写 ══════════════════ */
uint8_t I2CBus_WriteRaw(uint8_t addr8, const uint8_t *pData, uint16_t len)
{
    uint32_t timeout;
    uint16_t i;

    /* 发送从机地址 + 写方向。
     * 【I2C 地址的第 0 位不是地址，是方向位】：
     *   bit0 = 0 → 主机要「写」给从机      bit0 = 1 → 主机要「读」从机
     * 所以 7 位地址要先左移一位、最低位填方向，凑成一个字节发出去。
     * 这里的 & 0xFE 就是把最低位清零（= 写）。
     * BH1750 的 7 位地址是 0x23 → 总线字节 0x46(写)/0x47(读)；
     * 24C02 的 7 位地址是 0x50 → 总线字节 0xA0(写)/0xA1(读)。 */
    timeout = I2CBUS_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }   /* 等总线空闲(带超时) */
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    /* 产生起始信号 */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    I2C1->DR = addr8 & 0xFE;
    timeout = I2CBUS_TIMEOUT;
    /* 等 ADDR 标志：它表示「从机回了 ACK」，即这个地址确实有设备应答。
     * 如果这里超时，八成是地址写错、器件没接、或者上行电阻没焊 —— 先量硬件再怀疑代码。 */
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }
    (void)I2C1->SR1; (void)I2C1->SR2;                 /* 读 SR1/SR2 清 ADDR 标志 */

    /* 发送数据 */
    for (i = 0; i < len; i++)
    {
        timeout = I2CBUS_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) { }
        if (timeout == 0) { I2CBus_Recovery(); return 1; }
        I2C1->DR = pData[i];
    }

    /* 等待发送完成 */
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--)   { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    /* 停止信号 */
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}


/* ══════════════════ 读段(ReadRaw 与 WriteReadRaw 共用) ══════════════════
 * 进入前提：地址+读方向已发出、SR1 的 ADDR 已置位(尚未清除)。
 * 返回 0 = 读走 len 个字节并已 STOP；1 = 超时(调用方负责恢复)。 */
static uint8_t i2c_read_phase(uint8_t *pData, uint16_t len)
{
    uint32_t timeout;
    uint16_t i;

    if (len == 1U)
    {
        /* 单字节读：【顺序不能反】—— 必须先清 ACK，再清 ADDR。
         *
         * 为什么：清 ADDR 标志的那一刻，I2C 外设就立刻开始接收数据了。
         *   - 如果先清 ADDR：此时 ACK 还是使能的，硬件收完这 1 个字节会自动回 ACK，
         *     等于告诉从机「继续发」，而从机继续发的时候主机已经在发 STOP 了，
         *     总线时序就乱了。
         *   - 所以要先清 ACK（关掉自动应答 = 收完这字节回 NACK = 告诉从机"够了"），
         *     再清 ADDR 启动接收。 */
        I2C1->CR1 &= ~I2C_CR1_ACK;      /* 关自动应答 → 收完本字节回 NACK */
        (void)I2C1->SR1; (void)I2C1->SR2;  /* 清 ADDR，此时才开始真正接收 */
        I2C1->CR1 |= I2C_CR1_STOP;

        timeout = I2CBUS_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--) { }
        if (timeout == 0) { return 1U; }
        pData[0] = (uint8_t)I2C1->DR;

        I2C1->CR1 |= I2C_CR1_ACK;   /* 恢复 ACK */
    }
    else
    {
        (void)I2C1->SR1; (void)I2C1->SR2;             /* 读 SR1/SR2 清 ADDR */
        for (i = 0; i < len; i++)
        {
            if (i == len - 1)
                I2C1->CR1 &= ~I2C_CR1_ACK;            /* 最后一个字节不应答 */
            timeout = I2CBUS_TIMEOUT;
            while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--) { }
            if (timeout == 0) { return 1U; }
            pData[i] = (uint8_t)I2C1->DR;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->CR1 |= I2C_CR1_ACK;                     /* 恢复 ACK */
    }
    return 0;
}


uint8_t I2CBus_ReadRaw(uint8_t addr8, uint8_t *pData, uint16_t len)
{
    uint32_t timeout;

    if ((pData == NULL) || (len == 0U))
    {
        return 1U;
    }

    /* 等待总线空闲 */
    timeout = I2CBUS_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    /* 产生起始信号 */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    /* 发送从机地址 + R */
    I2C1->DR = addr8 | 0x01;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    if (i2c_read_phase(pData, len) != 0U)
    {
        I2CBus_Recovery();
        return 1U;
    }
    return 0;
}


/* ══════════════════ 先写后读的组合事务 ══════════════════ */
uint8_t I2CBus_WriteReadRaw(uint8_t addr8, const uint8_t *pWr, uint16_t wrLen,
                            uint8_t *pRd, uint16_t rdLen)
{
    uint32_t timeout;
    uint16_t i;

    if ((pWr == NULL) || (pRd == NULL) || (wrLen == 0U) || (rdLen == 0U))
    {
        return 1U;
    }

    /* ── 第一段：写(发寄存器/存储地址)，不发 STOP ── */
    timeout = I2CBUS_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    I2C1->DR = addr8 & 0xFE;             /* 写方向 */
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }
    (void)I2C1->SR1; (void)I2C1->SR2;

    for (i = 0; i < wrLen; i++)
    {
        timeout = I2CBUS_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) { }
        if (timeout == 0) { I2CBus_Recovery(); return 1; }
        I2C1->DR = pWr[i];
    }
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--)   { }   /* 写段数据全部发完 */
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    /* ── 第二段：RESTART 转读 ──
     * 不 STOP 直接再 START：一是保住总线的原子性(别的节点插不进来)，
     * 二是对 EEPROM 而言，STOP 会让它把"写字地址"当成一次写操作、
     * 进入写周期，后面就什么都读不到了。 */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    I2C1->DR = addr8 | 0x01;             /* 读方向 */
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1; }

    if (i2c_read_phase(pRd, rdLen) != 0U)
    {
        I2CBus_Recovery();
        return 1U;
    }
    return 0;
}


/* ══════════════════ ACK 探测 ══════════════════ */
uint8_t I2CBus_Probe(uint8_t addr8)
{
    uint32_t timeout;
    uint16_t sr1;

    timeout = I2CBUS_TIMEOUT;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1U; }

    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2CBUS_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)  { }
    if (timeout == 0) { I2CBus_Recovery(); return 1U; }

    I2C1->DR = addr8 & 0xFE;
    /* 等 ADDR(有应答) 或 AF(明确无应答)——AF 先到就立刻退出，
     * 不用傻等整个超时计数跑完。这正是 ACK 轮询能"快速失败"的原因。 */
    timeout = I2CBUS_TIMEOUT;
    for (;;)
    {
        sr1 = I2C1->SR1;
        if (sr1 & I2C_SR1_AF)
        {
            /* 无应答：清 AF(写 0 清除)，发 STOP 释放总线，返回"不在"。
             * 注意这不算总线故障，不触发 Recovery —— EEPROM 写周期内
             * 不应答是【正常行为】，恰恰是 ACK 轮询在等的东西。 */
            I2C1->SR1 = (uint16_t)~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 1U;
        }
        if (sr1 & I2C_SR1_ADDR)
        {
            break;                      /* 有 ACK */
        }
        if (timeout-- == 0U)
        {
            I2CBus_Recovery();
            return 1U;
        }
    }
    (void)I2C1->SR1; (void)I2C1->SR2;   /* 清 ADDR */
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0U;
}
