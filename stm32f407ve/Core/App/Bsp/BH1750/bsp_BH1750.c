/* bsp_BH1750.c
 * BH1750 光照传感器驱动：I2C1 寄存器直配，读连续高分辨率模式，输出 float lux。
 * 硬件：SCL=PB6 / SDA=PB7，开漏上拉，100kHz，7 位地址 0x23。
 * 注意：上电后第一次转换要等 120ms；总线 BUSY 死锁时发 9 个 SCL 脉冲加 STOP 恢复。
 *
 * ══════ I2C 的基础（理解后面每一个寄存器配置都靠这一段）══════
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
 *   记住这条，就能看懂 I2C1_BusRecovery 里那几行为什么这么排顺序。
 *
 * ── ACK 应答 ──
 *   主机每发完 8 位（1 字节）就释放 SDA，从机若收到就拉低 SDA 一个时钟 → 这叫 ACK。
 *   主机收到 ACK 才知道「从机在、地址没写错」；没收到就说明地址错了或器件不在。
 *   读方向反过来：主机收完一个字节要回 ACK 表示「继续发」，
 *   收到最后那个字节时回 【NACK】（不拉低）告诉从机「够了，别发了」。
 *
 * ── 为什么会出现「总线死锁」 ──
 *   如果从机在传输中途掉电或复位，它可能正好卡在「把 SDA 拉低」这个状态松不开手。
 *   此时主机要发起始条件（需要 SDA 由高变低），可 SDA 一直被从机按着不放，永远等不到。
 *   解法：主机手动打 9 个 SCL 脉冲，把从机内部那个没发完的字节「推」完，
 *   它就有机会释放 SDA 了 —— 这就是 I2C1_BusRecovery 干的事。
 */
#include "bsp_BH1750.h"


/*全局标志*/
static uint8_t  s_bh1750_ready;   /* 初始化完成标志 */


/*I2C1 寄存器直配*/
/**
  * 功    能：配好 I2C1 的引脚（PB6/PB7 复用开漏）和时钟，给读光照传感器打基础
  * 参    数：无
  * 返 回 值：无
  */
static void I2C1_HardwareInit(void)
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


/**
  * 功    能：I2C 总线死锁恢复：从机把 SDA 一直拉低不放时，主设备手动打 9 个时钟脉冲把它顶出来，
  * 参    数：无
  * 返 回 值：无
  * 补    充：再补一个 STOP。传感器掉电重启后经常需要这一步，否则后续所有读写都超时
  */
static void I2C1_BusRecovery(void)
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

    /*重新初始化 I2C1*/
    I2C1_HardwareInit();
}


/**
  * 功    能：I2C1 主机模式发一串字节（起始 → 从机地址 → 数据 → 停止）
  * 参    数：addr 8 位从机地址；pData 待发数据；len 字节数 1~255
  * 返 回 值：0=成功，1=超时(已尝试总线恢复)
  */
static uint8_t I2C1_MasterWrite(uint8_t addr, const uint8_t *pData, uint8_t len)
{
    uint32_t timeout;

    /* 等待总线空闲 */
    timeout = 0xFFFF;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    /* 产生起始信号 */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 0xFFFF;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    /* 发送从机地址 + 写方向。
     * 【I2C 地址的第 0 位不是地址，是方向位】：
     *   bit0 = 0 → 主机要「写」给从机      bit0 = 1 → 主机要「读」从机
     * 所以 7 位地址要先左移一位、最低位填方向，凑成一个字节发出去。
     * 这里的 & 0xFE 就是把最低位清零（= 写）；下面读函数用的是 | 0x01（= 读）。
     * BH1750 的 7 位地址是 0x23，实际出现在总线上的字节是 0x46(写) / 0x47(读)。 */
    I2C1->DR = addr & 0xFE;
    timeout = 0xFFFF;
    /* 等 ADDR 标志：它表示「从机回了 ACK」，即这个地址确实有设备应答。
     * 如果这里超时，八成是地址写错、器件没接、或者上行电阻没焊 —— 先量硬件再怀疑代码。 */
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }
    (void)I2C1->SR1; (void)I2C1->SR2;                 /* 读 SR1/SR2 清 ADDR 标志 */

    /* 发送数据 */
    for (uint8_t i = 0; i < len; i++)
    {
        timeout = 0xFFFF;
        while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) { }
        if (timeout == 0) { I2C1_BusRecovery(); return 1; }
        I2C1->DR = pData[i];
    }

    /* 等待发送完成 */
    timeout = 0xFFFF;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--)   { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    /* 停止信号 */
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}


/**
  * 功    能：I2C1 主机模式读一串字节。len 传 0 直接判失败（没有可读长度）
  * 参    数：addr 8 位从机地址；pData 接收缓冲；len 字节数，传 0 直接失败
  * 返 回 值：0=成功，1=参数非法或某一步超时(已尝试总线恢复)
  */
static uint8_t I2C1_MasterRead(uint8_t addr, uint8_t *pData, uint8_t len)
{
    uint32_t timeout;

    if (len == 0) return 1;

    /* 等待总线空闲 */
    timeout = 0xFFFF;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)  { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    /* 产生起始信号 */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 0xFFFF;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)    { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    /* 发送从机地址 + R */
    I2C1->DR = addr | 0x01;
    timeout = 0xFFFF;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }

    if (len == 1)
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

        timeout = 0xFFFF;
        while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--) { }
        if (timeout == 0) { I2C1_BusRecovery(); return 1; }
        pData[0] = (uint8_t)I2C1->DR;

        I2C1->CR1 |= I2C_CR1_ACK;   /* 恢复 ACK */
    }
    else
    {
        (void)I2C1->SR1; (void)I2C1->SR2;             /* 读 SR1/SR2 清 ADDR */
        for (uint8_t i = 0; i < len; i++)
        {
            if (i == len - 1)
                I2C1->CR1 &= ~I2C_CR1_ACK;            /* 最后一个字节不应答 */
            timeout = 0xFFFF;
            while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--) { }
            if (timeout == 0) { I2C1_BusRecovery(); return 1; }
            pData[i] = (uint8_t)I2C1->DR;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->CR1 |= I2C_CR1_ACK;                     /* 恢复 ACK */
    }
    return 0;
}


/*BH1750 对外接口*/

/**
  * 功    能：初始化光照传感器：上电、设成连续高分辨率模式，失败会重试 3 次
  * 参    数：无
  * 返 回 值：BH1750_OK=成功，BH1750_ERROR=重试 3 次仍失败
  */
BH1750_Status_t BH1750_Init(void)
{
    uint8_t cmd;
    uint8_t retry;

    s_bh1750_ready = 0;

    /*初始化 I2C1 硬件*/
    I2C1_HardwareInit();

    /*整条序列最多重试 BH1750_RETRY_CNT 次*/
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        /*上电*/
        cmd = BH1750_CMD_POWER_ON;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)                /* 7 位 0x23 左移 1 位 = 0x46(W) */
            continue;

        /*复位清数据寄存器，不做会读到上次的残留值*/
        cmd = BH1750_CMD_RESET;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)
            continue;

        /*连续高分辨率：1 lux，转换 ≤120ms*/
        cmd = BH1750_CMD_CONT_H_MODE;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)
            continue;

        /*等第一次转换*/
        HAL_Delay(120);

        s_bh1750_ready = 1;
        return BH1750_OK;
    }

    return BH1750_ERROR;
}


/* 读一次光照值，单位 lux（0~54612），读成功后把 is_valid 置 1。
 * 返回 BH1750_OK 成功；BH1750_ERROR 表示 data 为空、没初始化或重试 3 次仍失败。 */
/* ---------------------------------------------------------------------------
 * 连续高分辨率模式数据(只读)：
 *   名称   类型/位宽  含义               单位/范围     谁写/谁读
 *   DataH  uint8      高 8 位            0x00~0xFF     BH1750 写 / 主机读
 *   DataL  uint8      低 8 位            0x00~0xFF     BH1750 写 / 主机读
 *   raw    uint16     (DataH<<8)|DataL   0~65535       主机算
 *   lux    float      raw / 1.2          0~54612      主机算
 * ------------------------------------------------------------------------- */
BH1750_Status_t BH1750_Read(BH1750_Data_t *data)
{
    uint8_t  buf[2];
    uint16_t raw;
    uint8_t  retry;

    if (data == NULL)
        return BH1750_ERROR;

    data->is_valid = 0;

    if (!s_bh1750_ready)
        return BH1750_ERROR;

    /*重试 3 次，过滤偶发干扰*/
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        /*连续模式下直接读 2 字节*/
        if (I2C1_MasterRead(BH1750_I2C_ADDR, buf, 2) == 0)                  /* 0x47(R): 直接读最近一次转换结果 */
        {
            raw  = ((uint16_t)buf[0] << 8) | buf[1];
            data->lux      = (float)raw / 1.2f;    /* 换算系数 1.2 出自 BH1750 数据手册 */
            data->is_valid = 1;
            return BH1750_OK;
        }
    }

    /*读失败清标志，下次必须重新 Init*/
    s_bh1750_ready = 0;
    return BH1750_ERROR;
}
