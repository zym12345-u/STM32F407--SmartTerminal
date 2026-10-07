/**
  ******************************************************************************
  * @file    bsp_BH1750.c
  * @brief   BH1750 环境光强度传感器驱动实现(硬件 I2C1,寄存器直配)
  *
  *          说明:
  *            1. 本工程 HAL 库为精简子集(未包含 stm32f4xx_hal_i2c.c/.h),
  *               因此 BH1750 的 I2C1 通信直接操作 I2C1 寄存器实现,
  *               风格与 bsp_LCD_ILI9341.c 中 FSMC 直配寄存器一致。
  *            2. I2C1 工作在标准模式 100kHz,PB6=SCL、PB7=SDA,开漏+上拉。
  ******************************************************************************
  */
#include "bsp_BH1750.h"


/* ====================== 全局标志 ====================== */
static uint8_t  s_bh1750_ready;   /* 初始化完成标志 */


/* ====================== I2C1 寄存器直配 ====================== */
/**
  * @brief  配置 PB6/PB7 为 I2C1 复用开漏,并初始化 I2C1 外设(100kHz)
  * @note   APB1 = 42MHz (HCLK 168MHz / 4)
  */
static void I2C1_HardwareInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. 开启 GPIOB 与 I2C1 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* 2. PB6/PB7 -> I2C1, 开漏, 上拉, 高速 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 3. I2C1 外设配置(先关外设再配寄存器) */
    I2C1->CR1  &= ~I2C_CR1_PE;                          /* 关闭 I2C1 */

    I2C1->CR2   = 42U;                                  /* FREQ = APB1 = 42MHz */
    I2C1->CCR   = 210U;                                 /* 100kHz: CCR = 42M/(2*100k) = 210 */
    I2C1->TRISE = 43U;                                  /* 1000ns/23.8ns + 1 ≈ 43 */

    I2C1->OAR1  = 0;                                    /* 自身地址(主机可忽略) */
    I2C1->CR1  |= I2C_CR1_PE;                           /* 使能 I2C1 */
    I2C1->CR1  |= I2C_CR1_ACK;                          /* 使能应答 */
}


/**
  * @brief  I2C1 总线恢复:当从机异常把 SDA 钳位在低电平时,
  *         手动产生 9 个 SCL 脉冲迫使从机释放 SDA,再发 STOP。
  * @note   调用前 I2C1 可能已死锁(BUSY 永远置位)。
  */
static void I2C1_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t i;

    /* 1. 关闭 I2C1 外设 */
    I2C1->CR1 &= ~I2C_CR1_PE;

    /* 2. PB6(SCL)/PB7(SDA) 切回 GPIO 开漏输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 先释放 SDA/SCL 为高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    /* 3. 产生最多 9 个 SCL 脉冲,直到 SDA 被释放 */
    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   /* SCL 低 */
        for (volatile uint32_t d = 0; d < 100; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL 高 */
        for (volatile uint32_t d = 0; d < 100; d++) { }
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
            break;   /* SDA 已被从机释放 */
    }

    /* 4. 产生 STOP 条件: SDA 低→高(在 SCL 高期间) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   /* SDA 低 */
    for (volatile uint32_t d = 0; d < 100; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL 高 */
    for (volatile uint32_t d = 0; d < 100; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);     /* SDA 高 = STOP */
    for (volatile uint32_t d = 0; d < 100; d++) { }

    /* 5. 重新初始化 I2C1 外设 */
    I2C1_HardwareInit();
}


/**
  * @brief  I2C1 主机发送 N 字节
  * @param  addr   从机 8 位地址(已左移,含 W=0)
  * @param  pData  数据缓冲区
  * @param  len    字节数
  * @retval 0-成功,1-失败(已尝试总线恢复)
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

    /* 发送从机地址 + W */
    I2C1->DR = addr & 0xFE;
    timeout = 0xFFFF;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)  { }
    if (timeout == 0) { I2C1_BusRecovery(); return 1; }
    (void)I2C1->SR1; (void)I2C1->SR2;                 /* 读 SR1/SR2 清 ADDR */

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
  * @brief  I2C1 主机接收 N 字节
  * @param  addr   从机 8 位地址(已左移)
  * @param  pData  接收缓冲区
  * @param  len    字节数
  * @retval 0-成功,1-失败
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
        /* 单字节接收:清 ACK,再读 SR1/SR2 清 ADDR */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR1; (void)I2C1->SR2;
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


/* ====================== BH1750 对外接口 ====================== */

BH1750_Status_t BH1750_Init(void)
{
    uint8_t cmd;
    uint8_t retry;

    s_bh1750_ready = 0;

    /* 1. 初始化 I2C1 硬件 */
    I2C1_HardwareInit();

    /* 整个初始化序列最多重试 BH1750_RETRY_CNT 次 */
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        /* 2. 上电 */
        cmd = BH1750_CMD_POWER_ON;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)
            continue;

        /* 3. 复位(清数据寄存器) */
        cmd = BH1750_CMD_RESET;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)
            continue;

        /* 4. 配置为连续高分辨率模式(1lux,转换时间≤120ms) */
        cmd = BH1750_CMD_CONT_H_MODE;
        if (I2C1_MasterWrite(BH1750_I2C_ADDR, &cmd, 1) != 0)
            continue;

        /* 5. 等待第一次转换完成 */
        HAL_Delay(120);

        s_bh1750_ready = 1;
        return BH1750_OK;
    }

    return BH1750_ERROR;
}


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

    /* 读取最多重试 BH1750_RETRY_CNT 次,排除偶发干扰 */
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        /* 连续模式下直接读取 2 字节数据 */
        if (I2C1_MasterRead(BH1750_I2C_ADDR, buf, 2) == 0)
        {
            raw  = ((uint16_t)buf[0] << 8) | buf[1];
            data->lux      = (float)raw / 1.2f;    /* BH1750 数据手册公式 */
            data->is_valid = 1;
            return BH1750_OK;
        }
    }

    /* 多次读取失败,尝试重新初始化传感器 */
    s_bh1750_ready = 0;
    return BH1750_ERROR;
}
