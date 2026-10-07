/**
  ******************************************************************************
  * @file    bsp_DHT11.c
  * @brief   DHT11 温湿度传感器驱动实现(单总线 + DWT 微秒计时)
  ******************************************************************************
  */
#include "bsp_DHT11.h"
#include "cmsis_os2.h"   /* osDelay: 起始信号等待期间让出 CPU 给其他任务 */


/* ====================== 微秒级计时(DWT 周期计数器) ====================== */
/**
  * @brief  使能 DWT 周期计数器(Cortex-M3/M4/M7 通用)
  */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能 DWT */
    /* CYCCNT 的清零由 FreeRTOS 运行时统计模块(vConfigureTimerForRunTimeStats)
     * 统一做一次,这里不清零,避免运行中复位导致统计窗口异常 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* 开启周期计数 */
}

/**
  * @brief  读取当前 cycle 数
  */
static __INLINE uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/**
  * @brief  微秒级延时(基于 DWT)
  * @param  us  延时微秒数
  */
static void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT_GetCycles();
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT_GetCycles() - start) < ticks) { ; }
}

/**
  * @brief  获取自 start 以来经过的微秒数
  */
static uint32_t DWT_GetElapsedUs(uint32_t start)
{
    return (DWT_GetCycles() - start) / (SystemCoreClock / 1000000U);
}


/* ====================== 引脚操作 ====================== */
/**
  * @brief  配置 DATA 引脚为推挽输出
  */
static void DHT11_PinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  配置 DATA 引脚为输入(上拉)
  */
static void DHT11_PinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

#define DHT11_ReadPin()    HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)
#define DHT11_WritePin(x)  HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, (x))


/* ====================== 对外接口 ====================== */

DHT11_Status_t DHT11_Init(void)
{
    DHT11_GPIO_CLK_EN();
    DWT_Init();
    DHT11_PinInput();     /* 空闲时为输入,由外部上拉保持高电平 */
    return DHT11_OK;
}


DHT11_Status_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t  buf[5] = {0};
    uint8_t  i, j;
    uint32_t t;

    if (data == NULL)
        return DHT11_ERROR;

    data->is_valid = 0;

    /* ---------- 1. 主机发送起始信号 ---------- */
    DHT11_PinOutput();
    DHT11_WritePin(GPIO_PIN_RESET);      /* 拉低至少 18ms */
    osDelay(20);                         /* 阻塞让出 CPU: 引脚锁存低电平,信号时序不受影响 */
    DHT11_WritePin(GPIO_PIN_SET);        /* 释放总线(拉高) */
    DWT_DelayUs(30);                      /* 等待 20~40us */
    DHT11_PinInput();                     /* 切换为输入,等待 DHT11 响应 */

    /* ---------- 2. 等待 DHT11 响应(拉低约 80us) ---------- */
    t = DWT_GetCycles();
    while (DHT11_ReadPin() == GPIO_PIN_SET)
    {
        if (DWT_GetElapsedUs(t) > 100)    return DHT11_TIMEOUT;
    }
    t = DWT_GetCycles();
    while (DHT11_ReadPin() == GPIO_PIN_RESET)
    {
        if (DWT_GetElapsedUs(t) > 100)    return DHT11_TIMEOUT;
    }

    /* ---------- 3. 等待响应高电平结束(约 80us) ---------- */
    t = DWT_GetCycles();
    while (DHT11_ReadPin() == GPIO_PIN_SET)
    {
        if (DWT_GetElapsedUs(t) > 100)    return DHT11_TIMEOUT;
    }

    /* ---------- 4. 读取 40 位数据 ---------- */
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            /* 每 bit 起始: 先拉低约 50us */
            t = DWT_GetCycles();
            while (DHT11_ReadPin() == GPIO_PIN_RESET)
            {
                if (DWT_GetElapsedUs(t) > 70)    return DHT11_TIMEOUT;
            }

            /* 测量高电平持续时间: 26~28us 为 0, 70us 为 1 */
            t = DWT_GetCycles();
            while (DHT11_ReadPin() == GPIO_PIN_SET)
            {
                if (DWT_GetElapsedUs(t) > 100)    return DHT11_TIMEOUT;
            }

            buf[i] <<= 1;
            if (DWT_GetElapsedUs(t) > 40)    /* 高电平 > 40us 判定为 1 */
                buf[i] |= 0x01;
        }
    }

    /* ---------- 5. 校验 ---------- */
    if (((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3])) != buf[4])
        return DHT11_CHECKSUM_ERR;

    /* ---------- 6. 解析数据 ---------- */
    data->humidity    = (float)buf[0] + (float)buf[1] * 0.1f;
    data->temperature = (float)buf[2] + (float)buf[3] * 0.1f;
    data->is_valid    = 1;

    return DHT11_OK;
}
