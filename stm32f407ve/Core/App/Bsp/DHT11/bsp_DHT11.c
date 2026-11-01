/* bsp_DHT11.c
 * DHT11 温湿度驱动：单总线软件时序，DWT 微秒计时，读 40 位并校验。
 * 硬件：DATA=PC1，主机侧推挽/浮空切换，需外接 4.7k 上拉至 3.3V。
 * 注意：采样间隔 ≥1s；高电平 26~28us 判 0、约 70us 判 1，门限取 40us。
 */
#include "bsp_DHT11.h"
#include "cmsis_os2.h"   /* osDelay: 起始信号等待期间让出 CPU 给其他任务 */


/*微秒级计时(DWT 周期计数器)*/
/* 使能 DWT 的周期计数器（CYCCNT），给 DHT11 的微秒级时序当基准。
 * 没有它做不了 us 级延时 —— DHT11 每个电平的判定都在几十微秒量级。 */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* TRCENA 不置 1，DWT 不计数 */
    /* CYCCNT 不清零: 清掉会打乱 FreeRTOS 运行时统计窗口 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* 开启周期计数 */
}

/* 读当前内核周期数。168MHz 下约 25.6s 回绕一次，
 * 用无符号减法算差值可以正确跨过回绕点。 */
static __INLINE uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* 用 DWT 计数做微秒级忙等（会占着 CPU，但精度够）。
 * 常用 30us —— 主机发完起始信号、释放总线后要等这一小段。 */
static void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT_GetCycles();
    uint32_t ticks = us * (SystemCoreClock / 1000000U);   /* 168MHz 下 1us = 168 个周期 */
    while ((DWT_GetCycles() - start) < ticks) { ; }
}

/* 算从 start 到现在过了多少微秒。DHT11 里每个「等电平变化」的超时判断都靠它，
 * 也是防止时序错乱时死等下去的唯一手段。 */
static uint32_t DWT_GetElapsedUs(uint32_t start)
{
    return (DWT_GetCycles() - start) / (SystemCoreClock / 1000000U);
}


/*引脚操作*/
/* 把数据引脚切成输出 —— 主机要拉低总线发起始信号时才用。 */
static void DHT11_PinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/* 把数据引脚切回输入 —— 发完起始信号、准备读传感器回的电平。 */
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


/*对外接口*/

/* 初始化 DHT11：配引脚、初始化 DWT 时基。
 * 注意它恒返回 OK —— 这个驱动没有硬件检测手段，传感器掉线要等第一次读超时才发现。 */
DHT11_Status_t DHT11_Init(void)
{
    DHT11_GPIO_CLK_EN();
    DWT_Init();
    DHT11_PinInput();     /* 空闲为输入上拉，靠外部上拉保持高电平 */
    return DHT11_OK;
}


/* 读一次温湿度：拉低总线发起始 → 等传感器响应 → 逐位读 40bit → 校验和。
 * 每一步等待都有微秒级超时兜底，不会因为传感器没接就卡死在这里。
 * 返回 OK / ERROR（data 为空）/ TIMEOUT（等电平超时）/ CHECKSUM_ERR（校验错）。 */
DHT11_Status_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t  buf[5] = {0};
    uint8_t  i, j;
    uint32_t t;

    if (data == NULL)
        return DHT11_ERROR;

    data->is_valid = 0;

    /*主机发起始信号*/
    DHT11_PinOutput();
    DHT11_WritePin(GPIO_PIN_RESET);      /* 拉低至少 18ms */
    osDelay(20);                         /* 引脚已锁存低电平，让出 CPU 不影响时序 */
    DHT11_WritePin(GPIO_PIN_SET);        /* 释放总线(拉高) */
    DWT_DelayUs(30);                      /* 等待 20~40us */
    DHT11_PinInput();                     /* 切回输入，否则和从机抢总线 */

    /*等 DHT11 拉低约 80us 的响应*/
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

    /*等响应高电平结束(约 80us)*/
    t = DWT_GetCycles();
    while (DHT11_ReadPin() == GPIO_PIN_SET)
    {
        if (DWT_GetElapsedUs(t) > 100)    return DHT11_TIMEOUT;
    }

    /*读 40 位数据*/
    /* -----------------------------------------------------------------------
     * DHT11 40 位帧(每字节高位先出，共 5 字节)：
     *   buf[0]  湿度整数   20~90 %RH             DHT11 写 / 主机读
     *   buf[1]  湿度小数   恒为 0                DHT11 写 / 主机读
     *   buf[2]  温度整数   0~50 °C               DHT11 写 / 主机读
     *   buf[3]  温度小数   恒为 0                DHT11 写 / 主机读
     *   buf[4]  校验和     (buf[0]~buf[3]) & 0xFF DHT11 写 / 主机读
     *   位判别  每位先 50us 低电平，随后高电平 26~28us 为 0、约 70us 为 1
     * ----------------------------------------------------------------------- */
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            /*每位先拉低约 50us*/
            t = DWT_GetCycles();
            while (DHT11_ReadPin() == GPIO_PIN_RESET)
            {
                if (DWT_GetElapsedUs(t) > 70)    return DHT11_TIMEOUT;
            }

            /*高电平持续时间决定该位*/
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

    /*校验：只取低 8 位*/
    if (((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3])) != buf[4])    /* 校验和只取低 8 位 */
        return DHT11_CHECKSUM_ERR;

    /*解析成温度与湿度*/
    data->humidity    = (float)buf[0] + (float)buf[1] * 0.1f;    /* 整数 + 0.1×小数(DHT11 小数为 0) */
    data->temperature = (float)buf[2] + (float)buf[3] * 0.1f;    /* 同上; DHT11 量程 0~50 °C, 无负数 */
    data->is_valid    = 1;

    return DHT11_OK;
}
