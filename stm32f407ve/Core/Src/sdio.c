/* sdio.c
 * SDIO 外设初始化：PC8~PC11 = D0~D3，PC12 = CK，PD2 = CMD，AF12。
 *
 * 时钟说明：本工程 PLLQ=4，PLL48CLK 实际为 84MHz。
 * SDIO_CK = PLL48CLK / (2 * ClockDiv)，ClockDiv=2 时为 21MHz，
 * 留足余量给杜邦线/长走线；等硬件稳定后可改成 ClockDiv=1(42MHz)。
 */
#include "sdio.h"

SD_HandleTypeDef hsd;

/**
  * 功    能：初始化 SDIO 并识别 SD 卡（4-bit 轮询模式）
  * 参    数：无
  * 返 回 值：无；结果看 hsd.State，HAL_SD_STATE_READY 表示卡已就绪
  * 补    充：不在这里调用 Error_Handler，没插卡时由上层按"无卡"处理
  */
void MX_SDIO_SD_Init(void)
{
    MX_SDIO_SD_InitEx(SDIO_BUS_WIDE_1B, 4U);
}

/**
  * 功    能：按指定总线宽度和分频初始化 SDIO
  * 参    数：busWide   SDIO_BUS_WIDE_1B / SDIO_BUS_WIDE_4B
  *           clockDiv  分频值，SDIO_CK = PLL48CLK / (2 * clockDiv)
  * 返 回 值：无；结果看 hsd.State
  */
void MX_SDIO_SD_InitEx(uint32_t busWide, uint32_t clockDiv)
{
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide             = busWide;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv            = clockDiv;

    (void)HAL_SD_Init(&hsd);
}

/**
  * 功    能：SDIO 底层引脚与时钟初始化，由 HAL_SD_Init 回调
  * 参    数：hsd  SDIO 句柄
  * 返 回 值：无
  */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hsd->Instance != SDIO)
    {
        return;
    }

    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PC8=D0 PC9=D1 PC10=D2 PC11=D3 PC12=CK */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                                GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PD2=CMD */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * 功    能：SDIO 底层反初始化
  * 参    数：hsd  SDIO 句柄
  * 返 回 值：无
  */
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance != SDIO)
    {
        return;
    }

    __HAL_RCC_SDIO_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                           GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}
