/* bsp_SDCard.c
 * TF/SD 卡 BSP：SDIO(4-bit 轮询) + FatFS 挂载。
 */
#include "bsp_SDCard.h"

#include "sdio.h"
#include "fatfs.h"
#include "ff.h"
#include "usart.h"

static FATFS s_fs;
static uint8_t s_inited;
static uint8_t s_mountResult = 0xFFU;
static uint8_t s_busWidth = 1U;

/**
  * 功    能：按指定总线宽度尝试初始化，并用一次真实读扇区确认数据线可用
  * 参    数：wide  总线宽度（SDIO_BUS_WIDE_1B / SDIO_BUS_WIDE_4B）
  *           div   时钟分频
  * 返 回 值：1=可用，0=不可用
  * 补    充：卡能识别只说明 CMD 通，必须真读一个扇区才能证明 D0~D3 通
  */
static uint8_t SdCard_TryMode(uint32_t wide, uint32_t div)
{
    static uint8_t probe[512];

    if ((hsd.State == HAL_SD_STATE_READY) || (hsd.State == HAL_SD_STATE_TRANSFER))
    {
        (void)HAL_SD_DeInit(&hsd);
    }

    /*必须先用 1-bit 完成卡识别：ACMD6 切宽总线只能在卡初始化之后发。
     *直接把 BusWide=4B 塞进 HAL_SD_Init 只会改主机 CLKCR.WIDBUS，
     *卡仍工作在 1-bit，4 线读时 D1~D3 没有起始位，SDIO 报 STBITERR*/
    MX_SDIO_SD_InitEx(SDIO_BUS_WIDE_1B, div);

    if ((hsd.State != HAL_SD_STATE_READY) && (hsd.State != HAL_SD_STATE_TRANSFER))
    {
        return 0U;
    }

    /*真正切宽总线：这一步才会给卡发 ACMD6*/
    if (wide == SDIO_BUS_WIDE_4B)
    {
        if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
        {
            return 0U;
        }
    }

    if (HAL_SD_ReadBlocks(&hsd, probe, 0U, 1U, 3000U) != HAL_OK)
    {
        return 0U;
    }

    s_busWidth = (wide == SDIO_BUS_WIDE_4B) ? 4U : 1U;
    return 1U;
}

/**
  * 功    能：初始化 SDIO 并挂载 FatFS
  * 参    数：无
  * 返 回 值：SDCARD_* 状态码
  */
uint8_t SDCard_Init(void)
{
    if (s_inited != 0U)
    {
        return SDCARD_OK;
    }

    /*优先 4-bit；失败再回退 1-bit*/
    if (SdCard_TryMode(SDIO_BUS_WIDE_4B, 4U) == 0U)
    {
        if (SdCard_TryMode(SDIO_BUS_WIDE_1B, 4U) == 0U)
        {
            return SDCARD_ERR_NO_CARD;
        }
    }

    MX_FATFS_Init();

    s_mountResult = (uint8_t)f_mount(&s_fs, SD_Path, 1U);
    if (s_mountResult != (uint8_t)FR_OK)
    {
        return SDCARD_ERR_MOUNT;
    }

    s_inited = 1U;
    return SDCARD_OK;
}

/**
  * 功    能：卸载并反初始化
  * 参    数：无
  * 返 回 值：无
  */
void SDCard_DeInit(void)
{
    if (s_inited == 0U)
    {
        return;
    }

    (void)f_mount(NULL, SD_Path, 0U);
    s_inited = 0U;

    (void)HAL_SD_DeInit(&hsd);
}

/**
  * 功    能：查询卡是否就绪
  * 参    数：无
  * 返 回 值：1=就绪，0=未就绪
  */
uint8_t SDCard_IsReady(void)
{
    if ((hsd.State == HAL_SD_STATE_READY) || (hsd.State == HAL_SD_STATE_TRANSFER))
    {
        return 1U;
    }
    return 0U;
}

/**
  * 功    能：读取容量 KB
  * 参    数：无
  * 返 回 值：容量(KB)，未就绪返回 0
  */
uint32_t SDCard_GetCapacityKB(void)
{
    uint64_t bytes;

    if (SDCard_IsReady() == 0U)
    {
        return 0U;
    }

    /*2GB 卡的字节数会超过 32 位，中间必须用 64 位*/
    bytes = (uint64_t)hsd.SdCard.BlockNbr * (uint64_t)hsd.SdCard.BlockSize;
    return (uint32_t)(bytes / 1024ULL);
}

/**
  * 功    能：4-bit 诊断：试切 4-bit 读一个扇区，并打印 SDIO 关键寄存器
  * 参    数：无
  * 返 回 值：1=4-bit 可读写，0=失败
  * 补    充：仅供排查时调用；结束时尝试恢复 1-bit
  */
uint8_t SDCard_Diag4Bit(void)
{
    uint8_t  ok;
    uint32_t clkcr;
    uint32_t dctrl;
    uint32_t sta;
    uint32_t err;

    ok = SdCard_TryMode(SDIO_BUS_WIDE_4B, 4U);

    clkcr = SDIO->CLKCR;
    dctrl = SDIO->DCTRL;
    sta   = SDIO->STA;
    err   = hsd.ErrorCode;

    /*CLKCR[15:14]=WIDBUS(00=1bit,01=4bit)；STA 的 DTIMEOUT/DCRCFAIL/RXOVERR 指向不同故障*/
    UART_Printf("[SD4] ok=%u clkcr=0x%08lX dctrl=0x%08lX sta=0x%08lX err=0x%08lX type=0x%08lX\r\n",
                (unsigned)ok,
                (unsigned long)clkcr,
                (unsigned long)dctrl,
                (unsigned long)sta,
                (unsigned long)err,
                (unsigned long)hsd.SdCard.CardType);

    (void)HAL_SD_DeInit(&hsd);
    UART_Printf("[SD4] restore1=%u\r\n", (unsigned)SdCard_TryMode(SDIO_BUS_WIDE_1B, 4U));

    return ok;
}

/**
  * 功    能：取最近一次挂载返回码
  * 参    数：无
  * 返 回 值：FRESULT，0xFF 表示还没挂载过
  */
uint8_t SDCard_GetMountResult(void)
{
    return s_mountResult;
}

/**
  * 功    能：取当前实际使用的总线宽度
  * 参    数：无
  * 返 回 值：1 或 4
  */
uint8_t SDCard_GetBusWidth(void)
{
    return s_busWidth;
}
