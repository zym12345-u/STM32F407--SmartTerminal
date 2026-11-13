/* sd_diskio.c
 * FatFS 磁盘接口的 SD 实现：把 disk_read/disk_write/disk_ioctl 转给 HAL_SD。
 *
 * 当前使用轮询模式（未启用 DMA）：单次 512B 读写只占用几百微秒，
 * 框架阶段先保证稳定，后续要提速再切换 HAL_SD_ReadBlocks_DMA。
 */
#include "sd_diskio.h"
#include "sdio.h"

#define SD_READ_TIMEOUT_MS   5000U   /*单次读写超时，远大于实际耗时，只防卡死*/

static volatile DSTATUS s_stat = STA_NOINIT;

/**
  * 功    能：轮询等待卡回到 TRANSFER 状态
  * 参    数：timeout_ms  最长等待毫秒数
  * 返 回 值：1=已就绪，0=超时
  * 补    充：写扇区后卡要内部编程几百微秒到几毫秒，只查一次会误判，
  *           必须循环等到它真正退出 PROGRAMMING
  */
static uint8_t SD_WaitTransfer(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return 0U;
        }
    }
    return 1U;
}

static DSTATUS SD_CheckStatus(BYTE lun);

/**
  * 功    能：刷新驱动状态，判断卡是否可用
  */
static DSTATUS SD_CheckStatus(BYTE lun)
{
    (void)lun;

    s_stat = STA_NOINIT;
    if ((hsd.State == HAL_SD_STATE_READY) || (hsd.State == HAL_SD_STATE_TRANSFER))
    {
        s_stat &= (DSTATUS)~STA_NOINIT;
    }
    return s_stat;
}

/**
  * 功    能：初始化磁盘（FatFS 首次访问时调用）
  */
DSTATUS SD_initialize(BYTE lun)
{
    s_stat = STA_NOINIT;

    if (hsd.State == HAL_SD_STATE_RESET)
    {
        MX_SDIO_SD_Init();
    }
    return SD_CheckStatus(lun);
}

/**
  * 功    能：查询磁盘状态
  */
DSTATUS SD_status(BYTE lun)
{
    return SD_CheckStatus(lun);
}

/**
  * 功    能：读扇区
  */
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    (void)lun;

    if (s_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }
    if (HAL_SD_ReadBlocks(&hsd, buff, sector, count, SD_READ_TIMEOUT_MS) != HAL_OK)
    {
        return RES_ERROR;
    }
    return RES_OK;
}

/**
  * 功    能：写扇区
  */
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    (void)lun;

    if (s_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }
    if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, sector, count, SD_READ_TIMEOUT_MS) != HAL_OK)
    {
        return RES_ERROR;
    }

    /*写完必须等卡退出 PROGRAMMING 状态再返回：
     *卡内部编程需要几毫秒，立刻发起下一条读写会 DATA_TIMEOUT，
     *表现就是 f_mount 成功、f_open 却报 FR_DISK_ERR*/
    if (SD_WaitTransfer(SD_READ_TIMEOUT_MS) == 0U)
    {
        return RES_ERROR;
    }

    return RES_OK;
}

/**
  * 功    能：磁盘控制命令
  * 参    数：cmd  磁盘控制命令码，取值见 diskio.h
  */
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;

    (void)lun;

    if (s_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        *(DWORD *)buff = (DWORD)hsd.SdCard.BlockNbr;
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = (WORD)hsd.SdCard.BlockSize;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1U;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
}

Diskio_drvTypeDef SD_Driver =
{
    SD_initialize,
    SD_status,
    SD_read,
    SD_write,
    SD_ioctl
};
