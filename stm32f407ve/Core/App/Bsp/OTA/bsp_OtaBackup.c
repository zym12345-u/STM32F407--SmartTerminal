/* bsp_OtaBackup.c
 * 槽 A 整区备份：BootLoader 回滚的唯一基线。
 * 原实现在 GuiTask（BK_* 系列），远程 MQTT 下载同样需要它，下沉到 Bsp 共用。
 */
#include "bsp_OtaInternal.h"
#include "OtaData.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include <string.h>


/*══════ 槽 A 整区备份：BootLoader 回滚的唯一基线 ══════
 *
 * ── 为什么按整区 448KB 备份，而不是只备份"实际镜像大小" ──
 *   回滚时 BootLoader 的做法是「整段擦掉 App 区、整段写回」，
 *   备份也按同样粒度存，两边的长度、偏移、CRC 口径就永远一致，
 *   不用再在槽里维护一份"这份镜像到底多少字节"的额外信息。
 *   多抄的尾部 0xFF 不影响回滚结果，代价只是多花一秒左右。
 *
 * ── 记录为什么最后写，而且记录 CRC 最后写 ──
 *   判断"槽里有可用备份"只看首 4KB 的 magic + 记录 CRC。
 *   如果先写记录再写镜像，中途断电会留下一条"记录说完整、镜像却只写了一半"的假备份，
 *   BootLoader 拿它回滚就把 App 写坏。所以顺序是：擦记录扇区 → 写镜像 → 回读校验 → 才写记录。
 *   记录内部同理：前 5 个字的 CRC 放在第 6 个字，最后落盘，写完这条记录才算数。
 */
#define OTA_BK_SECTORS  (OTA_APP_SIZE / W25Q128_SECTOR_SIZE)    /*448KB / 4KB = 112*/
#define OTA_BK_PAGES    (W25Q128_SECTOR_SIZE / W25Q128_PAGE_SIZE)/*4KB / 256B = 16*/

static uint32_t OTA_BkPageBuf[W25Q128_PAGE_SIZE / 4U];  /*256B 页缓冲，不占调用方任务栈*/

/**
  * 功    能：检查 App 区开头是否像一个能跑的程序
  * 参    数：无
  * 返 回 值：1 = 向量表合理；0 = 空白/写了一半
  * 补    充：判据与 BootLoader 跳转前一致：栈顶在 SRAM 且 8 字节对齐，
  *            复位向量落在 App 区内且 bit0=1（Thumb）。备份前先拦一次，
  *            免得把一份坏镜像当宝贝存进槽里。
  */
static uint8_t OTA_AppVectorOk(void)
{
    uint32_t sp = *(volatile const uint32_t *)OTA_APP_BASE;
    uint32_t pc = *(volatile const uint32_t *)(OTA_APP_BASE + 4UL);

    if ((sp < 0x20000000UL) || (sp > 0x20020000UL) || ((sp & 7UL) != 0UL))
    {
        return 0U;
    }
    if ((pc < OTA_APP_BASE) || (pc >= (OTA_APP_BASE + OTA_APP_SIZE)) || ((pc & 1UL) == 0UL))
    {
        return 0U;
    }
    return 1U;
}

uint8_t OTA_ReadSlotABackupVer(uint32_t *ver)
{
    uint32_t hdr[8];
    uint32_t crc;

    if (ver == NULL)
    {
        return 0U;
    }
    *ver = 0U;

    if (W25Q128_ReadData(W25Q128_FW_SLOT_A_ADDR, (uint8_t *)hdr, 32UL) != HAL_OK)
    {
        return 0U;
    }
    if ((hdr[0] != W25Q128_FW_MAGIC) || (hdr[1] != OTA_REC_FMT_BK))
    {
        return 0U;
    }
    /*记录 CRC 只覆盖前 20B：与 OtaStore_WriteRecord / BootLoader ota2_rec_read 的口径一致*/
    crc = OtaCrc32(0xFFFFFFFFUL, (const uint8_t *)hdr, 20UL) ^ 0xFFFFFFFFUL;
    if (crc != hdr[5])
    {
        return 0U;
    }

    *ver = hdr[2];
    return 1U;
}

/**
  * 功    能：往槽 A 首 4KB 写整区备份记录（32B，格式 1）
  * 参    数：ver  版本号(主<<8|次)；crc  镜像 CRC32
  * 返 回 值：1 = 写入成功；0 = 失败
  * 补    充：调用前记录扇区必须已擦成全 0xFF
  */
static uint8_t OTA_BkWriteHeader(uint32_t ver, uint32_t crc)
{
    uint32_t hdr[8];
    uint8_t  i;

    for (i = 0U; i < 8U; i++)
    {
        hdr[i] = 0xFFFFFFFFUL;
    }
    hdr[0] = W25Q128_FW_MAGIC;
    hdr[1] = OTA_REC_FMT_BK;                /*格式 1：整区备份，ota2.c 回滚按这个格式验*/
    hdr[2] = ver;
    hdr[3] = OTA_APP_SIZE;
    hdr[4] = crc;
    hdr[5] = OtaCrc32(0xFFFFFFFFUL, (const uint8_t *)hdr, 20UL) ^ 0xFFFFFFFFUL;

    if (W25Q128_WriteRange(W25Q128_FW_SLOT_A_ADDR, (const uint8_t *)hdr, 32UL) != HAL_OK)
    {
        UART_Printf("[BK] header write FAIL\r\n");
        return 0U;
    }
    UART_Printf("[BK] header written: ver=%lu.%lu size=0x%lX crc=0x%08lX\r\n",
                (unsigned long)((ver >> 8U) & 0xFFUL), (unsigned long)(ver & 0xFFUL),
                (unsigned long)OTA_APP_SIZE, (unsigned long)crc);
    return 1U;
}

uint8_t OTA_BackupAppToSlotA(uint32_t ver, OTA_ProgressCb_t cb)
{
    uint32_t sect;
    uint32_t page;
    uint32_t i;
    uint32_t src;
    uint32_t dst;
    uint32_t imgBase = W25Q128_FW_SLOT_A_ADDR + W25Q128_FW_HDR_SIZE;
    uint32_t crc     = 0xFFFFFFFFUL;
    uint32_t readCrc;
    uint32_t t0;

    if (OTA_AppVectorOk() == 0U)
    {
        UART_Printf("[BK] app vector invalid, abort\r\n");
        return 0U;
    }
    if (W25Q128_ReadJedecId() != W25Q128_JEDEC_ID_EXPECT)
    {
        UART_Printf("[BK] W25Q not ready, abort\r\n");
        return 0U;
    }

    UART_Printf("[BK] start slot A size=0x%lX ver=%lu.%lu\r\n",
                (unsigned long)OTA_APP_SIZE,
                (unsigned long)((ver >> 8U) & 0xFFUL), (unsigned long)(ver & 0xFFUL));
    t0 = HAL_GetTick();

    /*记录扇区先擦空：镜像全部校验通过后才往里写记录*/
    if (W25Q128_EraseRange(W25Q128_FW_SLOT_A_ADDR, W25Q128_FW_HDR_SIZE) != HAL_OK)
    {
        UART_Printf("[BK] erase header sector FAIL\r\n");
        return 0U;
    }

    for (sect = 0UL; sect < OTA_BK_SECTORS; sect++)
    {
        src = OTA_APP_BASE + (sect * W25Q128_SECTOR_SIZE);
        dst = imgBase + (sect * W25Q128_SECTOR_SIZE);

        if (W25Q128_SectorErase4K(dst) != HAL_OK)
        {
            UART_Printf("[BK] erase FAIL at 0x%08lX\r\n", (unsigned long)dst);
            return 0U;
        }

        for (page = 0UL; page < OTA_BK_PAGES; page++)
        {
            uint32_t off = page * W25Q128_PAGE_SIZE;
            const volatile uint32_t *s32 = (const volatile uint32_t *)(src + off);

            /*内部 Flash 是内存映射，按字搬比按字节快；同时顺手累计内部镜像的 CRC*/
            for (i = 0UL; i < (W25Q128_PAGE_SIZE / 4U); i++)
            {
                OTA_BkPageBuf[i] = s32[i];
            }
            crc = OtaCrc32(crc, (const uint8_t *)OTA_BkPageBuf, W25Q128_PAGE_SIZE);

            if (W25Q128_PageProgram(dst + off, (const uint8_t *)OTA_BkPageBuf,
                                    W25Q128_PAGE_SIZE) != HAL_OK)
            {
                UART_Printf("[BK] program FAIL at 0x%08lX\r\n", (unsigned long)(dst + off));
                return 0U;
            }
        }

        /*单扇区"擦 4KB + 写 16 页"约 50~100ms，112 个扇区累计 8 秒上下：
         *调用方是 GuiTask 还是 WifiTask 都可能踩 IWDG 判死线，必须逐扇区补报*/
        Watchdog_Heartbeat(WDT_TASK_WIFI);
        if (cb != NULL)
        {
            cb((uint8_t)(((sect + 1UL) * 100UL) / OTA_BK_SECTORS));
        }
    }
    crc ^= 0xFFFFFFFFUL;

    /*回读整段再算一遍 CRC：擦/写/读三条链路任何一处出错都会在这里暴露*/
    readCrc = W25Q128_Crc32Range(imgBase, OTA_APP_SIZE);
    UART_Printf("[BK] crc internal=0x%08lX readback=0x%08lX\r\n",
                (unsigned long)crc, (unsigned long)readCrc);
    if (crc != readCrc)
    {
        UART_Printf("[BK] FAIL: crc mismatch\r\n");
        return 0U;
    }

    if (OTA_BkWriteHeader(ver, crc) == 0U)
    {
        return 0U;
    }

    UART_Printf("[BK] PASS %lu ms\r\n", (unsigned long)(HAL_GetTick() - t0));
    return 1U;
}

/*下载流程用：备份进度映射到 g_otaStatus 的 0~15 段，GUI 进度条不用改*/
static void OTA_DlBackupProgress(uint8_t percent)
{
    /*只刷进度；状态由 OtaBackup_EnsureForDownload 管理，
     *避免 GUI 串口 IAP 备份失败后全局状态滞留在 BACKUP，无法重试*/
    g_otaStatus.percent = (uint8_t)((percent * 15UL) / 100UL);
}

/**
  * 功    能：下载前把回滚基线（槽 A）立住：版本一致跳过，否则整区备份
  * 参    数：无
  * 返 回 值：1 = 槽 A 可用；0 = 备份失败（本次下载中止）
  * 补    充：GUI 与远程 MQTT 下载两条路都汇到 OTA_DownloadToSlotB，
  *            备份检查放这里就自动覆盖两条路——没有这一步，用户从没做过备份时，
  *            远程下载成功后 BootLoader 会因槽 A 缺备份 apply refused，几百 KB 白下
  *            （ota2.c 三道门的第一道）。版本一致就跳过，避免每次升级都白磨 448KB。
  *            备份进度映射到 g_otaStatus 的 0~15 段，GUI 进度条不用改
  */
uint8_t OtaBackup_EnsureForDownload(void)
{
    uint16_t curVer  = 0U;
    uint32_t slotVer = 0U;

    (void)OTA_GetCurrentVer(&curVer);
    if ((OTA_ReadSlotABackupVer(&slotVer) != 0U) && (slotVer == (uint32_t)curVer))
    {
        UART_Printf("[OTA] slot A backup v%u.%u ok, skip\r\n",
                    (unsigned)(curVer >> 8U), (unsigned)(curVer & 0xFFU));
        return 1U;
    }

    UART_Printf("[OTA] slot A backup start (cur v%u.%u)\r\n",
                (unsigned)(curVer >> 8U), (unsigned)(curVer & 0xFFU));
    /*仅 WiFi 下载流程使用全局备份状态；串口 IAP 自己维护界面，失败后仍可重试*/
    g_otaStatus.state   = OTA_ST_BACKUP;
    g_otaStatus.percent = 0U;
    if (OTA_BackupAppToSlotA((uint32_t)curVer, OTA_DlBackupProgress) == 0U)
    {
        return 0U;
    }
    g_otaStatus.state = OTA_ST_ERASE;   /*备份完成，进入擦槽 B 阶段*/
    return 1U;
}
