/* bsp_OTA.c
 * WiFi OTA 主流程：检查更新、下载到槽 B、写槽记录、置搬运标志、延时复位。
 *
 * ── 完整链路（App 侧）──
 *   WifiTask 收到 OTA 命令 → 擦槽 B → GET app.meta → GET app.bin
 *   → 边收边写槽 B → 整包 CRC + 回读 CRC → 写槽记录（格式 2）
 *   → BootFlag_RequestApply()：24C02 页 0xE8 写“下次上电把槽 B 搬进 App 区”
 * 本文件【不切换启动】：下载成功且搬运标志写妥后延时 3s 复位（OTA_RebootToApply），
 * 复位后的搬运与回滚由 BootLoader 做。
 * 下载失败只是这一次失败：槽 B 没记录、App 照常跑、24C02 也不留搬运标志。
 *
 * ── 拆分后的文件分工 ──
 *   bsp_OtaHttp.c  ：TCP/HTTP、应答头、meta 与镜像下载
 *   bsp_OtaStore.c ：CRC、槽 B 攒页写入、擦除、槽记录
 *   bsp_OtaBackup.c：槽 A 整区备份与记录
 *   本文件         ：对外 API 与主流程编排、当前版本读取
 *
 * ── 与 BootLoader/core/ota2.c 的分工 ──
 *   本文件：把镜像“安全地放进外部 Flash”——链路不可控（会断网/断电），
 *           所以允许失败，代价只是槽 B 里一份垃圾数据。
 *   ota2.c：擦内部 App 区 + 搬运 + 回滚 + 启动确认——不可逆，必须一次成。
 *   两段之间只有两个契约：槽首 32B 记录（magic 'FWSL'）与 24C02 的 8B 状态页。
 *
 * ── 为什么整槽先擦一遍 ──
 *   W25Q 扇区擦除一次要几十毫秒，期间 CPU 被 BUSY 轮询占住，串口里的 +IPD 数据
 *   会继续往 2KB 环形缓冲里堆。如果边收边擦，一次擦除就可能冲爆缓冲区丢数据。
 *   所以先花几秒把 112 个扇区全部擦掉，下载阶段只做 256B 页写（约 2ms/页），
 *   接收缓冲才不会溢出。代价是擦除期 4~5 秒没有进度细粒度，但进度条按扇区更新。
 *
 * ── 为什么记录最后写 ──
 *   槽记录是 BootLoader 判断“这个槽有可用镜像”的唯一依据。下载/校验任何一步失败
 *   都不写记录，槽 B 就是一块“没有记录的垃圾区”，BootLoader 绝不会拿它去覆盖 App。
 */
#include "bsp_OTA.h"
#include "bsp_OtaInternal.h"
#include "bsp_OtaCfg.h"
#include "bsp_bootflag.h"
#include "OtaData.h"
#include "bsp_ESP8266.h"
#include "CanData.h"
#include "main.h"          /* HAL_NVIC_SystemReset */
#include "cmsis_os2.h"     /* osDelay：复位前延时让出 CPU */
#include "usart.h"
#include "bsp_IWDG.h"

#include <stdio.h>
#include <string.h>


OtaStatus_t g_otaStatus;


/**
  * 功    能：取当前固件版本：元数据可信就用它，否则退回 CanData.h 的编译期常量
  * 参    数：ver  输出 (主<<8|次)，不能为空
  * 返 回 值：1 = 读自 sector 3 元数据；0 = 用编译期常量
  * 补    充：判据与 GUI 原有实现/BootLoader meta_read 一致：magic + 记录格式 +
  *            大小合理 + 记录自身 CRC + 镜像末尾 'MEND' 标记。
  *            Keil 直接烧 App 时不会更新 sector 3，旧元数据会留在那儿，
  *            标记不在就说明这份元数据不是当前镜像的，必须退回编译期版本。
  */
uint8_t OTA_GetCurrentVer(uint16_t *ver)
{
    uint32_t i;
    uint32_t addr;
    uint32_t size = 0UL;
    uint32_t v    = 0UL;
    uint8_t  found = 0U;

    if (ver == NULL)
    {
        return 0U;
    }
    *ver = (uint16_t)((((uint16_t)CAN_FW_VER_MAJOR & 0xFFU) << 8U) |
                      ((uint16_t)CAN_FW_VER_MINOR & 0xFFU));

    for (i = 0UL; i < OTA_META_SLOTS; i++)
    {
        addr = OTA_META_ADDR + (i * OTA_META_SLOT);
        if (*(volatile uint32_t *)addr != OTA_META_MAGIC)
        {
            continue;
        }
        if (*(volatile uint32_t *)(addr + 4UL) != OTA_META_FMT_REC)
        {
            continue;
        }
        size = *(volatile uint32_t *)(addr + 12UL);
        if ((size < 8UL) || (size > OTA_APP_SIZE) || ((size & 3UL) != 0UL))
        {
            continue;
        }
        if ((OtaCrc32(0xFFFFFFFFUL, (const uint8_t *)addr, 20UL) ^ 0xFFFFFFFFUL) !=
            *(volatile uint32_t *)(addr + 20UL))
        {
            continue;   /*掉电写了一半的记录，不算数*/
        }
        v     = *(volatile uint32_t *)(addr + 8UL);
        found = 1U;     /*继续往后扫，最后一条才是最新的*/
    }

    if (found != 0U)
    {
        if ((size < OTA_APP_SIZE) &&
            (*(volatile uint32_t *)(OTA_APP_BASE + size) != OTA_META_END))
        {
            return 0U;  /*标记不在：元数据不是当前这份镜像的*/
        }
        *ver = (uint16_t)((((v >> 8U) & 0xFFUL) << 8U) | (v & 0xFFUL));
        return 1U;
    }

    /*旧 v2 五字布局兼容：0 偏移处*/
    if (*(volatile uint32_t *)OTA_META_ADDR != OTA_META_MAGIC)
    {
        return 0U;
    }
    if (*(volatile uint32_t *)(OTA_META_ADDR + 4UL) != OTA_META_FMT)
    {
        return 0U;
    }
    v    = *(volatile uint32_t *)(OTA_META_ADDR + 8UL);
    size = *(volatile uint32_t *)(OTA_META_ADDR + 12UL);
    if ((size < 8UL) || (size > OTA_APP_SIZE) || ((size & 3UL) != 0UL))
    {
        return 0U;
    }
    if ((size < OTA_APP_SIZE) &&
        (*(volatile uint32_t *)(OTA_APP_BASE + size) != OTA_META_END))
    {
        return 0U;
    }

    *ver = (uint16_t)((((v >> 8U) & 0xFFUL) << 8U) | (v & 0xFFUL));
    return 1U;
}

/**
  * 功    能：只 GET /app.meta 检查是否有更高版本：不擦槽、不下载、不写记录
  * 参    数：newVer  输出发现的更高版本；无更新或失败时为 0
  * 返 回 值：OTA_ERR_NONE = 检查完成；其它 = 失败原因
  * 补    充：仅 WifiTask 调用；meta 请求最多 3 次（1s/3s 退避）；
  *            版本比较按 (主<<8|次)，相等或更低都算“已是最新”，不提示降级
  */
uint8_t OTA_CheckUpdate(uint16_t *newVer)
{
    uint32_t size = 0UL;
    uint32_t crc  = 0UL;
    uint16_t ver  = 0U;
    uint16_t cur  = 0U;
    uint8_t  err;

    if (newVer != NULL)
    {
        *newVer = 0U;
    }

    err = OtaHttp_FetchMetaRetry(&size, &crc, &ver);
    if (err != OTA_ERR_NONE)
    {
        return err;
    }
    if (ver == 0U)
    {
        return OTA_ERR_META;    /* meta 缺 version，不能当作“最新” */
    }

    (void)OTA_GetCurrentVer(&cur);

    if (ver > cur)
    {
        if (newVer != NULL)
        {
            *newVer = ver;
        }
        UART_Printf("[OTA] check cur=%u.%u new=%u.%u -> update available\r\n",
                    (unsigned)(cur >> 8U), (unsigned)(cur & 0xFFU),
                    (unsigned)(ver >> 8U), (unsigned)(ver & 0xFFU));
    }
    else
    {
        UART_Printf("[OTA] check cur=%u.%u new=%u.%u -> up to date\r\n",
                    (unsigned)(cur >> 8U), (unsigned)(cur & 0xFFU),
                    (unsigned)(ver >> 8U), (unsigned)(ver & 0xFFU));
    }
    (void)size;
    (void)crc;
    return OTA_ERR_NONE;
}

/**
  * 功    能：下载成功并写妥搬运标志后，延时 3s 自动复位交给 BootLoader
  * 参    数：无
  * 返 回 值：无，NVIC 复位不返回
  * 补    充：仅 WifiTask 调用；3s 内每 500ms 补一次 WiFi 心跳；
  *            复位前必须已经确认 BootFlag_RequestApply() 成功，否则不能调用本函数
  */
void OTA_RebootToApply(void)
{
    uint8_t i;

    UART_Printf("[OTA] reboot to apply in 3s\r\n");
    for (i = 0U; i < 6U; i++)
    {
        (void)osDelay(500U);
        Watchdog_Heartbeat(WDT_TASK_WIFI);
    }
    HAL_NVIC_SystemReset();
}

/**
  * 功    能：OTA 失败登记（不阻塞、不关连接，供 WifiTask 在“还没开始”时调用）
  * 参    数：err  OtaErr_t
  * 返 回 值：无
  * 补    充：GUI 轮询到 FAIL 后按 err 显示具体原因
  */
void OTA_ReportFail(uint8_t err)
{
    g_otaStatus.state   = OTA_ST_FAIL;
    g_otaStatus.err     = err;
    g_otaStatus.percent = 0U;
    UART_Printf("[OTA] not started, err=%u\r\n", (unsigned)err);
}

/**
  * 功    能：OTA 主流程：立回滚基线（槽 A）→ 擦槽 B → 取 meta → 下载镜像 → 双重校验 → 写记录
  * 参    数：无
  * 返 回 值：1 = 成功；0 = 失败
  * 补    充：仅 WifiTask 调用；成功是“槽 B 就绪 + 搬运标志已写”，
  *            此时 rebootPending=1，由 WifiTask 调 OTA_RebootToApply() 完成 3s 复位
  */
uint8_t OTA_DownloadToSlotB(void)
{
    OtaServerCfg_t cfg;
    uint32_t size = 0UL;
    uint32_t crc  = 0UL;
    uint16_t ver  = 0U;
    uint8_t  err  = OTA_ERR_NONE;
    uint32_t t0;

    g_otaStatus.state         = OTA_ST_ERASE;
    g_otaStatus.err           = OTA_ERR_NONE;
    g_otaStatus.percent       = 0U;
    g_otaStatus.newVer        = 0U;
    g_otaStatus.got           = 0UL;
    g_otaStatus.total         = 0UL;
    g_otaStatus.rebootPending = 0U;
    t0 = HAL_GetTick();

    OTA_Config_Get(&cfg);
    UART_Printf("\r\n[OTA] download start: %s:%u %s\r\n",
                cfg.host, (unsigned)cfg.port, OTA_BIN_PATH);

    if (W25Q128_ReadJedecId() != W25Q128_JEDEC_ID_EXPECT)
    {
        err = OTA_ERR_FLASH;
    }
    else if (OtaBackup_EnsureForDownload() == 0U)    /*先立回滚基线，再动槽 B*/
    {
        err = OTA_ERR_BACKUP;   /*备份失败与擦槽失败分开报，UI 文案不误导*/
    }
    else if (OtaStore_EraseSlot() == 0U)
    {
        err = OTA_ERR_ERASE;
    }
    else
    {
        g_otaStatus.state   = OTA_ST_META;
        g_otaStatus.percent = 20U;
        err = OtaHttp_FetchMetaRetry(&size, &crc, &ver);

        if (err == OTA_ERR_NONE)
        {
            g_otaStatus.newVer  = ver;
            g_otaStatus.total   = size;
            g_otaStatus.state   = OTA_ST_DOWNLOAD;
            g_otaStatus.percent = 25U;
            UART_Printf("[OTA] meta: size=%lu crc=0x%08lX ver=%u.%u\r\n",
                        (unsigned long)size, (unsigned long)crc,
                        (unsigned)(ver >> 8U), (unsigned)(ver & 0xFFU));

            err = OtaHttp_FetchImage(size, crc);

            if (err == OTA_ERR_NONE)
            {
                g_otaStatus.state   = OTA_ST_VERIFY;
                g_otaStatus.percent = 96U;
                if (OtaStore_WriteRecord(size, crc, ver) == 0U)
                {
                    err = OTA_ERR_RECORD;
                }
            }
        }
    }

    if (err == OTA_ERR_NONE)
    {
        /*到这里：镜像写进槽 B 了、两重 CRC 都过了、槽记录也回读验证了。
         *DONE 的真实含义是「槽 B 可搬」，不是「已经升级」：
         *本函数不复位，也不碰内部 App 区，实际切换要等用户复位后由 BootLoader 做。
         *GUI 的状态文案必须按这个口径写，否则用户会以为"点完就升级好了"。*/
        g_otaStatus.state    = OTA_ST_DONE;
        g_otaStatus.percent  = 100U;
        g_otaStatus.got      = size;
        g_otaStatus.checkVer = 0U;   /*升级包已在槽 B，主屏不再提示"有新版本"*/

        /*向 BootLoader 递"下次上电请搬槽 B"的请求（24C02 页 0xE8，8 字节）。
         *必须先确认写成功再把 rebootPending 置 1：
         *写失败时停在 DONE、不自动复位，否则会复位回旧固件却假装升级完成。*/
        if (BootFlag_RequestApply() == 0U)
        {
            g_otaStatus.rebootPending = 1U;
            UART_Printf("[OTA] boot apply flag set\r\n");
        }
        else
        {
            g_otaStatus.rebootPending = 0U;
            UART_Printf("[OTA] boot apply flag write FAIL\r\n");
        }
        UART_Printf("[OTA] PASS, slot B ready (%lu ms)\r\n",
                    (unsigned long)(HAL_GetTick() - t0));
        return 1U;
    }

    ESP8266_TcpClose();
    g_otaStatus.state = OTA_ST_FAIL;
    g_otaStatus.err   = err;
    UART_Printf("[OTA] FAIL err=%u at %u%% (%lu ms)\r\n",
                (unsigned)err, (unsigned)g_otaStatus.percent,
                (unsigned long)(HAL_GetTick() - t0));
    return 0U;
}
