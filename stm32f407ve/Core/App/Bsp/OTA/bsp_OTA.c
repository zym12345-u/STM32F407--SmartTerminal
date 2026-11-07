/* bsp_OTA.c
 * WiFi OTA 第一阶段：HTTP 下载 app.bin 到 W25Q 槽 B。
 *
 * ── 完整链路 ──
 *   WifiTask 收到 OTA 命令 → 擦槽 B → GET app.meta → GET app.bin
 *   → 边收边写槽 B → 整包 CRC + 回读 CRC → 写槽记录（格式 2）
 * 本阶段【不复位、不切换启动】，下载失败也只是这一次失败，App 照常运行。
 * 真正的“槽 B → 内部 Flash 搬运 + 回滚”在 BootLoader 第二阶段做。
 *
 * ── 为什么整槽先擦一遍 ──
 *   W25Q 扇区擦除一次要几十毫秒，期间 CPU 被 BUSY 轮询占住，串口里的 +IPD 数据
 *   会继续往 2KB 环形缓冲里堆。如果边收边擦，一次擦除就可能冲爆缓冲区丢数据。
 *   所以先花几秒把 112 个扇区全部擦掉，下载阶段只做 256B 页写（约 1~2ms/页），
 *   接收缓冲才不会溢出。代价是擦除期 4~5 秒没有进度细粒度，但进度条按扇区更新。
 *
 * ── 为什么记录最后写 ──
 *   槽记录是 BootLoader 判断“这个槽有可用镜像”的唯一依据。下载/校验任何一步失败
 *   都不写记录，槽 B 就是一块“没有记录的垃圾区”，BootLoader 绝不会拿它去覆盖 App。
 */
#include "bsp_OTA.h"
#include "bsp_bootflag.h"
#include "OtaData.h"
#include "bsp_ESP8266.h"
#include "usart.h"
#include "bsp_IWDG.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*HTTP 头最多 512B；正文若和头同一批到达，先暂存在 s_pre*/
#define OTA_HDR_MAX     512U
#define OTA_PRE_MAX     512U
#define OTA_META_MAX    512U
#define OTA_RX_CHUNK    512U

/*槽 B 的镜像区从槽基址 + 4KB 开始，最大 444KB*/
#define OTA_IMG_BASE    (W25Q128_FW_SLOT_B_ADDR + W25Q128_FW_HDR_SIZE)
#define OTA_IMG_MAX     W25Q128_FW_IMG_MAX

/*这些缓冲必须是 static：WifiTask 栈只有 3KB，放栈上直接溢出*/
static uint8_t  s_hdr[OTA_HDR_MAX];
static uint8_t  s_pre[OTA_PRE_MAX];
static uint16_t s_preLen;
static uint8_t  s_meta[OTA_META_MAX];
static uint8_t  s_chunk[OTA_RX_CHUNK];
static uint8_t  s_page[W25Q128_PAGE_SIZE];
static uint16_t s_pageFill;
static uint32_t s_written;
static uint32_t s_crc;

OtaStatus_t g_otaStatus;

/*内部函数*/
static uint32_t OTA_Crc32(uint32_t crc, const uint8_t *buf, uint32_t n);
static char *OTA_StrCaseStr(const char *hay, const char *needle);
static uint8_t OTA_HttpGetStart(const char *path);
static uint8_t OTA_ReadHeader(uint32_t *contentLen);
static uint8_t OTA_FetchMeta(uint32_t *size, uint32_t *crc, uint16_t *ver);
static uint8_t OTA_FetchImage(uint32_t size, uint32_t expectCrc);
static uint8_t OTA_EraseSlot(void);
static uint8_t OTA_WriteRecord(uint32_t size, uint32_t crc, uint16_t ver);

/**
  * 功    能：CRC32（逐位，多项式 0xEDB88320），与 W25Q128_Crc32Range / iap_upgrade.py 同算法
  * 参    数：crc  初值传 0xFFFFFFFF；buf  数据；n  字节数
  * 返 回 值：中间值（最终结果要再异或 0xFFFFFFFF）
  * 补    充：边收边算，避免为 300KB 镜像再申请一块缓冲
  */
static uint32_t OTA_Crc32(uint32_t crc, const uint8_t *buf, uint32_t n)
{
    uint32_t i;
    uint8_t  bit;

    for (i = 0UL; i < n; i++)
    {
        crc ^= (uint32_t)buf[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
  * 功    能：不分大小写的子串查找（C 库的 strstr 区分大小写）
  * 参    数：hay  被查找串；needle  关键字
  * 返 回 值：首次出现位置，找不到返回 NULL
  * 补    充：HTTP 头字段大小写不统一，用这个找 Content-Length
  */
static char *OTA_StrCaseStr(const char *hay, const char *needle)
{
    uint16_t i;
    uint16_t nl = (uint16_t)strlen(needle);

    if ((hay == NULL) || (needle == NULL) || (nl == 0U))
    {
        return NULL;
    }

    while (*hay != '\0')
    {
        for (i = 0U; i < nl; i++)
        {
            char a = hay[i];
            char b = needle[i];

            if ((a >= 'A') && (a <= 'Z'))
            {
                a = (char)(a + ('a' - 'A'));
            }
            if ((b >= 'A') && (b <= 'Z'))
            {
                b = (char)(b + ('a' - 'A'));
            }
            if (a != b)
            {
                break;
            }
        }
        if (i >= nl)
        {
            return (char *)hay;
        }
        hay++;
    }
    return NULL;
}

/**
  * 功    能：建 TCP 连接并发出 HTTP GET（Connection: close，收完服务器主动断开）
  * 参    数：path  资源路径，如 "/app.meta"
  * 返 回 值：1 = 连接且请求已发出，0 = 失败
  * 补    充：每次请求都重新建连，不复用 keep-alive，逻辑最简单也最不容易串包
  */
static uint8_t OTA_HttpGetStart(const char *path)
{
    char req[192];
    uint16_t len;

    if (ESP8266_TcpConnect(OTA_SRV_HOST, OTA_SRV_PORT) == 0U)
    {
        return 0U;
    }

    (void)snprintf(req, sizeof(req),
                   "GET %s HTTP/1.1\r\nHost: %s:%u\r\nConnection: close\r\n\r\n",
                   path, OTA_SRV_HOST, (unsigned)OTA_SRV_PORT);
    len = (uint16_t)strlen(req);
    return ESP8266_TcpSend((const uint8_t *)req, len);
}

/**
  * 功    能：收完 HTTP 应答头，解析状态码与 Content-Length
  * 参    数：contentLen  输出，头里没写则回 0
  * 返 回 值：1 = 头完整且是 200，0 = 失败
  * 补    充：头后面的正文若同批到达，存进 s_pre / s_preLen 交给调用方先消费
  */
static uint8_t OTA_ReadHeader(uint32_t *contentLen)
{
    uint16_t n      = 0U;
    uint16_t i;
    uint16_t end    = 0U;
    uint32_t start  = HAL_GetTick();
    const char *p;

    *contentLen = 0UL;
    s_preLen = 0U;

    while ((uint32_t)(HAL_GetTick() - start) < 5000U)
    {
        uint16_t r;

        if (n >= OTA_HDR_MAX)
        {
            break;
        }
        r = ESP8266_TcpRead(&s_hdr[n], (uint16_t)(OTA_HDR_MAX - n), 500U);
        if (r == 0U)
        {
            continue;      /* 500ms 没动静就接着等，总上限 5s */
        }
        n = (uint16_t)(n + r);
        for (i = 4U; i < n; i++)
        {
            if ((s_hdr[i - 3U] == '\r') && (s_hdr[i - 2U] == '\n') &&
                (s_hdr[i - 1U] == '\r') && (s_hdr[i] == '\n'))
            {
                end = (uint16_t)(i + 1U);
                break;
            }
        }
        if (end != 0U)
        {
            break;
        }
    }

    if ((end == 0U) || (end >= OTA_HDR_MAX))
    {
        UART_Printf("[OTA] HTTP header incomplete\r\n");
        return 0U;
    }

    /*头尾可能夹着正文：先搬出来，后面按 Content-Length 继续收*/
    s_preLen = (uint16_t)(n - end);
    if (s_preLen > OTA_PRE_MAX)
    {
        return 0U;
    }
    if (s_preLen > 0U)
    {
        memcpy(s_pre, &s_hdr[end], s_preLen);
    }
    s_hdr[end] = '\0';

    if (strstr((char *)s_hdr, " 200 ") == NULL)
    {
        /*只打印状态行前 32 字节，避免登录页/错误页把串口刷爆*/
        UART_Printf("[OTA] bad status: %.32s\r\n", (char *)s_hdr);
        return 0U;
    }

    p = OTA_StrCaseStr((char *)s_hdr, "content-length:");
    if (p != NULL)
    {
        *contentLen = (uint32_t)strtoul(p + 15, NULL, 10);
    }
    return 1U;
}

/**
  * 功    能：取 app.meta 并解析 version / size / crc
  * 参    数：size  输出镜像字节数；crc  输出整包 CRC32；ver  输出 (主<<8|次)
  * 返 回 值：OTA_ERR_NONE 或对应错误码
  * 补    充：meta 只有几十字节，用“收到 Content-Length 为止”的短循环即可
  */
static uint8_t OTA_FetchMeta(uint32_t *size, uint32_t *crc, uint16_t *ver)
{
    uint32_t contentLen = 0UL;
    uint16_t m;
    uint32_t start;
    const char *p;
    unsigned int  major = 0U;
    unsigned int  minor = 0U;
    unsigned long sz = 0UL;
    unsigned long cr = 0UL;

    if (OTA_HttpGetStart(OTA_META_PATH) == 0U)
    {
        ESP8266_TcpClose();
        UART_Printf("[OTA] meta connect fail\r\n");
        return OTA_ERR_CONNECT;
    }
    if (OTA_ReadHeader(&contentLen) == 0U)
    {
        ESP8266_TcpClose();
        return OTA_ERR_HTTP;
    }
    if ((contentLen == 0UL) || (contentLen >= OTA_META_MAX))
    {
        ESP8266_TcpClose();
        return OTA_ERR_META;
    }

    m = s_preLen;
    if (m > (uint16_t)(OTA_META_MAX - 1U))
    {
        ESP8266_TcpClose();
        return OTA_ERR_META;
    }
    if (m > 0U)
    {
        memcpy(s_meta, s_pre, m);
    }

    start = HAL_GetTick();
    while ((uint32_t)m < contentLen)
    {
        uint16_t want = (uint16_t)(contentLen - (uint32_t)m);
        uint16_t r;

        if (want > (uint16_t)(OTA_META_MAX - 1U - m))
        {
            want = (uint16_t)(OTA_META_MAX - 1U - m);
        }
        r = ESP8266_TcpRead(&s_meta[m], want, 500U);
        if (r > 0U)
        {
            m = (uint16_t)(m + r);
            start = HAL_GetTick();
        }
        else if ((uint32_t)(HAL_GetTick() - start) >= 3000U)
        {
            break;
        }
    }
    ESP8266_TcpClose();

    if ((uint32_t)m < contentLen)
    {
        UART_Printf("[OTA] meta timeout %u/%lu\r\n", (unsigned)m, (unsigned long)contentLen);
        return OTA_ERR_META;
    }
    s_meta[m] = '\0';

    p = strstr((char *)s_meta, "version=");
    if ((p == NULL) || (sscanf(p + 8, "%u.%u", &major, &minor) != 2))
    {
        UART_Printf("[OTA] meta missing version\r\n");
        return OTA_ERR_META;
    }
    p = strstr((char *)s_meta, "size=");
    if ((p == NULL) || (sscanf(p + 5, "%lu", &sz) != 1))
    {
        UART_Printf("[OTA] meta missing size\r\n");
        return OTA_ERR_META;
    }
    p = strstr((char *)s_meta, "crc=");
    if ((p == NULL) || (sscanf(p + 4, "%lu", &cr) != 1))
    {
        UART_Printf("[OTA] meta missing crc\r\n");
        return OTA_ERR_META;
    }

    if ((major > 255U) || (minor > 255U))
    {
        return OTA_ERR_META;
    }
    /*size 必须是 4 的倍数：BootLoader 搬运时按 32 位字写内部 Flash*/
    if ((sz < 64UL) || (sz > (unsigned long)OTA_IMG_MAX) || ((sz & 3UL) != 0UL))
    {
        UART_Printf("[OTA] meta size invalid: %lu (max %lu)\r\n",
                    sz, (unsigned long)OTA_IMG_MAX);
        return OTA_ERR_SIZE;
    }

    *ver  = (uint16_t)((((uint16_t)major & 0xFFU) << 8U) | ((uint16_t)minor & 0xFFU));
    *size = (uint32_t)sz;
    *crc  = (uint32_t)cr;
    return OTA_ERR_NONE;
}

/**
  * 功    能：把一段收到的正文算 CRC 并按 256B 攒页写进槽 B
  * 参    数：data  数据；n  字节数
  * 返 回 值：1 = 成功，0 = 页编程失败
  * 补    充：攒满一页才写，减少 SPI 事务；最后不足一页由调用方收尾
  */
static uint8_t OTA_Feed(const uint8_t *data, uint16_t n)
{
    s_crc = OTA_Crc32(s_crc, data, n);

    while (n > 0U)
    {
        uint16_t room = (uint16_t)(W25Q128_PAGE_SIZE - s_pageFill);
        uint16_t take = (n < room) ? n : room;

        memcpy(&s_page[s_pageFill], data, take);
        s_pageFill = (uint16_t)(s_pageFill + take);
        data += take;
        n = (uint16_t)(n - take);

        if (s_pageFill >= W25Q128_PAGE_SIZE)
        {
            if (W25Q128_PageProgram(OTA_IMG_BASE + s_written, s_page, W25Q128_PAGE_SIZE) != HAL_OK)
            {
                UART_Printf("[OTA] page program fail @0x%06lX\r\n",
                            (unsigned long)(OTA_IMG_BASE + s_written));
                return 0U;
            }
            s_written += W25Q128_PAGE_SIZE;
            s_pageFill = 0U;
            Watchdog_Heartbeat(WDT_TASK_WIFI);
        }
    }
    return 1U;
}

/**
  * 功    能：下载 app.bin 到槽 B，边收边写，最后整包 CRC + 回读 CRC 双重校验
  * 参    数：size  期望字节数；expectCrc  期望 CRC32
  * 返 回 值：OTA_ERR_NONE 或错误码
  * 补    充：Content-Length 必须与 meta 完全一致；5s 没有新数据判超时
  */
static uint8_t OTA_FetchImage(uint32_t size, uint32_t expectCrc)
{
    uint32_t contentLen = 0UL;
    uint32_t got = 0UL;
    uint32_t lastData;
    uint32_t crc;
    uint32_t rbCrc;

    if (OTA_HttpGetStart(OTA_BIN_PATH) == 0U)
    {
        ESP8266_TcpClose();
        UART_Printf("[OTA] bin connect fail\r\n");
        return OTA_ERR_CONNECT;
    }
    if (OTA_ReadHeader(&contentLen) == 0U)
    {
        ESP8266_TcpClose();
        return OTA_ERR_HTTP;
    }
    if ((contentLen != size) || (s_preLen > size))
    {
        UART_Printf("[OTA] size mismatch: http=%lu meta=%lu pre=%u\r\n",
                    (unsigned long)contentLen, (unsigned long)size, (unsigned)s_preLen);
        ESP8266_TcpClose();
        return OTA_ERR_SIZE;
    }

    s_crc      = 0xFFFFFFFFUL;
    s_pageFill = 0U;
    s_written  = 0UL;

    if (s_preLen > 0U)
    {
        if (OTA_Feed(s_pre, s_preLen) == 0U)
        {
            ESP8266_TcpClose();
            return OTA_ERR_FLASH;
        }
        got = (uint32_t)s_preLen;
    }
    lastData = HAL_GetTick();

    while (got < size)
    {
        uint32_t left = size - got;
        uint16_t want = (left > (uint32_t)OTA_RX_CHUNK) ? OTA_RX_CHUNK : (uint16_t)left;
        uint16_t r    = ESP8266_TcpRead(s_chunk, want, 1000U);

        if (r == 0U)
        {
            if ((uint32_t)(HAL_GetTick() - lastData) >= 5000U)
            {
                UART_Printf("[OTA] stall at %lu/%lu\r\n",
                            (unsigned long)got, (unsigned long)size);
                ESP8266_TcpClose();
                return OTA_ERR_TIMEOUT;
            }
            continue;
        }

        if (OTA_Feed(s_chunk, r) == 0U)
        {
            ESP8266_TcpClose();
            return OTA_ERR_FLASH;
        }
        got = (uint32_t)(got + r);
        lastData = HAL_GetTick();

        g_otaStatus.got     = got;
        g_otaStatus.percent = (uint8_t)(25UL + ((got * 70UL) / size));   /* 下载段占 25~95 */
        Watchdog_Heartbeat(WDT_TASK_WIFI);
    }

    /*最后不足一页：起始地址是 256 对齐的，页内部分写合法*/
    if (s_pageFill > 0U)
    {
        if (W25Q128_PageProgram(OTA_IMG_BASE + s_written, s_page, s_pageFill) != HAL_OK)
        {
            ESP8266_TcpClose();
            return OTA_ERR_FLASH;
        }
        s_written += (uint32_t)s_pageFill;
        s_pageFill = 0U;
    }
    ESP8266_TcpClose();

    if (s_written != size)
    {
        return OTA_ERR_SIZE;
    }

    crc = s_crc ^ 0xFFFFFFFFUL;
    UART_Printf("[OTA] crc calc=0x%08lX expect=0x%08lX\r\n",
                (unsigned long)crc, (unsigned long)expectCrc);
    if (crc != expectCrc)
    {
        return OTA_ERR_CRC;
    }

    /*回读再算一遍：确认真写进了 Flash，而不是“以为写了”*/
    rbCrc = W25Q128_Crc32Range(OTA_IMG_BASE, size);
    if (rbCrc != crc)
    {
        UART_Printf("[OTA] readback crc=0x%08lX mismatch\r\n", (unsigned long)rbCrc);
        return OTA_ERR_CRC;
    }
    return OTA_ERR_NONE;
}

/**
  * 功    能：擦除槽 B 的全部 112 个扇区（含记录扇区）
  * 参    数：无
  * 返 回 值：1 = 全部擦完，0 = 某个扇区失败
  * 补    充：逐扇区调用以便按扇区报进度、补 WIFI 心跳（底层只报 STORAGE 心跳）
  */
static uint8_t OTA_EraseSlot(void)
{
    uint32_t i;
    uint32_t sectors = W25Q128_FW_SLOT_SIZE / W25Q128_SECTOR_SIZE;

    for (i = 0UL; i < sectors; i++)
    {
        if (W25Q128_SectorErase4K(W25Q128_FW_SLOT_B_ADDR + (i * W25Q128_SECTOR_SIZE)) != HAL_OK)
        {
            UART_Printf("[OTA] erase fail sector %lu\r\n", (unsigned long)i);
            return 0U;
        }
        Watchdog_Heartbeat(WDT_TASK_WIFI);
        g_otaStatus.percent = (uint8_t)(20UL * (i + 1UL) / sectors);   /* 擦除段占 0~20 */
    }
    return 1U;
}

/**
  * 功    能：在槽 B 首扇区写 32B 记录（格式 2 = OTA 镜像），并回读校验
  * 参    数：size  镜像字节数；crc  整包 CRC；ver  目标版本
  * 返 回 值：1 = 写入且回读一致，0 = 失败
  * 补    充：记录布局与槽 A 备份一致：magic / 格式 / 版本 / 大小 / CRC / 记录自身 CRC
  *            记录最后写，失败随时中止都不会留下“看起来有效”的槽
  */
static uint8_t OTA_WriteRecord(uint32_t size, uint32_t crc, uint16_t ver)
{
    uint32_t hdr[8];
    uint32_t rd[8];
    uint8_t  i;
    uint32_t recCrc;

    for (i = 0U; i < 8U; i++)
    {
        hdr[i] = 0xFFFFFFFFUL;
    }
    hdr[0] = W25Q128_FW_MAGIC;
    hdr[1] = OTA_REC_FMT;
    hdr[2] = (uint32_t)ver;
    hdr[3] = size;
    hdr[4] = crc;
    recCrc = OTA_Crc32(0xFFFFFFFFUL, (const uint8_t *)hdr, 20UL) ^ 0xFFFFFFFFUL;
    hdr[5] = recCrc;

    if (W25Q128_WriteRange(W25Q128_FW_SLOT_B_ADDR, (const uint8_t *)hdr, 32UL) != HAL_OK)
    {
        return 0U;
    }
    if (W25Q128_ReadData(W25Q128_FW_SLOT_B_ADDR, (uint8_t *)rd, 32UL) != HAL_OK)
    {
        return 0U;
    }
    if ((rd[0] != W25Q128_FW_MAGIC) || (rd[1] != OTA_REC_FMT) ||
        (rd[2] != (uint32_t)ver) || (rd[3] != size) || (rd[4] != crc))
    {
        return 0U;
    }
    if ((OTA_Crc32(0xFFFFFFFFUL, (const uint8_t *)rd, 20UL) ^ 0xFFFFFFFFUL) != rd[5])
    {
        return 0U;
    }
    return 1U;
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
  * 功    能：OTA 主流程：擦槽 B → 取 meta → 下载镜像 → 双重校验 → 写记录
  * 参    数：无
  * 返 回 值：1 = 成功；0 = 失败
  * 补    充：仅 WifiTask 调用；成功也只是“槽 B 就绪”，不会自动复位
  */
uint8_t OTA_DownloadToSlotB(void)
{
    uint32_t size = 0UL;
    uint32_t crc  = 0UL;
    uint16_t ver  = 0U;
    uint8_t  err  = OTA_ERR_NONE;
    uint32_t t0;

    g_otaStatus.state   = OTA_ST_ERASE;
    g_otaStatus.err     = OTA_ERR_NONE;
    g_otaStatus.percent = 0U;
    g_otaStatus.newVer  = 0U;
    g_otaStatus.got     = 0UL;
    g_otaStatus.total   = 0UL;
    t0 = HAL_GetTick();

    UART_Printf("\r\n[OTA] download start: %s:%u %s\r\n",
                OTA_SRV_HOST, (unsigned)OTA_SRV_PORT, OTA_BIN_PATH);

    if (W25Q128_ReadJedecId() != W25Q128_JEDEC_ID_EXPECT)
    {
        err = OTA_ERR_FLASH;
    }
    else if (OTA_EraseSlot() == 0U)
    {
        err = OTA_ERR_ERASE;
    }
    else
    {
        g_otaStatus.state   = OTA_ST_META;
        g_otaStatus.percent = 20U;
        err = OTA_FetchMeta(&size, &crc, &ver);

        if (err == OTA_ERR_NONE)
        {
            g_otaStatus.newVer  = ver;
            g_otaStatus.total   = size;
            g_otaStatus.state   = OTA_ST_DOWNLOAD;
            g_otaStatus.percent = 25U;
            UART_Printf("[OTA] meta: size=%lu crc=0x%08lX ver=%u.%u\r\n",
                        (unsigned long)size, (unsigned long)crc,
                        (unsigned)(ver >> 8U), (unsigned)(ver & 0xFFU));

            err = OTA_FetchImage(size, crc);

            if (err == OTA_ERR_NONE)
            {
                g_otaStatus.state   = OTA_ST_VERIFY;
                g_otaStatus.percent = 96U;
                if (OTA_WriteRecord(size, crc, ver) == 0U)
                {
                    err = OTA_ERR_RECORD;
                }
            }
        }
    }

    if (err == OTA_ERR_NONE)
    {
        g_otaStatus.state   = OTA_ST_DONE;
        g_otaStatus.percent = 100U;
        g_otaStatus.got     = size;
        if (BootFlag_RequestApply() == 0U)
        {
            UART_Printf("[OTA] boot apply flag set\r\n");
        }
        else
        {
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
