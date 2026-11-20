/* bsp_OtaHttp.c
 * OTA 的 HTTP 客户端：建连、读应答头、取 app.meta、下载 app.bin。
 * 只被 bsp_OTA.c 的主流程调用；写 Flash 通过 bsp_OtaStore.c 的接口完成。
 */
#include "bsp_OtaInternal.h"
#include "bsp_OtaCfg.h"
#include "OtaData.h"
#include "bsp_ESP8266.h"
#include "bsp_IWDG.h"
#include "usart.h"
#include "cmsis_os2.h"     /* osDelay：重试退避 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/*HTTP 头最多 512B；正文若和头同一批到达，先暂存在 s_pre*/
#define OTA_HDR_MAX     512U
#define OTA_PRE_MAX     512U
#define OTA_META_MAX    512U
#define OTA_RX_CHUNK    512U

/*这些缓冲必须是 static：WifiTask 栈只有 3KB，放栈上直接溢出*/
static uint8_t  s_hdr[OTA_HDR_MAX];
static uint8_t  s_pre[OTA_PRE_MAX];
static uint16_t s_preLen;
static uint8_t  s_meta[OTA_META_MAX];
static uint8_t  s_chunk[OTA_RX_CHUNK];


static char *OTA_StrCaseStr(const char *hay, const char *needle);
static uint8_t OTA_HttpGetStart(const char *path);
static uint8_t OTA_ReadHeader(uint32_t *contentLen);
static uint8_t OTA_FetchMeta(uint32_t *size, uint32_t *crc, uint16_t *ver);

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
    OtaServerCfg_t cfg;
    char req[192];
    uint16_t len;

    /*每次请求都重新读缓存：用户在 OTA 页前改了地址，下一次请求就生效*/
    OTA_Config_Get(&cfg);

    if (ESP8266_TcpConnect(cfg.host, cfg.port) == 0U)
    {
        return 0U;
    }

    (void)snprintf(req, sizeof(req),
                   "GET %s HTTP/1.1\r\nHost: %s:%u\r\nConnection: close\r\n\r\n",
                   path, cfg.host, (unsigned)cfg.port);
    len = (uint16_t)strlen(req);
    return ESP8266_TcpSend((const uint8_t *)req, len);
}

/**
  * 功    能：收完 HTTP 应答头，解析状态码与 Content-Length
  * 参    数：contentLen  输出，头里没写则回 0
  * 返 回 值：1 = 头完整且是 200，0 = 失败
  * 补    充：头后面的正文若同批到达，存进 s_pre / s_preLen 交给调用方先消费。
  *
  * ── 为什么"头尾夹着正文"是常态，不是意外 ──
  *   +IPD 的分包边界由 ESP8266 和网络栈决定，和 HTTP 的消息边界毫无关系。
  *   一个 +IPD 包里完全可能是"头 + 正文前 300 字节"，所以收到 \r\n\r\n 之后
  *   必须先检查后面还有没有剩料，不能默认"头收完了，正文会重新来一包"。
  *
  * ── 为什么找 \r\n\r\n 而不是 \n\n ──
  *   HTTP/1.1 规范要求行结束是 CRLF；Python 的 http.server 严格照做。
  *   只找 \n\n 在遇到正文里恰好出现 0x0A0A 时会误判（二进制正文什么都可能有）。
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

    while ((uint32_t)(HAL_GetTick() - start) < 5000U)   /*总上限 5s，防止服务端半死不活*/
    {
        uint16_t r;

        /*头最多等 5s，超过 IWDG 3s 判死线，循环里必须补心跳*/
        Watchdog_Heartbeat(WDT_TASK_WIFI);

        if (n >= OTA_HDR_MAX)
        {
            break;      /*头超过 512B：不可能是正常应答，按失败处理*/
        }
        r = ESP8266_TcpRead(&s_hdr[n], (uint16_t)(OTA_HDR_MAX - n), 500U);
        if (r == 0U)
        {
            continue;      /* 500ms 没动静就接着等，总上限 5s */
        }
        n = (uint16_t)(n + r);
        /*每来一批就从头重扫：\r\n\r\n 可能正好被切在两次读取的边界上。
         *从 i=4 开始是因为至少要 4 个字节才能凑出一个 \r\n\r\n*/
        for (i = 4U; i < n; i++)
        {
            if ((s_hdr[i - 3U] == '\r') && (s_hdr[i - 2U] == '\n') &&
                (s_hdr[i - 1U] == '\r') && (s_hdr[i] == '\n'))
            {
                end = (uint16_t)(i + 1U);   /*end 指向正文第一个字节*/
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
    s_hdr[end] = '\0';      /*只为了后面能当 C 字符串用；end < 512 已在上面的检查里保证*/

    if (strstr((char *)s_hdr, " 200 ") == NULL)
    {
        /*只打印状态行前 32 字节，避免登录页/错误页把串口刷爆*/
        UART_Printf("[OTA] bad status: %.32s\r\n", (char *)s_hdr);
        return 0U;
    }

    /*头字段大小写不统一（Python 发 Content-Length，别的服务器可能发 content-length），
     *所以用不分大小写的查找；找不到就留 contentLen=0，交给调用方按"长度未知"处理*/
    p = OTA_StrCaseStr((char *)s_hdr, "content-length:");
    if (p != NULL)
    {
        *contentLen = (uint32_t)strtoul(p + 15, NULL, 10);   /*跳过 "content-length:" 共 15 字符*/
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

        Watchdog_Heartbeat(WDT_TASK_WIFI);

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
  * 功    能：带心跳的分段延时（500ms 一步）
  * 参    数：ms  延时时长
  * 返 回 值：无
  * 补    充：IWDG 判死线 3s，整段 3s 的退避不能一口气等，必须中途补报
  */
static void OTA_Delay(uint32_t ms)
{
    while (ms > 0U)
    {
        uint32_t step = (ms > 500U) ? 500U : ms;

        (void)osDelay(step);
        Watchdog_Heartbeat(WDT_TASK_WIFI);
        ms -= step;
    }
}

/**
  * 功    能：取 app.meta，失败最多重试到 3 次（退避 1s / 3s）
  * 参    数：size/crc/ver  同 OTA_FetchMeta
  * 返 回 值：OTA_ERR_NONE 或最后一次的错误码
  * 补    充：只重试廉价的 meta 请求；镜像下载不自动重试，避免故障时间翻倍
  */
uint8_t OtaHttp_FetchMetaRetry(uint32_t *size, uint32_t *crc, uint16_t *ver)
{
    uint8_t attempt;
    uint8_t err = OTA_ERR_META;

    for (attempt = 1U; attempt <= 3U; attempt++)
    {
        err = OTA_FetchMeta(size, crc, ver);
        if (err == OTA_ERR_NONE)
        {
            return OTA_ERR_NONE;
        }
        if (attempt < 3U)
        {
            uint32_t waitMs = (attempt == 1U) ? 1000U : 3000U;

            UART_Printf("[OTA] meta retry %u/3 after %lus\r\n",
                        (unsigned)(attempt + 1U), (unsigned long)(waitMs / 1000U));
            OTA_Delay(waitMs);
        }
    }
    UART_Printf("[OTA] meta failed after 3 attempts, err=%u\r\n", (unsigned)err);
    return err;
}

/**
  * 功    能：下载 app.bin 到槽 B，边收边写，最后整包 CRC + 回读 CRC 双重校验
  * 参    数：size  期望字节数；expectCrc  期望 CRC32（都来自 app.meta）
  * 返 回 值：OTA_ERR_NONE 或错误码
  * 补    充：Content-Length 必须与 meta 完全一致；5s 没有新数据判超时。
  *
  * ── 为什么要拿 HTTP 头的 Content-Length 去卡 meta.size ──
  *   meta 和 bin 是两次独立的 GET，中间服务端的文件完全可能被换掉
  *   （重新编译、重新生成 meta）。长度对不上就说明"这两份不是一套"，
  *   此时继续下载会得到一个 CRC 必然对不上的镜像，白白花 30 秒。
  *   提前在头解析完就判死，失败成本从"30 秒 + 一次擦写"降到"1 秒"。
  */
uint8_t OtaHttp_FetchImage(uint32_t size, uint32_t expectCrc)
{
    uint32_t contentLen = 0UL;
    uint32_t got = 0UL;
    uint32_t lastData;
    uint32_t written = 0UL;
    uint32_t crc = 0UL;
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
        /*两种异常合并成一个错误码：头里长度与 meta 不符，或者头后面夹的正文
         *反而比整个镜像还长（说明服务器返回的根本不是这个文件）*/
        UART_Printf("[OTA] size mismatch: http=%lu meta=%lu pre=%u\r\n",
                    (unsigned long)contentLen, (unsigned long)size, (unsigned)s_preLen);
        ESP8266_TcpClose();
        return OTA_ERR_SIZE;
    }

    OtaStore_Begin();

    if (s_preLen > 0U)           /*头后面夹着的正文先吃掉，否则会少掉开头几十到几百字节*/
    {
        if (OtaStore_Feed(s_pre, s_preLen) == 0U)
        {
            ESP8266_TcpClose();
            return OTA_ERR_FLASH;
        }
        got = (uint32_t)s_preLen;
    }
    lastData = HAL_GetTick();

    while (got < size)
    {
        uint32_t left = size - got;      /*只收"还差多少"，绝不多收：多收的部分属于下一份数据*/
        uint16_t want = (left > (uint32_t)OTA_RX_CHUNK) ? OTA_RX_CHUNK : (uint16_t)left;
        uint16_t r    = ESP8266_TcpRead(s_chunk, want, 1000U);

        if (r == 0U)
        {
            /*1 秒没数据不算断，连续 5 秒没有新字节才判死。
             *这里限的是"无进展"而不是"总耗时"——弱网下总耗时波动很大，
             *但只要还在进字节就说明链路活着，继续等即可*/
            if ((uint32_t)(HAL_GetTick() - lastData) >= 5000U)
            {
                UART_Printf("[OTA] stall at %lu/%lu\r\n",
                            (unsigned long)got, (unsigned long)size);
                ESP8266_TcpClose();
                return OTA_ERR_TIMEOUT;
            }
            continue;
        }

        if (OtaStore_Feed(s_chunk, r) == 0U)
        {
            ESP8266_TcpClose();
            return OTA_ERR_FLASH;
        }
        got = (uint32_t)(got + r);
        lastData = HAL_GetTick();

        g_otaStatus.got     = got;
        /*百分比映射：25 + 已收/总量 × 70，把 25~95 这段留给下载，
         *和备份+擦除(0~20)、取 meta(20~25)、校验+写记录(96~100)拼成完整进度条*/
        g_otaStatus.percent = (uint8_t)(25UL + ((got * 70UL) / size));
        Watchdog_Heartbeat(WDT_TASK_WIFI);
    }

    ESP8266_TcpClose();          /*这里就先断开，后面的校验不需要网络*/

    if (OtaStore_Finish(&written, &crc) == 0U)
    {
        return OTA_ERR_FLASH;
    }
    if (written != size)         /*防御：收到的东西和声明的长度不一致*/
    {
        return OTA_ERR_SIZE;
    }

    /*第一重校验：网络收到的字节是否就是 meta 里那份*/
    UART_Printf("[OTA] crc calc=0x%08lX expect=0x%08lX\r\n",
                (unsigned long)crc, (unsigned long)expectCrc);
    if (crc != expectCrc)
    {
        return OTA_ERR_CRC;
    }

    /*第二重校验：把这些字节从 Flash 读回来再算一遍。
     *第一重只证明"网上收到的没错"，证明不了"真的写进 Flash 了"——
     *写保护、供电跌落、页编程半途失败都可能让写操作静默失效。
     *两重都过才敢去写槽记录（记录 = 告诉 BootLoader 这份镜像可信）*/
    rbCrc = W25Q128_Crc32Range(OTA_IMG_BASE, size);
    if (rbCrc != crc)
    {
        UART_Printf("[OTA] readback crc=0x%08lX mismatch\r\n", (unsigned long)rbCrc);
        return OTA_ERR_CRC;
    }
    return OTA_ERR_NONE;
}
