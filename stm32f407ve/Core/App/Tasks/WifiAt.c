/* WifiAt.c
 * ESP8266 AT 链路：模块探测、连热点、SNTP 对时，以及 CWLAP 扫描与结果发布。
 * 只被 WifiTask 调用；共享状态通过 WifiTaskInternal.h 的接口读写。
 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_ESP8266.h"
#include "WifiNet.h"
#include "WifiTaskInternal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>


/* SNTP：时区 8=北京，3 个服务器依次尝试 */
#define SNTP_CONFIG_CMD      "AT+CIPSNTPCFG=1,8,\"202.120.2.101\",\"time.nist.gov\",\"ntp.aliyun.com\"\r\n"
#define FIRST_SYNC_TRIES     20U     /* 首次对时最多轮询 20 次 x1s */


/* CWLAP 完整应答解析缓冲(static,不放栈) */
static char s_resp[2048];


/*AT 连接与对时*/
/**
  * 功    能：探测模块在不在线：发 AT 最多试 3 次。返回 1 有响应、0 全都失败
  * 参    数：无
  * 返 回 值：1=模块有响应，0=3 次探测均失败
  */
static uint8_t WiFi_Ready(void)
{
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        if (ESP8266_Cmd("AT\r\n", "OK", 1000U) != 0U)
        {
            return 1U;
        }
        osDelay(500U);                     /* 探测间隔 500ms,3 次合计约 3s */
    }
    return 0U;
}

/**
  * 功    能：连热点：先切成 STA 模式，再用 CWJAP 连（最多试 3 次）。返回 1 连上
  * 参    数：ssid  热点名，32 字节内；pwd  密码，64 字节内，开放网络传空串
  * 返 回 值：1=连上，0=设模式失败或 3 次 CWJAP 均失败
  * 补    充：这一步最长会等到 20s，所以里面必须周期报心跳，否则会被看门狗判死
  */
static uint8_t WiFi_JoinAp(const char *ssid, const char *pwd)
{
    char joinCmd[128];
    uint8_t i;

    (void)ESP8266_Cmd("ATE0\r\n", "OK", 1000U);                 /*关回显*/
    if (ESP8266_Cmd("AT+CWMODE=1\r\n", "OK", 3000U) == 0U)   /* CWMODE=1:Station 模式 */
    {
        return 0U;
    }
    (void)snprintf(joinCmd, sizeof(joinCmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    for (i = 0U; i < 3U; i++)
    {
        UART_Printf("[WIFI] joining %s (try %d)\r\n", ssid, (int)(i + 1U));
        /*等待联网：单次最长 20 秒，期间只能靠 ESP8266_Cmd() 每 500ms 补报心跳*/
        if (ESP8266_Cmd(joinCmd, "OK", 20000U) != 0U)
        {
            return 1U;
        }
        osDelay(1000U);                    /* CWJAP 失败后间隔 1s 再重试 */
    }
    return 0U;
}

/**
  * 功    能：配好 SNTP 并等模块回时间，解析成功就写进 now。返回 1 对时成功
  * 参    数：now  输出时钟，调用方传入已清零的栈变量
  * 返 回 值：1=对时成功，0=SNTP 配置失败/超时/模块掉线
  */
static uint8_t WiFi_WaitSync(ClockData_t *now)
{
    ESP8266_Time_t net;
    uint8_t i;

    if (ESP8266_Cmd(SNTP_CONFIG_CMD, "OK", 3000U) == 0U)
    {
        UART_Printf("[WIFI] SNTP config fail (need AT v1.6+)\r\n");
        return 0U;
    }
    /*每轮 1s 回主循环可正常喂狗；FetchTime 单次约 0.1s，补报靠 ESP8266_Cmd 入口必报*/
    for (i = 0U; i < FIRST_SYNC_TRIES; i++)
    {
        osDelay(1000U);
        {
            uint8_t r = ESP8266_FetchTime(&net);
            if (r == 1U)
            {
                now->year   = net.year;
                now->month  = net.month;
                now->day    = net.day;
                now->hour   = net.hour;
                now->min    = net.min;
                now->sec    = net.sec;
                now->week   = net.week;
                now->synced = 1U;
                UART_Printf("[WIFI] synced: %04d-%02d-%02d %02d:%02d:%02d\r\n",
                            (int)now->year, (int)now->month, (int)now->day,
                            (int)now->hour, (int)now->min, (int)now->sec);
                return 1U;
            }
            if (r == 2U)
            {
                return 0U;     /* 模块掉线 */
            }
        }
    }
    return 0U;                 /* 一直 1970,对时超时 */
}

/**
  * 功    能：完整的上线流程：探测模块 → 连热点 → 网络对时，三步都成功才返回 1
  * 参    数：ssid  热点名；pwd  密码；now  输出时钟，仅对时成功时填充
  * 返 回 值：1=三步全成功，0=任一步失败
  * 补    充：任一步失败就停在那里，不会硬着头皮往下走
  */
uint8_t WiFi_FullConnect(const char *ssid, const char *pwd, ClockData_t *now)
{
    if (WiFi_Ready() == 0U)
    {
        UART_Printf("[WIFI] no AT response (check jumper/USART2 PA2-PA3)\r\n");
        return 0U;
    }
    if (WiFi_JoinAp(ssid, pwd) == 0U)
    {
        UART_Printf("[WIFI] join %s failed\r\n", ssid);
        return 0U;
    }
    UART_Printf("[WIFI] joined, SNTP syncing...\r\n");
    if (WiFi_WaitSync(now) == 0U)
    {
        UART_Printf("[WIFI] time sync failed\r\n");
        return 0U;
    }
    return 1U;
}


/*CWLAP 扫描与解析*/
static WifiAp_t s_top[WIFI_AP_MAX];
static uint8_t  s_topN;

/**
  * 功    能：把扫到的一个热点放进候选表（连同信号强度和加密方式）
  * 参    数：ssid  热点名，最长 32 字节；rssi  信号强度 dBm；sec  CWLAP 的 ecn，0=开放
  * 返 回 值：无
  */
static void TopList_Insert(const char *ssid, int rssi, uint8_t sec)
{
    uint8_t i;
    uint8_t weakIdx = 0U;
    int8_t  weakRssi = 127;

    /* 同名去重:保留信号更强的一条 */
    for (i = 0U; i < s_topN; i++)
    {
        if (strncmp(s_top[i].ssid, ssid, WIFI_SSID_LEN + 1U) == 0)
        {
            if (rssi > s_top[i].rssi)
            {
                s_top[i].rssi = (int8_t)rssi;
                s_top[i].sec  = sec;
            }
            return;
        }
    }

    if (s_topN < WIFI_AP_MAX)
    {
        (void)strncpy(s_top[s_topN].ssid, ssid, WIFI_SSID_LEN);
        s_top[s_topN].ssid[WIFI_SSID_LEN] = '\0';
        s_top[s_topN].rssi = (int8_t)rssi;
        s_top[s_topN].sec  = sec;
        s_topN++;
        return;
    }

    /* 已满:只在比列表中最弱信号更强时替换 */
    for (i = 0U; i < WIFI_AP_MAX; i++)
    {
        if (s_top[i].rssi < weakRssi)
        {
            weakRssi = s_top[i].rssi;
            weakIdx  = i;
        }
    }
    if (rssi > weakRssi)
    {
        (void)strncpy(s_top[weakIdx].ssid, ssid, WIFI_SSID_LEN);
        s_top[weakIdx].ssid[WIFI_SSID_LEN] = '\0';
        s_top[weakIdx].rssi = (int8_t)rssi;
        s_top[weakIdx].sec  = sec;
    }
}

/* 对候选表按信号强度从强到弱排序，界面上信号好的排前面。 */
/* 简单选择排序:信号从强到弱 */
static void TopList_Sort(void)
{
    uint8_t i, j;
    for (i = 0U; (uint8_t)(i + 1U) < s_topN; i++)
    {
        uint8_t best = i;
        for (j = (uint8_t)(i + 1U); j < s_topN; j++)
        {
            if (s_top[j].rssi > s_top[best].rssi)
            {
                best = j;
            }
        }
        if (best != i)
        {
            WifiAp_t tmp = s_top[i];
            s_top[i] = s_top[best];
            s_top[best] = tmp;
        }
    }
}

/**
  * 功    能：打印应答的前若干字节，调试用。内部自动截到 79 字节，避免把一行刷爆
  * 参    数：tag  日志标签；n  应答总字节数，内部自动截到 79 字节
  * 返 回 值：无
  */
static void DumpRespHead(const char *tag, uint16_t n)
{
    char head[80];
    uint16_t len = (n >= (uint16_t)sizeof(head) - 1U) ? (uint16_t)sizeof(head) - 1U : n;
    memcpy(head, s_resp, len);
    head[len] = '\0';
    UART_Printf("[WIFI] %s, %d bytes: %s\r\n", tag, (int)n, head);
}

/**
  * 功    能：执行一次扫描（AT+CWLAP）：解析结果整理成列表，再发「扫描完成」事件给 GUI
  * 参    数：无
  * 返 回 值：无，结果经 g_wifiScan 与 WIFI_EV_SCAN_DONE 发布
  */
void DoScan(void)
{
    char *p;
    uint16_t n;
    uint8_t attempt;

    Wifi_SetStatus(WIFI_ST_SCANNING);
    s_topN = 0U;
    UART_Printf("[WIFI] scan start\r\n");

    /*探测模块*/
    if (ESP8266_Cmd("AT\r\n", "OK", 1000U) == 0U)
    {
        n = ESP8266_CopyResponse(s_resp, (uint16_t)sizeof(s_resp));
        UART_Printf("[WIFI] AT probe FAIL (%d bytes; check jumper PA2/PA3 / module power)\r\n", (int)n);
        if (n > 0U)
        {
            DumpRespHead("AT raw", n);
        }
        Wifi_SetStatus(WIFI_ST_SCAN_FAIL);
        Wifi_PostEvent(WIFI_EV_SCAN_DONE);
        return;
    }

    /*切 STA 模式：CWLAP 需要 WiFi 已使能，模式字只写一次 Flash*/
    (void)ESP8266_Cmd("ATE0\r\n", "OK", 1000U);
    (void)ESP8266_Cmd("AT+CWMODE=1\r\n", "OK", 3000U);      /* 1=Station;CWLAP 需 WiFi 已使能 */

    /*扫描热点：耗时 3~15s，模块回 busy 时重试 3 轮*/
    for (attempt = 0U; attempt < 3U; attempt++)
    {
        if (ESP8266_Cmd("AT+CWLAP\r\n", "OK", 20000U) != 0U)
        {
            break;
        }
        n = ESP8266_CopyResponse(s_resp, (uint16_t)sizeof(s_resp));
        if (strstr(s_resp, "busy") != NULL)
        {
            UART_Printf("[WIFI] CWLAP busy, retry %d\r\n", (int)(attempt + 1U));
            osDelay(2000U);
            continue;
        }
        DumpRespHead("CWLAP no OK", n);
        Wifi_SetStatus(WIFI_ST_SCAN_FAIL);
        Wifi_PostEvent(WIFI_EV_SCAN_DONE);
        return;
    }
    if (attempt >= 3U)
    {
        UART_Printf("[WIFI] CWLAP timeout x3\r\n");
        Wifi_SetStatus(WIFI_ST_SCAN_FAIL);
        Wifi_PostEvent(WIFI_EV_SCAN_DONE);
        return;
    }

    n = ESP8266_CopyResponse(s_resp, (uint16_t)sizeof(s_resp));
    UART_Printf("[WIFI] CWLAP ok, raw %d bytes\r\n", (int)n);

    /* 每行形如: +CWLAP:(3,"SSID",-58,"aa:bb:cc:dd:ee:ff",6,0,0,0,1,1) */
    p = s_resp;
    while ((p = strstr(p, "+CWLAP:(")) != NULL)
    {
        int  ecn = 0, rssi = 0, got;
        char ssid[WIFI_SSID_LEN + 1U];

        ssid[0] = '\0';
        got = sscanf(p, "+CWLAP:(%d,\"%32[^\"]\",%d,", &ecn, ssid, &rssi);
        if ((got == 3) && (ssid[0] != '\0'))
        {
            /*SSID 原样透传给 AT+CWJAP，含中文也能显示与连接*/
            TopList_Insert(ssid, rssi, (uint8_t)ecn);
        }
        p += 8U;     /* 跳过 "+CWLAP:(" 继续找下一条 */
    }

    TopList_Sort();

    /*发布结果：扫描已结束，GUI 收到事件后才读*/
    taskENTER_CRITICAL();
    g_wifiScan.count = s_topN;
    if (s_topN > 0U)
    {
        memcpy(g_wifiScan.aps, s_top, sizeof(WifiAp_t) * s_topN);
    }
    taskEXIT_CRITICAL();

    Wifi_SetStatus((s_topN > 0U) ? WIFI_ST_SCAN_OK : WIFI_ST_SCAN_FAIL);
    UART_Printf("[WIFI] scan done, %d AP(s)\r\n", (int)s_topN);
    Wifi_PostEvent(WIFI_EV_SCAN_DONE);
}
