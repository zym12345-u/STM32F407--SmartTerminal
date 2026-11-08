/* WifiTask.c
 * WiFi 任务：扫描热点、配网、联网、SNTP 校时，结果写 g_wifiScan 与 g_clockData。
 * 周期：未联网 200ms、已联网 1s 轮询 cmdQ；优先级 Normal，栈 3072 字节。
 * 注意：AT+CWJAP 单次最长阻塞 20s，期间回不到主循环，喂狗只能靠 ESP8266_Cmd() 内部补报。
 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_ESP8266.h"
#include "bsp_OTA.h"
#include "bsp_OtaCfg.h"
#include "OtaData.h"
#include "bsp_WifiStore.h"
#include "bsp_WifiStoreLfs.h"
#include "bsp_IWDG.h"
#include "ClockData.h"
#include "WifiNet.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* SNTP：时区 8=北京，3 个服务器依次尝试 */
#define SNTP_CONFIG_CMD      "AT+CIPSNTPCFG=1,8,\"202.120.2.101\",\"time.nist.gov\",\"ntp.aliyun.com\"\r\n"

#define WIFI_TASK_STACK_SIZE 3072U   /* 字节;s_resp/s_snap 均为 static,栈只放调用帧 */
#define FIRST_SYNC_TRIES     20U     /* 首次对时最多轮询 20 次 x1s */
#define RESYNC_PERIOD_MS     (30U * 60U * 1000U)
#define RECONNECT_PERIOD_MS  10000U  /* 掉线后自动重连间隔 */
#define OTA_BOOT_CHECK_DELAY_MS 60000U  /* 首次联网后静默检查更新的延时：每 boot 最多一次 */


/*任务间队列与共享状态*/
static osMessageQueueId_t s_cmdQ;    /* GUI -> 任务 */
static osMessageQueueId_t s_evtQ;    /* 任务 -> GUI */

WifiScanState_t g_wifiScan = {0};

/* 全局时钟数据实体(ClockData.h 中 extern,GuiTask 临界区读取) */
ClockData_t g_clockData;

/* CWLAP 完整应答解析缓冲(static,不放栈) */
static char s_resp[2048];

/* 已保存凭据(连接成功/开机加载后驻留,掉线重连用) */
static WifiCred_t s_cred;
static uint8_t    s_haveCred = 0U;
static uint8_t    s_connected = 0U;

/*上电静默检查：从"首次连接成功"起算 60s；60s 内掉线则重新等下次连上。
 *s_bootChecked 保证每次上电最多查一次，失败也不重试。*/
static uint32_t s_bootConnTick = 0U;
static uint8_t  s_bootArmed = 0U;
static uint8_t  s_bootChecked = 0U;
static uint8_t  s_prevConnected = 0U;

/*配网/联网状态机，g_wifiScan.status 迁移
 * 状态             含义         进入条件                    超时
 * WIFI_ST_IDLE     无凭据       无保存凭据或自动连失败       无
 * WIFI_ST_SCANNING 扫描中       收到 WIFI_CMD_SCAN          CWLAP 20000ms×3
 * WIFI_ST_SCAN_OK  扫描出热点   解析到至少 1 个热点        无
 * WIFI_ST_SCAN_FAIL 扫描失败     AT 无响应/无 OK/busy 3 轮   无
 * WIFI_ST_JOINING  联网中       收到 WIFI_CMD_JOIN          CWJAP 20000ms×3
 * WIFI_ST_JOIN_OK  联网成功     CWJAP+SNTP 全成功           无，s_connected=1
 * WIFI_ST_JOIN_FAIL 联网失败    任一步失败                  无，s_connected=0
 * WIFI_ST_CONNECTED 已联网      上一次连接成功              每 30000ms 重新 SNTP
 * 掉线：s_connected=0 且 s_haveCred=1 时，每 10000ms 用保存凭据重连一次。
 */


/*队列 API*/
/**
  * 功    能：让任务去扫一遍热点：投一条命令进队列就返回，不等结果
  * 参    数：无
  * 返 回 值：无
  */
void Wifi_PostScan(void)
{
    WifiCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_SCAN;
    if (s_cmdQ != NULL)
    {
        /*超时 0：GuiTask 调用，队列满直接丢弃，不能卡界面*/
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        UART_Printf("[WIFI] PostScan dropped: cmdQ NULL (heap alloc failed)\r\n");
    }
}

/**
  * 功    能：让任务去连指定热点：投命令进队列就返回
  * 参    数：ssid  热点名，最长 32 字节，超长截断；pwd  密码，最长 64 字节，开放网络传 ""
  * 返 回 值：无
  * 补    充：ssid 最长 32 字节、pwd 最长 64 字节，超出部分截断；开放网络密码传空串
  */
void Wifi_PostJoin(const char *ssid, const char *pwd)
{
    WifiCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_JOIN;
    if (ssid != NULL)
    {
        strncpy(cmd.ssid, ssid, WIFI_SSID_LEN);
    }
    if (pwd != NULL)
    {
        strncpy(cmd.pwd, pwd, WIFI_PWD_LEN);
    }
    if (s_cmdQ != NULL)
    {
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        UART_Printf("[WIFI] PostJoin dropped: cmdQ NULL (heap alloc failed)\r\n");
    }
}

/**
  * 功    能：请求 WifiTask 下载固件到槽 B：投一条命令就返回，进度由 GUI 轮询 g_otaStatus
  * 参    数：无
  * 返 回 值：无
  * 补    充：必须走 WifiTask —— ESP8266 是单消费者，且 W25Q 与 WifiStore 共用 SPI1，
  *            放在同一个任务里执行天然串行，不需要额外加锁
  */
void Wifi_PostOta(void)
{
    WifiCmd_t cmd;

    /*整条命令【按值】投递，不 malloc 指针：
     *队列里存的是 WifiCmd_t 的结构体副本，GUI 投完就可以直接返回，
     *不存在"谁负责 free"的问题，也不会因为 GUI 任务栈上的变量被回收而踩空。
     *代价是每次投递多拷一份结构体，但 OTA 一天也点不了几次。*/
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_OTA;

    if (s_cmdQ != NULL)
    {
        /*超时传 0：GUI 侧（lv_timer 上下文）绝对不能阻塞等队列。
         *队列深度只有 2，理论上连点两下就满——GUI 的按钮禁用是主要防护，
         *这里投递失败时【不额外处理】，属于已知的静默丢弃（OTA 页会停在 Idle）*/
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        /*任务还没创建（s_cmdQ 未初始化）：直接把原因写进共享状态，
         *GUI 轮询到 FAIL 会显示"队列不可用"，比什么都不发生强*/
        OTA_ReportFail(OTA_ERR_QUEUE);
    }
}

/**
  * 功    能：请求 WifiTask 保存 OTA 服务器地址：投一条命令就返回
  * 参    数：host  服务器地址 1~63 字符；port  端口 1~65535
  * 返 回 值：无
  * 补    充：LittleFS 只有 WifiTask 能写；队列不可用时静默丢弃并打日志
  */
void Wifi_PostSetOtaCfg(const char *host, uint16_t port)
{
    WifiCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_SET_OTA_CFG;
    cmd.port = port;
    if (host != NULL)
    {
        strncpy(cmd.host, host, sizeof(cmd.host) - 1U);
    }

    if (s_cmdQ != NULL)
    {
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        UART_Printf("[WIFI] PostSetOtaCfg dropped: cmdQ NULL\r\n");
    }
}

/**
  * 功    能：请求 WifiTask 检查更新：投一条命令就返回，结果写 g_otaStatus
  * 参    数：manual  1=用户手动点 Check 0=上电静默检查
  * 返 回 值：无
  * 补    充：队列不可用时直接把检查状态置 FAIL，GUI 能从 checkErr 看出原因
  */
void Wifi_PostCheckOta(uint8_t manual)
{
    WifiCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_CHECK_OTA;
    cmd.manual = (manual != 0U) ? 1U : 0U;

    if (s_cmdQ != NULL)
    {
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        g_otaStatus.checkManual = cmd.manual;
        g_otaStatus.checkErr    = OTA_ERR_QUEUE;
        g_otaStatus.checkState  = OTA_CHK_FAIL;
        UART_Printf("[WIFI] PostCheckOta dropped: cmdQ NULL\r\n");
    }
}

/*超时 0 轮询：GUI 必须持续刷界面，不能阻塞等事件*/
/**
  * 功    能：非阻塞取一个 WiFi 事件（扫描完成 / 连接成功 / 连接失败）
  * 参    数：ev  输出缓冲，必须非 NULL
  * 返 回 值：1=取到事件，0=队列空或参数非法
  */
uint8_t Wifi_TryGetEvent(WifiEvent_t *ev)
{
    if ((s_evtQ != NULL) && (ev != NULL) &&
        (osMessageQueueGet(s_evtQ, ev, NULL, 0U) == osOK))
    {
        return 1U;
    }
    return 0U;
}

/**
  * 功    能：往事件队列里投一个事件码，供 GUI 那边轮询取
  * 参    数：ev  事件码，WIFI_EV_SCAN_DONE / JOIN_OK / JOIN_FAIL
  * 返 回 值：无
  */
static void Wifi_PostEvent(WifiEvent_t ev)
{
    (void)osMessageQueuePut(s_evtQ, &ev, 0U, 0U);
}

/**
  * 功    能：更新 WiFi 状态，状态有变化才打日志（GUI 的状态栏也是读这个值显示）
  * 参    数：st  目标状态，WIFI_ST_IDLE ~ WIFI_ST_CONNECTED
  * 返 回 值：无
  */
static void Wifi_SetStatus(WifiStatus_t st)
{
    g_wifiScan.status = (uint8_t)st;
}


/*本地走秒*/
/**
  * 功    能：判断闰年，日期进位时用：能被 4 整除但不能被 100 整除，或能被 400 整除
  * 参    数：year  公元年份，如 2026
  * 返 回 值：1=闰年，0=平年
  */
static uint8_t IsLeapYear(uint16_t year)
{
    return (((year % 4U == 0U) && (year % 100U != 0U)) || (year % 400U == 0U)) ? 1U : 0U;
}

/**
  * 功    能：返回某年某月有多少天。2 月要看是不是闰年；month 越界会读越界，调用方保证 1~12
  * 参    数：year  年份，仅 2 月闰年判断用；month  月份 1~12，越界会读越界
  * 返 回 值：当月天数 28/29/30/31
  */
static uint8_t DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31U, 28U, 31U, 30U, 31U, 30U,
                                     31U, 31U, 30U, 31U, 30U, 31U};
    if ((month == 2U) && (IsLeapYear(year) != 0U))
    {
        return 29U;
    }
    return days[month - 1U];
}

/**
  * 功    能：时钟每秒进一位：秒进分、分进时、时进日、日进月、月进年，逐级处理进位
  * 参    数：t  待进位的时钟实体，调用方传栈上的 now
  * 返 回 值：无
  * 补    充：t 传的是调用方栈上的副本，进完位由调用方发布出去
  */
static void Clock_TickOneSecond(ClockData_t *t)
{
    t->sec++;
    if (t->sec < 60U) return;
    t->sec = 0U;
    t->min++;
    if (t->min < 60U) return;
    t->min = 0U;
    t->hour++;
    if (t->hour < 24U) return;
    t->hour = 0U;
    t->week = (uint8_t)((t->week % 7U) + 1U);
    t->day++;
    if (t->day <= DaysInMonth(t->year, t->month)) return;
    t->day = 1U;
    t->month++;
    if (t->month <= 12U) return;
    t->month = 1U;
    t->year++;
}


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
static uint8_t WiFi_FullConnect(const char *ssid, const char *pwd, ClockData_t *now)
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
static void DoScan(void)
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


/*屏幕提交的连接命令*/
/**
  * 功    能：执行连接命令：跑完整上线流程，结果同时更新状态和事件队列
  * 参    数：cmd  命令体，ssid/pwd 为用户选定热点与密码
  * 返 回 值：无，结果经 g_wifiScan.status 与 evtQ 发布
  */
static void DoJoinCmd(const WifiCmd_t *cmd)
{
    ClockData_t now;

    memset(&now, 0, sizeof(now));
    Wifi_SetStatus(WIFI_ST_JOINING);

    if (WiFi_FullConnect(cmd->ssid, cmd->pwd, &now) == 0U)
    {
        s_connected = 0U;
        Wifi_SetStatus(WIFI_ST_JOIN_FAIL);
        Wifi_PostEvent(WIFI_EV_JOIN_FAIL);
        return;
    }

    /*保存凭据只擦 4KB 块(busy 33ms)，不冻结 CPU，无需放宽 IWDG 窗口*/
    memset(&s_cred, 0, sizeof(s_cred));
    s_cred.magic = WIFI_FLASH_MAGIC;
    (void)strncpy(s_cred.ssid, cmd->ssid, WIFI_SSID_LEN);
    (void)strncpy(s_cred.pwd,  cmd->pwd,  WIFI_PWD_LEN);
    if (WifiStoreLfs_Save(&s_cred) != 0U)
    {
        s_haveCred = 1U;
    }
    else
    {
        UART_Printf("[WIFI] WARN: credential lfs save failed\r\n");
    }

    s_connected = 1U;
    ClockData_Set(&now);
    Wifi_SetStatus(WIFI_ST_JOIN_OK);
    Wifi_PostEvent(WIFI_EV_JOIN_OK);
}


/**
  * 功    能：执行一次检查更新，结果写 g_otaStatus 的 checkState/checkErr/checkVer
  * 参    数：manual  1=手动 0=上电静默
  * 返 回 值：无
  * 补    充：只 GET app.meta；检查期间补 WIFI 心跳（最长十几秒）；
  *            静默检查失败只打日志，不弹窗
  */
static void DoCheckOta(uint8_t manual)
{
    uint16_t ver = 0U;
    uint8_t  err;

    g_otaStatus.checkManual = (manual != 0U) ? 1U : 0U;
    g_otaStatus.checkErr    = OTA_ERR_NONE;
    g_otaStatus.checkVer    = 0U;
    g_otaStatus.checkState  = OTA_CHK_CHECKING;

    if (manual != 0U)
    {
        UART_Printf("[OTA] check start manual\r\n");
    }
    else
    {
        UART_Printf("[OTA] check start boot\r\n");
    }

    if (s_connected == 0U)
    {
        err = OTA_ERR_WIFI;
    }
    else
    {
        err = OTA_CheckUpdate(&ver);
    }

    if (err == OTA_ERR_NONE)
    {
        g_otaStatus.checkVer   = ver;
        g_otaStatus.checkState = OTA_CHK_DONE;
    }
    else
    {
        UART_Printf("[OTA] check fail err=%u\r\n", (unsigned)err);
        if (manual != 0U)
        {
            g_otaStatus.checkErr   = err;
            g_otaStatus.checkState = OTA_CHK_FAIL;
        }
        else
        {
            /*静默检查失败只留日志，不回写失败状态，避免用户进 OTA 页看到过期错误*/
            g_otaStatus.checkErr   = OTA_ERR_NONE;
            g_otaStatus.checkState = OTA_CHK_IDLE;
        }
    }
}


/*任务入口*/
/**
  * 功    能：WiFi 任务主循环：等命令队列 → 处理扫描/连接 → 走时钟 → 报心跳。死循环不返回
  * 参    数：argument  未使用，创建时传 NULL
  * 返 回 值：无，死循环不返回
  * 补    充：取命令带超时，所以没命令时也会按周期报心跳，不会被判卡死
  */
void StartWifiTask(void *argument)
{
    (void)argument;
    ClockData_t now;
    uint32_t    lastResync = 0U;
    uint32_t    lastReconnect = 0U;

    memset(&now, 0, sizeof(now));
    ESP8266_Init();
    UART_Printf("[WIFI] ESP8266 USART2(PA2/PA3) init ok\r\n");

    osDelay(2000U);    /* 等模块上电稳定 + 避开开机 LVGL 首屏渲染 */

    /* A. 开机：有保存凭据则自动连(先迁后读) */
    (void)WifiStoreLfs_Migrate();

    /* 凭据迁移完成后再读 OTA 配置：同一 LittleFS，且只有本任务能碰 */
    OTA_Config_Init();

    if (WifiStoreLfs_Load(&s_cred) != 0U)
    {
        s_haveCred = 1U;
        UART_Printf("[WIFI] saved credential: %s, auto connecting...\r\n", s_cred.ssid);
        Wifi_SetStatus(WIFI_ST_JOINING);
        if (WiFi_FullConnect(s_cred.ssid, s_cred.pwd, &now) != 0U)
        {
            s_connected = 1U;
            lastResync = osKernelGetTickCount();
            ClockData_Set(&now);
            Wifi_SetStatus(WIFI_ST_CONNECTED);
        }
        else
        {
            s_connected = 0U;
            lastReconnect = osKernelGetTickCount();
            Wifi_SetStatus(WIFI_ST_IDLE);
        }
    }
    else
    {
        UART_Printf("[WIFI] no saved credential, use screen to configure\r\n");
        Wifi_SetStatus(WIFI_ST_IDLE);
    }

    for (;;)
    {
        WifiCmd_t   cmd;
        osStatus_t  st;

        /*下面 DoScan()/DoJoinCmd() 一跑几秒到 20s，此处走不到，存活靠 ESP8266_Cmd() 内部补报*/
        Watchdog_Heartbeat(WDT_TASK_WIFI);

        /*连接状态跳变：首次连上/重连成功都从这里开始计 60s；
         *60s 内掉线则 s_bootArmed 清零，等下次连上重新计时*/
        if ((s_connected != 0U) && (s_prevConnected == 0U))
        {
            s_bootConnTick = osKernelGetTickCount();
            s_bootArmed    = 1U;
        }
        else if ((s_connected == 0U) && (s_prevConnected != 0U))
        {
            s_bootArmed = 0U;
        }
        s_prevConnected = s_connected;

        /*已连接 1s 醒一次走秒，未连接 200ms 醒一次以快速响应配网/重连*/
        st = osMessageQueueGet(s_cmdQ, &cmd, NULL, s_connected ? 1000U : 200U);

        if (st == osOK)
        {
            if (cmd.type == WIFI_CMD_SCAN)
            {
                DoScan();
            }
            else if (cmd.type == WIFI_CMD_JOIN)
            {
                DoJoinCmd(&cmd);
                if (s_connected)
                {
                    /*DoJoinCmd 已写 ClockData，走秒基准从当前值继续*/
                    ClockData_Get(&now);
                    lastResync = osKernelGetTickCount();
                }
            }
            else if (cmd.type == WIFI_CMD_OTA)
            {
                /*没联网就别开始：OTA 不做自动连接，避免把 20s×3 的联网时间藏进下载流程*/
                if (s_connected == 0U)
                {
                    OTA_ReportFail(OTA_ERR_WIFI);
                }
                else if (OTA_DownloadToSlotB() != 0U)
                {
                    /*成功且搬运标志已写：3s 后自动复位，交给 BootLoader 搬运。
                     *标志写失败时 rebootPending=0，停在 DONE 提示手动复位，绝不假装升级*/
                    if (g_otaStatus.rebootPending != 0U)
                    {
                        OTA_RebootToApply();
                    }
                    else
                    {
                        UART_Printf("[OTA] reboot skipped: apply flag not set\r\n");
                    }
                }
            }
            else if (cmd.type == WIFI_CMD_SET_OTA_CFG)
            {
                /*写 /ota.conf 只能在 WifiTask 里做；失败时缓存保持原值，运行不受影响*/
                if (OTA_Config_Set(cmd.host, cmd.port) == 0U)
                {
                    UART_Printf("[WIFI] ota cfg save failed\r\n");
                }
            }
            else if (cmd.type == WIFI_CMD_CHECK_OTA)
            {
                DoCheckOta(cmd.manual);
            }
            continue;
        }

        if (s_connected)
        {
            /* 本地走秒 */
            Clock_TickOneSecond(&now);
            ClockData_Set(&now);

            /*30 分钟重同步，校准本地漂移*/
            if ((uint32_t)(osKernelGetTickCount() - lastResync) >= RESYNC_PERIOD_MS)
            {
                ESP8266_Time_t net;
                lastResync = osKernelGetTickCount();
                if (ESP8266_FetchTime(&net) == 1U)
                {
                    now.year = net.year; now.month = net.month; now.day = net.day;
                    now.hour = net.hour; now.min = net.min;     now.sec = net.sec;
                    now.week = net.week; now.synced = 1U;
                    ClockData_Set(&now);
                }
            }

            /*上电静默检查：首次连上起算 60s，仍在线且本 boot 未查过才执行一次。
             *只 GET meta（最长十几秒，内部补心跳），失败不重试、不弹窗*/
            if ((s_bootArmed != 0U) && (s_bootChecked == 0U) &&
                ((uint32_t)(osKernelGetTickCount() - s_bootConnTick) >= OTA_BOOT_CHECK_DELAY_MS))
            {
                s_bootArmed   = 0U;
                s_bootChecked = 1U;
                DoCheckOta(0U);
            }
        }
        else if (s_haveCred)
        {
            /* B. 掉线/自动连失败：每 10s 用保存凭据重试 */
            if ((uint32_t)(osKernelGetTickCount() - lastReconnect) >= RECONNECT_PERIOD_MS)
            {
                lastReconnect = osKernelGetTickCount();
                UART_Printf("[WIFI] reconnecting saved AP...\r\n");
                if (WiFi_FullConnect(s_cred.ssid, s_cred.pwd, &now) != 0U)
                {
                    s_connected = 1U;
                    lastResync = osKernelGetTickCount();
                    ClockData_Set(&now);
                    Wifi_SetStatus(WIFI_ST_CONNECTED);
                }
            }
        }
    }
}


/**
  * 功    能：创建 WiFi 任务，由 freertos.c 在启动调度器前调一次
  * 参    数：无
  * 返 回 值：无
  */
void WifiTask_Create(void)
{
    static const osThreadAttr_t wifiTaskAttr = {
        .name = "WifiTask",
        .stack_size = WIFI_TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
    };

    /*队列必须先于任务创建；句柄来自 FreeRTOS 堆，堆不足返回 NULL，GUI 命令会被静默丢弃*/
    s_cmdQ = osMessageQueueNew(2U, sizeof(WifiCmd_t), NULL);   /* 深 2:命令只是"请求",不做排队积压 */
    s_evtQ = osMessageQueueNew(8U, sizeof(WifiEvent_t), NULL); /* 深 8:事件短时堆积也不丢 */
    UART_Printf("[WIFI] queues: cmdQ=%d evtQ=%d, free heap after create=%u bytes\r\n",
                (int)(s_cmdQ != NULL), (int)(s_evtQ != NULL),
                (unsigned int)xPortGetFreeHeapSize());

    (void)osThreadNew(StartWifiTask, NULL, &wifiTaskAttr);
}
