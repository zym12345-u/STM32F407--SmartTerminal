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
#include "bsp_Mqtt.h"
#include "OtaData.h"
#include "bsp_WifiStore.h"
#include "bsp_WifiStoreLfs.h"
#include "bsp_IWDG.h"
#include "ClockData.h"
#include "SensorData.h"
#include "LEDType.h"
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
#define MQTT_PUB_PERIOD_MS   1000U   /* 遥测上报周期：1Hz，本地 Broker 足够 */

/* LED 队列句柄由 freertos.c 创建，WifiTask 收到 MQTT 命令后投递点灯消息 */
extern osMessageQueueId_t LEDQueueHandle;


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

/*远程 OTA 通知：检查和下载都要先把 ESP8266 让给 HTTP，结果当场发不出去，
 *所以先存一条待发消息，等 MQTT 重连后由主循环补发。
 *两个来源标志保证只有"远程发起的操作"才回传结果，GUI 本地点按钮不往外发。*/
static char     s_otaOut[128];
static uint8_t  s_otaOutPending = 0U;
static uint8_t  s_otaCheckNotify = 0U;
static uint8_t  s_otaDlNotify = 0U;
static uint16_t s_curVer = 0U;        /*本 boot 运行固件版本，开机读一次，避免 1Hz 遥测反复读 Flash*/

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
  * 功    能：请求 WifiTask 保存 MQTT Broker 地址：投一条命令就返回
  * 参    数：host  Broker 地址 1~63 字符；port  端口 1~65535
  * 返 回 值：无
  * 补    充：LittleFS 只有 WifiTask 能写；队列不可用时静默丢弃并打日志
  */
void Wifi_PostSetMqttCfg(const char *host, uint16_t port)
{
    WifiCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_SET_MQTT_CFG;
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
        UART_Printf("[WIFI] PostSetMqttCfg dropped: cmdQ NULL\r\n");
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

/**
  * 功    能：调试入口：把一条原始 AT 指令转发给 ESP8266（仅 WifiTask 能碰 AT 串口）
  * 参    数：atcmd  不含 CRLF 的 AT 指令，如 "AT+GMR"
  * 返 回 值：无
  * 补    充：只用于联调；队列不可用时打日志丢弃
  */
void Wifi_PostAtProbe(const char *atcmd)
{
    WifiCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_AT_PROBE;
    if (atcmd != NULL)
    {
        strncpy(cmd.atcmd, atcmd, sizeof(cmd.atcmd) - 1U);
    }

    if (s_cmdQ != NULL)
    {
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        UART_Printf("[WIFI] PostAtProbe dropped: cmdQ NULL\r\n");
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


/**
  * 功    能：调试：发一条原始 AT 指令并把模块应答逐行打到调试串口
  * 参    数：atcmd  不含 CRLF 的 AT 指令
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；应答快照放 static，避免占 3KB 任务栈
  */
static void DoAtProbe(const char *atcmd)
{
    static char buf[512];
    char line[96];
    uint16_t n;
    uint16_t i;
    uint16_t k = 0U;
    char     tmp[80];

    if (atcmd == NULL)
    {
        return;
    }

    (void)snprintf(tmp, sizeof(tmp), "%s\r\n", atcmd);
    UART_Printf("[AT] >> %s\r\n", atcmd);

    /* 不关心成败：ASSOC/ERROR/超时都要把原始应答打出来 */
    (void)ESP8266_Cmd(tmp, "OK", 2000U);

    n = ESP8266_CopyResponse(buf, (uint16_t)(sizeof(buf) - 1U));
    buf[n] = '\0';

    for (i = 0U; i < n; i++)
    {
        char c = buf[i];
        if ((c == '\r') || (c == '\n'))
        {
            if (k > 0U)
            {
                line[k] = '\0';
                UART_Printf("[AT] << %s\r\n", line);
                k = 0U;
            }
        }
        else if (k < (uint16_t)(sizeof(line) - 1U))
        {
            line[k++] = c;
        }
    }
    if (k > 0U)
    {
        line[k] = '\0';
        UART_Printf("[AT] << %s\r\n", line);
    }
}

/*MQTT 下行命令与遥测*/
/**
  * 功    能：在 JSON 文本里找 "key" 后的值起点（跳过空格和冒号）
  * 参    数：json  以 '\0' 结尾的 JSON；key  字段名（不带引号）
  * 返 回 值：值首字符指针；找不到/格式不对返回 NULL
  * 补    充：只做演示用的极简解析，不处理嵌套和转义
  */
static const char *Wifi_JsonValue(const char *json, const char *key)
{
    char        pat[24];
    const char *p;
    int         n;

    if ((json == NULL) || (key == NULL))
    {
        return NULL;
    }
    n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if ((n <= 0) || (n >= (int)sizeof(pat)))
    {
        return NULL;
    }

    p = strstr(json, pat);
    if (p == NULL)
    {
        return NULL;
    }
    p += strlen(pat);
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    if (*p != ':')
    {
        return NULL;
    }
    p++;
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    return p;
}

/**
  * 功    能：版本号转文本，0x0105 -> "1.5"
  * 参    数：ver  版本 (主<<8|次)；out  输出缓冲；cap  容量
  * 返 回 值：无
  */
static void Wifi_VerText(uint16_t ver, char *out, size_t cap)
{
    if ((out == NULL) || (cap == 0U))
    {
        return;
    }
    (void)snprintf(out, cap, "%u.%u",
                   (unsigned)((ver >> 8U) & 0xFFU), (unsigned)(ver & 0xFFU));
}

/**
  * 功    能：从 JSON 里取版本号并转成 (主<<8|次)
  * 参    数：json  以 '\0' 结尾的 JSON；key  字段名
  * 返 回 值：版本号；字段缺失/格式非法返回 0
  * 补    充：兼容 "ver":"1.6" 与 "ver":1.6 两种写法；0 不是合法固件版本，可当失败哨兵
  */
static uint16_t Wifi_JsonVer(const char *json, const char *key)
{
    const char *p = Wifi_JsonValue(json, key);
    unsigned    major = 0U;
    unsigned    minor = 0U;

    if (p == NULL)
    {
        return 0U;
    }
    if (*p == '"')
    {
        p++;                        /*跳过字符串引号，数字写法这里不动*/
    }
    if ((*p < '0') || (*p > '9'))
    {
        return 0U;
    }
    while ((*p >= '0') && (*p <= '9'))
    {
        major = major * 10U + (unsigned)(*p - '0');
        if (major > 255U)
        {
            return 0U;              /*版本编码只有 1 字节主版本，超了就是不合法*/
        }
        p++;
    }
    if (*p == '.')
    {
        p++;
        while ((*p >= '0') && (*p <= '9'))
        {
            minor = minor * 10U + (unsigned)(*p - '0');
            if (minor > 255U)
            {
                return 0U;
            }
            p++;
        }
    }
    if ((major == 0U) && (minor == 0U))
    {
        return 0U;
    }
    return (uint16_t)((major << 8U) | minor);
}

/**
  * 功    能：float 转四舍五入后的十分位整数（避免 printf 浮点支持问题）
  * 参    数：v  物理量
  * 返 回 值：v×10 取整
  */
static int Wifi_ToTenths(float v)
{
    return (int)(v * 10.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
}

/**
  * 功    能：float 转四舍五入整数
  * 参    数：v  物理量
  * 返 回 值：取整结果
  */
static int Wifi_ToInt(float v)
{
    return (int)(v + ((v >= 0.0f) ? 0.5f : -0.5f));
}

/*远程 OTA：待发消息缓冲与回执*/
/**
  * 功    能：把一条 JSON 存进待发缓冲（覆盖上一条）
  * 参    数：json  以 '\0' 结尾的消息体
  * 返 回 值：无
  * 补    充：OTA 结果只关心最新状态，不做队列；重连后主循环发一次就清标志
  */
static void Wifi_OtaOutSet(const char *json)
{
    if (json == NULL)
    {
        return;
    }
    (void)strncpy(s_otaOut, json, sizeof(s_otaOut) - 1U);
    s_otaOut[sizeof(s_otaOut) - 1U] = '\0';
    s_otaOutPending = 1U;
}

/**
  * 功    能：回一条 OTA 消息：能发就当场发，发不出去存起来等重连补发
  * 参    数：json  以 '\0' 结尾的消息体
  * 返 回 值：无
  */
static void Wifi_OtaReply(const char *json)
{
    if (Mqtt_Publish(MQTT_TOPIC_OTA, json, MQTT_QOS1, 0U) == 0U)
    {
        Wifi_OtaOutSet(json);
    }
}

/**
  * 功    能：远程触发 OTA：排进命令队列，由主循环先断开 MQTT 再执行
  * 参    数：download  1=下载并自动复位；0=只检查版本
  * 返 回 值：无
  * 补    充：只在 WifiTask 内调用；不能在本函数里直接做 HTTP，
  *            否则 MQTT 长连接还占着 CIPSTART，两条 TCP 会互相踩
  */
static void Wifi_OtaRemoteTrigger(uint8_t download)
{
    WifiCmd_t q;
    char      ack[128];
    char      cur[12];
    osStatus_t st;

    Wifi_VerText(s_curVer, cur, sizeof(cur));
    memset(&q, 0, sizeof(q));

    if (download != 0U)
    {
        q.type = WIFI_CMD_OTA;
        st = osMessageQueuePut(s_cmdQ, &q, 0U, 0U);
        if (st == osOK)
        {
            s_otaDlNotify = 1U;
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"download\",\"state\":\"start\",\"cur\":\"%s\"}", cur);
        }
        else
        {
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"download\",\"state\":\"busy\",\"cur\":\"%s\"}", cur);
        }
    }
    else
    {
        q.type   = WIFI_CMD_CHECK_OTA;
        q.manual = 1U;
        st = osMessageQueuePut(s_cmdQ, &q, 0U, 0U);
        if (st == osOK)
        {
            s_otaCheckNotify = 1U;
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"check\",\"state\":\"start\",\"cur\":\"%s\"}", cur);
        }
        else
        {
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"check\",\"state\":\"busy\",\"cur\":\"%s\"}", cur);
        }
    }

    UART_Printf("[MQTT] ota remote trigger: %s (%d)\r\n",
                (download != 0U) ? "download" : "check", (int)st);
    Wifi_OtaReply(ack);
}

/**
  * 功    能：把检查结果组织成 OTA 主题消息存进待发缓冲
  * 参    数：无
  * 返 回 值：无
  * 补    充：检查期间 MQTT 已 Stop，只能等重启连接后补发
  */
static void Wifi_OtaQueueCheckResult(void)
{
    char json[128];
    char cur[12];

    Wifi_VerText(s_curVer, cur, sizeof(cur));

    if (g_otaStatus.checkState == OTA_CHK_DONE)
    {
        if (g_otaStatus.checkVer != 0U)
        {
            (void)snprintf(json, sizeof(json),
                           "{\"ack\":\"ota\",\"event\":\"check\",\"cur\":\"%s\","
                           "\"new\":\"%u.%u\",\"result\":\"available\"}",
                           cur,
                           (unsigned)((g_otaStatus.checkVer >> 8U) & 0xFFU),
                           (unsigned)(g_otaStatus.checkVer & 0xFFU));
        }
        else
        {
            (void)snprintf(json, sizeof(json),
                           "{\"ack\":\"ota\",\"event\":\"check\",\"cur\":\"%s\","
                           "\"result\":\"latest\"}", cur);
        }
    }
    else
    {
        (void)snprintf(json, sizeof(json),
                       "{\"ack\":\"ota\",\"event\":\"check\",\"cur\":\"%s\","
                       "\"result\":\"fail\",\"err\":%u}",
                       cur, (unsigned)g_otaStatus.checkErr);
    }
    Wifi_OtaOutSet(json);
}

/**
  * 功    能：把下载结果组织成 OTA 主题消息存进待发缓冲
  * 参    数：state  结果短语；err  失败原因（成功填 0）
  * 返 回 值：无
  * 补    充：下载成功会 3s 内复位，来不及发；升级后的版本由遥测 fw 字段体现
  */
static void Wifi_OtaQueueDlResult(const char *state, uint8_t err)
{
    char json[128];
    char cur[12];

    Wifi_VerText(s_curVer, cur, sizeof(cur));
    (void)snprintf(json, sizeof(json),
                   "{\"ack\":\"ota\",\"event\":\"download\",\"cur\":\"%s\","
                   "\"state\":\"%s\",\"err\":%u}",
                   cur, state, (unsigned)err);
    Wifi_OtaOutSet(json);
}

/**
  * 功    能：处理 ota 主题消息：版本通知（ver）或升级指令（cmd=check/download）
  * 参    数：msg  下行消息，不可为 NULL
  * 返 回 值：无
  * 补    充：版本通知只更新本地"发现新版本"状态，不联网；
  *            真正检查/下载都排队列，执行前由主循环先让出 MQTT
  */
static void Wifi_HandleOtaTopic(const MqttCommand_t *msg)
{
    const char *cmd;
    char        ack[128];
    char        cur[12];
    uint16_t    target;

    if (msg == NULL)
    {
        return;
    }

    /*本机的回执也发在 ota 主题上，Broker 会原样投回来。
     *带 ack 字段的一律是本机发的，直接丢弃，否则会"自己回自己"无限循环*/
    if (Wifi_JsonValue(msg->payload, "ack") != NULL)
    {
        return;
    }

    Wifi_VerText(s_curVer, cur, sizeof(cur));

    if (Wifi_JsonValue(msg->payload, "ver") != NULL)
    {
        target = Wifi_JsonVer(msg->payload, "ver");
        if ((target != 0U) && (target > s_curVer))
        {
            /*服务端已经告知了更高版本，GUI 的"发现新版本"直接置位，
             *不必为同一件事再发一次 HTTP：省一次 ESP8266 独占和十几秒等待*/
            g_otaStatus.checkErr   = OTA_ERR_NONE;
            g_otaStatus.checkVer   = target;
            g_otaStatus.checkState = OTA_CHK_DONE;
            UART_Printf("[MQTT] ota notify v%u.%u\r\n",
                        (unsigned)((target >> 8U) & 0xFFU), (unsigned)(target & 0xFFU));
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"notify\",\"cur\":\"%s\","
                           "\"ver\":\"%u.%u\",\"result\":\"available\"}",
                           cur, (unsigned)((target >> 8U) & 0xFFU), (unsigned)(target & 0xFFU));
        }
        else
        {
            (void)snprintf(ack, sizeof(ack),
                           "{\"ack\":\"ota\",\"event\":\"notify\",\"cur\":\"%s\","
                           "\"result\":\"latest\"}", cur);
        }
        Wifi_OtaReply(ack);
        return;
    }

    cmd = Wifi_JsonValue(msg->payload, "cmd");
    if ((cmd != NULL) && (strncmp(cmd, "\"check\"", 7U) == 0))
    {
        Wifi_OtaRemoteTrigger(0U);
    }
    else if ((cmd != NULL) && (strncmp(cmd, "\"download\"", 10U) == 0))
    {
        Wifi_OtaRemoteTrigger(1U);
    }
    else
    {
        /*不认识的消息只记日志、不回包：这条主题是收发共用的，
         *任何自动回包都可能被 Broker 投回给自己，宁可不答也别制造回环*/
        UART_Printf("[MQTT] ota ignored: %s\r\n", msg->payload);
    }
}

/**
  * 功    能：把一条点灯消息投给 LEDTask（WifiTask 不直接碰 GPIO）
  * 参    数：color  0=绿 1=蓝；state  0=灭 1=亮
  * 返 回 值：无
  */
static void Wifi_PostLed(LEDColor color, LEDState state)
{
    LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));

    if (msg == NULL)
    {
        UART_Printf("[MQTT] led dropped: no heap\r\n");
        return;
    }
    msg->color = color;
    msg->state = state;
    if (osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
        vPortFree(msg);
    }
}

/**
  * 功    能：把当前传感器与时间打成 JSON 发到遥测主题
  * 参    数：now  当前时钟，不可为 NULL
  * 返 回 值：无
  * 补    充：全部用整数打印，不依赖 printf 的 %f；1Hz 调用，失败下轮自动重试
  */
static void Wifi_PublishTelemetry(const ClockData_t *now)
{
    char json[192];
    char fw[12];
    int  t10;
    int  h10;
    int  lux;
    int  n;

    if (now == NULL)
    {
        return;
    }

    t10 = Wifi_ToTenths(g_sensorData.temperature);
    h10 = Wifi_ToTenths(g_sensorData.humidity);
    lux = Wifi_ToInt(g_sensorData.light);
    Wifi_VerText(s_curVer, fw, sizeof(fw));

    /*带 fw：远程端不用额外查询就能确认升级后的设备是否已跑在新版本上*/
    n = snprintf(json, sizeof(json),
                 "{\"temp\":%d.%d,\"humi\":%d.%d,\"lux\":%d,\"ok\":%u,"
                 "\"ts\":\"%04u-%02u-%02u %02u:%02u:%02u\",\"fw\":\"%s\"}",
                 t10 / 10, ((t10 < 0) ? -t10 : t10) % 10,
                 h10 / 10, ((h10 < 0) ? -h10 : h10) % 10,
                 lux,
                 (unsigned)g_sensorData.temp_valid,
                 (unsigned)now->year, (unsigned)now->month, (unsigned)now->day,
                 (unsigned)now->hour, (unsigned)now->min, (unsigned)now->sec,
                 fw);

    if (n > 0)
    {
        (void)Mqtt_Publish(MQTT_TOPIC_TELEMETRY, json, MQTT_QOS0, 0U);
    }
}

/**
  * 功    能：处理 MQTT 下行消息：cmd 主题走控制命令，ota 主题走升级通知
  * 参    数：无
  * 返 回 值：无
  * 补    充：命令 JSON 形如 {"cmd":"ping"} / {"cmd":"led","color":"blue","on":1} /
  *            {"cmd":"ota","action":"check|download"}
  */
static void Wifi_HandleMqttCommands(void)
{
    MqttCommand_t c;
    char          ack[96];

    while (Mqtt_TakeCommand(&c) != 0U)
    {
        const char *cmd;

        /*来源主题决定语义：ota 主题的 ver 通知不能当成普通控制命令解析*/
        if (strcmp(c.topic, MQTT_TOPIC_OTA) == 0)
        {
            Wifi_HandleOtaTopic(&c);
            continue;
        }

        cmd = Wifi_JsonValue(c.payload, "cmd");
        if ((cmd != NULL) && (strncmp(cmd, "\"ping\"", 6U) == 0))
        {
            (void)Mqtt_Publish(MQTT_TOPIC_TELEMETRY, "{\"ack\":\"pong\"}", MQTT_QOS1, 0U);
        }
        else if ((cmd != NULL) && (strncmp(cmd, "\"led\"", 5U) == 0))
        {
            const char *color = Wifi_JsonValue(c.payload, "color");
            const char *on    = Wifi_JsonValue(c.payload, "on");
            LEDColor    ledColor = ((color != NULL) && (strncmp(color, "\"blue\"", 6U) == 0))
                                   ? LEDColor_Blue : LEDColor_Green;
            LEDState    ledState = ((on != NULL) && (on[0] == '1'))
                                   ? LEDState_On : LEDState_Off;

            Wifi_PostLed(ledColor, ledState);
            (void)snprintf(ack, sizeof(ack), "{\"ack\":\"led\",\"ok\":1,\"on\":%u}",
                           (unsigned)((ledState == LEDState_On) ? 1U : 0U));
            (void)Mqtt_Publish(MQTT_TOPIC_TELEMETRY, ack, MQTT_QOS1, 0U);
        }
        else if ((cmd != NULL) && (strncmp(cmd, "\"ota\"", 5U) == 0))
        {
            const char *action = Wifi_JsonValue(c.payload, "action");

            /*action 缺省按 check：远程只想问"有没有新版本"时不必带参数*/
            Wifi_OtaRemoteTrigger(((action != NULL) &&
                                   (strncmp(action, "\"download\"", 10U) == 0)) ? 1U : 0U);
        }
        else
        {
            (void)Mqtt_Publish(MQTT_TOPIC_TELEMETRY, "{\"ack\":\"unknown\"}", MQTT_QOS0, 0U);
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
    uint32_t    lastClockTick;
    uint32_t    lastPubTick;

    memset(&now, 0, sizeof(now));
    ESP8266_Init();
    Mqtt_Init();       /*只清本地状态，真正建链等 WiFi 连上后 Mqtt_Start */
    UART_Printf("[WIFI] ESP8266 USART2(PA2/PA3) init ok\r\n");

    osDelay(2000U);    /* 等模块上电稳定 + 避开开机 LVGL 首屏渲染 */

    /* A. 开机：有保存凭据则自动连(先迁后读) */
    (void)WifiStoreLfs_Migrate();

    /* 凭据迁移完成后再读 OTA / MQTT 配置：同一 LittleFS，且只有本任务能碰 */
    OTA_Config_Init();
    Mqtt_Config_Init();
    Mqtt_ReloadConfig();

    /*版本只读一次：遥测 1Hz 都带 fw 字段，不能每次都去翻内部 Flash 元数据*/
    (void)OTA_GetCurrentVer(&s_curVer);
    UART_Printf("[WIFI] running fw v%u.%u\r\n",
                (unsigned)((s_curVer >> 8U) & 0xFFU), (unsigned)(s_curVer & 0xFFU));

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

    lastClockTick = osKernelGetTickCount();
    lastPubTick   = lastClockTick;

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
            Mqtt_Start();          /*WiFi 刚连上：允许 MQTT 建链*/
        }
        else if ((s_connected == 0U) && (s_prevConnected != 0U))
        {
            s_bootArmed = 0U;
            Mqtt_Stop(0U);         /*WiFi 已掉：不发 AT，只清 MQTT 本地状态*/
        }
        s_prevConnected = s_connected;

        /*已连接 50ms 醒一次跑 MQTT 收包/心跳；未连接 200ms 醒一次以快速响应配网/重连*/
        st = osMessageQueueGet(s_cmdQ, &cmd, NULL, s_connected ? 50U : 200U);

        if (st == osOK)
        {
            if (cmd.type == WIFI_CMD_SCAN)
            {
                /*扫描会长时间占用 AT 串口，先把 MQTT 长连接让出去*/
                Mqtt_Stop(1U);
                DoScan();
                if (s_connected)
                {
                    Mqtt_Start();
                }
            }
            else if (cmd.type == WIFI_CMD_JOIN)
            {
                Mqtt_Stop(1U);
                DoJoinCmd(&cmd);
                if (s_connected)
                {
                    /*DoJoinCmd 已写 ClockData，走秒基准从当前值继续*/
                    ClockData_Get(&now);
                    lastResync    = osKernelGetTickCount();
                    lastClockTick = lastResync;
                    Mqtt_Start();      /*换 AP 后原 TCP 已断，重新建链*/
                }
            }
            else if (cmd.type == WIFI_CMD_OTA)
            {
                /*没联网就别开始：OTA 不做自动连接，避免把 20s×3 的联网时间藏进下载流程*/
                if (s_connected == 0U)
                {
                    OTA_ReportFail(OTA_ERR_WIFI);
                    /*远程发起的下载也要给回执，否则上位机一直等不到结果*/
                    if (s_otaDlNotify != 0U)
                    {
                        s_otaDlNotify = 0U;
                        Wifi_OtaQueueDlResult("fail", OTA_ERR_WIFI);
                    }
                }
                else
                {
                    /*OTA 要独占 ESP8266 的 TCP 通道，必须先断开 MQTT 再 CIPSTART*/
                    Mqtt_Stop(1U);
                    if (OTA_DownloadToSlotB() != 0U)
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
                            if (s_otaDlNotify != 0U)
                            {
                                s_otaDlNotify = 0U;
                                Wifi_OtaQueueDlResult("done_no_reboot", OTA_ERR_NONE);
                            }
                            Mqtt_Start();
                        }
                    }
                    else
                    {
                        if (s_otaDlNotify != 0U)
                        {
                            s_otaDlNotify = 0U;
                            Wifi_OtaQueueDlResult("fail", g_otaStatus.err);
                        }
                        Mqtt_Start();      /*下载失败留在当前固件，恢复 MQTT*/
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
            else if (cmd.type == WIFI_CMD_SET_MQTT_CFG)
            {
                UART_Printf("[WIFI] set mqtt cfg %s:%u\r\n", cmd.host, (unsigned)cmd.port);
                /*写 /mqtt.conf 只能在本任务；成功后断开旧 Broker 并按新地址重连*/
                if (Mqtt_Config_Set(cmd.host, cmd.port) != 0U)
                {
                    Mqtt_Stop(1U);
                    Mqtt_ReloadConfig();
                    Mqtt_Start();
                }
                else
                {
                    UART_Printf("[WIFI] mqtt cfg save failed\r\n");
                }
            }
            else if (cmd.type == WIFI_CMD_CHECK_OTA)
            {
                /*检查更新走一次性 HTTP，与 MQTT 长连接互斥*/
                Mqtt_Stop(1U);
                DoCheckOta(cmd.manual);
                /*只有远程发起的检查才需要回传结果；GUI 本地点按钮看屏幕就行*/
                if (s_otaCheckNotify != 0U)
                {
                    s_otaCheckNotify = 0U;
                    Wifi_OtaQueueCheckResult();
                }
                Mqtt_Start();
            }
            else if (cmd.type == WIFI_CMD_AT_PROBE)
            {
                /*裸 AT 调试会打乱 MQTT 字节流，先断开再探测*/
                Mqtt_Stop(1U);
                DoAtProbe(cmd.atcmd);
                Mqtt_Start();
            }
            continue;
        }

        if (s_connected)
        {
            uint32_t tickNow;

            /* 本地走秒：50ms 轮询下按 1s 边界推进，避免固定 1s 超时导致 MQTT 收包延迟。
             *用 +=1000 而不是 =tickNow：轮询周期有抖动，直接取当前 tick 会让走秒变慢；
             *长阻塞（如 OTA 检查）返回后这里会补上错过的秒数。*/
            tickNow = osKernelGetTickCount();
            while ((uint32_t)(tickNow - lastClockTick) >= 1000U)
            {
                lastClockTick += 1000U;
                Clock_TickOneSecond(&now);
                ClockData_Set(&now);
            }

            /*MQTT 收包/心跳/重连；连接阶段会阻塞数秒，内部已补喂看门狗*/
            Mqtt_Poll();
            Wifi_HandleMqttCommands();

            /*OTA 结果补发：检查/下载期间 MQTT 被让出，重连后把最后一条结果发出去。
             *只保留最新一条，发成功才清标志，失败下轮继续试*/
            if ((s_otaOutPending != 0U) && (Mqtt_IsConnected() != 0U))
            {
                if (Mqtt_Publish(MQTT_TOPIC_OTA, s_otaOut, MQTT_QOS1, 0U) != 0U)
                {
                    s_otaOutPending = 0U;
                }
            }

            /*遥测 1Hz；未连 Broker 时 Mqtt_Publish 直接返回 0，不阻塞*/
            tickNow = osKernelGetTickCount();
            if ((uint32_t)(tickNow - lastPubTick) >= MQTT_PUB_PERIOD_MS)
            {
                lastPubTick = tickNow;
                Wifi_PublishTelemetry(&now);
            }

            /*30 分钟重同步，校准本地漂移。SNTP 走 AT 命令，会清串口环，
             *必须先把 MQTT 断开，否则收到的半包会被清掉导致字节流错位*/
            if ((uint32_t)(osKernelGetTickCount() - lastResync) >= RESYNC_PERIOD_MS)
            {
                ESP8266_Time_t net;
                lastResync = osKernelGetTickCount();
                Mqtt_Stop(1U);
                if (ESP8266_FetchTime(&net) == 1U)
                {
                    now.year = net.year; now.month = net.month; now.day = net.day;
                    now.hour = net.hour; now.min = net.min;     now.sec = net.sec;
                    now.week = net.week; now.synced = 1U;
                    ClockData_Set(&now);
                }
                lastClockTick = osKernelGetTickCount();
                Mqtt_Start();
            }

            /*上电静默检查：首次连上起算 60s，仍在线且本 boot 未查过才执行一次。
             *只 GET meta（最长十几秒，内部补心跳），失败不重试、不弹窗*/
            if ((s_bootArmed != 0U) && (s_bootChecked == 0U) &&
                ((uint32_t)(osKernelGetTickCount() - s_bootConnTick) >= OTA_BOOT_CHECK_DELAY_MS))
            {
                s_bootArmed   = 0U;
                s_bootChecked = 1U;
                Mqtt_Stop(1U);
                DoCheckOta(0U);
                Mqtt_Start();
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
                    lastClockTick = lastResync;
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
