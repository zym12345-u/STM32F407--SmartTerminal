/**
  ******************************************************************************
  * @file    WifiTask.c
  * @brief   WiFi 网络任务:屏幕配网 + SNTP 网络对时
  *
  *          两种取凭据途径(凭据保存在 W25Q128 的 LittleFS /wifi.conf):
  *            A. 开机自动连:从 LittleFS 读凭据,连上后 SNTP 对时;
  *            B. 屏幕配网:GuiTask 扫到热点、用户输密码,经命令队列提交,
  *               连接成功后写入 LittleFS,下次开机走 A 自动连。
  *
  *          V1.2 起凭据不再放内部 Flash sector11(降级打标灰色地带,
  *          擦 128KB 会冻结 CPU 1~2s),首次开机会自动把旧凭据迁过来。
  *
  *          队列(在 WifiTask_Create 中创建,早于任务首次运行):
  *            GuiTask --WifiCmd_t-->  本任务  扫描/连接命令(元素约 100B,深 4)
  *            本任务 --WifiEvent-->  GuiTask 扫描完成/连接结果(深 8)
  *
  *          运行中本地每秒走秒(公历进位+星期),每 30 分钟重新 SNTP 校准;
  *          掉线自动重连已保存凭据。
  *
  *          ⚠️ 本任务是全工程最容易被看门狗咬死的地方:
  *            AT+CWJAP 单次调用最多连续阻塞 20s,期间本任务回不到主循环,
  *            调不到 Watchdog_Heartbeat(),只能靠 ESP8266_Cmd() 内部补报。
  *            详见 WiFi_JoinAp() / DoJoinCmd() 里的注释。
  *            (V1.2 起凭据保存走 LittleFS 不再冻结 CPU,不再是咬狗因素)
  ******************************************************************************
  */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_ESP8266.h"
#include "bsp_WifiStore.h"
#include "bsp_WifiStoreLfs.h"
#include "bsp_IWDG.h"
#include "ClockData.h"
#include "WifiNet.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* NTP:时区 8=北京;3 个服务器依次尝试(与网盘 17-3 例程一致) */
#define SNTP_CONFIG_CMD      "AT+CIPSNTPCFG=1,8,\"202.120.2.101\",\"time.nist.gov\",\"ntp.aliyun.com\"\r\n"

#define WIFI_TASK_STACK_SIZE 3072U   /* 字节;s_resp/s_snap 均为 static,栈只放调用帧 */
#define FIRST_SYNC_TRIES     20U     /* 首次对时最多轮询 20 次 x1s */
#define RESYNC_PERIOD_MS     (30U * 60U * 1000U)
#define RECONNECT_PERIOD_MS  10000U  /* 掉线后自动重连间隔 */


/* ====================== 任务间队列与共享状态 ====================== */
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


/* ============================ 队列 API ============================ */
void Wifi_PostScan(void)
{
    WifiCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = WIFI_CMD_SCAN;
    if (s_cmdQ != NULL)
    {
        /* 超时 0 = 不阻塞:这是给 GuiTask 调用的界面线程,
         * 队列满就直接丢弃,绝不能让界面卡在这里等 WifiTask */
        (void)osMessageQueuePut(s_cmdQ, &cmd, 0U, 0U);
    }
    else
    {
        UART_Printf("[WIFI] PostScan dropped: cmdQ NULL (heap alloc failed)\r\n");
    }
}

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

/* GuiTask 每轮界面刷新时轮询一次:超时 0 不阻塞,取不到就继续刷界面。
 * 用轮询而不是阻塞等待,是因为 GUI 必须持续响应触摸和刷新 */
uint8_t Wifi_TryGetEvent(WifiEvent_t *ev)
{
    if ((s_evtQ != NULL) && (ev != NULL) &&
        (osMessageQueueGet(s_evtQ, ev, NULL, 0U) == osOK))
    {
        return 1U;
    }
    return 0U;
}

static void Wifi_PostEvent(WifiEvent_t ev)
{
    (void)osMessageQueuePut(s_evtQ, &ev, 0U, 0U);
}

static void Wifi_SetStatus(WifiStatus_t st)
{
    g_wifiScan.status = (uint8_t)st;
}


/* ====================== 本地走秒(公历/闰年/星期) ====================== */
static uint8_t IsLeapYear(uint16_t year)
{
    return (((year % 4U == 0U) && (year % 100U != 0U)) || (year % 400U == 0U)) ? 1U : 0U;
}

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


/* ====================== AT 连接/对时原语 ====================== */
static uint8_t WiFi_Ready(void)
{
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        if (ESP8266_Cmd("AT\r\n", "OK", 1000U) != 0U)
        {
            return 1U;
        }
        osDelay(500U);
    }
    return 0U;
}

static uint8_t WiFi_JoinAp(const char *ssid, const char *pwd)
{
    char joinCmd[128];
    uint8_t i;

    (void)ESP8266_Cmd("ATE0\r\n", "OK", 1000U);
    if (ESP8266_Cmd("AT+CWMODE=1\r\n", "OK", 3000U) == 0U)
    {
        return 0U;
    }
    (void)snprintf(joinCmd, sizeof(joinCmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    for (i = 0U; i < 3U; i++)
    {
        UART_Printf("[WIFI] joining %s (try %d)\r\n", ssid, (int)(i + 1U));
        /* ⚠️ 这一行最多阻塞 20s:ESP8266_Cmd 要一直等到模块回 OK 或超时。
         * 这 20s 是"一次调用连续阻塞",本任务完全回不到主循环,调不到
         * Watchdog_Heartbeat();而判死线只有 3s。
         * 所以 ESP8266_Cmd() 内部必须每 500ms 补报心跳,否则必被 IWDG 复位 */
        if (ESP8266_Cmd(joinCmd, "OK", 20000U) != 0U)
        {
            return 1U;
        }
        osDelay(1000U);
    }
    return 0U;
}

/* 配置 SNTP 并轮询等待有效网络时间,成功填充 now */
static uint8_t WiFi_WaitSync(ClockData_t *now)
{
    ESP8266_Time_t net;
    uint8_t i;

    if (ESP8266_Cmd(SNTP_CONFIG_CMD, "OK", 3000U) == 0U)
    {
        UART_Printf("[WIFI] SNTP config fail (need AT v1.6+)\r\n");
        return 0U;
    }
    /* 注意这里的 20s 和 WiFi_JoinAp() 的 20s 不是一回事:
     * 这里是 20 次 x osDelay(1s),每次都能回到主循环报心跳,天然安全;
     * 但 ESP8266_FetchTime() 单次只有 ~0.1s,比喂狗节流门槛 500ms 还短,
     * 若只做"循环内节流补报"会永远够不着门槛 → 整轮零心跳。
     * 所以 ESP8266_Cmd() 除了节流,还必须有"入口必报" */
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

/* 完整链路:AT 就绪 -> 连接 -> SNTP 对时 */
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


/* ====================== CWLAP 扫描与解析(Top N) ====================== */
static WifiAp_t s_top[WIFI_AP_MAX];
static uint8_t  s_topN;

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

/* 打印应答快照开头若干字符(原始内容,帮助判断波特率/固件返回) */
static void DumpRespHead(const char *tag, uint16_t n)
{
    char head[80];
    uint16_t len = (n >= (uint16_t)sizeof(head) - 1U) ? (uint16_t)sizeof(head) - 1U : n;
    memcpy(head, s_resp, len);
    head[len] = '\0';
    UART_Printf("[WIFI] %s, %d bytes: %s\r\n", tag, (int)n, head);
}

static void DoScan(void)
{
    char *p;
    uint16_t n;
    uint8_t attempt;

    Wifi_SetStatus(WIFI_ST_SCANNING);
    s_topN = 0U;
    UART_Printf("[WIFI] scan start\r\n");

    /* 模块没就绪时扫描也会失败;CWLAP 在已关联状态下同样可用 */
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

    /* 保证处于 STA 模式(CWLAP 需要 WiFi 使能;模式字写入 Flash,仅一次) */
    (void)ESP8266_Cmd("ATE0\r\n", "OK", 1000U);
    (void)ESP8266_Cmd("AT+CWMODE=1\r\n", "OK", 3000U);

    /* 扫描耗时随热点数量 3~15s 不等;模块可能先回 busy s.../busy p...,重试 3 轮 */
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
            /* 保留含中文等非 ASCII 字节的 SSID:已启用 simsun CJK 字体,
             * 可正常显示中文名热点;SSID 字节原样透传给 AT+CWJAP,ESP8266 可连 */
            TopList_Insert(ssid, rssi, (uint8_t)ecn);
        }
        p += 8U;     /* 跳过 "+CWLAP:(" 继续找下一条 */
    }

    TopList_Sort();

    /* 发布结果(扫描已结束,GUI 收到事件后才读,无需临界区) */
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


/* ====================== 屏幕提交的连接命令 ====================== */
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

    /* V1.2: 凭据已迁到 LittleFS,保存只擦 4KB 块(busy 33ms),不再冻结 CPU,
     * 也无需放宽 IWDG 窗口。 */
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


/* ============================ 任务入口 ============================ */
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

    /* A. 开机:有保存凭据则自动连(先迁后读) */
    (void)WifiStoreLfs_Migrate();
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

        /* ⚠️ 别以为有这一句就安全:下面 DoScan()/DoJoinCmd() 一跑就是几秒到 20s,
         * 这期间根本走不到这里。本任务的存活实际取决于 ESP8266_Cmd()
         * 内部有没有做心跳注入,而不是这一句 */
        Watchdog_Heartbeat(WDT_TASK_WIFI);

        /* 已连接时每秒醒一次走秒(顺带满足 3s 判死线);
         * 未连接时 200ms 醒一次,好快速响应配网/重连命令 */
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
                    /* DoJoinCmd 已发布 now,本地走秒基准从 ClockData 取当前值 */
                    ClockData_Get(&now);
                    lastResync = osKernelGetTickCount();
                }
            }
            continue;
        }

        if (s_connected)
        {
            /* 本地走秒 */
            Clock_TickOneSecond(&now);
            ClockData_Set(&now);

            /* 30 分钟重同步,校准本地漂移 */
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
        }
        else if (s_haveCred)
        {
            /* B. 掉线/开机自动连失败:每 10s 用保存的凭据重试,不打扰用户 */
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


void WifiTask_Create(void)
{
    static const osThreadAttr_t wifiTaskAttr = {
        .name = "WifiTask",
        .stack_size = WIFI_TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
    };

    /* 队列先于任务创建,GuiTask 任何时刻 Post 都安全。
     * 队列从 FreeRTOS 堆分配(每个命令元素约 99B),堆不足会返回 NULL,
     * 这里必须检查并打印,否则 GUI 的扫描命令会被静默丢弃 */
    s_cmdQ = osMessageQueueNew(2U, sizeof(WifiCmd_t), NULL);
    s_evtQ = osMessageQueueNew(8U, sizeof(WifiEvent_t), NULL);
    UART_Printf("[WIFI] queues: cmdQ=%d evtQ=%d, free heap after create=%u bytes\r\n",
                (int)(s_cmdQ != NULL), (int)(s_evtQ != NULL),
                (unsigned int)xPortGetFreeHeapSize());

    (void)osThreadNew(StartWifiTask, NULL, &wifiTaskAttr);
}
