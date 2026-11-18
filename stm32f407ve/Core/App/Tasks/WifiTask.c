/* WifiTask.c
 * WiFi 任务核心：命令/事件队列、联网状态、本地走秒、掉线重连与主循环。
 * AT 链路与扫描见 WifiAt.c，单次命令见 WifiCmd.c，MQTT 遥测见 WifiMqtt.c。
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
#include "WifiNet.h"
#include "WifiTaskInternal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>


#define WIFI_TASK_STACK_SIZE 3072U   /* 字节;s_resp/s_snap 均为 static,栈只放调用帧 */
#define RESYNC_PERIOD_MS     (30U * 60U * 1000U)
#define RECONNECT_PERIOD_MS  10000U  /* 掉线后自动重连间隔 */
#define OTA_BOOT_CHECK_DELAY_MS 60000U  /* 首次联网后静默检查更新的延时：每 boot 最多一次 */
#define MQTT_PUB_PERIOD_MS   1000U   /* 遥测上报周期：1Hz，本地 Broker 足够 */


/*任务间队列与共享状态*/
static osMessageQueueId_t s_cmdQ;    /* GUI -> 任务 */
static osMessageQueueId_t s_evtQ;    /* 任务 -> GUI */

WifiScanState_t g_wifiScan = {0};

/* 全局时钟数据实体(ClockData.h 中 extern,GuiTask 临界区读取) */
ClockData_t g_clockData;

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
void Wifi_PostEvent(WifiEvent_t ev)
{
    (void)osMessageQueuePut(s_evtQ, &ev, 0U, 0U);
}

/**
  * 功    能：更新 WiFi 状态，状态有变化才打日志（GUI 的状态栏也是读这个值显示）
  * 参    数：st  目标状态，WIFI_ST_IDLE ~ WIFI_ST_CONNECTED
  * 返 回 值：无
  */
void Wifi_SetStatus(WifiStatus_t st)
{
    g_wifiScan.status = (uint8_t)st;
}

/**
  * 功    能：查询"网络是否真的连着"（给 GUI 判断能否发起 OTA 用）
  * 参    数：无
  * 返 回 值：1 = 已联网；0 = 未联网
  * 补    充：g_wifiScan.status 是配网页的状态机，扫描会把它改成 SCANNING/SCAN_OK，
  *            用它判断联网会把"网络好着"误判成"未连接"，所以统一走 s_connected
  */
uint8_t Wifi_IsConnected(void)
{
    return s_connected;
}

/*内部接口：给拆分后的 WifiAt/WifiCmd/WifiMqtt 用，不是对外 API*/
/**
  * 功    能：更新连接标志（配网连接成功/失败、掉线重连用）
  * 参    数：up  1=已联网 0=未联网
  * 返 回 值：无
  */
void Wifi_SetConnected(uint8_t up)
{
    s_connected = (up != 0U) ? 1U : 0U;
}

/**
  * 功    能：取本 boot 运行固件版本（开机只读一次 Flash，之后都从这里取）
  * 参    数：无
  * 返 回 值：版本号 (主<<8|次)
  */
uint16_t Wifi_CurVer(void)
{
    return s_curVer;
}

/**
  * 功    能：往命令队列投一条命令，供 WifiMqtt 远程 OTA 复用
  * 参    数：cmd  待投递命令
  * 返 回 值：osOK=投递成功，osError=队列未创建或参数非法
  */
osStatus_t Wifi_CmdQPut(const WifiCmd_t *cmd)
{
    if ((s_cmdQ == NULL) || (cmd == NULL))
    {
        return osError;
    }
    return osMessageQueuePut(s_cmdQ, cmd, 0U, 0U);
}

/**
  * 功    能：取凭据实体，配网成功后由 DoJoinCmd 填充并落盘
  * 参    数：无
  * 返 回 值：指向任务内静态凭据的指针，不可为 NULL
  */
WifiCred_t *Wifi_CredBuffer(void)
{
    return &s_cred;
}

/**
  * 功    能：标记已保存可用凭据，掉线后据此自动重连
  * 参    数：无
  * 返 回 值：无
  */
void Wifi_CredMarkSaved(void)
{
    s_haveCred = 1U;
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
                    Wifi_OtaNotifyDlResult("fail", OTA_ERR_WIFI);
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
                            Wifi_OtaNotifyDlResult("done_no_reboot", OTA_ERR_NONE);
                            Mqtt_Start();
                        }
                    }
                    else
                    {
                        Wifi_OtaNotifyDlResult("fail", g_otaStatus.err);
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
                Wifi_OtaNotifyCheckResult();
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
            Wifi_OtaFlushPending();

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
