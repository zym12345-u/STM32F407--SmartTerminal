/* WifiMqtt.c
 * MQTT 下行命令、遥测上报与远程 OTA 回执。
 * 只被 WifiTask 调用；共享状态通过 WifiTaskInternal.h 的接口读写。
 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"

#include "bsp_Mqtt.h"
#include "OtaData.h"
#include "ClockData.h"
#include "SensorData.h"
#include "LEDType.h"
#include "bsp_SDCard.h"
#include "WifiNet.h"
#include "WifiTaskInternal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* LED 队列句柄由 freertos.c 创建，WifiTask 收到 MQTT 命令后投递点灯消息 */
extern osMessageQueueId_t LEDQueueHandle;


/*远程 OTA 通知：检查和下载都要先把 ESP8266 让给 HTTP，结果当场发不出去，
 *所以先存一条待发消息，等 MQTT 重连后由主循环补发。
 *两个来源标志保证只有"远程发起的操作"才回传结果，GUI 本地点按钮不往外发。*/
static char     s_otaOut[128];
static uint8_t  s_otaOutPending = 0U;
static uint8_t  s_otaCheckNotify = 0U;
static uint8_t  s_otaDlNotify = 0U;


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

    Wifi_VerText(Wifi_CurVer(), cur, sizeof(cur));
    memset(&q, 0, sizeof(q));

    if (download != 0U)
    {
        q.type = WIFI_CMD_OTA;
        st = Wifi_CmdQPut(&q);
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
        st = Wifi_CmdQPut(&q);
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

    Wifi_VerText(Wifi_CurVer(), cur, sizeof(cur));

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

    Wifi_VerText(Wifi_CurVer(), cur, sizeof(cur));
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
    uint16_t    curVer;

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

    curVer = Wifi_CurVer();
    Wifi_VerText(curVer, cur, sizeof(cur));

    if (Wifi_JsonValue(msg->payload, "ver") != NULL)
    {
        target = Wifi_JsonVer(msg->payload, "ver");
        if ((target != 0U) && (target > curVer))
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
void Wifi_PublishTelemetry(const ClockData_t *now)
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
    Wifi_VerText(Wifi_CurVer(), fw, sizeof(fw));

    /*带 fw：远程端不用额外查询就能确认升级后的设备是否已跑在新版本上；
     * 带 sd/sdkb：远程确认 TF 卡是否被识别及容量，便于 SDIO 联调期间免串口观察*/
    n = snprintf(json, sizeof(json),
                 "{\"temp\":%d.%d,\"humi\":%d.%d,\"lux\":%d,\"ok\":%u,"
                 "\"ts\":\"%04u-%02u-%02u %02u:%02u:%02u\",\"fw\":\"%s\","
                 "\"sd\":%u,\"sdkb\":%lu}",
                 t10 / 10, ((t10 < 0) ? -t10 : t10) % 10,
                 h10 / 10, ((h10 < 0) ? -h10 : h10) % 10,
                 lux,
                 (unsigned)g_sensorData.temp_valid,
                 (unsigned)now->year, (unsigned)now->month, (unsigned)now->day,
                 (unsigned)now->hour, (unsigned)now->min, (unsigned)now->sec,
                 fw,
                 (unsigned)SDCard_IsReady(),
                 (unsigned long)SDCard_GetCapacityKB());

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
void Wifi_HandleMqttCommands(void)
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


/*主循环用的 OTA 回执接口：只有远程发起的操作才回传，GUI 本地点按钮不往外发*/
/**
  * 功    能：远程发起的检查结束后调用，有待发标志才组织并缓存回执
  * 参    数：无
  * 返 回 值：无
  */
void Wifi_OtaNotifyCheckResult(void)
{
    if (s_otaCheckNotify != 0U)
    {
        s_otaCheckNotify = 0U;
        Wifi_OtaQueueCheckResult();
    }
}

/**
  * 功    能：远程发起的下载结束后调用，有待发标志才组织并缓存回执
  * 参    数：state  结果短语；err  失败原因（成功填 0）
  * 返 回 值：无
  */
void Wifi_OtaNotifyDlResult(const char *state, uint8_t err)
{
    if (s_otaDlNotify != 0U)
    {
        s_otaDlNotify = 0U;
        Wifi_OtaQueueDlResult(state, err);
    }
}

/**
  * 功    能：MQTT 重连后补发最后一条 OTA 结果，发成功才清标志，失败下轮继续试
  * 参    数：无
  * 返 回 值：无
  */
void Wifi_OtaFlushPending(void)
{
    if ((s_otaOutPending != 0U) && (Mqtt_IsConnected() != 0U))
    {
        if (Mqtt_Publish(MQTT_TOPIC_OTA, s_otaOut, MQTT_QOS1, 0U) != 0U)
        {
            s_otaOutPending = 0U;
        }
    }
}
