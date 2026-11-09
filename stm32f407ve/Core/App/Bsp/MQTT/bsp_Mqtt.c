/* bsp_Mqtt.c
 * MQTT 会话层：基于 ESP8266 的 TCP 通道维护一条长连接。
 *
 * ── 状态机 ──
 *   OFF        WiFi 未连接/被主动停止，不做任何事
 *   BACKOFF    连接失败后等待退避时间：5s → 10s → 30s → 60s（之后固定 60s）
 *   CONNECTING 正在 CIPSTART + 发 CONNECT + 等 CONNACK（阻塞数秒）
 *   CONNECTED  已订阅 stm32/dev01/cmd，周期收包 + 每 20s PING
 *
 * ── 收包为什么先搬到自己的缓冲 ──
 *   ESP8266_Cmd() 每次进来第一件事是清接收环，而 CIPSEND 内部就要调它。
 *   如果串口环里已经有 +IPD 数据，直接发就会把半包冲掉，MQTT 流从此错位。
 *   所以每次发送前先 Mqtt_RxDrain() 把环里的原始 TCP 字节搬进 s_rx，
 *   再按 MQTT 剩余长度切帧解析。
 *
 * ── 断线判定 ──
 *   ESP8266_TcpRead 会把 "CLOSED" 文本丢掉，所以拿不到关闭事件。
 *   这里用应用层心跳兜底：20s 发一次 PINGREQ，60s 收不到任何报文就重连。
 */
#include "bsp_Mqtt.h"
#include "bsp_MqttKit.h"
#include "bsp_ESP8266.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>


#define MQTT_RX_BUF_SIZE      512U      /* 单帧上限；本项目消息都 <200B */
#define MQTT_TX_BUF_SIZE      512U
#define MQTT_CMD_QUEUE_LEN    4U
#define MQTT_CONNACK_TIMEOUT  4000U     /* 本地 Broker 通常几十毫秒，4s 足够 */
#define MQTT_DRAIN_ROUNDS     8U        /* 单次最多读 8×96 字节，避免饿死界面 */


typedef enum
{
    MQTT_ST_OFF = 0,
    MQTT_ST_BACKOFF,
    MQTT_ST_CONNECTING,
    MQTT_ST_CONNECTED
} MqttState_t;


static MqttState_t s_state = MQTT_ST_OFF;

static char     s_host[64];
static uint16_t s_port;

static uint8_t  s_rx[MQTT_RX_BUF_SIZE];
static uint16_t s_rxLen;
static uint8_t  s_tx[MQTT_TX_BUF_SIZE];

static uint16_t s_pktId;            /* 订阅/发布共用，从 1 递增 */
static uint32_t s_nextTryTick;
static uint32_t s_lastPingTick;
static uint32_t s_lastRxTick;
static uint8_t  s_backoffIdx;

static uint8_t  s_connAckSeen;
static uint8_t  s_connAckRc;
static uint8_t  s_needReconnect;

static MqttCommand_t s_cmdQ[MQTT_CMD_QUEUE_LEN];
static uint8_t       s_cmdHead;
static uint8_t       s_cmdTail;


/*内部函数*/
static void    Mqtt_EnterBackoff(void);
static uint8_t Mqtt_Connect(void);
static uint8_t Mqtt_Append(const uint8_t *data, uint16_t len);
static uint8_t Mqtt_RxDrain(void);
static void    Mqtt_RxPump(void);
static void    Mqtt_ProcessRx(void);
static void    Mqtt_HandlePublish(uint32_t total);
static uint8_t Mqtt_SendRaw(uint16_t len);
static void    Mqtt_EnqueueCommand(const uint8_t *payload, uint32_t len);


/**
  * 功    能：初始化会话层，清所有状态并装载默认 Broker 地址
  * 参    数：无
  * 返 回 值：无
  */
void Mqtt_Init(void)
{
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_tx, 0, sizeof(s_tx));

    s_state        = MQTT_ST_OFF;
    s_rxLen        = 0U;
    s_pktId        = 0U;
    s_nextTryTick  = 0U;
    s_lastPingTick = 0U;
    s_lastRxTick   = 0U;
    s_backoffIdx   = 0U;
    s_connAckSeen  = 0U;
    s_connAckRc    = 0xFFU;
    s_needReconnect = 0U;

    (void)strncpy(s_host, MQTT_BROKER_HOST, sizeof(s_host) - 1U);
    s_host[sizeof(s_host) - 1U] = '\0';
    s_port = MQTT_BROKER_PORT;

    s_cmdHead = 0U;
    s_cmdTail = 0U;
}

/**
  * 功    能：允许 MQTT 工作，下一次 Poll 立即尝试连接
  * 参    数：无
  * 返 回 值：无
  */
void Mqtt_Start(void)
{
    if (s_state != MQTT_ST_OFF)
    {
        return;
    }
    s_backoffIdx   = 0U;
    s_rxLen        = 0U;
    s_needReconnect = 0U;
    s_state        = MQTT_ST_BACKOFF;
    s_nextTryTick  = HAL_GetTick();     /* 0 延迟：下一次 Poll 直接连 */
    UART_Printf("[MQTT] start, broker %s:%u\r\n", s_host, (unsigned)s_port);
}

/**
  * 功    能：停止 MQTT；graceful=1 时先 DISCONNECT 再关 TCP
  * 参    数：graceful  1=WiFi 还在，可以发 AT；0=WiFi 已掉线，只清本地状态
  * 返 回 值：无
  */
void Mqtt_Stop(uint8_t graceful)
{
    if (s_state == MQTT_ST_OFF)
    {
        return;
    }

    if (graceful != 0U)
    {
        if (s_state == MQTT_ST_CONNECTED)
        {
            uint32_t n;
            /* 先发 retained 离线状态，订阅方立即看到 online=0；
             * 再发 DISCONNECT 让 Broker 丢弃遗嘱（不发的话会重复离线通知）*/
            (void)Mqtt_Publish(MQTT_TOPIC_STATUS, MQTT_PAYLOAD_OFFLINE, MQTT_QOS1, 1U);
            n = MqttKit_BuildDisconnect(s_tx, sizeof(s_tx));
            if (n > 0U)
            {
                (void)Mqtt_SendRaw((uint16_t)n);    /* 等 SEND OK，保证 CIPCLOSE 不被 busy 拒掉 */
            }
        }
        ESP8266_TcpClose();
    }
    else
    {
        ESP8266_TcpRxReset();
    }

    s_state        = MQTT_ST_OFF;
    s_rxLen        = 0U;
    s_needReconnect = 0U;
    s_cmdHead      = 0U;
    s_cmdTail      = 0U;
    UART_Printf("[MQTT] stopped (graceful=%u)\r\n", (unsigned)graceful);
}

/**
  * 功    能：MQTT 主循环：按状态推进连接、收包、心跳与超时
  * 参    数：无
  * 返 回 值：无
  */
void Mqtt_Poll(void)
{
    uint32_t now;

    if (s_state == MQTT_ST_OFF)
    {
        return;
    }

    now = HAL_GetTick();

    if (s_state == MQTT_ST_BACKOFF)
    {
        if ((int32_t)(now - s_nextTryTick) < 0)
        {
            return;
        }
        s_state = MQTT_ST_CONNECTING;
        UART_Printf("[MQTT] connecting %s:%u ...\r\n", s_host, (unsigned)s_port);
    }

    if (s_state == MQTT_ST_CONNECTING)
    {
        if (Mqtt_Connect() == 0U)
        {
            Mqtt_EnterBackoff();
        }
        return;
    }

    /* CONNECTED */
    Mqtt_RxPump();
    if (s_needReconnect != 0U)
    {
        UART_Printf("[MQTT] link error, reconnect\r\n");
        Mqtt_EnterBackoff();
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_lastRxTick) >= MQTT_RX_TIMEOUT_MS)
    {
        UART_Printf("[MQTT] no packet for %u ms, reconnect\r\n", (unsigned)MQTT_RX_TIMEOUT_MS);
        Mqtt_EnterBackoff();
        return;
    }

    if ((uint32_t)(now - s_lastPingTick) >= MQTT_PING_PERIOD_MS)
    {
        uint32_t n = MqttKit_BuildPingReq(s_tx, sizeof(s_tx));

        if ((n == 0U) || (Mqtt_SendRaw((uint16_t)n) == 0U))
        {
            Mqtt_EnterBackoff();
            return;
        }
        s_lastPingTick = now;
    }
}

uint8_t Mqtt_Publish(const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    uint32_t n;
    uint32_t len;
    uint16_t id = 0U;

    if ((s_state != MQTT_ST_CONNECTED) || (topic == NULL) || (payload == NULL) || (qos > MQTT_QOS1))
    {
        return 0U;
    }

    len = (uint32_t)strlen(payload);
    if (len > 65535U)
    {
        return 0U;
    }
    if (qos > 0U)
    {
        s_pktId = (s_pktId == 0xFFFFU) ? 1U : (uint16_t)(s_pktId + 1U);
        id = s_pktId;
    }

    n = MqttKit_BuildPublish(s_tx, sizeof(s_tx), topic, (const uint8_t *)payload,
                             (uint16_t)len, qos, retain, id);
    if (n == 0U)
    {
        return 0U;
    }
    return Mqtt_SendRaw((uint16_t)n);
}

uint8_t Mqtt_TakeCommand(MqttCommand_t *out)
{
    if ((out == NULL) || (s_cmdTail == s_cmdHead))
    {
        return 0U;
    }
    *out = s_cmdQ[s_cmdTail];
    s_cmdTail = (uint8_t)((s_cmdTail + 1U) % MQTT_CMD_QUEUE_LEN);
    return 1U;
}

uint8_t Mqtt_IsConnected(void)
{
    return (s_state == MQTT_ST_CONNECTED) ? 1U : 0U;
}


/*══════ 连接与收包 ══════*/
/**
  * 功    能：进入退避等待，按 5/10/30/60s 递增
  * 参    数：无
  * 返 回 值：无
  */
static void Mqtt_EnterBackoff(void)
{
    static const uint32_t delays[4] = {5000U, 10000U, 30000U, 60000U};
    uint32_t d = delays[s_backoffIdx];

    if (s_backoffIdx < 3U)
    {
        s_backoffIdx++;
    }

    s_state         = MQTT_ST_BACKOFF;
    s_nextTryTick   = HAL_GetTick() + d;
    s_rxLen         = 0U;
    s_needReconnect = 0U;
    s_connAckSeen   = 0U;

    UART_Printf("[MQTT] retry in %u ms\r\n", (unsigned)d);
}

/**
  * 功    能：完整建链：CIPSTART → CONNECT → 等 CONNACK → SUBSCRIBE cmd
  * 参    数：无
  * 返 回 值：1=成功进入 CONNECTED；0=任一步失败
  * 补    充：阻塞数秒；CIPSTART 内部与握手循环都会补喂看门狗
  */
static uint8_t Mqtt_Connect(void)
{
    uint32_t n;
    uint32_t deadline;

    if (ESP8266_TcpConnect(s_host, s_port) == 0U)
    {
        UART_Printf("[MQTT] tcp connect fail\r\n");
        return 0U;
    }

    s_connAckSeen = 0U;
    s_connAckRc   = 0xFFU;
    s_needReconnect = 0U;

    n = MqttKit_BuildConnect(s_tx, sizeof(s_tx), MQTT_CLIENT_ID, MQTT_KEEPALIVE_SEC,
                             MQTT_TOPIC_STATUS, MQTT_PAYLOAD_OFFLINE, MQTT_QOS1, 1U);
    if ((n == 0U) || (ESP8266_TcpSendWait(s_tx, (uint16_t)n, 1500U) == 0U))
    {
        UART_Printf("[MQTT] send CONNECT fail\r\n");
        return 0U;
    }

    /*等 CONNACK：ESP8266_TcpRead 单次最多等 50ms，循环里自己补心跳，
     *否则 4s 握手期间可能超过 3s 的 IWDG 窗口*/
    deadline = HAL_GetTick() + MQTT_CONNACK_TIMEOUT;
    while ((int32_t)(deadline - HAL_GetTick()) > 0)
    {
        uint8_t  tmp[64];
        uint16_t got = ESP8266_TcpRead(tmp, (uint16_t)sizeof(tmp), 50U);

        if (got > 0U)
        {
            if (Mqtt_Append(tmp, got) == 0U)
            {
                return 0U;
            }
            Mqtt_ProcessRx();
            if (s_connAckSeen != 0U)
            {
                break;
            }
        }
        Watchdog_Heartbeat(WDT_TASK_WIFI);
    }

    if ((s_connAckSeen == 0U) || (s_connAckRc != 0U))
    {
        UART_Printf("[MQTT] CONNACK fail rc=%u\r\n", (unsigned)s_connAckRc);
        return 0U;
    }

    /*订阅命令主题；SUBACK 不阻塞等，收到时只打日志*/
    s_pktId = (s_pktId == 0xFFFFU) ? 1U : (uint16_t)(s_pktId + 1U);
    n = MqttKit_BuildSubscribe(s_tx, sizeof(s_tx), MQTT_TOPIC_CMD, MQTT_QOS1, s_pktId);
    if ((n == 0U) || (Mqtt_SendRaw((uint16_t)n) == 0U))
    {
        UART_Printf("[MQTT] subscribe fail\r\n");
        return 0U;
    }

    s_state        = MQTT_ST_CONNECTED;
    s_backoffIdx   = 0U;
    s_lastPingTick = HAL_GetTick();
    s_lastRxTick   = s_lastPingTick;
    s_needReconnect = 0U;

    UART_Printf("[MQTT] connected, sub %s\r\n", MQTT_TOPIC_CMD);

    /*上线状态（retained）：后订阅的设备/客户端也能立即看到在线*/
    (void)Mqtt_Publish(MQTT_TOPIC_STATUS, MQTT_PAYLOAD_ONLINE, MQTT_QOS1, 1U);
    return 1U;
}

/**
  * 功    能：把新收到的字节追加到接收缓冲
  * 参    数：data  数据；len  字节数
  * 返 回 值：1=成功；0=缓冲不够（置重连标志）
  */
static uint8_t Mqtt_Append(const uint8_t *data, uint16_t len)
{
    if ((uint32_t)s_rxLen + (uint32_t)len > (uint32_t)sizeof(s_rx))
    {
        UART_Printf("[MQTT] rx buf full: have=%u add=%u b0=%02X b1=%02X\r\n",
                    (unsigned)s_rxLen, (unsigned)len,
                    (unsigned)data[0], (unsigned)data[1]);
        s_rxLen         = 0U;
        s_needReconnect = 1U;
        return 0U;
    }
    memcpy(&s_rx[s_rxLen], data, len);
    s_rxLen = (uint16_t)(s_rxLen + len);
    s_lastRxTick = HAL_GetTick();
    return 1U;
}

/**
  * 功    能：把串口环里已到的 TCP 原始字节全部搬进 s_rx
  * 参    数：无
  * 返 回 值：1=正常；0=缓冲溢出
  * 补    充：发送前必须先调用，否则 CIPSEND 的 ESP8266_Cmd 会清环丢数据
  */
static uint8_t Mqtt_RxDrain(void)
{
    uint8_t  tmp[96];
    uint16_t n;
    uint8_t  round;

    for (round = 0U; round < MQTT_DRAIN_ROUNDS; round++)
    {
        n = ESP8266_TcpRead(tmp, (uint16_t)sizeof(tmp), 0U);
        if (n == 0U)
        {
            break;
        }
        if (Mqtt_Append(tmp, n) == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

/**
  * 功    能：收一轮数据并解析
  * 参    数：无
  * 返 回 值：无
  */
static void Mqtt_RxPump(void)
{
    (void)Mqtt_RxDrain();
    Mqtt_ProcessRx();
}

/**
  * 功    能：按剩余长度切帧，循环处理所有完整报文
  * 参    数：无
  * 返 回 值：无
  */
static void Mqtt_ProcessRx(void)
{
    while (s_rxLen >= 2U)
    {
        int32_t total = MqttKit_FrameLength(s_rx, (uint32_t)s_rxLen);
        uint8_t type;

        if (total == 0)
        {
            break;                      /* 还没收全，等下一轮 */
        }
        if ((total < 0) || ((uint32_t)total > (uint32_t)s_rxLen))
        {
            UART_Printf("[MQTT] bad frame, drop buffer\r\n");
            s_rxLen         = 0U;
            s_needReconnect = 1U;
            return;
        }

        type = (uint8_t)(s_rx[0] >> 4U);
        if (type == MQTT_PKT_CONNACK)
        {
            uint8_t rc = 0xFFU;
            if (MqttKit_ParseConnAck(s_rx, (uint32_t)total, &rc) == 0)
            {
                s_connAckSeen = 1U;
                s_connAckRc   = rc;
            }
        }
        else if (type == MQTT_PKT_PUBLISH)
        {
            Mqtt_HandlePublish((uint32_t)total);
        }
        else if (type == MQTT_PKT_SUBACK)
        {
            uint8_t rc = 0xFFU;
            if (MqttKit_ParseSubAck(s_rx, (uint32_t)total, &rc) == 0)
            {
                UART_Printf("[MQTT] SUBACK rc=0x%02X\r\n", (unsigned)rc);
            }
        }
        else
        {
            /* PINGRESP / PUBACK 等：收到即说明链路活着，时间戳已在 Append 更新 */
        }

        /*解析期间可能又追加了新数据，所以这里用最新的 s_rxLen*/
        memmove(s_rx, &s_rx[total], (size_t)(s_rxLen - (uint16_t)total));
        s_rxLen = (uint16_t)(s_rxLen - (uint16_t)total);
    }
}

/**
  * 功    能：处理 PUBLISH：QoS1 回 PUBACK，命中 cmd 主题则入队
  * 参    数：total  整帧字节数
  * 返 回 值：无
  */
static void Mqtt_HandlePublish(uint32_t total)
{
    MqttMessage_t msg;

    if (MqttKit_ParsePublish(s_rx, total, &msg) != 0)
    {
        return;
    }

    if ((msg.qos == MQTT_QOS1) && (msg.pktId != 0U))
    {
        uint32_t n = MqttKit_BuildPubAck(s_tx, sizeof(s_tx), msg.pktId);
        if (n > 0U)
        {
            (void)Mqtt_SendRaw((uint16_t)n);
        }
    }

    if ((msg.topicLen == (uint16_t)strlen(MQTT_TOPIC_CMD)) &&
        (memcmp(msg.topic, MQTT_TOPIC_CMD, msg.topicLen) == 0))
    {
        Mqtt_EnqueueCommand(msg.payload, msg.payloadLen);
    }
}

/**
  * 功    能：把下行命令复制进命令队列；队列满则丢弃并打日志
  * 参    数：payload  消息体；len  字节数
  * 返 回 值：无
  */
static void Mqtt_EnqueueCommand(const uint8_t *payload, uint32_t len)
{
    uint8_t next;
    MqttCommand_t *slot;

    if ((payload == NULL) || (len == 0U) || (len > MQTT_CMD_PAYLOAD_MAX))
    {
        UART_Printf("[MQTT] cmd dropped: len=%u\r\n", (unsigned)len);
        return;
    }

    next = (uint8_t)((s_cmdHead + 1U) % MQTT_CMD_QUEUE_LEN);
    if (next == s_cmdTail)
    {
        UART_Printf("[MQTT] cmd dropped: queue full\r\n");
        return;
    }

    slot = &s_cmdQ[s_cmdHead];
    (void)strncpy(slot->topic, MQTT_TOPIC_CMD, sizeof(slot->topic) - 1U);
    slot->topic[sizeof(slot->topic) - 1U] = '\0';
    memcpy(slot->payload, payload, len);
    slot->payload[len] = '\0';
    slot->len = (uint16_t)len;

    s_cmdHead = next;
    UART_Printf("[MQTT] cmd in: %s\r\n", slot->payload);
}

/**
  * 功    能：发送 s_tx 里的数据；发送前先把 +IPD 搬空
  * 参    数：len  待发字节数
  * 返 回 值：1=已写入 ESP8266；0=失败（置重连标志）
  */
static uint8_t Mqtt_SendRaw(uint16_t len)
{
    if ((len == 0U) || (len > 2048U))
    {
        return 0U;
    }
    if (Mqtt_RxDrain() == 0U)
    {
        return 0U;
    }
    /*必须等 SEND OK：CONNECT/SUBSCRIBE/上线消息/遥测是背靠背连发的，
     *不等上一条发完，模块会回 busy p... 导致本包直接丢失*/
    if (ESP8266_TcpSendWait(s_tx, len, 1500U) == 0U)
    {
        UART_Printf("[MQTT] CIPSEND fail, len=%u\r\n", (unsigned)len);
        s_needReconnect = 1U;
        return 0U;
    }
    return 1U;
}
