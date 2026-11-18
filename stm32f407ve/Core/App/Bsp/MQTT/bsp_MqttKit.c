/* bsp_MqttKit.c
 * MQTT 3.1.1 报文组包/解包实现。纯字节操作，不依赖 RTOS/网络，便于单独验证。
 */
#include "bsp_MqttKit.h"

#include <string.h>


/*══════ 组包：带边界检查的顺序写入器 ══════*/
typedef struct
{
    uint8_t *buf;
    uint32_t cap;
    uint32_t len;
    uint8_t  err;      /*1=曾经越界，最终返回 0*/
} KitWriter_t;

/**
  * 功    能：写一个字节；缓冲不够只置错误位，不越界
  * 参    数：w  写入器；v  字节
  * 返 回 值：无
  */
static void Kit_PutU8(KitWriter_t *w, uint8_t v)
{
    if (w->err != 0U)
    {
        return;
    }
    if (w->len >= w->cap)
    {
        w->err = 1U;
        return;
    }
    w->buf[w->len++] = v;
}

/**
  * 功    能：写大端 16 位
  * 参    数：w  写入器；v  数值
  * 返 回 值：无
  */
static void Kit_PutU16(KitWriter_t *w, uint16_t v)
{
    Kit_PutU8(w, (uint8_t)(v >> 8));
    Kit_PutU8(w, (uint8_t)(v & 0xFFU));
}

/**
  * 功    能：写原始字节块
  * 参    数：w  写入器；data  数据；len  字节数
  * 返 回 值：无
  */
static void Kit_PutBytes(KitWriter_t *w, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;

    if ((p == NULL) && (len > 0U))
    {
        w->err = 1U;
        return;
    }
    for (i = 0U; i < len; i++)
    {
        Kit_PutU8(w, p[i]);
    }
}

/**
  * 功    能：写「2 字节长度 + 内容」的 UTF-8 字符串
  * 参    数：w  写入器；s  以 '\0' 结尾的字符串，不允许 NULL
  * 返 回 值：无
  */
static void Kit_PutString(KitWriter_t *w, const char *s)
{
    uint32_t n = (uint32_t)strlen(s);

    if (n > 65535U)
    {
        w->err = 1U;
        return;
    }
    Kit_PutU16(w, (uint16_t)n);
    Kit_PutBytes(w, s, n);
}

/**
  * 功    能：写 MQTT 剩余长度（每字节 7 位，最高位表示后续还有）
  * 参    数：w  写入器；v  剩余长度
  * 返 回 值：无
  */
static void Kit_PutRemain(KitWriter_t *w, uint32_t v)
{
    do
    {
        uint8_t b = (uint8_t)(v % 128U);
        v /= 128U;
        if (v > 0U)
        {
            b |= 0x80U;
        }
        Kit_PutU8(w, b);
    } while (v > 0U);
}

/**
  * 功    能：收尾：有错误返回 0，否则返回写入字节数
  * 参    数：w  写入器
  * 返 回 值：写入字节数或 0
  */
static uint32_t Kit_Finish(const KitWriter_t *w)
{
    return (w->err != 0U) ? 0U : w->len;
}

/**
  * 功    能：解析固定头：跳过首字节，读剩余长度字段
  * 参    数：buf  整帧首地址；len  整帧长度（调用方保证 >=1）
  *            hdrLen  输出可变头起始偏移；remain  输出剩余长度
  * 返 回 值：0=成功；-1=长度字段不完整或超 4 字节
  */
static int8_t Kit_DecodeHeader(const uint8_t *buf, uint32_t len,
                               uint32_t *hdrLen, uint32_t *remain)
{
    uint32_t i = 1U;
    uint32_t mult = 1U;
    uint32_t value = 0U;
    uint8_t  digits = 0U;

    while (i < len)
    {
        uint8_t b = buf[i++];

        value += (uint32_t)(b & 0x7FU) * mult;
        digits++;
        if ((b & 0x80U) == 0U)
        {
            *hdrLen = i;
            *remain = value;
            return 0;
        }
        mult <<= 7U;
        if (digits >= 4U)
        {
            return -1;      /* MQTT 剩余长度最多 4 字节 */
        }
    }
    return -1;
}


/*══════ 对外：组包 ══════*/
/**
  * 功    能：计算一帧报文的总长度（固定头 + 剩余长度字段 + 剩余内容）
  * 参    数：buf  接收流首地址；len  当前已收字节数
  * 返 回 值：>0 = 整帧总字节数；0 = 数据还不够一帧；-1 = 长度字段非法
  * 补    充：用于从 TCP 字节流里切帧，调用方按返回值判断是否收齐。
  *
  * ── MQTT 的"剩余长度"为什么这么麻烦 ──
  *   它是【变长编码】：每字节只有低 7 位是数据，最高位是"后面还有"的续位。
  *   127 以内 1 字节，16383 以内 2 字节……最多 4 字节，表示到 256MB。
  *   所以只看第一个字节根本不知道这帧多长——必须从第 2 字节起逐字节解码，
  *   解到"没有续位"为止才知道固定头到底占了几个字节。
  *   这也是 TCP 粘包/半包的标准解法：FrameLength 返回 0 就继续等下一批字节。
  */
int32_t MqttKit_FrameLength(const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t mult = 1U;
    uint32_t remain = 0U;
    uint8_t  digits = 0U;

    if ((buf == NULL) || (len < 2U))
    {
        return 0;
    }

    for (i = 1U; i < len; i++)
    {
        uint8_t b = buf[i];

        remain += (uint32_t)(b & 0x7FU) * mult;
        digits++;
        if ((b & 0x80U) == 0U)
        {
            uint32_t total = i + 1U + remain;
            if (total > 65535U)     /* 本项目单帧都远小于 64KB，超了按非法处理 */
            {
                return -1;
            }
            if (len < total)        /* 头收全了但正文还没到齐，继续等 */
            {
                return 0;
            }
            return (int32_t)total;
        }
        mult <<= 7U;
        if (digits >= 4U)
        {
            return -1;
        }
    }
    return 0;       /* 长度字段还没收全 */
}

/**
  * 功    能：组 CONNECT 报文（clean session，无用户名密码，可带遗嘱）
  * 参    数：buf  输出缓冲；cap  容量；clientId  客户端 ID；keepAliveSec  保活秒数
  *            willTopic/willPayload  遗嘱主题与内容，传 NULL 表示不带遗嘱
  *            willQos  遗嘱 QoS（0/1）；willRetain  1=遗嘱保留
  * 返 回 值：写入字节数，0=参数非法或缓冲不足
  * 补    充：CONNECT 是唯一的可变头 10 字节的报文：
  *            "MQTT"(4B) + 协议级别(1B) + 连接标志(1B) + keepAlive(2B) = 8，
  *            加上协议名自己的 2 字节长度前缀正好 10。
  *            遗嘱（LWT）是 Broker 替设备发的"遗言"：设备断电/断网没来得及发
  *            DISCONNECT 时，Broker 把 willPayload 发到 willTopic——
  *            本项目用它把 stm32/dev01/status 置成 online=0，Web 端才知道设备掉了。
  */
uint32_t MqttKit_BuildConnect(uint8_t *buf, uint32_t cap,
                              const char *clientId, uint16_t keepAliveSec,
                              const char *willTopic, const char *willPayload,
                              uint8_t willQos, uint8_t willRetain)
{
    KitWriter_t w;
    uint32_t idLen;
    uint32_t willTopicLen = 0U;
    uint32_t willPayLen   = 0U;
    uint32_t remain;
    uint8_t  flags = 0x02U;             /* bit1=clean session */

    if ((buf == NULL) || (clientId == NULL) || (willQos > MQTT_QOS1))
    {
        return 0U;
    }
    idLen = (uint32_t)strlen(clientId);
    if (idLen > 65535U)
    {
        return 0U;
    }

    /*可变头 10 字节 + 客户端 ID + 遗嘱主题/内容（各 2 字节长度 + 内容）*/
    remain = 10U + 2U + idLen;
    if ((willTopic != NULL) && (willPayload != NULL))
    {
        willTopicLen = (uint32_t)strlen(willTopic);
        willPayLen   = (uint32_t)strlen(willPayload);
        if ((willTopicLen == 0U) || (willTopicLen > 65535U) || (willPayLen > 65535U))
        {
            return 0U;
        }
        remain += 2U + willTopicLen + 2U + willPayLen;
        flags |= 0x04U;                     /* bit2=will flag */
        flags |= (uint8_t)(willQos << 3U);  /* bit3~4=will QoS */
        if (willRetain != 0U)
        {
            flags |= 0x20U;                 /* bit5=will retain */
        }
    }

    w.buf = buf;
    w.cap = cap;
    w.len = 0U;
    w.err = 0U;

    Kit_PutU8(&w, (uint8_t)(MQTT_PKT_CONNECT << 4));
    Kit_PutRemain(&w, remain);
    Kit_PutString(&w, "MQTT");          /* 协议名 */
    Kit_PutU8(&w, 4U);                  /* 协议级别 3.1.1 */
    Kit_PutU8(&w, flags);               /* 连接标志：clean session + 可选遗嘱 */
    Kit_PutU16(&w, keepAliveSec);
    Kit_PutString(&w, clientId);        /* 载荷：客户端 ID */
    if (willTopicLen > 0U)
    {
        Kit_PutString(&w, willTopic);
        Kit_PutString(&w, willPayload);
    }

    return Kit_Finish(&w);
}

/**
  * 功    能：组 PUBLISH 报文
  * 参    数：buf/cap  输出缓冲；topic  主题；payload  消息体，可为 NULL（长度为 0）
  *            payloadLen  消息体字节数；qos  0/1；retain  1=保留消息；pktId  QoS>0 时的标识
  * 返 回 值：写入字节数，0=参数非法或缓冲不足
  * 补    充：PUBLISH 的固定头低 4 位本身就是 flags：bit1~2=QoS、bit0=retain，
  *            所以首字节不能像其他报文那样只写类型，要类型<<4 再拼 flags。
  *            QoS1 时报文标识 pktId 紧跟在 topic 后面、payload 前面——
  *            位置是协议规定的，Broker 靠它把 PUBACK 对回这条消息。
  */
uint32_t MqttKit_BuildPublish(uint8_t *buf, uint32_t cap, const char *topic,
                              const uint8_t *payload, uint16_t payloadLen,
                              uint8_t qos, uint8_t retain, uint16_t pktId)
{
    KitWriter_t w;
    uint32_t topicLen;
    uint32_t remain;
    uint8_t  flags;

    if ((buf == NULL) || (topic == NULL) || (qos > MQTT_QOS1))
    {
        return 0U;
    }
    topicLen = (uint32_t)strlen(topic);
    if ((topicLen == 0U) || (topicLen > 65535U))
    {
        return 0U;
    }

    remain = 2U + topicLen + ((qos > 0U) ? 2U : 0U) + (uint32_t)payloadLen;

    w.buf = buf;
    w.cap = cap;
    w.len = 0U;
    w.err = 0U;

    flags = (uint8_t)(MQTT_PKT_PUBLISH << 4);
    flags |= (uint8_t)(qos << 1);
    if (retain != 0U)
    {
        flags |= 0x01U;
    }

    Kit_PutU8(&w, flags);
    Kit_PutRemain(&w, remain);
    Kit_PutU16(&w, (uint16_t)topicLen);
    Kit_PutBytes(&w, topic, topicLen);
    if (qos > 0U)
    {
        Kit_PutU16(&w, pktId);
    }
    Kit_PutBytes(&w, payload, (uint32_t)payloadLen);

    return Kit_Finish(&w);
}

/**
  * 功    能：组 SUBSCRIBE 报文（单主题）
  * 参    数：buf/cap  输出缓冲；topic  主题过滤器；qos  请求 QoS；pktId  报文标识（不能为 0）
  * 返 回 值：写入字节数，0=失败
  * 补    充：固定头低 4 位固定写 0x02（协议规定 SUBSCRIBE 必须带这个标志位，
  *            不是 QoS）；载荷顺序是 pktId + (2B 长度 + topic) + 1B 请求 QoS。
  *            协议本允许一条 SUBSCRIBE 带多个主题，这里只组单主题——
  *            组包代码简单一半，调用方多组几条就行（见 bsp_Mqtt.c 的两连发）。
  */
uint32_t MqttKit_BuildSubscribe(uint8_t *buf, uint32_t cap, const char *topic,
                                uint8_t qos, uint16_t pktId)
{
    KitWriter_t w;
    uint32_t topicLen;
    uint32_t remain;

    if ((buf == NULL) || (topic == NULL) || (qos > MQTT_QOS1) || (pktId == 0U))
    {
        return 0U;
    }
    topicLen = (uint32_t)strlen(topic);
    if ((topicLen == 0U) || (topicLen > 65535U))
    {
        return 0U;
    }

    remain = 2U + 2U + topicLen + 1U;

    w.buf = buf;
    w.cap = cap;
    w.len = 0U;
    w.err = 0U;

    Kit_PutU8(&w, (uint8_t)((MQTT_PKT_SUBSCRIBE << 4) | 0x02U));
    Kit_PutRemain(&w, remain);
    Kit_PutU16(&w, pktId);
    Kit_PutU16(&w, (uint16_t)topicLen);
    Kit_PutBytes(&w, topic, topicLen);
    Kit_PutU8(&w, qos);

    return Kit_Finish(&w);
}

/**
  * 功    能：组 PINGREQ（2 字节）
  * 参    数：buf/cap  输出缓冲
  * 返 回 值：2，0=失败
  * 补    充：整个 MQTT 协议里最小的报文：1 字节类型 + 1 字节剩余长度 0。
  *            keepAlive 只有 CONNECT 时声明，真正续命靠的是这条；
  *            Broker 在 1.5 倍 keepAlive 内没收到任何报文就断开连接。
  */
uint32_t MqttKit_BuildPingReq(uint8_t *buf, uint32_t cap)
{
    if ((buf == NULL) || (cap < 2U))
    {
        return 0U;
    }
    buf[0] = (uint8_t)(MQTT_PKT_PINGREQ << 4);
    buf[1] = 0U;
    return 2U;
}

/**
  * 功    能：组 DISCONNECT（2 字节）
  * 参    数：buf/cap  输出缓冲
  * 返 回 值：2，0=失败
  * 补    充：主动下线必须发它：Broker 收到后才【不发】遗嘱。
  *            不发 DISCONNECT 直接断 TCP（拔网线场景），Broker 会替你把
  *            遗嘱发出去——主动下线和意外掉线就是靠这一条报文区分的。
  */
uint32_t MqttKit_BuildDisconnect(uint8_t *buf, uint32_t cap)
{
    if ((buf == NULL) || (cap < 2U))
    {
        return 0U;
    }
    buf[0] = (uint8_t)(MQTT_PKT_DISCONNECT << 4);
    buf[1] = 0U;
    return 2U;
}

/**
  * 功    能：组 PUBACK（4 字节）
  * 参    数：buf/cap  输出缓冲；pktId  要确认的报文标识
  * 返 回 值：4，0=失败
  * 补    充：QoS1 的"收到确认"：设备收到 QoS1 的 PUBLISH 后必须回 PUBACK，
  *            pktId 原样带回，Broker 才知道哪条投递成功了；
  *            不回的话 Broker 会按重发间隔用 dup=1 重投同一条消息。
  */
uint32_t MqttKit_BuildPubAck(uint8_t *buf, uint32_t cap, uint16_t pktId)
{
    if ((buf == NULL) || (cap < 4U))
    {
        return 0U;
    }
    buf[0] = (uint8_t)(MQTT_PKT_PUBACK << 4);
    buf[1] = 2U;
    buf[2] = (uint8_t)(pktId >> 8);
    buf[3] = (uint8_t)(pktId & 0xFFU);
    return 4U;
}


/*══════ 对外：解包 ══════*/
/**
  * 功    能：解 CONNACK
  * 参    数：buf/len  整帧报文；rc  输出连接返回码，0=成功
  * 返 回 值：0=解析成功；-1=不是合法 CONNACK
  * 补    充：载荷 2 字节：第 0 字节是连接确认标志（当前必为 0），
  *            第 1 字节才是返回码——0=接受、1=协议版本错、2=clientId 被拒、
  *            5=未授权。调试时 rc 打出来基本能定位"连不上"的配置问题。
  */
int8_t MqttKit_ParseConnAck(const uint8_t *buf, uint32_t len, uint8_t *rc)
{
    uint32_t hdrLen;
    uint32_t remain;

    if ((buf == NULL) || (rc == NULL) || (len < 2U))
    {
        return -1;
    }
    if ((buf[0] >> 4) != MQTT_PKT_CONNACK)
    {
        return -1;
    }
    if (Kit_DecodeHeader(buf, len, &hdrLen, &remain) != 0)
    {
        return -1;
    }
    if ((remain < 2U) || ((uint32_t)hdrLen + remain > len))
    {
        return -1;
    }
    *rc = buf[hdrLen + 1U];     /* 第 0 字节是连接确认标志，第 1 字节是返回码 */
    return 0;
}

/**
  * 功    能：解 PUBLISH，topic/payload 指针指向 buf 内部
  * 参    数：buf/len  整帧报文；out  输出解析结果
  * 返 回 值：0=成功；-1=报文非法
  * 补    充：【零拷贝设计】topic/payload 不复制，直接指向原缓冲里的位置，
  *            生命周期 = 调用方的那块 buf——调用方处理完才能动缓冲。
  *            省一次拷贝看起来小事，但命令队列里存的是【拷贝后的副本】
  *            （见 bsp_Mqtt.c Mqtt_EnqueueCommand），这里的指针只活到入队前。
  */
int8_t MqttKit_ParsePublish(const uint8_t *buf, uint32_t len, MqttMessage_t *out)
{
    uint32_t hdrLen;
    uint32_t remain;
    uint32_t pos;
    uint16_t topicLen;

    if ((buf == NULL) || (out == NULL) || (len < 2U))
    {
        return -1;
    }
    if ((buf[0] >> 4) != MQTT_PKT_PUBLISH)
    {
        return -1;
    }
    if (Kit_DecodeHeader(buf, len, &hdrLen, &remain) != 0)
    {
        return -1;
    }
    if (((uint32_t)hdrLen + remain > len) || (remain < 2U))
    {
        return -1;
    }

    memset(out, 0, sizeof(*out));
    out->type    = MQTT_PKT_PUBLISH;
    out->flags   = (uint8_t)(buf[0] & 0x0FU);
    out->retain  = (uint8_t)(out->flags & 0x01U);
    out->qos     = (uint8_t)((out->flags >> 1) & 0x03U);
    out->dup     = (uint8_t)((out->flags >> 3) & 0x01U);
    if (out->qos > MQTT_QOS1)
    {
        return -1;              /* 本项目不处理 QoS2 */
    }

    pos = hdrLen;
    topicLen = (uint16_t)(((uint16_t)buf[pos] << 8) | buf[pos + 1U]);
    pos += 2U;
    if ((uint32_t)topicLen + 2U > remain)
    {
        return -1;
    }

    out->topic    = (const char *)&buf[pos];
    out->topicLen = topicLen;
    pos += topicLen;
    remain -= (uint32_t)topicLen + 2U;

    if (out->qos > 0U)
    {
        if (remain < 2U)
        {
            return -1;
        }
        out->pktId = (uint16_t)(((uint16_t)buf[pos] << 8) | buf[pos + 1U]);
        pos += 2U;
        remain -= 2U;
    }

    out->payload    = &buf[pos];
    out->payloadLen = remain;
    return 0;
}

/**
  * 功    能：解 SUBACK
  * 参    数：buf/len  整帧报文；rc  输出第一个主题的返回码（0/1/2 成功，0x80 失败）
  * 返 回 值：0=解析成功；-1=不是合法 SUBACK
  * 补    充：返回码与请求的 QoS 对应：0=QoS0、1=QoS1、2=QoS2 通过并降级，
  *            0x80=Broker 拒绝订阅（主题无权限等）。本项目只发 QoS1，
  *            所以正常应回 0x01；收到 0x80 说明 Broker 配置有问题。
  */
int8_t MqttKit_ParseSubAck(const uint8_t *buf, uint32_t len, uint8_t *rc)
{
    uint32_t hdrLen;
    uint32_t remain;

    if ((buf == NULL) || (rc == NULL) || (len < 2U))
    {
        return -1;
    }
    if ((buf[0] >> 4) != MQTT_PKT_SUBACK)
    {
        return -1;
    }
    if (Kit_DecodeHeader(buf, len, &hdrLen, &remain) != 0)
    {
        return -1;
    }
    /*剩余长度 = 2 字节报文标识 + 至少 1 字节返回码*/
    if ((remain < 3U) || ((uint32_t)hdrLen + remain > len))
    {
        return -1;
    }
    *rc = buf[hdrLen + 2U];
    return 0;
}
