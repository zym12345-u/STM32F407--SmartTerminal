/* bsp_MqttKit.h
 * MQTT 3.1.1 报文组包/解包：只做字节层，不碰网络、不 malloc。
 * 设计：组包写入调用方给的缓冲，解包返回指向原缓冲的指针，生命周期由调用方保证。
 * 支持：CONNECT（含遗嘱）/ PUBLISH(QoS0/1) / SUBSCRIBE(QoS0/1) / PUBACK / PINGREQ / DISCONNECT。
 * 不支持：QoS2、用户名密码（本地 Mosquitto allow_anonymous 用不到）。
 */
#ifndef __BSP_MQTTKIT_H
#define __BSP_MQTTKIT_H

#include <stdint.h>


/*固定头部报文类型（高 4 位）*/
#define MQTT_PKT_CONNECT      1U
#define MQTT_PKT_CONNACK      2U
#define MQTT_PKT_PUBLISH      3U
#define MQTT_PKT_PUBACK       4U
#define MQTT_PKT_SUBSCRIBE    8U
#define MQTT_PKT_SUBACK       9U
#define MQTT_PKT_PINGREQ      12U
#define MQTT_PKT_PINGRESP     13U
#define MQTT_PKT_DISCONNECT   14U

/*QoS 等级：本项目只用 0/1*/
#define MQTT_QOS0             0U
#define MQTT_QOS1             1U


/*解包结果：topic 与 payload 指向原始报文缓冲，不复制*/
typedef struct
{
    uint8_t        type;          /*报文类型，MQTT_PKT_xxx*/
    uint8_t        flags;         /*固定头低 4 位*/
    uint8_t        qos;           /*0/1，PUBLISH 有效*/
    uint8_t        retain;        /*1= retained 消息*/
    uint8_t        dup;           /*1=重复投递*/
    uint16_t       pktId;         /*QoS>0 时的报文标识*/
    const char    *topic;         /*主题，不带结尾 '\0'*/
    uint16_t       topicLen;      /*主题字节数*/
    const uint8_t *payload;       /*消息体，二进制安全*/
    uint32_t       payloadLen;    /*消息体字节数*/
} MqttMessage_t;


/**
  * 功    能：计算一帧报文的总长度（固定头 + 剩余长度字段 + 剩余内容）
  * 参    数：buf  接收流首地址；len  当前已收字节数
  * 返 回 值：>0 = 整帧总字节数；0 = 数据还不够一帧；-1 = 长度字段非法
  * 补    充：用于从 TCP 字节流里切帧，调用方按返回值判断是否收齐
  */
int32_t MqttKit_FrameLength(const uint8_t *buf, uint32_t len);

/**
  * 功    能：组 CONNECT 报文（clean session，无用户名密码，可带遗嘱）
  * 参    数：buf  输出缓冲；cap  容量；clientId  客户端 ID；keepAliveSec  保活秒数
  *            willTopic/willPayload  遗嘱主题与内容，传 NULL 表示不带遗嘱
  *            willQos  遗嘱 QoS（0/1）；willRetain  1=遗嘱保留
  * 返 回 值：写入字节数，0=参数非法或缓冲不足
  */
uint32_t MqttKit_BuildConnect(uint8_t *buf, uint32_t cap,
                              const char *clientId, uint16_t keepAliveSec,
                              const char *willTopic, const char *willPayload,
                              uint8_t willQos, uint8_t willRetain);

/**
  * 功    能：组 PUBLISH 报文
  * 参    数：buf/cap  输出缓冲；topic  主题；payload  消息体，可为 NULL（长度为 0）
  *            payloadLen  消息体字节数；qos  0/1；retain  1=保留消息；pktId  QoS>0 时的标识
  * 返 回 值：写入字节数，0=参数非法或缓冲不足
  */
uint32_t MqttKit_BuildPublish(uint8_t *buf, uint32_t cap, const char *topic,
                              const uint8_t *payload, uint16_t payloadLen,
                              uint8_t qos, uint8_t retain, uint16_t pktId);

/**
  * 功    能：组 SUBSCRIBE 报文（单主题）
  * 参    数：buf/cap  输出缓冲；topic  主题过滤器；qos  请求 QoS；pktId  报文标识（不能为 0）
  * 返 回 值：写入字节数，0=失败
  */
uint32_t MqttKit_BuildSubscribe(uint8_t *buf, uint32_t cap, const char *topic,
                                uint8_t qos, uint16_t pktId);

/**
  * 功    能：组 PINGREQ（2 字节）
  * 参    数：buf/cap  输出缓冲
  * 返 回 值：2，0=失败
  */
uint32_t MqttKit_BuildPingReq(uint8_t *buf, uint32_t cap);

/**
  * 功    能：组 DISCONNECT（2 字节）
  * 参    数：buf/cap  输出缓冲
  * 返 回 值：2，0=失败
  */
uint32_t MqttKit_BuildDisconnect(uint8_t *buf, uint32_t cap);

/**
  * 功    能：组 PUBACK（4 字节）
  * 参    数：buf/cap  输出缓冲；pktId  要确认的报文标识
  * 返 回 值：4，0=失败
  */
uint32_t MqttKit_BuildPubAck(uint8_t *buf, uint32_t cap, uint16_t pktId);

/**
  * 功    能：解 CONNACK
  * 参    数：buf/len  整帧报文；rc  输出连接返回码，0=成功
  * 返 回 值：0=解析成功；-1=不是合法 CONNACK
  */
int8_t MqttKit_ParseConnAck(const uint8_t *buf, uint32_t len, uint8_t *rc);

/**
  * 功    能：解 PUBLISH，topic/payload 指针指向 buf 内部
  * 参    数：buf/len  整帧报文；out  输出解析结果
  * 返 回 值：0=成功；-1=报文非法
  */
int8_t MqttKit_ParsePublish(const uint8_t *buf, uint32_t len, MqttMessage_t *out);

/**
  * 功    能：解 SUBACK
  * 参    数：buf/len  整帧报文；rc  输出第一个主题的返回码（0/1/2 成功，0x80 失败）
  * 返 回 值：0=解析成功；-1=不是合法 SUBACK
  */
int8_t MqttKit_ParseSubAck(const uint8_t *buf, uint32_t len, uint8_t *rc);


#endif /* __BSP_MQTTKIT_H */
