/* bsp_Mqtt.h
 * MQTT 会话层：跑在 WifiTask 里的长连接客户端。
 * 依赖 ESP8266 的 TCP 通道（CIPSTART + CIPSEND + +IPD），由 bsp_MqttKit 组包/解包。
 *
 * ── 使用顺序（必须在 WifiTask 内调用）──
 *   Mqtt_Init()                 开机一次
 *   WiFi 连上后 Mqtt_Start()    进入自动连接状态
 *   WiFi 掉线后 Mqtt_Stop(0)    不碰 AT，只清本地状态
 *   OTA/扫描/SNTP 前 Mqtt_Stop(1)  先 DISCONNECT + CIPCLOSE，把串口和 TCP 让出去
 *   主循环每 50ms 调一次 Mqtt_Poll()，内部完成重连、收包、心跳、超时检测
 *
 * ── 约束 ──
 *   1) ESP8266 是单消费者：本模块所有函数只能在 WifiTask 里调；
 *   2) 每次发数据前会把串口环里的 +IPD 先搬进自己的接收缓冲，避免被 CIPSEND 的
 *      ESP8266_Cmd() 清环冲掉；因此不要在 Mqtt_Poll 之外混用 AT 命令；
 *   3) MQTT 是长连接，OTA 下载必须先 Mqtt_Stop(1)，否则两条 TCP 会抢 CIPSTART。
 */
#ifndef __BSP_MQTT_H
#define __BSP_MQTT_H

#include <stdint.h>

#include "bsp_MqttKit.h"    /* 复用 MQTT_QOS0/1 等协议常量 */
#include "bsp_MqttCfg.h"    /* Broker 地址可配置：宏默认 + /mqtt.conf */


/*══════ 部署参数：编译期默认值 ══════
 * 本地 Mosquitto：PC 192.168.10.3:1883，allow_anonymous true。
 * 运行时可用串口命令 mqttcfg <host> [port] 改 /mqtt.conf 覆盖，无需重编。*/
#define MQTT_BROKER_HOST      "192.168.10.3"
#define MQTT_BROKER_PORT      1883U
#define MQTT_CLIENT_ID        "stm32f407ve-dev01"

/*主题按项目约定：遥测上行 / 命令下行 / 在线状态（retained）/ OTA 预留*/
#define MQTT_TOPIC_TELEMETRY  "stm32/dev01/telemetry"
#define MQTT_TOPIC_CMD        "stm32/dev01/cmd"
#define MQTT_TOPIC_STATUS     "stm32/dev01/status"
#define MQTT_TOPIC_OTA        "stm32/dev01/ota"

/*在线状态载荷：上线发布 online=1（retained），遗嘱与主动下线发布 online=0*/
#define MQTT_PAYLOAD_ONLINE   "{\"online\":1}"
#define MQTT_PAYLOAD_OFFLINE  "{\"online\":0}"

/*心跳：CONNECT 里声明 60s，实际每 20s 发一次 PINGREQ；
 *连续 60s 没收到任何报文就判定连接失效并退避重连*/
#define MQTT_KEEPALIVE_SEC    60U
#define MQTT_PING_PERIOD_MS   20000U
#define MQTT_RX_TIMEOUT_MS    60000U

/*单条下行命令最大长度：MQTT.fx 手动发 JSON，128 字节足够*/
#define MQTT_CMD_PAYLOAD_MAX  128U


/*下行命令：Mqtt_Poll 收到 PUBLISH 后按队列交给 WifiTask 处理*/
typedef struct
{
    char     topic[40];                         /* 命令主题，'\0' 结尾 */
    char     payload[MQTT_CMD_PAYLOAD_MAX + 1U];/* 命令 JSON，'\0' 结尾 */
    uint16_t len;                               /* payload 实际字节数 */
} MqttCommand_t;


/**
  * 功    能：初始化会话层（清状态、装载默认 Broker 地址），开机调用一次
  * 参    数：无
  * 返 回 值：无
  * 补    充：只初始化本地变量，不碰 AT 串口
  */
void Mqtt_Init(void);

/**
  * 功    能：从 Mqtt_Config 缓存重新装载 Broker 地址（改配置后调用）
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；调用前应先 Mqtt_Stop(1)，调用后 Mqtt_Start()
  */
void Mqtt_ReloadConfig(void);

/**
  * 功    能：允许 MQTT 工作；下一次 Mqtt_Poll 会立即尝试连接 Broker
  * 参    数：无
  * 返 回 值：无
  * 补    充：WiFi 连上后调用；重复调用无副作用
  */
void Mqtt_Start(void);

/**
  * 功    能：停止 MQTT
  * 参    数：graceful  1=先发 DISCONNECT 再 CIPCLOSE（WiFi 还在时用）
  *                      0=只清本地状态，不发 AT（WiFi 已掉线时用）
  * 返 回 值：无
  * 补    充：OTA/扫描/SNTP 等要独占 ESP8266 的流程前必须先 graceful 停止
  */
void Mqtt_Stop(uint8_t graceful);

/**
  * 功    能：MQTT 会话主循环：重连、收包、心跳、超时检测
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；连接阶段可能阻塞数秒（内部会补喂看门狗）；
  *            已连接时单次调用只做一次非阻塞收包，耗时几十毫秒以内
  */
void Mqtt_Poll(void);

/**
  * 功    能：发布一条消息
  * 参    数：topic  主题；payload  以 '\0' 结尾的文本消息；qos  0/1；retain  1=保留消息
  * 返 回 值：1=已写入 ESP8266；0=未连接/发送失败
  * 补    充：QoS1 只做一次发送，不重传；遥测用 QoS0，在线状态用 QoS1+retain
  */
uint8_t Mqtt_Publish(const char *topic, const char *payload, uint8_t qos, uint8_t retain);

/**
  * 功    能：取一条下行命令（非阻塞）
  * 参    数：out  输出命令，不可为 NULL
  * 返 回 值：1=取到；0=没有命令
  * 补    充：仅 WifiTask 调用；命令在 Mqtt_Poll 内入队，本函数只搬运
  */
uint8_t Mqtt_TakeCommand(MqttCommand_t *out);

/**
  * 功    能：查询 MQTT 是否已连接
  * 参    数：无
  * 返 回 值：1=CONNACK 成功且未掉线；0=其他
  */
uint8_t Mqtt_IsConnected(void);


#endif /* __BSP_MQTT_H */
