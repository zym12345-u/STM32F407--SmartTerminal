/* bsp_MqttCfg.h
 * MQTT Broker 地址配置：编译期宏做出厂默认，LittleFS /mqtt.conf 运行时覆盖。
 *
 * ── 优先级 ──
 *   上电先把 bsp_Mqtt.h 的 MQTT_BROKER_HOST / MQTT_BROKER_PORT 填进缓存；
 *   WifiTask 在 LittleFS 挂载完成后调用 Mqtt_Config_Init()，
 *   文件合法则覆盖缓存，否则保持宏默认并打印 [MQTT] cfg invalid。
 *
 * ── 线程约束 ──
 *   LittleFS 不是线程安全的，Init/Set 只能在 WifiTask 里调用；
 *   其它任务（GUI、CommandTask）只能调 Mqtt_Config_Get() 读缓存。
 */
#ifndef __BSP_MQTT_CFG_H
#define __BSP_MQTT_CFG_H

#include <stdint.h>

/* Broker 地址缓存；host 最长 63 字符，port 1~65535 */
typedef struct
{
    char     host[64];
    uint16_t port;
} MqttBrokerCfg_t;

/**
  * 功    能：加载 /mqtt.conf；文件缺失/损坏/键缺失时整体回退宏默认
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；文件非法时不删除、不重写，只回退
  */
void Mqtt_Config_Init(void);

/**
  * 功    能：读当前生效的 Broker 地址（只读缓存，不碰 LittleFS）
  * 参    数：out  输出缓冲，不能为空
  * 返 回 值：无
  */
void Mqtt_Config_Get(MqttBrokerCfg_t *out);

/**
  * 功    能：校验并保存新地址到 /mqtt.conf，成功后同步更新缓存
  * 参    数：host  Broker 地址 1~63 可打印 ASCII；port  1~65535
  * 返 回 值：1 = 保存成功；0 = 参数非法或文件写入失败
  * 补    充：仅 WifiTask 调用；写文件前后各补一次 WiFi 心跳，写完 sync
  */
uint8_t Mqtt_Config_Set(const char *host, uint16_t port);

#endif /* __BSP_MQTT_CFG_H */
