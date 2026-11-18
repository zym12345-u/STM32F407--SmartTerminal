/* WifiTaskInternal.h
 * WifiTask 拆分后的内部接口：仅 Core/App/Tasks 下 Wifi*.c 之间使用，不属于对外 API。
 * 对外接口仍在 WifiNet.h（Wifi_PostScan/Wifi_PostJoin/...）。
 */
#ifndef __WIFI_TASK_INTERNAL_H
#define __WIFI_TASK_INTERNAL_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "WifiNet.h"
#include "ClockData.h"


/* 核心 WifiTask.c：状态/事件/连接标志/凭据/版本/命令队列 */
void        Wifi_SetStatus(WifiStatus_t st);      /* 写 g_wifiScan.status，WifiAt/WifiCmd 用 */
void        Wifi_PostEvent(WifiEvent_t ev);       /* 任务->GUI 事件，WifiAt/WifiCmd 用 */
void        Wifi_SetConnected(uint8_t up);        /* 写 s_connected，WifiCmd 用 */
uint16_t    Wifi_CurVer(void);                    /* 本 boot 运行固件版本，WifiMqtt 用 */
osStatus_t  Wifi_CmdQPut(const WifiCmd_t *cmd);   /* 投命令进 cmdQ，WifiMqtt 远程 OTA 用 */
WifiCred_t *Wifi_CredBuffer(void);                /* 凭据实体，WifiCmd 保存成功后写 */
void        Wifi_CredMarkSaved(void);             /* 标记已有可用凭据 */

/* AT 链路与扫描 WifiAt.c */
uint8_t WiFi_FullConnect(const char *ssid, const char *pwd, ClockData_t *now);
void    DoScan(void);

/* 屏幕/远程命令 WifiCmd.c */
void DoJoinCmd(const WifiCmd_t *cmd);
void DoCheckOta(uint8_t manual);
void DoAtProbe(const char *atcmd);

/* MQTT 遥测与远程 OTA WifiMqtt.c */
void Wifi_HandleMqttCommands(void);
void Wifi_PublishTelemetry(const ClockData_t *now);
void Wifi_OtaNotifyCheckResult(void);                        /* 远程检查回执，没有待发标志则忽略 */
void Wifi_OtaNotifyDlResult(const char *state, uint8_t err); /* 远程下载回执，同上 */
void Wifi_OtaFlushPending(void);                             /* MQTT 重连后补发最后一条 OTA 结果 */


#endif /* __WIFI_TASK_INTERNAL_H */
