/**
  ******************************************************************************
  * @file    WifiNet.h
  * @brief   屏幕配网:GuiTask 与 WifiTask 之间的命令/事件/数据定义
  *
  *          通信模型(两条单向队列,均由 WifiTask_Create 创建):
  *            GuiTask --WifiCmd_t-->  WifiTask   请求扫描 / 提交密码连接
  *            WifiTask --WifiEvent--> GuiTask   扫描完成 / 连接成功失败
  *          热点列表内容较大(8 个 AP),不走队列:WifiTask 扫描后写入
  *          g_wifiScan,GUI 收到 SCAN_DONE 事件后直接读(单写单读,事件同步)。
  ******************************************************************************
  */
#ifndef __WIFI_NET_H
#define __WIFI_NET_H

#include <stdint.h>


#define WIFI_SSID_LEN   32U    /* 802.11 SSID 最长 32 字节,+1 收尾 */
#define WIFI_PWD_LEN    64U    /* WPA2 预共享密钥最长 63 字节,+1 收尾 */
#define WIFI_AP_MAX     8U     /* 屏幕列表最多显示的热点数(按信号取前 N) */


/* 掉电保存到 STM32 内部 Flash 的 WiFi 凭据 */
typedef struct
{
    uint32_t magic;                 /* WIFI_FLASH_MAGIC,判定是否写过凭据 */
    char     ssid[WIFI_SSID_LEN + 1U];
    char     pwd[WIFI_PWD_LEN + 1U];
    uint32_t checksum;              /* 其余字段的简单加和校验 */
} WifiCred_t;

/* 扫描到的单个热点 */
typedef struct
{
    char    ssid[WIFI_SSID_LEN + 1U];
    int8_t  rssi;                   /* dBm,如 -55;越大越好 */
    uint8_t sec;                    /* CWLAP ecn:0=开放,其余=加密 */
} WifiAp_t;

/* WifiTask 当前状态(配网页状态文字用) */
typedef enum
{
    WIFI_ST_IDLE = 0,       /* 空闲,等待命令 */
    WIFI_ST_SCANNING,       /* 正在扫描 */
    WIFI_ST_SCAN_OK,        /* 扫描完成,列表有效 */
    WIFI_ST_SCAN_FAIL,      /* 扫描失败 */
    WIFI_ST_JOINING,        /* 正在用提交的密码连接+对时 */
    WIFI_ST_JOIN_OK,        /* 连接并对时成功 */
    WIFI_ST_JOIN_FAIL,      /* 连接失败 */
    WIFI_ST_CONNECTED       /* 已联网(开机自动连上或配网后运行中) */
} WifiStatus_t;

/* 扫描结果共享区:WifiTask 写,GuiTask 在收到事件后读 */
typedef struct
{
    volatile uint8_t status;        /* WifiStatus_t */
    volatile uint8_t count;         /* aps 中有效条目数 */
    WifiAp_t         aps[WIFI_AP_MAX];
} WifiScanState_t;

extern WifiScanState_t g_wifiScan;


/* GUI -> WifiTask 命令 */
#define WIFI_CMD_SCAN   1U
#define WIFI_CMD_JOIN   2U
typedef struct
{
    uint8_t type;                       /* WIFI_CMD_xxx */
    char    ssid[WIFI_SSID_LEN + 1U];   /* JOIN 时有效 */
    char    pwd[WIFI_PWD_LEN + 1U];
} WifiCmd_t;

/* WifiTask -> GUI 事件 */
typedef enum
{
    WIFI_EV_SCAN_DONE = 1,  /* 扫描结束(成功或失败,读 g_wifiScan.status) */
    WIFI_EV_JOIN_OK,        /* 提交的密码连接+对时成功 */
    WIFI_EV_JOIN_FAIL       /* 连接失败/超时 */
} WifiEvent_t;


/* GuiTask 调用:请求扫描周围热点(非阻塞,结果由事件通知) */
void Wifi_PostScan(void);

/* GuiTask 调用:提交选中的 SSID 与密码发起连接(非阻塞) */
void Wifi_PostJoin(const char *ssid, const char *pwd);

/* GuiTask 在 LVGL 定时器里非阻塞轮询事件:1=取到一个事件 */
uint8_t Wifi_TryGetEvent(WifiEvent_t *ev);


#endif /* __WIFI_NET_H */
