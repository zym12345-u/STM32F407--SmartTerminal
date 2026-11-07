/* WifiNet.h
 * 屏幕配网：GuiTask 与 WifiTask 之间的命令、事件与共享数据定义，含掉电保存的凭据与热点列表结构。
 */
/*两条单向队列由 WifiTask_Create 创建：GuiTask 发 WifiCmd_t，WifiTask 回 WifiEvent_t*/
/*热点列表 8 个 AP 较大不走队列：WifiTask 扫描后写 g_wifiScan，GUI 收到 SCAN_DONE 后直接读(单写单读靠事件同步)*/
#ifndef __WIFI_NET_H
#define __WIFI_NET_H

#include <stdint.h>


/* 长度均为字符/条目个数，编译期常量，决定 WifiCred_t/WifiCmd_t/WifiAp_t 的数组长度 */
#define WIFI_SSID_LEN   32U    /* SSID 最长 32 字节，+1 收尾 */
#define WIFI_PWD_LEN    64U    /* WPA2 预共享密钥最长 63 字节，+1 收尾 */
#define WIFI_AP_MAX     8U     /* 列表最多显示的热点数(按信号取前 N) */


/* 掉电保存到内部 Flash 的 WiFi 凭据 */
typedef struct
{
    uint32_t magic;                 /* WIFI_FLASH_MAGIC，判定是否写过凭据；保存流程写，开机比对 */
    char     ssid[WIFI_SSID_LEN + 1U];   /* 热点名 0~32 字符；保存流程写，开机连接读 */
    char     pwd[WIFI_PWD_LEN + 1U];     /* 密码 0~63 字符(WPA2 上限)；保存流程写，开机连接读 */
    uint32_t checksum;              /* 其余字段的简单加和，防损坏；保存流程写，读取时校验 */
} WifiCred_t;

/* 扫描到的单个热点 */
typedef struct
{
    char    ssid[WIFI_SSID_LEN + 1U];   /* 热点名 0~32 字符；WifiTask 扫描后写，GuiTask 读 */
    int8_t  rssi;                       /* 信号强度 dBm，-127~0，越大越好；WifiTask 写，GuiTask 排序/显示 */
    uint8_t sec;                        /* 加密类型 CWLAP ecn，0=开放 1~5=各类加密；WifiTask 写，GuiTask 读 */
} WifiAp_t;

/* WifiTask 状态 0~7，配网页状态文字用；WifiTask 写 g_wifiScan.status，GuiTask 读 */
typedef enum
{
    WIFI_ST_IDLE = 0,       /* 空闲，等命令 */
    WIFI_ST_SCANNING,       /* 正在扫描 */
    WIFI_ST_SCAN_OK,        /* 扫描完成，列表有效 */
    WIFI_ST_SCAN_FAIL,      /* 扫描失败 */
    WIFI_ST_JOINING,        /* 正用提交的密码连接并对时 */
    WIFI_ST_JOIN_OK,        /* 连接并对时成功 */
    WIFI_ST_JOIN_FAIL,      /* 连接失败 */
    WIFI_ST_CONNECTED       /* 已联网(开机自动连上或配网后) */
} WifiStatus_t;

/* 扫描结果共享区：WifiTask 写，GuiTask 收到事件后读 */
typedef struct
{
    volatile uint8_t status;        /* WifiStatus_t 0~7；WifiTask 写，GuiTask 读 */
    volatile uint8_t count;         /* aps 有效条目数 0~WIFI_AP_MAX；WifiTask 写，GuiTask 读 */
    WifiAp_t         aps[WIFI_AP_MAX];   /* 扫描结果，下标 0~(count-1) 有效；WifiTask 写，GuiTask 读 */
} WifiScanState_t;

extern WifiScanState_t g_wifiScan;


/* 命令码 1~3；GuiTask 填并投队列，WifiTask 读 */
#define WIFI_CMD_SCAN   1U
#define WIFI_CMD_JOIN   2U
#define WIFI_CMD_OTA    3U                  /* 下载固件到 W25Q 槽 B，进度看 g_otaStatus */
typedef struct
{
    uint8_t type;                       /* 1=SCAN 2=JOIN 3=OTA；GuiTask 写，WifiTask 读 */
    char    ssid[WIFI_SSID_LEN + 1U];   /* 目标热点名 0~32 字符，仅 JOIN 有效；GuiTask 写，WifiTask 读 */
    char    pwd[WIFI_PWD_LEN + 1U];     /* 密码 0~63 字符，仅 JOIN 有效；GuiTask 写，WifiTask 读 */
} WifiCmd_t;

/* 事件 1~3；WifiTask 填并投队列，GuiTask 在 LVGL 定时器里轮询 */
typedef enum
{
    WIFI_EV_SCAN_DONE = 1,  /* 扫描结束，成败看 g_wifiScan.status */
    WIFI_EV_JOIN_OK,        /* 连接并对时成功 */
    WIFI_EV_JOIN_FAIL       /* 连接失败/超时 */
} WifiEvent_t;


/*非阻塞，可在 GUI 事件回调里直接调用，队列满静默丢弃*/
/**
  * 功    能：让 WifiTask 去扫一遍热点：只投一条命令进队列就返回，不等结果
  * 参    数：无
  * 返 回 值：无
  */
void Wifi_PostScan(void);

/*非阻塞；字符串在函数内拷入命令结构体，调用方缓冲可立即释放*/
/**
  * 功    能：让 WifiTask 连指定热点：投命令进队列就返回，连接结果通过事件队列拿
  * 参    数：ssid  热点名 0~32 字符，不可为 NULL；pwd  密码 0~63 字符，开放热点传空串
  * 返 回 值：无
  * 补    充：ssid 不能为 NULL；开放热点密码传空串
  */
void Wifi_PostJoin(const char *ssid, const char *pwd);

/**
  * 功    能：请求 WifiTask 从 HTTP 服务器下载固件到 W25Q 槽 B（非阻塞，投队列就返回）
  * 参    数：无
  * 返 回 值：无
  * 补    充：下载进度/结果读 g_otaStatus；只能在本页 WiFi 已连接时发起，否则 err=OTA_ERR_WIFI
  */
void Wifi_PostOta(void);

/*0 tick 超时，绝不阻塞 GUI；事件队列长度由 WifiTask_Create 决定*/
/**
  * 功    能：非阻塞取一个 WiFi 事件（扫描完成 / 连接成功 / 连接失败）
  * 参    数：ev  输出事件，取到时写入 1~3，不可为 NULL
  * 返 回 值：1=取到一个事件，0=当前无事件
  */
uint8_t Wifi_TryGetEvent(WifiEvent_t *ev);


#endif /* __WIFI_NET_H */
