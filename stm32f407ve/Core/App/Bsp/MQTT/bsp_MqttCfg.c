/* bsp_MqttCfg.c
 * MQTT Broker 地址配置：宏默认 + LittleFS /mqtt.conf 覆盖。
 *
 * 文件格式（两行键值，UTF-8/ASCII 纯文本）：
 *   host=192.168.10.3
 *   port=1883
 * 解析规则：逐行解析、行宽受限；host 只允许可打印 ASCII 且不含空格/'='；
 * port 必须 1~65535。任一项非法或键缺失，整体回退宏默认，且不改写文件。
 *
 * 为什么只允许 WifiTask 调用 Init/Set：
 *   LittleFS 与 W25Q 共用 SPI1，且文件系统本身没有锁；GUI/CommandTask 只读缓存，
 *   写入统一由 WifiTask 串行执行，避免两个任务同时改文件系统。
 */
#include "bsp_MqttCfg.h"
#include "bsp_Mqtt.h"          /* MQTT_BROKER_HOST / MQTT_BROKER_PORT 出厂默认值 */
#include "bsp_lfs_port.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MQTT_CFG_PATH      "/mqtt.conf"
#define MQTT_CFG_HOST_MAX  63U
#define MQTT_CFG_READ_MAX  160U

/* 缓存初值就是宏默认：即使 Init 还没跑，读到的也是有效地址 */
static MqttBrokerCfg_t s_cfg = { MQTT_BROKER_HOST, (uint16_t)MQTT_BROKER_PORT };

/**
  * 功    能：把缓存恢复成编译期宏默认值
  * 参    数：无
  * 返 回 值：无
  */
static void Mqtt_Cfg_Defaults(void)
{
    (void)strncpy(s_cfg.host, MQTT_BROKER_HOST, sizeof(s_cfg.host) - 1U);
    s_cfg.host[sizeof(s_cfg.host) - 1U] = '\0';
    s_cfg.port = (uint16_t)MQTT_BROKER_PORT;
}

/**
  * 功    能：校验 host：1~63 字符，只允许可打印 ASCII，且不含空格/'='
  * 参    数：host  待校验字符串
  * 返 回 值：1 = 合法，0 = 非法
  */
static uint8_t Mqtt_Cfg_HostValid(const char *host)
{
    uint16_t i;

    if (host == NULL)
    {
        return 0U;
    }
    for (i = 0U; host[i] != '\0'; i++)
    {
        char c = host[i];

        if (i >= MQTT_CFG_HOST_MAX)
        {
            return 0U;      /* 超过 63 字符 */
        }
        if ((c < 0x21) || (c > 0x7E) || (c == '='))
        {
            return 0U;      /* 控制符/空格/等号都不接受 */
        }
    }
    return (i == 0U) ? 0U : 1U;
}

/**
  * 功    能：解析 /mqtt.conf 文本；host 与 port 两个键都必须存在且合法
  * 参    数：buf  文件内容；len  字节数；out  解析结果
  * 返 回 值：1 = 合法并写入 out；0 = 任一键缺失/重复/值非法/行超长
  * 补    充：未知键忽略，给将来扩展留余地；空行跳过
  */
static uint8_t Mqtt_Cfg_Parse(const uint8_t *buf, uint32_t len, MqttBrokerCfg_t *out)
{
    char     host[MQTT_CFG_HOST_MAX + 1U];
    unsigned int port = 0U;
    uint8_t  haveHost = 0U;
    uint8_t  havePort = 0U;
    uint32_t i = 0UL;

    host[0] = '\0';

    while (i < len)
    {
        uint32_t start;
        uint32_t n;
        char     line[72];

        start = i;
        while ((i < len) && (buf[i] != '\n') && (buf[i] != '\r'))
        {
            i++;
        }
        n = i - start;
        while ((i < len) && ((buf[i] == '\n') || (buf[i] == '\r')))
        {
            i++;
        }
        if (n == 0UL)
        {
            continue;       /* 空行 */
        }
        if (n >= (uint32_t)sizeof(line))
        {
            return 0U;      /* 单行超长，按文件损坏处理 */
        }

        memcpy(line, &buf[start], n);
        line[n] = '\0';

        if (strncmp(line, "host=", 5U) == 0)
        {
            if ((haveHost != 0U) || (Mqtt_Cfg_HostValid(line + 5) == 0U))
            {
                return 0U;
            }
            (void)strncpy(host, line + 5, sizeof(host) - 1U);
            host[sizeof(host) - 1U] = '\0';
            haveHost = 1U;
        }
        else if (strncmp(line, "port=", 5U) == 0)
        {
            char          *endp = NULL;
            unsigned long  v;

            if (havePort != 0U)
            {
                return 0U;
            }
            v = strtoul(line + 5, &endp, 10);
            if ((endp == NULL) || (*endp != '\0') || (v < 1UL) || (v > 65535UL))
            {
                return 0U;
            }
            port = (unsigned int)v;
            havePort = 1U;
        }
        else
        {
            /* 未知键忽略 */
        }
    }

    if ((haveHost == 0U) || (havePort == 0U))
    {
        return 0U;
    }

    (void)strncpy(out->host, host, sizeof(out->host) - 1U);
    out->host[sizeof(out->host) - 1U] = '\0';
    out->port = (uint16_t)port;
    return 1U;
}

/**
  * 功    能：加载配置：先填默认值，再尝试 /mqtt.conf 覆盖
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；文件不存在按默认（不打印 invalid）
  */
void Mqtt_Config_Init(void)
{
    lfs_t      *lfs;
    lfs_file_t  file;
    int         err;
    lfs_ssize_t n;
    uint8_t     buf[MQTT_CFG_READ_MAX];

    Mqtt_Cfg_Defaults();

    lfs = LFS_Port_Handle();
    if (lfs == NULL)
    {
        UART_Printf("[MQTT] cfg host=%s port=%u (default)\r\n",
                    s_cfg.host, (unsigned)s_cfg.port);
        return;
    }

    Watchdog_Heartbeat(WDT_TASK_WIFI);

    err = lfs_file_open(lfs, &file, MQTT_CFG_PATH, LFS_O_RDONLY);
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[MQTT] cfg host=%s port=%u (default)\r\n",
                    s_cfg.host, (unsigned)s_cfg.port);
        return;
    }

    n = lfs_file_read(lfs, &file, buf, (lfs_size_t)(sizeof(buf) - 1U));
    (void)lfs_file_close(lfs, &file);

    Watchdog_Heartbeat(WDT_TASK_WIFI);

    /* n 到读缓冲上限说明文件超长，和解析失败一样整体回退 */
    if ((n <= 0) || (n >= (lfs_ssize_t)(sizeof(buf) - 1U) ||
        (Mqtt_Cfg_Parse(buf, (uint32_t)n, &s_cfg) == 0U)))
    {
        Mqtt_Cfg_Defaults();
        UART_Printf("[MQTT] cfg invalid, use default\r\n");
        return;
    }

    UART_Printf("[MQTT] cfg host=%s port=%u (lfs)\r\n",
                s_cfg.host, (unsigned)s_cfg.port);
}

/**
  * 功    能：读当前生效的 Broker 地址
  * 参    数：out  输出缓冲，不能为空
  * 返 回 值：无
  * 补    充：任意任务可调；只读缓存，不碰 LittleFS
  */
void Mqtt_Config_Get(MqttBrokerCfg_t *out)
{
    if (out == NULL)
    {
        return;
    }
    (void)strncpy(out->host, s_cfg.host, sizeof(out->host) - 1U);
    out->host[sizeof(out->host) - 1U] = '\0';
    out->port = s_cfg.port;
}

/**
  * 功    能：校验并保存新地址：写 /mqtt.conf → sync → 更新缓存
  * 参    数：host  Broker 地址；port  端口
  * 返 回 值：1 = 成功；0 = 参数非法/打开/写入/sync 失败
  * 补    充：仅 WifiTask 调用；写失败时缓存保持不变，运行中的地址不回退
  */
uint8_t Mqtt_Config_Set(const char *host, uint16_t port)
{
    lfs_t      *lfs;
    /*lfs_file_t 和文本缓冲放 static：WifiTask 栈只有 3KB，LittleFS 调用链较深*/
    static lfs_file_t s_file;
    static char       s_text[96];
    char        *text = s_text;
    int         err;
    int         len;
    lfs_ssize_t n;

    if ((Mqtt_Cfg_HostValid(host) == 0U) || (port == 0U))
    {
        UART_Printf("[MQTT] cfg invalid host/port\r\n");
        return 0U;
    }

    len = snprintf(text, sizeof(s_text), "host=%s\nport=%u\n", host, (unsigned)port);
    if ((len <= 0) || ((uint32_t)len >= (uint32_t)sizeof(s_text)))
    {
        return 0U;
    }

    lfs = LFS_Port_Handle();
    if (lfs == NULL)
    {
        UART_Printf("[MQTT] cfg save fail: lfs not mounted\r\n");
        return 0U;
    }

    Watchdog_Heartbeat(WDT_TASK_WIFI);

    err = lfs_file_open(lfs, &s_file, MQTT_CFG_PATH,
                        LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[MQTT] cfg save open fail err=%d\r\n", err);
        return 0U;
    }

    Watchdog_Heartbeat(WDT_TASK_WIFI);
    n = lfs_file_write(lfs, &s_file, text, (lfs_size_t)len);
    if (n != (lfs_ssize_t)len)
    {
        (void)lfs_file_close(lfs, &s_file);
        UART_Printf("[MQTT] cfg save write fail n=%d\r\n", (int)n);
        return 0U;
    }

    Watchdog_Heartbeat(WDT_TASK_WIFI);
    err = lfs_file_sync(lfs, &s_file);
    (void)lfs_file_close(lfs, &s_file);

    Watchdog_Heartbeat(WDT_TASK_WIFI);

    if (err != LFS_ERR_OK)
    {
        UART_Printf("[MQTT] cfg save sync fail err=%d\r\n", err);
        return 0U;
    }

    (void)strncpy(s_cfg.host, host, sizeof(s_cfg.host) - 1U);
    s_cfg.host[sizeof(s_cfg.host) - 1U] = '\0';
    s_cfg.port = port;

    UART_Printf("[MQTT] cfg saved host=%s port=%u\r\n", s_cfg.host, (unsigned)s_cfg.port);
    return 1U;
}
