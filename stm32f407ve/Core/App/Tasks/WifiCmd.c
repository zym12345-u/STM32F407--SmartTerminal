/* WifiCmd.c
 * 屏幕/远程下发的单次命令：配网连接、检查更新、裸 AT 调试。
 * 只被 WifiTask 调用；共享状态通过 WifiTaskInternal.h 的接口读写。
 */
#include "cmsis_os2.h"

#include "bsp_ESP8266.h"
#include "bsp_OTA.h"
#include "bsp_WifiStore.h"
#include "bsp_WifiStoreLfs.h"
#include "OtaData.h"
#include "ClockData.h"
#include "WifiNet.h"
#include "WifiTaskInternal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>


/*屏幕提交的连接命令*/
/**
  * 功    能：执行连接命令：跑完整上线流程，结果同时更新状态和事件队列
  * 参    数：cmd  命令体，ssid/pwd 为用户选定热点与密码
  * 返 回 值：无，结果经 g_wifiScan.status 与 evtQ 发布
  */
void DoJoinCmd(const WifiCmd_t *cmd)
{
    ClockData_t now;
    WifiCred_t *cred;

    memset(&now, 0, sizeof(now));
    Wifi_SetStatus(WIFI_ST_JOINING);

    if (WiFi_FullConnect(cmd->ssid, cmd->pwd, &now) == 0U)
    {
        Wifi_SetConnected(0U);
        Wifi_SetStatus(WIFI_ST_JOIN_FAIL);
        Wifi_PostEvent(WIFI_EV_JOIN_FAIL);
        return;
    }

    /*保存凭据只擦 4KB 块(busy 33ms)，不冻结 CPU，无需放宽 IWDG 窗口*/
    cred = Wifi_CredBuffer();
    memset(cred, 0, sizeof(*cred));
    cred->magic = WIFI_FLASH_MAGIC;
    (void)strncpy(cred->ssid, cmd->ssid, WIFI_SSID_LEN);
    (void)strncpy(cred->pwd,  cmd->pwd,  WIFI_PWD_LEN);
    if (WifiStoreLfs_Save(cred) != 0U)
    {
        Wifi_CredMarkSaved();
    }
    else
    {
        UART_Printf("[WIFI] WARN: credential lfs save failed\r\n");
    }

    Wifi_SetConnected(1U);
    ClockData_Set(&now);
    Wifi_SetStatus(WIFI_ST_JOIN_OK);
    Wifi_PostEvent(WIFI_EV_JOIN_OK);
}


/**
  * 功    能：执行一次检查更新，结果写 g_otaStatus 的 checkState/checkErr/checkVer
  * 参    数：manual  1=手动 0=上电静默
  * 返 回 值：无
  * 补    充：只 GET app.meta；检查期间补 WIFI 心跳（最长十几秒）；
  *            静默检查失败只打日志，不弹窗
  */
void DoCheckOta(uint8_t manual)
{
    uint16_t ver = 0U;
    uint8_t  err;

    g_otaStatus.checkManual = (manual != 0U) ? 1U : 0U;
    g_otaStatus.checkErr    = OTA_ERR_NONE;
    g_otaStatus.checkVer    = 0U;
    g_otaStatus.checkState  = OTA_CHK_CHECKING;

    if (manual != 0U)
    {
        UART_Printf("[OTA] check start manual\r\n");
    }
    else
    {
        UART_Printf("[OTA] check start boot\r\n");
    }

    if (Wifi_IsConnected() == 0U)
    {
        err = OTA_ERR_WIFI;
    }
    else
    {
        err = OTA_CheckUpdate(&ver);
    }

    if (err == OTA_ERR_NONE)
    {
        g_otaStatus.checkVer   = ver;
        g_otaStatus.checkState = OTA_CHK_DONE;
    }
    else
    {
        UART_Printf("[OTA] check fail err=%u\r\n", (unsigned)err);
        if (manual != 0U)
        {
            g_otaStatus.checkErr   = err;
            g_otaStatus.checkState = OTA_CHK_FAIL;
        }
        else
        {
            /*静默检查失败只留日志，不回写失败状态，避免用户进 OTA 页看到过期错误*/
            g_otaStatus.checkErr   = OTA_ERR_NONE;
            g_otaStatus.checkState = OTA_CHK_IDLE;
        }
    }
}


/**
  * 功    能：调试：发一条原始 AT 指令并把模块应答逐行打到调试串口
  * 参    数：atcmd  不含 CRLF 的 AT 指令
  * 返 回 值：无
  * 补    充：仅 WifiTask 调用；应答快照放 static，避免占 3KB 任务栈
  */
void DoAtProbe(const char *atcmd)
{
    static char buf[512];
    char line[96];
    uint16_t n;
    uint16_t i;
    uint16_t k = 0U;
    char     tmp[80];

    if (atcmd == NULL)
    {
        return;
    }

    (void)snprintf(tmp, sizeof(tmp), "%s\r\n", atcmd);
    UART_Printf("[AT] >> %s\r\n", atcmd);

    /* 不关心成败：ASSOC/ERROR/超时都要把原始应答打出来 */
    (void)ESP8266_Cmd(tmp, "OK", 2000U);

    n = ESP8266_CopyResponse(buf, (uint16_t)(sizeof(buf) - 1U));
    buf[n] = '\0';

    for (i = 0U; i < n; i++)
    {
        char c = buf[i];
        if ((c == '\r') || (c == '\n'))
        {
            if (k > 0U)
            {
                line[k] = '\0';
                UART_Printf("[AT] << %s\r\n", line);
                k = 0U;
            }
        }
        else if (k < (uint16_t)(sizeof(line) - 1U))
        {
            line[k++] = c;
        }
    }
    if (k > 0U)
    {
        line[k] = '\0';
        UART_Printf("[AT] << %s\r\n", line);
    }
}
