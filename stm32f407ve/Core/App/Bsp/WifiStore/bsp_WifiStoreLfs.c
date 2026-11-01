/* bsp_WifiStoreLfs.c
 * WiFi 凭据存 LittleFS /wifi.conf：本地组帧后整块写入并 sync，校验算法与旧 Flash 方案一致。
 * 硬件：SPI1 = PA5/PA6/PA7 + CS=PC13，经 bsp_lfs_port 访问 W25Q128。
 * 注意：写前后各报一次 WIFI 心跳，LittleFS 整理可能接近秒级，判死线 3s。
 */
#include "bsp_WifiStoreLfs.h"
#include "bsp_WifiStore.h"
#include "bsp_lfs_port.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include <string.h>

#define WIFI_LFS_PATH    "/wifi.conf"      /* LittleFS 内的凭据文件路径(只此一份) */

/**
  * 功    能：算凭据校验和：前 102 字节按 32 位累加，用于读出时验证内容完整
  * 参    数：c  WifiCred_t 指针，不能为空
  * 返 回 值：前 102 字节的 32 位加和(最大约 26010)
  */
static uint32_t WifiLfs_CalcChecksum(const WifiCred_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint32_t       sum = 0U;
    uint32_t       n;
    uint32_t       bodyLen = (uint32_t)((const uint8_t *)&c->checksum - p);

    for (n = 0U; n < bodyLen; n++)
    {
        sum += (uint32_t)p[n];
    }
    return sum;
}

/**
  * 功    能：读文件并校验长度、magic、校验和。任一项不符返回 0 并清空输出
  * 参    数：out  输出缓冲，不能为空，失败时清零
  * 返 回 值：1=有合法凭据，0=文件不存在/长度不符/magic 或校验失败
  */
uint8_t WifiStoreLfs_Load(WifiCred_t *out)
{
    lfs_t      *lfs = LFS_Port_Handle();
    lfs_file_t  file;
    int         err;
    lfs_ssize_t n;

    memset(out, 0, sizeof(*out));
    if (lfs == NULL)
    {
        return 0U;
    }

    err = lfs_file_open(lfs, &file, WIFI_LFS_PATH, LFS_O_RDONLY);   /* 只读打开 */
    if (err != LFS_ERR_OK)
    {
        return 0U;                   /* 文件不存在或读取失败 */
    }

    n = lfs_file_read(lfs, &file, out, sizeof(*out));   /* 必须读满整个结构才算有效 */
    (void)lfs_file_close(lfs, &file);

    if (n != (lfs_ssize_t)sizeof(*out))
    {
        memset(out, 0, sizeof(*out));
        return 0U;
    }
    if (out->magic != WIFI_FLASH_MAGIC)
    {
        memset(out, 0, sizeof(*out));
        return 0U;
    }
    if (out->checksum != WifiLfs_CalcChecksum(out))
    {
        memset(out, 0, sizeof(*out));
        return 0U;
    }
    return 1U;
}

/**
  * 功    能：写文件：打开 → 写满长度 → sync。任一步失败返回 0。超长字段截断
  * 参    数：in  待保存凭据，不能为空，ssid/pwd 超长部分截断
  * 返 回 值：1=打开/写入/写满长度/sync 全部成功，0=任一步失败
  */
uint8_t WifiStoreLfs_Save(const WifiCred_t *in)
{
    lfs_t      *lfs = LFS_Port_Handle();
    lfs_file_t  file;
    int         err;
    lfs_ssize_t n;

    if (lfs == NULL)
    {
        return 0U;
    }

    /* 先在本地缓冲里填好 magic 与校验和，避免写一半被打断留下脏数据 */
    WifiCred_t buf;
    memset(&buf, 0, sizeof(buf));
    buf.magic = WIFI_FLASH_MAGIC;
    strncpy(buf.ssid, in->ssid, WIFI_SSID_LEN);
    strncpy(buf.pwd,  in->pwd,  WIFI_PWD_LEN);
    buf.checksum = WifiLfs_CalcChecksum(&buf);

    /* 只动 1~2 个 4KB 块，busy 约 33ms，不必放宽 IWDG；入口先报一次心跳 */
    Watchdog_Heartbeat(WDT_TASK_WIFI);

    err = lfs_file_open(lfs, &file, WIFI_LFS_PATH,
                        LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);   /* 截断重写, 只保留一条 */
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[WIFI] lfs save open fail err=%d\r\n", err);
        return 0U;
    }

    n = lfs_file_write(lfs, &file, &buf, sizeof(buf));
    if (n != (lfs_ssize_t)sizeof(buf))
    {
        (void)lfs_file_close(lfs, &file);
        UART_Printf("[WIFI] lfs save write fail n=%d\r\n", (int)n);
        return 0U;
    }

    /* sync 保证掉电安全 */
    err = lfs_file_sync(lfs, &file);
    (void)lfs_file_close(lfs, &file);

    Watchdog_Heartbeat(WDT_TASK_WIFI);    /* 写完再报一次: 判死线 3s, 覆盖整段写耗时 */

    if (err != LFS_ERR_OK)
    {
        UART_Printf("[WIFI] lfs save sync fail err=%d\r\n", err);
        return 0U;
    }

    UART_Printf("[WIFI] credential saved to lfs:%s\r\n", WIFI_LFS_PATH);
    return 1U;
}

/**
  * 功    能：一次性迁移：旧扇区里有凭据且 LittleFS 还没文件时，把它写进文件
  * 参    数：无
  * 返 回 值：1=本次确实迁移了一条，0=无需迁移(LFS 已有/旧区为空/写失败)
  */
uint8_t WifiStoreLfs_Migrate(void)
{
    WifiCred_t legacy;
    WifiCred_t dummy;
    uint8_t    migrated = 0U;

    /* lfs 里已有合法凭据 = 之前迁移过, 不再重复 */
    if (WifiStoreLfs_Load(&dummy) != 0U)
    {
        return 0U;
    }

    /* 读旧的内部 Flash sector11 */
    if (WifiStore_Load(&legacy) == 0U)
    {
        return 0U;                   /* sector11 无合法凭据(校验失败), 无需迁移 */
    }

    /* 写入 LittleFS */
    if (WifiStoreLfs_Save(&legacy) == 0U)
    {
        UART_Printf("[WIFI] migrate: lfs write fail, keep legacy flash\r\n");
        return 0U;
    }

    /* 清掉 sector11 的 magic, 防止下次开机重复迁移 */
    {
        WifiCred_t blank;
        memset(&blank, 0, sizeof(blank));
        if (WifiStore_Save(&blank) != 0U)     /* 全 0 写入: magic=0, 下次不再迁移 */
        {
            UART_Printf("[WIFI] migrate: legacy sector11 cleared\r\n");
        }
        else
        {
            UART_Printf("[WIFI] migrate: legacy sector11 clear fail\r\n");
        }
    }

    UART_Printf("[WIFI] migrate: moved credential %s to lfs\r\n", legacy.ssid);
    migrated = 1U;
    return migrated;
}
