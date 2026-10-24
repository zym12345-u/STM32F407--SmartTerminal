/**
  ******************************************************************************
  * @file    bsp_WifiStoreLfs.c
  * @brief   WiFi 凭据 LittleFS 存储 —— 实现
  *
  *          与旧内部 Flash 方案对比:
  *            旧: 擦 128KB 扇区,CPU 停 1~2s,必须 IWDG_SetReload(500) 放宽窗口;
  *            新: LittleFS 只擦 1~2 个 4KB 块(busy 约 33ms),RTOS 正常运行,
  *                无需动 IWDG,也不冻结其他任务。
  *
  *          注意: 调用上下文必须是"已挂载 LittleFS 且单任务串行访问 SPI1"的地方
  *                (当前是 WifiTask 在连接成功后调用)。
  ******************************************************************************
  */
#include "bsp_WifiStoreLfs.h"
#include "bsp_WifiStore.h"
#include "bsp_lfs_port.h"
#include "usart.h"

#include <string.h>

#define WIFI_LFS_PATH    "/wifi.conf"

/* 与 bsp_WifiStore.c 相同的加和校验,保持二进制兼容 */
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

    err = lfs_file_open(lfs, &file, WIFI_LFS_PATH, LFS_O_RDONLY);
    if (err != LFS_ERR_OK)
    {
        return 0U;                   /* 文件不存在(首次)或挂载失败 */
    }

    n = lfs_file_read(lfs, &file, out, sizeof(*out));
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

    /* 先组帧到本地缓冲,校验通过后再写文件 */
    WifiCred_t buf;
    memset(&buf, 0, sizeof(buf));
    buf.magic = WIFI_FLASH_MAGIC;
    strncpy(buf.ssid, in->ssid, WIFI_SSID_LEN);
    strncpy(buf.pwd,  in->pwd,  WIFI_PWD_LEN);
    buf.checksum = WifiLfs_CalcChecksum(&buf);

    err = lfs_file_open(lfs, &file, WIFI_LFS_PATH,
                        LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
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
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[WIFI] lfs save sync fail err=%d\r\n", err);
        return 0U;
    }

    UART_Printf("[WIFI] credential saved to lfs:%s\r\n", WIFI_LFS_PATH);
    return 1U;
}

uint8_t WifiStoreLfs_Migrate(void)
{
    WifiCred_t legacy;
    WifiCred_t dummy;
    uint8_t    migrated = 0U;

    /* 文件已存在就不用迁 */
    if (WifiStoreLfs_Load(&dummy) != 0U)
    {
        return 0U;
    }

    /* 读内部 Flash sector11 */
    if (WifiStore_Load(&legacy) == 0U)
    {
        return 0U;                   /* sector11 为空或校验失败 */
    }

    /* 写进 LittleFS */
    if (WifiStoreLfs_Save(&legacy) == 0U)
    {
        UART_Printf("[WIFI] migrate: lfs write fail, keep legacy flash\r\n");
        return 0U;
    }

    /* 清掉 sector11 的 magic,防止下次又误读 */
    {
        WifiCred_t blank;
        memset(&blank, 0, sizeof(blank));
        if (WifiStore_Save(&blank) != 0U)
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
