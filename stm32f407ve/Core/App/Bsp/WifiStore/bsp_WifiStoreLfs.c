/**
  ******************************************************************************
  * @file    bsp_WifiStoreLfs.c
 * @brief   WiFi 凭据的 LittleFS 存储实现(V1.2 M5)
  *
 *          与旧方案(内部 Flash sector11)的对比:
 *            旧: 擦除 128KB 扇区, CPU 停 1~2s, 必须 IWDG_SetReload(500) 放宽窗口;
 *            新: LittleFS 只动 1~2 个 4KB 块(busy 约 33ms), RTOS 正常运行,
 *                不必碰 IWDG 窗口, 且自带磨损均衡与掉电恢复
  *
 *          调用时机: 只在 WifiTask 启动阶段执行一次, 且必须在 LittleFS 挂载之后。
 *                    注意 LFS_Port_Handle() 恒返回 &s_lfs(永不为 NULL),
 *                    所以这里的 NULL 检查挡不住"未挂载"——未挂载时是
 *                    lfs_file_open 直接失败, 迁移同样静默跳过
  ******************************************************************************
  */
#include "bsp_WifiStoreLfs.h"
#include "bsp_WifiStore.h"
#include "bsp_lfs_port.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include <string.h>

#define WIFI_LFS_PATH    "/wifi.conf"

/* 校验和算法与 bsp_WifiStore.c 保持一致, 便于迁移时交叉比对 */
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
        return 0U;                   /* 文件不存在或读取失败(首次开机属正常) */
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

    /* 先在本地缓冲里填好 magic 与校验和, 再整块写入 ——
     * 避免直接写 out 导致写一半被打断留下脏数据 */
    WifiCred_t buf;
    memset(&buf, 0, sizeof(buf));
    buf.magic = WIFI_FLASH_MAGIC;
    strncpy(buf.ssid, in->ssid, WIFI_SSID_LEN);
    strncpy(buf.pwd,  in->pwd,  WIFI_PWD_LEN);
    buf.checksum = WifiLfs_CalcChecksum(&buf);

    /* 心跳保险: LittleFS 的 erase+prog 可能跨多个 4KB 块, 加上元数据更新,
     * 最坏情况可能接近秒级。WiFi 任务判死线 3s, 这里入口报一次。 */
    Watchdog_Heartbeat(WDT_TASK_WIFI);

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

    Watchdog_Heartbeat(WDT_TASK_WIFI);    /* 写完再报一次 */

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
