/* bsp_WifiStoreLfs.h
 * WiFi 凭据 V2：存 W25Q128 上 LittleFS 的 /wifi.conf，二进制单条 WifiCred_t。
 * 硬件：SPI1 = PA5/PA6/PA7 + CS=PC13，经 bsp_lfs_port 访问。
 * 注意：只把旧 sector11 迁移一次，成功即清 magic，之后只走 LittleFS。
 */
#ifndef __BSP_WIFI_STORE_LFS_H
#define __BSP_WIFI_STORE_LFS_H

#include <stdint.h>
#include "WifiNet.h"

/**
  * 函    数：WifiStoreLfs_Load
  * 参    数：out  输出，不能为空，失败时清零
  * 返 回 值：1=成功，0=不存在/校验失败
  */
uint8_t WifiStoreLfs_Load(WifiCred_t *out);

/**
  * 函    数：WifiStoreLfs_Save
  * 参    数：in  待保存凭据，不能为空
  * 返 回 值：1=成功，0=失败
  */
uint8_t WifiStoreLfs_Save(const WifiCred_t *in);

/**
  * 函    数：WifiStoreLfs_Migrate
  * 参    数：无
  * 返 回 值：1=本次发生迁移，0=无需迁移(文件已存在或 sector11 为空)
  */
uint8_t WifiStoreLfs_Migrate(void);

#endif /* __BSP_WIFI_STORE_LFS_H */
