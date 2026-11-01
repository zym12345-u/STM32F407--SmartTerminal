/* bsp_WifiStoreLfs.h
 * WiFi 凭据 V2：存 W25Q128 上 LittleFS 的 /wifi.conf，二进制单条 WifiCred_t。
 * 硬件：SPI1 = PA5/PA6/PA7 + CS=PC13，经 bsp_lfs_port 访问。
 * 注意：只把旧 sector11 迁移一次，成功即清 magic，之后只走 LittleFS。
 */
#ifndef __BSP_WIFI_STORE_LFS_H
#define __BSP_WIFI_STORE_LFS_H

#include <stdint.h>
#include "WifiNet.h"

/* 从 LittleFS 的文件里读 WiFi 凭据。返回 1 成功，0 文件不存在或校验不过（输出会清零）。 */
uint8_t WifiStoreLfs_Load(WifiCred_t *out);

/* 把 WiFi 凭据写进 LittleFS 文件。返回 1 成功、0 失败。 */
uint8_t WifiStoreLfs_Save(const WifiCred_t *in);

/* 把老版本存在 Flash 扇区里的凭据搬到 LittleFS，只搬一次。
 * 返回 1 表示这次真搬了；0 表示不用搬（文件已存在或旧区是空的）。 */
uint8_t WifiStoreLfs_Migrate(void);

#endif /* __BSP_WIFI_STORE_LFS_H */
