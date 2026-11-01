/* bsp_WifiStore.h
 * WiFi 凭据内部 Flash 存储：sector11 单条记录，magic 0x57494649 + 32 位加和校验。
 * 硬件：0x080E0000 起 128KB，IROM 裁到 448KB 避开；记录 108B，checksum 在偏移 102。
 * 注意：保存要整扇区擦 128KB，CPU 停 1~2s，必须放宽 IWDG 窗口。
 */
#ifndef __BSP_WIFI_STORE_H
#define __BSP_WIFI_STORE_H

#include <stdint.h>
#include "WifiNet.h"


#define WIFI_FLASH_MAGIC   0x57494649U   /* 'WIFI',判定扇区里有没有写过凭据 */
#define WIFI_FLASH_BASE    0x080E0000U   /* sector 11 起始地址 */


/**
  * 功    能：从内部 Flash 读 WiFi 凭据。返回 1 有合法记录；
  * 参    数：out  输出，不能为空
  * 返 回 值：1=有合法记录，0=从未写过/校验失败(内容清零)
  */
uint8_t WifiStore_Load(WifiCred_t *out);

/**
  * 功    能：把 WiFi 凭据写进内部 Flash（擦除 + 写入 + 回读比对）。返回 1 成功、0 失败
  * 参    数：in  待保存凭据，不能为空
  * 返 回 值：1=成功，0=失败
  */
uint8_t WifiStore_Save(const WifiCred_t *in);


#endif /* __BSP_WIFI_STORE_H */
