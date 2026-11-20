/* bsp_OtaInternal.h
 * bsp_OTA 拆分后的内部接口与槽布局常量：仅 Bsp/OTA 下的 bsp_Ota*.c 使用。
 * 对外接口仍在 bsp_OTA.h（下载、检查、备份、读版本、复位）。
 */
#ifndef __BSP_OTA_INTERNAL_H
#define __BSP_OTA_INTERNAL_H

#include <stdint.h>
#include "bsp_W25Q128.h"
#include "bsp_OTA.h"


/* 槽 B 的镜像区从槽基址 + 4KB 开始，最大 444KB */
#define OTA_IMG_BASE    (W25Q128_FW_SLOT_B_ADDR + W25Q128_FW_HDR_SIZE)
#define OTA_IMG_MAX     W25Q128_FW_IMG_MAX

/* 取当前版本用的内部 Flash 元数据布局（与 GUI/BootLoader 保持一致）
 *  [0] magic  [1] 格式版本  [2] 固件版本(主<<8|次)  [3] 镜像字节数  [4] 整包 CRC32 */
#define OTA_META_ADDR     0x0800C000UL
#define OTA_META_MAGIC    0x4154454DUL  /*'M''E''T''A'*/
#define OTA_META_FMT      2UL           /*旧 v2 五字布局，只兼容读*/
#define OTA_META_FMT_REC  3UL           /*追加记录：带记录自身 CRC*/
#define OTA_META_SLOT     32UL          /*一条记录占 32 字节*/
#define OTA_META_SLOTS    (0x4000UL / OTA_META_SLOT)  /*sector 3 共 16KB，512 条*/
#define OTA_META_END      0x444E454DUL  /*'M''E''N''D'：镜像末尾标记*/
#define OTA_APP_BASE      0x08010000UL
#define OTA_APP_SIZE      0x00070000UL  /*448KB，与 BootLoader 的 APP_SIZE 同值*/


/* 槽 B 存储：页攒写、擦除、记录、CRC */
uint32_t OtaCrc32(uint32_t crc, const uint8_t *buf, uint32_t n);
void     OtaStore_Begin(void);                       /* 重置 CRC/页缓冲/已写字节 */
uint8_t  OtaStore_Feed(const uint8_t *data, uint16_t n);
uint8_t  OtaStore_Finish(uint32_t *written, uint32_t *crc);  /* 写尾部零页并回填统计 */
uint8_t  OtaStore_EraseSlot(void);
uint8_t  OtaStore_WriteRecord(uint32_t size, uint32_t crc, uint16_t ver);

/* HTTP 取 meta 与镜像 */
uint8_t OtaHttp_FetchMetaRetry(uint32_t *size, uint32_t *crc, uint16_t *ver);
uint8_t OtaHttp_FetchImage(uint32_t size, uint32_t expectCrc);

/* 槽 A 回滚基线：下载前确保备份有效 */
uint8_t OtaBackup_EnsureForDownload(void);


#endif /* __BSP_OTA_INTERNAL_H */
