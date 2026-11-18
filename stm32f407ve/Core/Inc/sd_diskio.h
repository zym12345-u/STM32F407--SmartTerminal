/* sd_diskio.h
 * FatFS 与 SDIO 之间的驱动桥接层，供 ff_gen_drv 链接。
 */
#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ff_gen_drv.h"

/* 挂到 FatFS 的 SD 驱动实例 */
extern Diskio_drvTypeDef SD_Driver;

#ifdef __cplusplus
}
#endif

#endif /* __SD_DISKIO_H */
