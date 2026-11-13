/* fatfs.h
 * FatFS 挂载入口，由 SD 驱动链接到卷 "0:/"。
 */
#ifndef __FATFS_APP_H
#define __FATFS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* 卷路径，内容为 "0:/"，由 FATFS_LinkDriver 填写 */
extern char SD_Path[4];

/* 把 SD_Driver 注册为 0 号盘；重复调用无副作用 */
void MX_FATFS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FATFS_APP_H */
