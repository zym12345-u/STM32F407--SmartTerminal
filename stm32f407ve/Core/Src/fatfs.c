/* fatfs.c
 * FatFS 卷注册：只把 SD 驱动挂到 0 号盘，不做实际读写。
 */
#include "fatfs.h"

char SD_Path[4];

/**
  * 功    能：注册 SD 驱动为默认卷，重复调用直接返回
  * 参    数：无
  * 返 回 值：无
  */
void MX_FATFS_Init(void)
{
    static uint8_t linked = 0U;

    if (linked != 0U)
    {
        return;
    }
    if (FATFS_LinkDriver(&SD_Driver, SD_Path) == 0U)
    {
        linked = 1U;
    }
}
