/* bsp_SDCard.h
 * TF/SD 卡 BSP 封装：初始化 SDIO、挂载 FatFS、查询容量。
 *
 * 典型用法（在任务或初始化流程里显式调用，不会自动执行）：
 *     if (SDCard_Init() == SDCARD_OK) {
 *         f_open(&fp, "0:/test.txt", FA_WRITE | FA_CREATE_ALWAYS);
 *     }
 * 没有插卡时 SDCard_Init() 返回 SDCARD_ERR_NO_CARD，不影响其他功能。
 */
#ifndef __BSP_SDCARD_H
#define __BSP_SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 返回值 */
#define SDCARD_OK             0U   /* 初始化并挂载成功 */
#define SDCARD_ERR_NO_CARD    1U   /* SDIO 已初始化但没识别到卡 */
#define SDCARD_ERR_MOUNT      2U   /* 卡在，但 FatFS 挂载失败(未格式化/文件系统异常) */

/**
  * 功    能：初始化 SDIO 并挂载 FatFS 卷 "0:/"
  * 参    数：无
  * 返 回 值：SDCARD_* 状态码
  * 补    充：可重复调用；成功后再次调用直接返回 SDCARD_OK
  */
uint8_t SDCard_Init(void);

/**
  * 功    能：卸载文件系统并反初始化 SDIO
  * 参    数：无
  * 返 回 值：无
  */
void SDCard_DeInit(void);

/**
  * 功    能：查询卡当前是否可用
  * 参    数：无
  * 返 回 值：1=可用，0=不可用
  */
uint8_t SDCard_IsReady(void);

/**
  * 功    能：读取卡容量
  * 参    数：无
  * 返 回 值：容量 KB；未就绪返回 0
  */
uint32_t SDCard_GetCapacityKB(void);

/**
  * 功    能：取最近一次挂载的 FatFS 原始返回码，便于区分"没卡/没格式化/读失败"
  * 参    数：无
  * 返 回 值：FRESULT，0=FR_OK；未挂载过返回 0xFF
  */
uint8_t SDCard_GetMountResult(void);

/**
  * 功    能：取当前实际使用的 SDIO 总线宽度
  * 参    数：无
  * 返 回 值：1 或 4
  */
uint8_t SDCard_GetBusWidth(void);

/**
  * 功    能：4-bit 兼容性诊断，打印 SDIO 寄存器并尝试恢复 1-bit
  * 参    数：无
  * 返 回 值：1=4-bit 可读写，0=失败
  * 补    充：仅排查用，正常流程不要调用
  */
uint8_t SDCard_Diag4Bit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SDCARD_H */
