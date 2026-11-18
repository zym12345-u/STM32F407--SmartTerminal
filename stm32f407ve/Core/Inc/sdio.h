/* sdio.h
 * SDIO 底层初始化接口：4-bit 模式，轮询收发（暂不使用 DMA）。
 */
#ifndef __SDIO_H
#define __SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SDIO 句柄，由 sd_diskio.c 直接引用 */
extern SD_HandleTypeDef hsd;

/* 初始化 SDIO 外设并识别卡；失败时 hsd.State 为 ERROR/TIMEOUT，不阻塞调用方 */
void MX_SDIO_SD_Init(void);

/* 按指定总线宽度/分频初始化，供上层做 4-bit→1-bit 回退；
 * busWide 取 SDIO_BUS_WIDE_1B/4B，clockDiv 见 SDIO_CK = PLL48CLK/(2*div) */
void MX_SDIO_SD_InitEx(uint32_t busWide, uint32_t clockDiv);

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H */
