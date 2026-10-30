/**
  ******************************************************************************
  * @file    bsp_lfs_port.h
  * @brief   LittleFS 块设备适配层 —— V1.2 里程碑 M3
  *
  *          分区约定(W25Q128 16MB):
  *            0x00000000 ~ 0x009FEFFF  LittleFS 数据区(2559 x 4KB ≈ 9.99MB)
  *            0x009FF000 ~ 0x009FFFFF  boot 自测扇区(M1.5 SelfTest 专用, 不进文件系统)
  *            0x00A00000 ~ 0x00FFFFFF  GBK 字库区(M2 已验证存在, 驱动层写保护)
  ******************************************************************************
  */
#ifndef __BSP_LFS_PORT_H
#define __BSP_LFS_PORT_H

#include <stdint.h>
#include "lfs.h"

/** 挂载文件系统; 首次遇到全 0xFF 空白 Flash 时自动格式化。
 *  返回 1 = 成功, 0 = 失败(失败绝不自动格式化, 避免瞬时 IO 错误抹掉数据) */
uint8_t LFS_Port_Mount(void);

/** 已挂载的文件系统实例(供后续 StorageTask 做文件读写) */
lfs_t *LFS_Port_Handle(void);

/** 已用块数(4KB/块), 失败返回 0xFFFFFFFF */
uint32_t LFS_Port_UsedBlocks(void);

#endif /* __BSP_LFS_PORT_H */
