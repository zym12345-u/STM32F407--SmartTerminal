/**
  ******************************************************************************
  * @file    bsp_lfs_port.h
  * @brief   LittleFS 块设备适配层 —— V1.2 里程碑 M3
  *
  *          分区约定(W25Q128 16MB):
  *            0x00000000 ~ 0x007FFFFF  LittleFS 数据区(2048 x 4KB = 8MB)
  *            0x00800000 ~ 0x008DFFFF  固件槽 A/B(各 448KB, 备份/OTA 用)
  *            0x008E0000 ~ 0x009FEFFF  预留
  *            0x009FF000 ~ 0x009FFFFF  boot 自测扇区(M1.5 SelfTest 专用, 不进文件系统)
  *            0x00A00000 ~ 0x00FFFFFF  GBK 字库区(M2 已验证存在, 驱动层写保护)
  ******************************************************************************
  */
#ifndef __BSP_LFS_PORT_H
#define __BSP_LFS_PORT_H

#include <stdint.h>
#include "lfs.h"

/** 挂载文件系统; 首次遇到全 0xFF 空白 Flash 时自动格式化。
 *  若 superblock 记录的块数与当前分区配置不符(分区缩容), 重建一次并重新挂载;
 *  其它错误(IO/损坏)直接返回失败, 不自动格式化。
 *  返回 1 = 成功, 0 = 失败 */
uint8_t LFS_Port_Mount(void);

/** 已挂载的文件系统实例(供后续 StorageTask 做文件读写) */
lfs_t *LFS_Port_Handle(void);

/** 已用块数(4KB/块), 失败返回 0xFFFFFFFF */
uint32_t LFS_Port_UsedBlocks(void);

#endif /* __BSP_LFS_PORT_H */
