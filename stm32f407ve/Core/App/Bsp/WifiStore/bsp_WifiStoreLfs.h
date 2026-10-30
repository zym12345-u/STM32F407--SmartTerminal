/**
  ******************************************************************************
  * @file    bsp_WifiStoreLfs.h
  * @brief   WiFi 凭据存储 V2 —— 从内部 Flash 迁到 W25Q128 上的 LittleFS
  *
  *          为什么迁:
  *            1. 内部 Flash 的 sector11(0x080E0000)是"1MB 硅片按 512KB 降级打标"
  *               的物理灰色地带,容量寄存器不担保,长期放凭据有风险;
  *            2. 每次保存要擦 128KB 扇区,CPU 停滞 1~2s,必须放宽 IWDG 窗口,
  *               全任务冻结,代价大;
  *            3. LittleFS 自带磨损均衡 + 掉电安全,适合小数据频繁更新。
  *
  *          迁移策略(一次性):
  *            开机 Load 时先看 /wifi.conf,没有再去读内部 Flash sector11;
  *            读到合法凭据就写进 /wifi.conf 并把 sector11 清零(magic=0),
  *            之后只走 LittleFS。整个过程只发生一次,用户无感。
  *
  *          文件格式: 二进制,直接存 WifiCred_t 结构(102B),与内部 Flash 相同,
  *                    便于调试时直接对比。
  ******************************************************************************
  */
#ifndef __BSP_WIFI_STORE_LFS_H
#define __BSP_WIFI_STORE_LFS_H

#include <stdint.h>
#include "WifiNet.h"

/** 从 LittleFS 读取凭据:成功返回 1;不存在/校验失败返回 0(out 清零) */
uint8_t WifiStoreLfs_Load(WifiCred_t *out);

/** 把凭据写入 LittleFS:成功返回 1,失败返回 0 */
uint8_t WifiStoreLfs_Save(const WifiCred_t *in);

/** 一次性迁移:内部 Flash sector11 -> LittleFS /wifi.conf。
 *  返回 1 = 发生了一次迁移, 0 = 无需迁移(文件已存在或 sector11 为空) */
uint8_t WifiStoreLfs_Migrate(void);

#endif /* __BSP_WIFI_STORE_LFS_H */
