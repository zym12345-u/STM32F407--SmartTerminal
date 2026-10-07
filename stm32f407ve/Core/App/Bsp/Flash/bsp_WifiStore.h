/**
  ******************************************************************************
  * @file    bsp_WifiStore.h
  * @brief   WiFi 凭据掉电存储:写入 STM32F407 内部 Flash 最后一个扇区
  *
  *          F407VET6 Flash 共 512KB,扇区 11 位于 0x080E0000、大小 128KB。
  *          Keil 工程 IROM 已裁成 448KB(0x08000000~0x080DFFFF),程序不会
  *          与本扇区冲突。整个扇区只存一个 WifiCred_t 结构。
  *
  *          注意:擦除 128KB 扇区期间 F4 内核会停滞约 1~2 秒(取指暂停),
  *          全部任务冻结;因此保存动作安排在"连接成功后"执行,界面停在
  *          Saving 状态,用户无感等待,保存完再进入走时。
  ******************************************************************************
  */
#ifndef __BSP_WIFI_STORE_H
#define __BSP_WIFI_STORE_H

#include <stdint.h>
#include "WifiNet.h"


#define WIFI_FLASH_MAGIC   0x57494649U   /* 'WIFI',判定扇区里有没有写过凭据 */
#define WIFI_FLASH_BASE    0x080E0000U   /* sector 11 起始地址 */


/* 读取凭据:有合法记录返回 1;从未写过/校验失败返回 0(out 内容清零) */
uint8_t WifiStore_Load(WifiCred_t *out);

/* 擦除扇区并写入新凭据:成功返回 1,失败返回 0 */
uint8_t WifiStore_Save(const WifiCred_t *in);


#endif /* __BSP_WIFI_STORE_H */
