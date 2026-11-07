/* bsp_bootflag.h
 * OTA-2 启动标志（24C02 页 0xE8）：App 请求 BootLoader 搬运槽 B，
 * 以及新固件启动后向 BootLoader 确认“我跑起来了”。
 *
 * 页布局与 BootLoader/port/bsp_ee.c 的 ota2_state_read/write 必须完全一致：
 *   0~3 magic 'OTA2' | 4 state | 5 未确认次数 | 6~7 保留
 */
#ifndef __BSP_BOOTFLAG_H
#define __BSP_BOOTFLAG_H

#include <stdint.h>

#define BOOTFLAG_MEM        0xE8U
#define BOOTFLAG_MAGIC      0x3241544FUL   /*'O''T''A''2'*/
#define BOOTFLAG_ST_APPLY   1U             /*请求搬运槽 B*/
#define BOOTFLAG_ST_CONFIRM 2U             /*已搬运，等待 App 确认*/

/**
  * 功    能：写“请求搬运槽 B”标志（同时把未确认次数清 0）
  * 参    数：无
  * 返 回 值：0 = 成功；1 = EEPROM 写失败
  * 补    充：OTA 下载成功后调用；下次复位由 BootLoader 搬运
  */
uint8_t BootFlag_RequestApply(void);

/**
  * 功    能：向 BootLoader 确认本次启动成功
  * 参    数：无
  * 返 回 值：2 = 已清掉待确认标志；1 = 无需确认；0 = EEPROM 读写失败
  * 补    充：只有 state=CONFIRM 时才写，普通启动读一次就返回，不产生写磨损
  */
uint8_t BootFlag_ConfirmBoot(void);

/**
  * 功    能：清整页标志
  * 参    数：无
  * 返 回 值：无
  * 补    充：串口 IAP 前调用，避免“串口升级”和“槽 B 搬运”两个请求叠加
  */
void BootFlag_Clear(void);

#endif /* __BSP_BOOTFLAG_H */
