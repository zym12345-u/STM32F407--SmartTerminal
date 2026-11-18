/* GuiOtaInternal.h
 * OTA 页拆分后的内部接口：仅 Gui/GuiOta*.c 之间使用，对外接口仍在 GuiCommon.h。
 */
#ifndef __GUI_OTA_INTERNAL_H
#define __GUI_OTA_INTERNAL_H

#include <stdint.h>
#include "lvgl.h"


/* GuiOtaSerial.c */
uint8_t GuiOta_IsBusy(void);                        /* 1=W25Q 正被 OTA 占用 */
uint8_t GuiOta_EnsureSlotABackup(lv_obj_t *lbl);    /* 1=槽 A 可用，0=备份失败 */
void    GuiOta_SerialUpdate(lv_obj_t *lbl);         /* 串口 IAP：备份后写标志并复位 */

/* GuiOtaScreen.c */
void GuiOta_EnterPage(lv_event_t *event);           /* 主屏 Update 按钮 -> 进页并发起检查 */


#endif /* __GUI_OTA_INTERNAL_H */
