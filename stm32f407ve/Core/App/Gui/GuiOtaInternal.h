/* GuiOtaInternal.h
 * OTA 页拆分后的内部接口：仅 Gui/GuiOta*.c 之间使用，对外接口仍在 GuiCommon.h。
 */
#ifndef __GUI_OTA_INTERNAL_H
#define __GUI_OTA_INTERNAL_H

#include <stdint.h>
#include "lvgl.h"


/* 页面级流程状态：只有本页发起的检查/下载才弹窗 */
#define GUI_OTA_FLOW_NONE     0U
#define GUI_OTA_FLOW_CHECK    1U
#define GUI_OTA_FLOW_DOWNLOAD 2U

/* 弹窗确认键的动作 */
#define GUI_OTA_ACT_NONE     0U
#define GUI_OTA_ACT_DOWNLOAD 1U
#define GUI_OTA_ACT_SERIAL   2U

/* OTA 页控件与显示缓存：GuiOtaScreen.c 负责填，GuiOtaStatus.c 负责读和刷新 */
typedef struct
{
    lv_obj_t *scr;          /* 页面根对象 */
    lv_obj_t *lblWifi;      /* 标题栏 WiFi 状态 */
    lv_obj_t *lblVer;       /* 当前/目标版本 */
    lv_obj_t *lblState;     /* 阶段/结果文案 */
    lv_obj_t *lblDetail;    /* 已收 KB / 总 KB */
    lv_obj_t *bar;          /* 进度条 */
    uint8_t   curMajor;     /* 当前固件主版本 */
    uint8_t   curMinor;     /* 当前固件次版本 */
    uint8_t   uiDirty;      /* 1=进页后强制重画一次 */
    uint8_t   flow;         /* GUI_OTA_FLOW_* */
    uint8_t   checkSeen;    /* 是否看到过 CHECKING */
    uint8_t   failNotified; /* 下载失败已弹过窗 */
    uint32_t  checkT0;      /* 发起检查的时刻，4s 兜底用 */
} GuiOtaUi_t;

extern GuiOtaUi_t g_guiOtaUi;


/* GuiOtaSerial.c */
uint8_t GuiOta_IsBusy(void);                        /* 1=W25Q 正被 OTA 占用 */
uint8_t GuiOta_EnsureSlotABackup(lv_obj_t *lbl);    /* 1=槽 A 可用，0=备份失败 */
void    GuiOta_SerialUpdate(lv_obj_t *lbl);         /* 串口 IAP：备份后写标志并复位 */

/* GuiOtaScreen.c */
void GuiOta_EnterPage(lv_event_t *event);           /* 主屏 Update 按钮 -> 进页并发起检查 */

/* GuiOtaStatus.c */
void GuiOta_ShowMsg(const char *txt, uint8_t confirm, const char *okText, uint8_t action);
void GuiOta_CloseMsg(void);


#endif /* __GUI_OTA_INTERNAL_H */
