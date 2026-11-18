/* GuiCommon.h
 * GUI 模块公共接口：颜色宏、样式对象、屏幕创建与轮询回调。
 * 拆分后各文件只通过本头文件交互，禁止 extern 其它文件的 static 控件。
 */
#ifndef __GUI_COMMON_H
#define __GUI_COMMON_H

#include <stdint.h>
#include "lvgl.h"

/*配色方案（RGB565）*/
#define COLOR_BG_TOP        lv_color_hex(0x1e293b)   /* 背景渐变顶 */
#define COLOR_BG_BOT        lv_color_hex(0x0f172a)   /* 背景渐变底 */
#define COLOR_CARD          lv_color_hex(0xf1f5f9)   /* 卡片底色(近白,高对比) */
#define COLOR_CARD_BORDER   lv_color_hex(0x94a3b8)   /* 卡片边框 */
#define COLOR_TEXT          lv_color_hex(0x0f172a)   /* 主文字(近黑,白卡上极清晰) */
#define COLOR_TEXT_MUTED    lv_color_hex(0x475569)   /* 次要文字(中灰) */
#define COLOR_CARD_TEXT_DARK lv_color_hex(0x334155)  /* 卡片标签文字 */

#define COLOR_ACCENT_GREEN  lv_color_hex(0x22c55e)   /* 绿灯强调色 */
#define COLOR_ACCENT_GREEN_D lv_color_hex(0x15803d)  /* 绿灯深色 */
#define COLOR_ACCENT_BLUE   lv_color_hex(0x3b82f6)   /* 蓝灯强调色 */
#define COLOR_ACCENT_BLUE_D lv_color_hex(0x1d4ed8)   /* 蓝灯深色 */
#define COLOR_ACCENT_TEMP   lv_color_hex(0xf97316)   /* 温度色 */
#define COLOR_ACCENT_HUMI   lv_color_hex(0x06b6d4)   /* 湿度色 */
#define COLOR_ACCENT_LIGHT  lv_color_hex(0xeab308)   /* 光照色 */

/* 可复用样式对象：由 GuiTask.c 初始化，其他 GUI 文件只读使用 */
typedef struct
{
  lv_style_t bg;        /* 全屏渐变背景 */
  lv_style_t card;      /* 传感器卡片 */
  lv_style_t cardLabel; /* 卡片数值文字 */
  lv_style_t cardSub;   /* 卡片标签文字 */
  lv_style_t btnGreen;  /* 绿灯按钮 */
  lv_style_t btnBlue;   /* 蓝灯按钮 */
  lv_style_t btnWifi;   /* WiFi 按钮 */
  lv_style_t status;    /* 状态栏文字 */
  lv_style_t accentBar; /* 卡片左侧色条 */
  lv_style_t swBg;      /* 开关背景(关) */
  lv_style_t swIndOn;   /* 开关指示器(开) */
  lv_style_t swKnob;    /* 开关旋钮 */
} GuiStyles_t;

GuiStyles_t *Gui_Styles(void);

void Gui_ShowMainScr(void);
void GUI_GetFwVersion(uint8_t *major, uint8_t *minor, uint8_t *fromMeta);

void GUI_CreateMainScreen(void);
void GUI_CreateWifiScreen(void);
void GUI_CreateOtaScreen(void);
void GUI_CreateWifiEntry(lv_obj_t *scr);
void GUI_CreateUpdateEntry(lv_obj_t *scr);

void GUI_RefreshMainOtaEntry(void);

void SensorRefreshTimerCb(lv_timer_t *timer);
void WifiEventTimerCb(lv_timer_t *timer);
void OtaStatusTimerCb(lv_timer_t *timer);

#endif /* __GUI_COMMON_H */
