/* GuiOtaScreen.c
 * OTA 下载页：页面搭建、进页检查、返回与串口升级按钮。
 * 状态轮询/弹窗/下载流程见 GuiOtaStatus.c，串口升级与槽 A 备份见 GuiOtaSerial.c，
 * 主屏入口见 GuiOtaEntry.c。所有 lv_* 只在 GuiTask 线程调用。
 */

#include "lvgl.h"
#include "bsp_font_gbk.h"
#include "bsp_OtaCfg.h"
#include "WifiNet.h"
#include "GuiCommon.h"
#include "GuiOtaText.h"
#include "GuiOtaInternal.h"
#include <stdio.h>

/* OTA 页控件与显示缓存：本文件负责填，GuiOtaStatus.c 负责刷新 */
GuiOtaUi_t g_guiOtaUi;

static lv_obj_t *s_lblOtaServer;

/**
  * 功    能：把当前生效的服务器地址刷到 OTA 页（读缓存，不碰 LittleFS）
  * 参    数：无
  * 返 回 值：无
  */
static void GUI_UpdateOtaServer(void)
{
  OtaServerCfg_t cfg;
  char txt[80];

  OTA_Config_Get(&cfg);
  snprintf(txt, sizeof(txt), "Server  %s : %u", cfg.host, (unsigned)cfg.port);
  lv_label_set_text(s_lblOtaServer, txt);
}

/**
  * 功    能：OTA 页标题栏的 Serial 按钮：先弹确认窗，防误触
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：按下会备份并复位，屏幕会黑 3~5 秒（BootLoader 不初始化背光），
  *            所以必须让用户明确确认；备份进度与失败原因仍打在状态标签上
  */
static void SerialUpdateEventCb(lv_event_t *event)
{
  (void)event;

  if (GuiOta_IsBusy() != 0U)
  {
    lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_BUSY);
    return;
  }

  GuiOta_ShowMsg(TXT_SERIAL_CONFIRM, 1U, TXT_SERIAL_OK, GUI_OTA_ACT_SERIAL);
}

/*════════════ WiFi OTA 下载页（第一阶段：只下载到槽 B）════════════
 * 为什么不复用弹窗：下载要几十秒，必须能看到“擦除 / 取 meta / 下载 / 校验 / 失败原因”；
 * 为什么退出页面还继续下载：下载由 WifiTask 执行，界面只是观察者，关页面不该中断传输。
 */

/**
  * 功    能：进入 OTA 页：刷新服务器地址/当前版本，并自动发起一次手动检查
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：用户只按一个 Update，不区分"检查"和"下载"；
  *            检查结果由 150ms 轮询在 OtaStatusTimerCb 里弹窗，
  *            这里只负责投命令并把 g_guiOtaUi.flow 立起来
  */
void GuiOta_EnterPage(lv_event_t *event)
{
  uint8_t fromMeta;
  char    txt[64];

  (void)event;
  GUI_UpdateOtaServer();
  GUI_GetFwVersion(&g_guiOtaUi.curMajor, &g_guiOtaUi.curMinor, &fromMeta);
  (void)fromMeta;
  snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
           (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor);
  lv_label_set_text(g_guiOtaUi.lblVer, txt);
  g_guiOtaUi.uiDirty = 1U;
  lv_scr_load_anim(g_guiOtaUi.scr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);

  /*下载占用 W25Q 期间不做检查：两段 TCP 收包会互相踩*/
  if (GuiOta_IsBusy() != 0U)
  {
    return;
  }
  if (Wifi_IsConnected() == 0U)
  {
    lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_NOWIFI);
    GuiOta_ShowMsg(TXT_NO_WIFI, 0U, NULL, GUI_OTA_ACT_NONE);
    return;
  }

  g_guiOtaUi.flow         = GUI_OTA_FLOW_CHECK;
  g_guiOtaUi.checkSeen    = 0U;
  g_guiOtaUi.failNotified = 0U;   /*重新检查就把上次的失败提示清掉*/
  g_guiOtaUi.checkT0      = lv_tick_get();
  Wifi_PostCheckOta(1U);
  lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_CHECKING);
}

/**
  * 功    能：OTA 页返回主屏
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：不取消下载；WifiTask 仍在跑，回来还能看到进度。
  *            离开页面要清掉本页的检查流程，否则结果会在主屏上弹窗
  */
static void OtaBackEventCb(lv_event_t *event)
{
  (void)event;
  g_guiOtaUi.flow = GUI_OTA_FLOW_NONE;
  GuiOta_CloseMsg();
  Gui_ShowMainScr();
}

/**
  * 功    能：搭 WiFi OTA 页：服务器信息、当前/目标版本、阶段、进度条、下载/返回
  * 参    数：无
  * 返 回 值：无
  * 补    充：控件全用现有样式；卡片高 96 是为了让失败原因能折两行不压进度条
  */
void GUI_CreateOtaScreen(void)
{
  lv_obj_t *lbl;
  lv_obj_t *card;
  char      txt[80];
  uint8_t   fromMeta;
  lv_obj_t *btn;

  /*建页时就取一次当前版本，避免刚进页面显示 v0.0*/
  GUI_GetFwVersion(&g_guiOtaUi.curMajor, &g_guiOtaUi.curMinor, &fromMeta);
  (void)fromMeta;

  g_guiOtaUi.scr = lv_obj_create(NULL);
  lv_obj_remove_style_all(g_guiOtaUi.scr);
  lv_obj_add_style(g_guiOtaUi.scr, &Gui_Styles()->bg, 0);

  lbl = lv_label_create(g_guiOtaUi.scr);
  lv_label_set_text(lbl, "WiFi OTA");
  lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(lbl, 8, 6);

  g_guiOtaUi.lblWifi = lv_label_create(g_guiOtaUi.scr);
  lv_label_set_text(g_guiOtaUi.lblWifi, "WiFi: --");
  lv_obj_add_style(g_guiOtaUi.lblWifi, &Gui_Styles()->status, 0);
  lv_obj_set_pos(g_guiOtaUi.lblWifi, 110, 8);

  card = lv_obj_create(g_guiOtaUi.scr);
  lv_obj_remove_style_all(card);
  lv_obj_add_style(card, &Gui_Styles()->card, 0);
  lv_obj_set_size(card, 304, 96);
  lv_obj_set_pos(card, 8, 28);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(card, 8, 0);

  s_lblOtaServer = lv_label_create(card);
  lv_label_set_text(s_lblOtaServer, "Server  --");
  lv_obj_add_style(s_lblOtaServer, &Gui_Styles()->cardSub, 0);
  lv_obj_set_pos(s_lblOtaServer, 0, 0);
  GUI_UpdateOtaServer();   /*建页时先按缓存填一次，Init 跑完/改配置后进页再刷*/

  g_guiOtaUi.lblVer = lv_label_create(card);
  snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
           (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor);
  lv_label_set_text(g_guiOtaUi.lblVer, txt);
  lv_obj_add_style(g_guiOtaUi.lblVer, &Gui_Styles()->cardSub, 0);
  lv_obj_set_pos(g_guiOtaUi.lblVer, 0, 20);

  g_guiOtaUi.lblState = lv_label_create(card);
  lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_IDLE);
  lv_label_set_long_mode(g_guiOtaUi.lblState, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(g_guiOtaUi.lblState, 288);
  lv_obj_set_style_text_color(g_guiOtaUi.lblState, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(g_guiOtaUi.lblState, BSP_Font_Gbk16(), 0);
  lv_obj_set_pos(g_guiOtaUi.lblState, 0, 42);

  /*进度条：轨道深色、指示蓝色；不用默认样式，避免主题自动加渐变/边框*/
  g_guiOtaUi.bar = lv_bar_create(g_guiOtaUi.scr);
  lv_obj_set_size(g_guiOtaUi.bar, 304, 14);
  lv_obj_set_pos(g_guiOtaUi.bar, 8, 130);
  lv_bar_set_range(g_guiOtaUi.bar, 0, 100);
  lv_obj_set_style_bg_opa(g_guiOtaUi.bar, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(g_guiOtaUi.bar, lv_color_hex(0x334155), 0);
  lv_obj_set_style_border_width(g_guiOtaUi.bar, 0, 0);
  lv_obj_set_style_radius(g_guiOtaUi.bar, 7, 0);
  lv_obj_set_style_bg_opa(g_guiOtaUi.bar, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(g_guiOtaUi.bar, COLOR_ACCENT_BLUE, LV_PART_INDICATOR);
  lv_obj_set_style_radius(g_guiOtaUi.bar, 7, LV_PART_INDICATOR);
  lv_bar_set_value(g_guiOtaUi.bar, 0, LV_ANIM_OFF);

  g_guiOtaUi.lblDetail = lv_label_create(g_guiOtaUi.scr);
  lv_label_set_text(g_guiOtaUi.lblDetail, "0 KB / 0 KB");
  lv_obj_add_style(g_guiOtaUi.lblDetail, &Gui_Styles()->status, 0);
  lv_obj_set_pos(g_guiOtaUi.lblDetail, 8, 148);

  lbl = lv_label_create(g_guiOtaUi.scr);
  lv_label_set_text(lbl, "PC: python Tools/ota_serve.py app_image.bin");
  lv_obj_add_style(lbl, &Gui_Styles()->status, 0);
  lv_obj_set_pos(lbl, 8, 166);

  lbl = lv_label_create(g_guiOtaUi.scr);
  lv_label_set_text(lbl, "Open = auto check, then confirm to install");
  lv_obj_add_style(lbl, &Gui_Styles()->status, 0);
  lv_obj_set_pos(lbl, 8, 182);

  /*Serial：串口 IAP 的老入口，进 OTA 页后按它才备份 + 复位进 3s 窗口*/
  btn = lv_btn_create(g_guiOtaUi.scr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &Gui_Styles()->btnBlue, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 64, 22);
  lv_obj_set_pos(btn, 248, 4);
  lv_obj_add_event_cb(btn, SerialUpdateEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Serial");
  lv_obj_center(lbl);

  btn = lv_btn_create(g_guiOtaUi.scr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &Gui_Styles()->btnWifi, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 100, 32);
  lv_obj_set_pos(btn, 210, 202);
  lv_obj_add_event_cb(btn, OtaBackEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_LEFT " Back");
  lv_obj_center(lbl);
}
