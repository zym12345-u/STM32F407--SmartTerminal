/* GuiOtaStatus.c
 * OTA 状态轮询、确认弹窗与下载流程：读取 g_otaStatus 与 g_guiOtaUi，
 * 只在当前页为 OTA 页时刷新控件。所有 lv_* 只在 GuiTask 线程调用。
 */

#include "lvgl.h"
#include "bsp_font_gbk.h"
#include "OtaData.h"
#include "WifiNet.h"
#include "GuiCommon.h"
#include "GuiOtaText.h"
#include "GuiOtaInternal.h"
#include <stdio.h>
#include <string.h>

static lv_obj_t *s_msgbox;
static uint8_t   s_msgAct = GUI_OTA_ACT_DOWNLOAD;

static void GUI_StartOtaDownload(void);

/**
  * 功    能：OTA 错误码对应的中文提示（弹窗用）
  * 参    数：err OTA_ERR_* 错误码
  * 返 回 值：中文描述
  * 补    充：状态行仍用英文短句（宽度只够一行），弹窗给人看所以用中文；
  *            每行控制在 14 个汉字内，16px 字库下正好放进 256px 的卡片
  */
static const char *GUI_OtaErrTextCn(uint8_t err)
{
  switch (err)
  {
    case OTA_ERR_WIFI:    return E_WIFI;
    case OTA_ERR_FLASH:   return E_FLASH;
    case OTA_ERR_ERASE:   return E_ERASE;
    case OTA_ERR_BACKUP:  return E_BACKUP;
    case OTA_ERR_CONNECT: return E_CONNECT;
    case OTA_ERR_HTTP:    return E_HTTP;
    case OTA_ERR_META:    return E_META;
    case OTA_ERR_SIZE:    return E_SIZE;
    case OTA_ERR_TIMEOUT: return E_TIMEOUT;
    case OTA_ERR_CRC:     return E_CRC;
    case OTA_ERR_RECORD:  return E_RECORD;
    case OTA_ERR_QUEUE:   return E_QUEUE;
    default:              return E_UNKNOWN;
  }
}

/**
  * 功    能：关掉当前弹窗（没有就什么都不做）
  * 参    数：无
  * 返 回 值：无
  */
void GuiOta_CloseMsg(void)
{
  if (s_msgbox != NULL)
  {
    lv_obj_del(s_msgbox);
    s_msgbox = NULL;
  }
}

/**
  * 功    能：弹窗里的 Later：只关窗，不改任何状态
  * 参    数：event LVGL 事件
  * 返 回 值：无
  */
static void MsgLaterEventCb(lv_event_t *event)
{
  (void)event;
  GuiOta_CloseMsg();
}

/**
  * 功    能：弹窗确认键：按 s_msgAct 分派到「下载」或「串口升级」
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：备份必须排在下载之前——BootLoader 搬运前验不到槽 A 就会 apply refused
  */
static void MsgConfirmEventCb(lv_event_t *event)
{
  (void)event;
  GuiOta_CloseMsg();

  if (s_msgAct == GUI_OTA_ACT_SERIAL)
  {
    GuiOta_SerialUpdate(g_guiOtaUi.lblState);
  }
  else
  {
    GUI_StartOtaDownload();
  }
}

/**
  * 功    能：弹模态提示框：全屏遮罩 + 居中卡片
  * 参    数：txt 正文（中文，用外挂 GBK 16px 字库）；confirm 1=给两个键；
  *            okText 确认键文案（NULL 用「升级」）；action 确认键动作 GUI_OTA_ACT_*
  * 返 回 值：无
  * 补    充：不用 lv_msgbox：默认样式的按钮在 240 高的屏上偏大、长文案会被裁；
  *            这里复用 Gui_Styles()->card / Gui_Styles()->btnGreen / Gui_Styles()->btnBlue，观感和页面一致
  */
void GuiOta_ShowMsg(const char *txt, uint8_t confirm, const char *okText, uint8_t action)
{
  lv_obj_t *mask;
  lv_obj_t *box;
  lv_obj_t *lbl;
  lv_obj_t *btn;

  GuiOta_CloseMsg();
  s_msgAct = action;

  mask = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(mask);
  lv_obj_set_style_bg_color(mask, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(mask, LV_OPA_50, 0);
  lv_obj_set_size(mask, 320, 240);
  lv_obj_clear_flag(mask, LV_OBJ_FLAG_SCROLLABLE);

  box = lv_obj_create(mask);
  lv_obj_remove_style_all(box);
  lv_obj_add_style(box, &Gui_Styles()->card, 0);
  lv_obj_set_size(box, 280, (confirm != 0U) ? 150 : 118);
  lv_obj_align(box, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(box, 12, 0);

  lbl = lv_label_create(box);
  lv_label_set_text(lbl, txt);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl, 256);
  lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(lbl, BSP_Font_Gbk16(), 0);
  lv_obj_set_pos(lbl, 0, 0);

  btn = lv_btn_create(box);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, (confirm != 0U) ? &Gui_Styles()->btnGreen : &Gui_Styles()->btnBlue, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 118, 30);
  lv_obj_set_pos(btn, (confirm != 0U) ? 0 : 69, (confirm != 0U) ? 104 : 64);
  lv_obj_add_event_cb(btn, (confirm != 0U) ? MsgConfirmEventCb : MsgLaterEventCb,
                      LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, (confirm != 0U) ? ((okText != NULL) ? okText : TXT_UPDATE) : TXT_OK);
  lv_obj_set_style_text_font(lbl, BSP_Font_Gbk16(), 0);
  lv_obj_center(lbl);

  if (confirm != 0U)
  {
    btn = lv_btn_create(box);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &Gui_Styles()->btnBlue, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(btn, 118, 30);
    lv_obj_set_pos(btn, 138, 104);
    lv_obj_add_event_cb(btn, MsgLaterEventCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, TXT_LATER);
    lv_obj_set_style_text_font(lbl, BSP_Font_Gbk16(), 0);
    lv_obj_center(lbl);
  }

  s_msgbox = mask;
}

/**
  * 功    能：下载前置：确保槽 A 有当前版本备份，再投 OTA 命令
  * 参    数：无
  * 返 回 值：无
  * 补    充：备份失败就中止、不发下载命令——"下载成功却搬不进去"比不下载更糟；
  *            备份耗时 5~7 秒，进度直接打在页面状态行上
  */
static void GUI_StartOtaDownload(void)
{
  if (GuiOta_IsBusy() != 0U)
  {
    return;
  }

  lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_BACKUP);
  lv_refr_now(NULL);

  if (GuiOta_EnsureSlotABackup(g_guiOtaUi.lblState) == 0U)
  {
    lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_BK_FAIL);
    GuiOta_ShowMsg(TXT_BK_FAIL, 0U, NULL, GUI_OTA_ACT_NONE);
    return;
  }

  Wifi_PostOta();
  g_guiOtaUi.flow         = GUI_OTA_FLOW_DOWNLOAD;   /*失败时由页面轮询负责弹窗*/
  g_guiOtaUi.failNotified = 0U;
  lv_label_set_text(g_guiOtaUi.lblState, TXT_ST_SENT);
}

/**
  * 功    能：150ms 轮询 g_otaStatus，只在值变化时才刷控件
  * 参    数：timer LVGL 定时器
  * 返 回 值：无
  * 补    充：跨任务读 volatile 字段；每次只取一个字段做显示，不做复合判断，
  *            避免读到“上一阶段的 percent + 下一阶段的 state”这种撕裂组合
  */
void OtaStatusTimerCb(lv_timer_t *timer)
{
  static uint8_t  lastPct    = 0xFFU;
  static uint8_t  lastWifi   = 0xFFU;
  static uint32_t lastKb     = 0xFFFFFFFFUL;
  static uint16_t lastVer    = 0xFFFFU;
  static char     lastStateTxt[96] = {0};
  uint8_t  st;
  uint8_t  err;
  uint8_t  chk;
  uint8_t  chkErr;
  char     txt[80];
  char     stateTxt[96];
  uint32_t kb;
  uint16_t targetVer;

  (void)timer;
  if (lv_scr_act() != g_guiOtaUi.scr)
  {
    return;        /*离开页面就不刷；下载在 WifiTask 里继续 */
  }

  /*重新进入页面：把上次的缓存作废，强制整页重画一次*/
  if (g_guiOtaUi.uiDirty != 0U)
  {
    g_guiOtaUi.uiDirty = 0U;
    lastPct    = 0xFFU;
    lastWifi   = 0xFFU;
    lastKb     = 0xFFFFFFFFUL;
    lastVer    = 0xFFFFU;
    lastStateTxt[0] = '\0';
  }

  /*WiFi 状态只在变化时刷；用 s_connected 而不是配网页的状态机，否则扫描后会被误判*/
  {
    uint8_t wifiUp = Wifi_IsConnected();

    if (lastWifi != wifiUp)
    {
      lastWifi = wifiUp;
      lv_label_set_text(g_guiOtaUi.lblWifi,
                        (wifiUp != 0U) ? "WiFi: connected" : "WiFi: not connected");
    }
  }

  st     = g_otaStatus.state;
  err    = g_otaStatus.err;
  chk    = g_otaStatus.checkState;
  chkErr = g_otaStatus.checkErr;

  /*下载失败：只弹一次中文窗，之后状态行不再停在错误文案上（否则看着像卡死）。
   *成功不弹窗——状态行会显示 rebooting，复位本身就是反馈*/
  if (g_guiOtaUi.flow == GUI_OTA_FLOW_DOWNLOAD)
  {
    if (st == OTA_ST_FAIL)
    {
      g_guiOtaUi.flow         = GUI_OTA_FLOW_NONE;
      g_guiOtaUi.failNotified = 1U;
      GuiOta_ShowMsg(GUI_OtaErrTextCn(err), 0U, NULL, GUI_OTA_ACT_NONE);
    }
    else if (st == OTA_ST_DONE)
    {
      g_guiOtaUi.flow = GUI_OTA_FLOW_NONE;
    }
  }

  /*状态行优先级：下载进行中 > 检查进行中 > 下载结果 > 检查结果 > 空闲。
   *这样检查失败不会覆盖下载失败文案，下载失败也不会被一次检查成功冲掉。*/
  if ((st == OTA_ST_BACKUP) || (st == OTA_ST_ERASE) || (st == OTA_ST_META) ||
      (st == OTA_ST_DOWNLOAD) || (st == OTA_ST_VERIFY))
  {
    switch (st)
    {
      case OTA_ST_BACKUP:   (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_BACKUP);   break;
      case OTA_ST_ERASE:    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_ERASE);    break;
      case OTA_ST_META:     (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_META);     break;
      case OTA_ST_DOWNLOAD: (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_DOWNLOAD); break;
      default:              (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_VERIFY);   break;
    }
  }
  else if (chk == OTA_CHK_CHECKING)
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_CHECKING);
  }
  else if (st == OTA_ST_DONE)
  {
    /*rebootPending=1 才是真的会自动复位；标志写失败时停在 DONE 等用户手动复位*/
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s",
                   (g_otaStatus.rebootPending != 0U) ? TXT_ST_REBOOT : TXT_ST_MANUAL);
  }
  else if ((st == OTA_ST_FAIL) && (g_guiOtaUi.failNotified == 0U))
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", GUI_OtaErrTextCn(err));
  }
  else if (g_guiOtaUi.failNotified != 0U)
  {
    /*失败原因已经弹窗说过一次，状态行只留重试提示*/
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_RETRY);
  }
  else if (chk == OTA_CHK_DONE)
  {
    if (g_otaStatus.checkVer != 0U)
    {
      (void)snprintf(stateTxt, sizeof(stateTxt), TXT_ST_NEWVER,
                     (unsigned)((g_otaStatus.checkVer >> 8U) & 0xFFU),
                     (unsigned)(g_otaStatus.checkVer & 0xFFU));
    }
    else
    {
      (void)snprintf(stateTxt, sizeof(stateTxt), TXT_ST_UPTODATE,
                     (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor);
    }
  }
  else if (chk == OTA_CHK_FAIL)
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", GUI_OtaErrTextCn(chkErr));
  }
  else
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", TXT_ST_IDLE);
  }

  if (strcmp(lastStateTxt, stateTxt) != 0)
  {
    (void)strncpy(lastStateTxt, stateTxt, sizeof(lastStateTxt) - 1U);
    lastStateTxt[sizeof(lastStateTxt) - 1U] = '\0';
    lv_label_set_text(g_guiOtaUi.lblState, lastStateTxt);
  }

  /*目标版本：优先显示下载时取到的 newVer，其次显示检查发现的 checkVer*/
  targetVer = (g_otaStatus.newVer != 0U) ? g_otaStatus.newVer : g_otaStatus.checkVer;
  if (lastVer != targetVer)
  {
    lastVer = targetVer;
    if (targetVer != 0U)
    {
      (void)snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New v%u.%u",
                     (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor,
                     (unsigned)((targetVer >> 8U) & 0xFFU),
                     (unsigned)(targetVer & 0xFFU));
    }
    else
    {
      (void)snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
                     (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor);
    }
    lv_label_set_text(g_guiOtaUi.lblVer, txt);
  }

  kb = g_otaStatus.got / 1024UL;
  if (lastKb != kb)
  {
    lastKb = kb;
    (void)snprintf(txt, sizeof(txt), "%lu KB / %lu KB",
                   (unsigned long)kb, (unsigned long)(g_otaStatus.total / 1024UL));
    lv_label_set_text(g_guiOtaUi.lblDetail, txt);
  }

  if (lastPct != g_otaStatus.percent)
  {
    lastPct = g_otaStatus.percent;
    lv_bar_set_value(g_guiOtaUi.bar, (int32_t)lastPct, LV_ANIM_OFF);
  }

  /*手动检查的结果在这里收口：只有本页发起的那一次才弹窗，
   *MQTT 远程检查和开机静默检查都不能在用户面前弹。
   *必须先看到 CHECKING 再接受结果，否则进页面时会把上一次遗留的 DONE 当成本次结果；
   *4 秒兜底覆盖"命令被队列丢弃、直接 IDLE → FAIL"这种永远等不到 CHECKING 的情况*/
  if (g_guiOtaUi.flow == GUI_OTA_FLOW_CHECK)
  {
    if (chk == OTA_CHK_CHECKING)
    {
      g_guiOtaUi.checkSeen = 1U;
    }
    if ((chk != OTA_CHK_CHECKING) &&
        ((g_guiOtaUi.checkSeen != 0U) || (lv_tick_elaps(g_guiOtaUi.checkT0) > 4000U)))
    {
      g_guiOtaUi.flow = GUI_OTA_FLOW_NONE;

      if (chk == OTA_CHK_DONE)
      {
        if (g_otaStatus.checkVer != 0U)
        {
          (void)snprintf(txt, sizeof(txt), TXT_NEW_VER,
                         (unsigned)((g_otaStatus.checkVer >> 8U) & 0xFFU),
                         (unsigned)(g_otaStatus.checkVer & 0xFFU));
          GuiOta_ShowMsg(txt, 1U, TXT_UPDATE, GUI_OTA_ACT_DOWNLOAD);
        }
        else
        {
          (void)snprintf(txt, sizeof(txt), TXT_UP_TO_DATE,
                         (unsigned)g_guiOtaUi.curMajor, (unsigned)g_guiOtaUi.curMinor);
          GuiOta_ShowMsg(txt, 0U, NULL, GUI_OTA_ACT_NONE);
        }
      }
      else if (chk == OTA_CHK_FAIL)
      {
        GuiOta_ShowMsg(GUI_OtaErrTextCn(chkErr), 0U, NULL, GUI_OTA_ACT_NONE);
      }
      else
      {
        GuiOta_ShowMsg(TXT_TIMEOUT, 0U, NULL, GUI_OTA_ACT_NONE);
      }
    }
  }
}
