/* GuiOtaScreen.c
 * OTA 下载页：页面搭建、进页检查、150ms 状态轮询、确认弹窗与下载流程。
 * 串口升级与槽 A 备份见 GuiOtaSerial.c，主屏入口见 GuiOtaEntry.c。
 * 所有 lv_* 只在 GuiTask 线程调用；进度数据由 WifiTask 写 g_otaStatus，本文件只读。
 */

#include "lvgl.h"
#include "bsp_font_gbk.h"
#include "bsp_OtaCfg.h"
#include "OtaData.h"
#include "WifiNet.h"
#include "GuiCommon.h"
#include "GuiOtaText.h"
#include "GuiOtaInternal.h"
#include <stdio.h>
#include <string.h>

/* WiFi OTA 下载页控件（只下载到槽 B，不切启动） */
static lv_obj_t *s_otaScr;
static lv_obj_t *s_lblOtaWifi;
static lv_obj_t *s_lblOtaServer;
static lv_obj_t *s_lblOtaVer;
static lv_obj_t *s_lblOtaState;
static lv_obj_t *s_lblOtaDetail;
static lv_obj_t *s_barOta;
static uint8_t   s_otaCurMajor;
static uint8_t   s_otaCurMinor;
static uint8_t   s_otaUiDirty = 1U;   /*进入页面时置 1，让轮询函数丢弃上次的显示缓存*/

/*弹窗确认键的动作：同一个弹窗要服务「下载」和「串口升级」两条路径*/
#define MSG_ACT_DOWNLOAD   1U
#define MSG_ACT_SERIAL     2U

static uint8_t s_msgAct = MSG_ACT_DOWNLOAD;

/*一键更新交互：进页面自动检查 → 按结果弹窗 → 确认后备份槽 A 再下载。
 * 为什么要有 s_otaFlow：检查是 WifiTask 异步做的（让出 MQTT、建 TCP、GET app.meta），
 * 晚 0.5~3 秒才有结果；只有本页发起的那一次才弹窗，
 * MQTT 远程检查和开机 60s 静默检查都不能在用户面前弹窗*/
#define OTA_UI_FLOW_NONE     0U
#define OTA_UI_FLOW_CHECK    1U
#define OTA_UI_FLOW_DOWNLOAD 2U   /*本页发起的下载：失败时弹一次中文窗*/

static uint8_t   s_otaFlow;
static uint8_t   s_otaCheckSeen;   /*是否看到过 CHECKING：防止把上一次遗留的结果当成本次的*/
static uint8_t   s_otaFailNotified;/*下载失败已弹过窗：状态行不再停在错误文案上*/
static uint32_t  s_otaCheckT0;
static lv_obj_t *s_msgbox;

static void GUI_ShowMsg(const char *txt, uint8_t confirm, const char *okText);
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
static void GUI_CloseMsg(void)
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
  GUI_CloseMsg();
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
  GUI_CloseMsg();

  if (s_msgAct == MSG_ACT_SERIAL)
  {
    GuiOta_SerialUpdate(s_lblOtaState);
  }
  else
  {
    GUI_StartOtaDownload();
  }
}

/**
  * 功    能：弹模态提示框：全屏遮罩 + 居中卡片
  * 参    数：txt 正文（中文，用外挂 GBK 16px 字库）；confirm 1=给两个键；
  *            okText 确认键文案（NULL 用「升级」）
  * 返 回 值：无
  * 补    充：不用 lv_msgbox：默认样式的按钮在 240 高的屏上偏大、长文案会被裁；
  *            这里复用 Gui_Styles()->card / Gui_Styles()->btnGreen / Gui_Styles()->btnBlue，观感和页面一致
  */
static void GUI_ShowMsg(const char *txt, uint8_t confirm, const char *okText)
{
  lv_obj_t *mask;
  lv_obj_t *box;
  lv_obj_t *lbl;
  lv_obj_t *btn;

  GUI_CloseMsg();

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

  lv_label_set_text(s_lblOtaState, TXT_ST_BACKUP);
  lv_refr_now(NULL);

  if (GuiOta_EnsureSlotABackup(s_lblOtaState) == 0U)
  {
    lv_label_set_text(s_lblOtaState, TXT_ST_BK_FAIL);
    GUI_ShowMsg(TXT_BK_FAIL, 0U, NULL);
    return;
  }

  Wifi_PostOta();
  s_otaFlow         = OTA_UI_FLOW_DOWNLOAD;   /*失败时由页面轮询负责弹窗*/
  s_otaFailNotified = 0U;
  lv_label_set_text(s_lblOtaState, TXT_ST_SENT);
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
  s_otaFlow = OTA_UI_FLOW_NONE;
  GUI_CloseMsg();
  Gui_ShowMainScr();
}

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
  *            所以必须让用户明确确认；备份进度与失败原因仍打在 s_lblOtaState 上
  */
static void SerialUpdateEventCb(lv_event_t *event)
{
  (void)event;

  if (GuiOta_IsBusy() != 0U)
  {
    lv_label_set_text(s_lblOtaState, TXT_ST_BUSY);
    return;
  }

  s_msgAct = MSG_ACT_SERIAL;
  GUI_ShowMsg(TXT_SERIAL_CONFIRM, 1U, TXT_SERIAL_OK);
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
  *            这里只负责投命令并把 s_otaFlow 立起来
  */
void GuiOta_EnterPage(lv_event_t *event)
{
  uint8_t fromMeta;
  char    txt[64];

  (void)event;
  GUI_UpdateOtaServer();
  GUI_GetFwVersion(&s_otaCurMajor, &s_otaCurMinor, &fromMeta);
  (void)fromMeta;
  snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
           (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
  lv_label_set_text(s_lblOtaVer, txt);
  s_otaUiDirty = 1U;
  lv_scr_load_anim(s_otaScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);

  /*下载占用 W25Q 期间不做检查：两段 TCP 收包会互相踩*/
  if (GuiOta_IsBusy() != 0U)
  {
    return;
  }
  if (Wifi_IsConnected() == 0U)
  {
    lv_label_set_text(s_lblOtaState, TXT_ST_NOWIFI);
    GUI_ShowMsg(TXT_NO_WIFI, 0U, NULL);
    return;
  }

  s_otaFlow         = OTA_UI_FLOW_CHECK;
  s_otaCheckSeen    = 0U;
  s_otaFailNotified = 0U;   /*重新检查就把上次的失败提示清掉*/
  s_otaCheckT0      = lv_tick_get();
  Wifi_PostCheckOta(1U);
  lv_label_set_text(s_lblOtaState, TXT_ST_CHECKING);
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
  if (lv_scr_act() != s_otaScr)
  {
    return;        /*离开页面就不刷；下载在 WifiTask 里继续 */
  }

  /*重新进入页面：把上次的缓存作废，强制整页重画一次*/
  if (s_otaUiDirty != 0U)
  {
    s_otaUiDirty = 0U;
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
      lv_label_set_text(s_lblOtaWifi,
                        (wifiUp != 0U) ? "WiFi: connected" : "WiFi: not connected");
    }
  }

  st     = g_otaStatus.state;
  err    = g_otaStatus.err;
  chk    = g_otaStatus.checkState;
  chkErr = g_otaStatus.checkErr;

  /*下载失败：只弹一次中文窗，之后状态行不再停在错误文案上（否则看着像卡死）。
   *成功不弹窗——状态行会显示 rebooting，复位本身就是反馈*/
  if (s_otaFlow == OTA_UI_FLOW_DOWNLOAD)
  {
    if (st == OTA_ST_FAIL)
    {
      s_otaFlow         = OTA_UI_FLOW_NONE;
      s_otaFailNotified = 1U;
      GUI_ShowMsg(GUI_OtaErrTextCn(err), 0U, NULL);
    }
    else if (st == OTA_ST_DONE)
    {
      s_otaFlow = OTA_UI_FLOW_NONE;
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
  else if ((st == OTA_ST_FAIL) && (s_otaFailNotified == 0U))
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", GUI_OtaErrTextCn(err));
  }
  else if (s_otaFailNotified != 0U)
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
                     (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
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
    lv_label_set_text(s_lblOtaState, lastStateTxt);
  }

  /*目标版本：优先显示下载时取到的 newVer，其次显示检查发现的 checkVer*/
  targetVer = (g_otaStatus.newVer != 0U) ? g_otaStatus.newVer : g_otaStatus.checkVer;
  if (lastVer != targetVer)
  {
    lastVer = targetVer;
    if (targetVer != 0U)
    {
      (void)snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New v%u.%u",
                     (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor,
                     (unsigned)((targetVer >> 8U) & 0xFFU),
                     (unsigned)(targetVer & 0xFFU));
    }
    else
    {
      (void)snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
                     (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
    }
    lv_label_set_text(s_lblOtaVer, txt);
  }

  kb = g_otaStatus.got / 1024UL;
  if (lastKb != kb)
  {
    lastKb = kb;
    (void)snprintf(txt, sizeof(txt), "%lu KB / %lu KB",
                   (unsigned long)kb, (unsigned long)(g_otaStatus.total / 1024UL));
    lv_label_set_text(s_lblOtaDetail, txt);
  }

  if (lastPct != g_otaStatus.percent)
  {
    lastPct = g_otaStatus.percent;
    lv_bar_set_value(s_barOta, (int32_t)lastPct, LV_ANIM_OFF);
  }

  /*手动检查的结果在这里收口：只有本页发起的那一次才弹窗，
   *MQTT 远程检查和开机静默检查都不能在用户面前弹。
   *必须先看到 CHECKING 再接受结果，否则进页面时会把上一次遗留的 DONE 当成本次结果；
   *4 秒兜底覆盖"命令被队列丢弃、直接 IDLE → FAIL"这种永远等不到 CHECKING 的情况*/
  if (s_otaFlow == OTA_UI_FLOW_CHECK)
  {
    if (chk == OTA_CHK_CHECKING)
    {
      s_otaCheckSeen = 1U;
    }
    if ((chk != OTA_CHK_CHECKING) &&
        ((s_otaCheckSeen != 0U) || (lv_tick_elaps(s_otaCheckT0) > 4000U)))
    {
      s_otaFlow = OTA_UI_FLOW_NONE;

      if (chk == OTA_CHK_DONE)
      {
        if (g_otaStatus.checkVer != 0U)
        {
          (void)snprintf(txt, sizeof(txt), TXT_NEW_VER,
                         (unsigned)((g_otaStatus.checkVer >> 8U) & 0xFFU),
                         (unsigned)(g_otaStatus.checkVer & 0xFFU));
          s_msgAct = MSG_ACT_DOWNLOAD;
          GUI_ShowMsg(txt, 1U, TXT_UPDATE);
        }
        else
        {
          (void)snprintf(txt, sizeof(txt), TXT_UP_TO_DATE,
                         (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
          GUI_ShowMsg(txt, 0U, NULL);
        }
      }
      else if (chk == OTA_CHK_FAIL)
      {
        GUI_ShowMsg(GUI_OtaErrTextCn(chkErr), 0U, NULL);
      }
      else
      {
        GUI_ShowMsg(TXT_TIMEOUT, 0U, NULL);
      }
    }
  }
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

  /*建页时就取一次当前版本，避免刚进页面显示 v0.0*/
  GUI_GetFwVersion(&s_otaCurMajor, &s_otaCurMinor, &fromMeta);
  (void)fromMeta;

  s_otaScr = lv_obj_create(NULL);
  lv_obj_remove_style_all(s_otaScr);
  lv_obj_add_style(s_otaScr, &Gui_Styles()->bg, 0);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "WiFi OTA");
  lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(lbl, 8, 6);

  s_lblOtaWifi = lv_label_create(s_otaScr);
  lv_label_set_text(s_lblOtaWifi, "WiFi: --");
  lv_obj_add_style(s_lblOtaWifi, &Gui_Styles()->status, 0);
  lv_obj_set_pos(s_lblOtaWifi, 110, 8);

  card = lv_obj_create(s_otaScr);
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

  s_lblOtaVer = lv_label_create(card);
  snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
           (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
  lv_label_set_text(s_lblOtaVer, txt);
  lv_obj_add_style(s_lblOtaVer, &Gui_Styles()->cardSub, 0);
  lv_obj_set_pos(s_lblOtaVer, 0, 20);

  s_lblOtaState = lv_label_create(card);
  lv_label_set_text(s_lblOtaState, TXT_ST_IDLE);
  lv_label_set_long_mode(s_lblOtaState, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(s_lblOtaState, 288);
  lv_obj_set_style_text_color(s_lblOtaState, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(s_lblOtaState, BSP_Font_Gbk16(), 0);
  lv_obj_set_pos(s_lblOtaState, 0, 42);

  /*进度条：轨道深色、指示蓝色；不用默认样式，避免主题自动加渐变/边框*/
  s_barOta = lv_bar_create(s_otaScr);
  lv_obj_set_size(s_barOta, 304, 14);
  lv_obj_set_pos(s_barOta, 8, 130);
  lv_bar_set_range(s_barOta, 0, 100);
  lv_obj_set_style_bg_opa(s_barOta, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(s_barOta, lv_color_hex(0x334155), 0);
  lv_obj_set_style_border_width(s_barOta, 0, 0);
  lv_obj_set_style_radius(s_barOta, 7, 0);
  lv_obj_set_style_bg_opa(s_barOta, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(s_barOta, COLOR_ACCENT_BLUE, LV_PART_INDICATOR);
  lv_obj_set_style_radius(s_barOta, 7, LV_PART_INDICATOR);
  lv_bar_set_value(s_barOta, 0, LV_ANIM_OFF);

  s_lblOtaDetail = lv_label_create(s_otaScr);
  lv_label_set_text(s_lblOtaDetail, "0 KB / 0 KB");
  lv_obj_add_style(s_lblOtaDetail, &Gui_Styles()->status, 0);
  lv_obj_set_pos(s_lblOtaDetail, 8, 148);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "PC: python Tools/ota_serve.py app_image.bin");
  lv_obj_add_style(lbl, &Gui_Styles()->status, 0);
  lv_obj_set_pos(lbl, 8, 166);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "Open = auto check, then confirm to install");
  lv_obj_add_style(lbl, &Gui_Styles()->status, 0);
  lv_obj_set_pos(lbl, 8, 182);

  /*Serial：串口 IAP 的老入口，进 OTA 页后按它才备份 + 复位进 3s 窗口*/
  lv_obj_t *btn = lv_btn_create(s_otaScr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &Gui_Styles()->btnBlue, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 64, 22);
  lv_obj_set_pos(btn, 248, 4);
  lv_obj_add_event_cb(btn, SerialUpdateEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Serial");
  lv_obj_center(lbl);

  btn = lv_btn_create(s_otaScr);
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
