/* GuiWifiScreen.c
 * WiFi 配网页：热点列表、密码面板、扫描/连接事件轮询。
 * 只由 GuiTask 线程调用 lv_*；与 WifiTask 通过命令队列/事件队列交互。
 */

#include "cmsis_os2.h"
#include "lvgl.h"
#include "bsp_font_gbk.h"
#include "WifiNet.h"
#include "GuiCommon.h"
#include <stdio.h>
#include <string.h>
static lv_obj_t *s_wifiScr;
static lv_obj_t *s_apList;
static lv_obj_t *s_lblWifiStatus;
static lv_obj_t *s_btnRescan;
static lv_obj_t *s_pwdPanel;
static lv_obj_t *s_pwdPrompt;
static lv_obj_t *s_pwdTa;
static lv_obj_t *s_kb;
static char      s_selSsid[WIFI_SSID_LEN + 1U];
static uint8_t   s_wifiBusy;

/*WiFi 屏幕配网界面*/

/*配网界面状态机，由当前页 + s_wifiBusy + evtQ 事件驱动
 * 状态     进入条件          动作                                       超时
 * 主屏     点击 WiFi Setup   清 evtQ、busy=1、切页、Wifi_PostScan()      无
 * 扫描中   busy=1            SCAN_DONE 后 busy=0，按 status 建列表或报错 CWLAP 20000ms×3
 * 选网     点击热点          sec=0 直接 JOIN，否则弹密码面板            无
 * 连接中   busy=1            发 JOIN，面板隐藏                          CWJAP 20000ms×3
 * 连接成功 JOIN_OK           提示 Connected!，1.5s 后回主屏              1500ms
 * 连接失败 JOIN_FAIL         隐藏密码面板，提示检查密码                 无
 * 退出     Back 或返回定时器 淡入切回主屏，页面对象保留复用              250ms
 * 界面不设超时，等待时长由 WifiTask 的 CWLAP/CWJAP 超时决定，最终必有事件回。
 */

static void ApItemEventCb(lv_event_t *event);
static void WifiPwdHideReady(lv_anim_t *a);

/* 动画执行回调包装:固定 selector=0 */
/**
  * 功    能：动画回调：按当前动画值设置对象的纵向偏移，用来做滑入滑出效果
  * 参    数：var  动画目标对象；v  当前动画值 px
  * 返 回 值：无
  */
static void AnimSetTranslateY(void *var, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)var, v, 0);
}

/**
  * 功    能：用最新的扫描结果重建热点列表
  * 参    数：无
  * 返 回 值：无
  */
static void WifiList_Refresh(void)
{
    uint8_t i;
    char item[64];

    lv_obj_clean(s_apList);

    if ((g_wifiScan.status != WIFI_ST_SCAN_OK) || (g_wifiScan.count == 0U))
    {
        (void)lv_list_add_btn(s_apList, NULL, "(no network)");
        return;
    }

    for (i = 0U; i < g_wifiScan.count; i++)
    {
        const WifiAp_t *ap = &g_wifiScan.aps[i];
        lv_obj_t *btn;
        lv_obj_t *lbl;
        lv_color_t sigColor;

        /* 信号强度色阶:>-50 绿,>-70 黄,否则灰 */
        if (ap->rssi > -50)      sigColor = COLOR_ACCENT_GREEN;
        else if (ap->rssi > -70) sigColor = COLOR_ACCENT_LIGHT;
        else                     sigColor = COLOR_TEXT_MUTED;

        snprintf(item, sizeof(item), "%s  %ddBm%s",
                 ap->ssid, (int)ap->rssi, (ap->sec != 0U) ? "  [*]" : "");
        btn = lv_list_add_btn(s_apList, NULL, item);
        lv_obj_set_user_data(btn, (void *)(uint32_t)i);
        lv_obj_add_event_cb(btn, ApItemEventCb, LV_EVENT_CLICKED, NULL);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(btn, COLOR_CARD, 0);
        lv_obj_set_style_border_color(btn, sigColor, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_pad_top(btn, 6, 0);
        lv_obj_set_style_pad_bottom(btn, 6, 0);

        lbl = lv_obj_get_child(btn, 0);
        if (lbl != NULL)
        {
            lv_obj_set_width(lbl, 280);                    /* 限宽 280,超长 SSID 省略号截断 */
            lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
            lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
            lv_obj_set_style_text_font(lbl, BSP_Font_Gbk16(), 0);
        }
    }
}

/**
  * 功    能：一次性定时器回调：延时到点后把界面切回去，进函数先把自己删掉
  * 参    数：timer  一次性定时器句柄，进入即删除
  * 返 回 值：无
  */
static void WifiReturnTimerCb(lv_timer_t *timer)
{
    lv_timer_del(timer);
    Gui_ShowMainScr();
}

/**
  * 功    能：提交连接请求：把选中的 SSID 和用户输入的密码投给 WifiTask，不等结果
  * 参    数：pwd  用户输入密码，开放网络传 ""
  * 返 回 值：无
  * 补    充：开放网络密码传空串
  */
static void WifiSubmitJoin(const char *pwd)
{
    s_wifiBusy = 1U;
    lv_obj_add_state(s_btnRescan, LV_STATE_DISABLED);
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_lblWifiStatus, "Connecting (up to 40s)...");
    Wifi_PostJoin(s_selSsid, pwd);
}

/**
  * 功    能：点某个热点的事件回调：记下选中的下标，开放网络直接连，加密的弹密码框
  * 参    数：event  LVGL 事件对象，user_data 为 aps 下标
  * 返 回 值：无
  */
static void ApItemEventCb(lv_event_t *event)
{
    lv_obj_t *btn = lv_event_get_target(event);
    uint32_t  idx;
    char prompt[48];

    if ((s_wifiBusy != 0U) || (g_wifiScan.count == 0U))
        return;
    idx = (uint32_t)lv_obj_get_user_data(btn);
    if (idx >= g_wifiScan.count)
        return;

    (void)snprintf(s_selSsid, sizeof(s_selSsid), "%s", g_wifiScan.aps[idx].ssid);

    if (g_wifiScan.aps[idx].sec == 0U)
    {
        WifiSubmitJoin("");
        return;
    }

    snprintf(prompt, sizeof(prompt), "Password: %.24s", s_selSsid);
    lv_label_set_text(s_pwdPrompt, prompt);
    lv_textarea_set_text(s_pwdTa, "");
    lv_obj_clear_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(s_kb, s_pwdTa);

    /* 密码面板从底部滑入 */
    lv_obj_set_style_translate_y(s_pwdPanel, 240, 0);   /* 屏高 240,先移到屏外再滑入 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_pwdPanel);
    lv_anim_set_exec_cb(&a, AnimSetTranslateY);
    lv_anim_set_values(&a, 240, 0);
    lv_anim_set_time(&a, 280);          /* 滑入 280ms */
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/**
  * 功    能：点「重新扫描」：投一条扫描命令给 WifiTask
  * 参    数：event  未使用
  * 返 回 值：无
  */
static void WifiRescanCb(lv_event_t *event)
{
    (void)event;
    if (s_wifiBusy != 0U)
        return;
    s_wifiBusy = 1U;
    lv_obj_add_state(s_btnRescan, LV_STATE_DISABLED);
    lv_obj_clean(s_apList);
    lv_label_set_text(s_lblWifiStatus, "Scanning...");
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    Wifi_PostScan();
}

/**
  * 功    能：点「返回」：从配网页退回主屏
  * 参    数：event  未使用
  * 返 回 值：无
  */
static void WifiBackCb(lv_event_t *event)
{
    (void)event;
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    Gui_ShowMainScr();
}

/**
  * 功    能：密码框点「确定」：取出输入内容并提交连接
  * 参    数：event  未使用
  * 返 回 值：无
  */
static void WifiPwdOkCb(lv_event_t *event)
{
    const char *pwd;
    (void)event;
    pwd = lv_textarea_get_text(s_pwdTa);
    WifiSubmitJoin((pwd != NULL) ? pwd : "");
}

/**
  * 功    能：密码框点「取消」：收起键盘和密码框，不连接
  * 参    数：event  未使用
  * 返 回 值：无
  */
static void WifiPwdCancelCb(lv_event_t *event)
{
    (void)event;
    lv_keyboard_set_textarea(s_kb, NULL);
    /* 滑出再隐藏 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_pwdPanel);
    lv_anim_set_exec_cb(&a, AnimSetTranslateY);
    lv_anim_set_values(&a, 0, 240);
    lv_anim_set_time(&a, 220);          /* 滑出 220ms */
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, WifiPwdHideReady);
    lv_anim_start(&a);
}

/**
  * 功    能：密码框隐藏动画结束时的回调，用来做收尾清理（比如把键盘也收掉）
  * 参    数：a  动画描述，未使用
  * 返 回 值：无
  */
static void WifiPwdHideReady(lv_anim_t *a)
{
    (void)a;
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_translate_y(s_pwdPanel, 0, 0);  /* 复位 translate,下次滑入从 0 开始 */
}

/* 定时器：轮询 WifiTask 的事件队列，处理「扫描完成」「连接成功/失败」。
 * GUI 就是这样拿到 WiFi 的异步结果的。 */
/*不在配网页时事件不取走，会一直堆在 evtQ，故进页时必须先清陈旧事件*/
void WifiEventTimerCb(lv_timer_t *timer)
{
    WifiEvent_t ev;

    (void)timer;
    if (lv_scr_act() != s_wifiScr)
        return;

    while (Wifi_TryGetEvent(&ev) != 0U)
    {
        if (ev == WIFI_EV_SCAN_DONE)
        {
            s_wifiBusy = 0U;
            lv_obj_clear_state(s_btnRescan, LV_STATE_DISABLED);
            if (g_wifiScan.status == WIFI_ST_SCAN_OK)
            {
                WifiList_Refresh();
                lv_label_set_text(s_lblWifiStatus, "Tap a network");
            }
            else
            {
                lv_obj_clean(s_apList);
                (void)lv_list_add_btn(s_apList, NULL, "(no network)");
                lv_label_set_text(s_lblWifiStatus, "Scan failed, press Rescan");
            }
        }
        else if (ev == WIFI_EV_JOIN_OK)
        {
            s_wifiBusy = 0U;
            lv_obj_clear_state(s_btnRescan, LV_STATE_DISABLED);
            lv_label_set_text(s_lblWifiStatus, "Connected!");
            (void)lv_timer_create(WifiReturnTimerCb, 1500U, NULL);   /* 成功提示停留 1.5s */
        }
        else if (ev == WIFI_EV_JOIN_FAIL)
        {
            s_wifiBusy = 0U;
            lv_obj_clear_state(s_btnRescan, LV_STATE_DISABLED);
            lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(s_lblWifiStatus, "Join failed, check password / Rescan");
        }
    }
}

/**
  * 功    能：点主屏的 WiFi 入口：切到配网页，并发起一次扫描
  * 参    数：event  未使用
  * 返 回 值：无
  */
static void WifiEntryEventCb(lv_event_t *event)
{
    WifiEvent_t stale;
    (void)event;

    /* 清空上次留下的陈旧事件(原因见 WifiEventTimerCb 的注释),
     * 否则刚进页面就会被历史事件覆盖掉状态 */
    while (Wifi_TryGetEvent(&stale) != 0U) { ; }

    s_wifiBusy = 1U;
    lv_obj_add_state(s_btnRescan, LV_STATE_DISABLED);
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clean(s_apList);
    lv_label_set_text(s_lblWifiStatus, "Scanning...");
    lv_scr_load_anim(s_wifiScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
    Wifi_PostScan();
}

/*创建 WiFi 配网页*/
/**
  * 功    能：把配网页搭起来：热点列表 + 密码输入框 + 软键盘
  * 参    数：无
  * 返 回 值：无
  */
void GUI_CreateWifiScreen(void)
{
    lv_obj_t *lblTitle;
    lv_obj_t *btn;
    lv_obj_t *lbl;

    s_wifiScr = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_wifiScr);
    lv_obj_add_style(s_wifiScr, &Gui_Styles()->bg, 0);

    /* 标题 */
    lblTitle = lv_label_create(s_wifiScr);
    lv_label_set_text(lblTitle, "WiFi Setup");
    lv_obj_set_style_text_color(lblTitle, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lblTitle, 8, 6);

    s_lblWifiStatus = lv_label_create(s_wifiScr);
    lv_label_set_text(s_lblWifiStatus, "Scanning...");
    lv_obj_add_style(s_lblWifiStatus, &Gui_Styles()->status, 0);
    lv_obj_set_pos(s_lblWifiStatus, 100, 8);

    /* 热点列表 */
    s_apList = lv_list_create(s_wifiScr);
    lv_obj_set_size(s_apList, 308, 168);             /* 308×168:底部留给 Rescan/Back 按钮 */
    lv_obj_set_pos(s_apList, 6, 28);
    lv_obj_set_style_bg_opa(s_apList, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_apList, COLOR_CARD, 0);
    lv_obj_set_style_border_color(s_apList, COLOR_CARD_BORDER, 0);
    lv_obj_set_style_border_width(s_apList, 1, 0);
    lv_obj_set_style_radius(s_apList, 10, 0);

    /* 底部按钮 */
    s_btnRescan = lv_btn_create(s_wifiScr);
    lv_obj_remove_style_all(s_btnRescan);
    lv_obj_add_style(s_btnRescan, &Gui_Styles()->btnWifi, 0);
    lv_obj_set_style_bg_opa(s_btnRescan, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(s_btnRescan, 150, 34);
    lv_obj_set_pos(s_btnRescan, 6, 200);
    lv_obj_add_event_cb(s_btnRescan, WifiRescanCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(s_btnRescan);
    lv_label_set_text(lbl, LV_SYMBOL_REFRESH "  Rescan");
    lv_obj_center(lbl);

    btn = lv_btn_create(s_wifiScr);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &Gui_Styles()->btnWifi, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(btn, 150, 34);
    lv_obj_set_pos(btn, 164, 200);
    lv_obj_add_event_cb(btn, WifiBackCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, LV_SYMBOL_LEFT "  Back");
    lv_obj_center(lbl);

    /* 密码输入弹层 */
    s_pwdPanel = lv_obj_create(s_wifiScr);
    lv_obj_remove_style_all(s_pwdPanel);
    lv_obj_set_size(s_pwdPanel, 320, 240);
    lv_obj_set_pos(s_pwdPanel, 0, 0);
    lv_obj_set_style_bg_color(s_pwdPanel, COLOR_BG_BOT, 0);
    lv_obj_set_style_bg_opa(s_pwdPanel, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_pwdPanel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_pwdPanel, 0, 0);
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);

    s_pwdPrompt = lv_label_create(s_pwdPanel);
    lv_label_set_text(s_pwdPrompt, "Password");
    lv_obj_set_style_text_color(s_pwdPrompt, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(s_pwdPrompt, BSP_Font_Gbk16(), 0);
    lv_obj_set_pos(s_pwdPrompt, 8, 2);

    s_pwdTa = lv_textarea_create(s_pwdPanel);
    lv_obj_set_size(s_pwdTa, 186, 28);
    lv_obj_set_pos(s_pwdTa, 8, 20);
    lv_textarea_set_one_line(s_pwdTa, true);
    lv_textarea_set_password_mode(s_pwdTa, true);
    lv_textarea_set_text(s_pwdTa, "");
    lv_obj_set_style_bg_opa(s_pwdTa, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_pwdTa, COLOR_CARD, 0);
    lv_obj_set_style_text_color(s_pwdTa, COLOR_TEXT, 0);
    lv_obj_set_style_border_color(s_pwdTa, COLOR_ACCENT_BLUE, 0);

    btn = lv_btn_create(s_pwdPanel);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &Gui_Styles()->btnWifi, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(btn, 56, 28);
    lv_obj_set_pos(btn, 202, 19);
    lv_obj_add_event_cb(btn, WifiPwdCancelCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Cancel");
    lv_obj_center(lbl);

    btn = lv_btn_create(s_pwdPanel);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &Gui_Styles()->btnGreen, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(btn, 52, 28);
    lv_obj_set_pos(btn, 262, 19);
    lv_obj_add_event_cb(btn, WifiPwdOkCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Join");
    lv_obj_center(lbl);

    /* 键盘:从 y=50 到屏幕底部 240,共 190px,确保 5 行完整显示 */
    s_kb = lv_keyboard_create(s_pwdPanel);
    lv_obj_set_size(s_kb, 320, 190);
    lv_obj_set_pos(s_kb, 0, 50);
    lv_obj_set_style_pad_all(s_kb, 1, 0);
    lv_obj_set_style_pad_row(s_kb, 1, 0);
    lv_obj_set_style_pad_column(s_kb, 2, 0);
    lv_obj_set_style_bg_color(s_kb, COLOR_BG_BOT, 0);
}

/**
  * 功    能：在主屏放一个 WiFi 入口，显示当前状态，点进去是配网页
  * 参    数：scr  主屏对象
  * 返 回 值：无
  */
void GUI_CreateWifiEntry(lv_obj_t *scr)
{
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &Gui_Styles()->btnWifi, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 150, 60);
  lv_obj_set_pos(btn, 8, 138);
  lv_obj_add_event_cb(btn, WifiEntryEventCb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_WIFI "\nWiFi Setup");
  lv_obj_center(lbl);
}
