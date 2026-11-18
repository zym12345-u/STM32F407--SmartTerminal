/* GuiOtaEntry.c
 * 主屏底部唯一的升级入口：按钮创建、新版本高亮、点击进入 OTA 页。
 * 所有 lv_* 只在 GuiTask 线程调用。
 */

#include "lvgl.h"
#include "OtaData.h"
#include "GuiCommon.h"
#include "GuiOtaInternal.h"

/* 主屏 OTA 入口按钮：检查到新版本时这里换成蓝色高亮，避免新开弹窗 */
static lv_obj_t *s_btnMainOta;
static lv_obj_t *s_lblMainOta;

/**
  * 功    能：点主屏 Upgrade 按钮：进入 OTA 页并按结果自动检查
  * 参    数：event LVGL 事件，原样转给页面入口
  * 返 回 值：无
  */
static void OtaEntryEventCb(lv_event_t *event)
{
  GuiOta_EnterPage(event);
}

/**
  * 功    能：主屏底部唯一的升级入口：点进去自动检查更新
  * 参    数：scr 主屏对象
  * 返 回 值：无
  * 补    充：原来「FW Update + OTA」两个按钮已合并——检查、确认、备份、下载都在同一条
  *            流程里；串口 IAP 收进 OTA 页标题栏的 Serial
  */
void GUI_CreateUpdateEntry(lv_obj_t *scr)
{
  s_btnMainOta = lv_btn_create(scr);
  lv_obj_remove_style_all(s_btnMainOta);
  lv_obj_add_style(s_btnMainOta, &Gui_Styles()->btnWifi, 0);
  lv_obj_set_style_bg_opa(s_btnMainOta, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(s_btnMainOta, 304, 28);   /*屏宽 320，左右各留 8px；底部空区 206~234*/
  lv_obj_set_pos(s_btnMainOta, 8, 206);
  lv_obj_add_event_cb(s_btnMainOta, OtaEntryEventCb, LV_EVENT_CLICKED, NULL);

  s_lblMainOta = lv_label_create(s_btnMainOta);
  lv_label_set_text(s_lblMainOta, LV_SYMBOL_DOWNLOAD "  Update");
  lv_obj_center(s_lblMainOta);
}

/**
  * 功    能：主屏 OTA 入口高亮：检查到新版本就换蓝底 + 上箭头，下载成功（checkVer 清零）后恢复
  * 参    数：无
  * 返 回 值：无
  * 补    充：只改局部样式，不动共享的 Gui_Styles()->btnWifi，避免影响其他按钮
  */
void GUI_RefreshMainOtaEntry(void)
{
  if (s_btnMainOta != NULL)
  {
    static uint8_t otaHi = 0U;
    uint8_t hi = (g_otaStatus.checkVer != 0U) ? 1U : 0U;

    if (hi != otaHi)
    {
      otaHi = hi;
      lv_obj_set_style_bg_color(s_btnMainOta,
                                (hi != 0U) ? COLOR_ACCENT_BLUE : lv_color_hex(0x334155), 0);
      lv_obj_set_style_bg_grad_color(s_btnMainOta,
                                     (hi != 0U) ? COLOR_ACCENT_BLUE_D : lv_color_hex(0x1e293b), 0);
      lv_obj_set_style_border_color(s_btnMainOta,
                                    (hi != 0U) ? COLOR_ACCENT_BLUE_D : COLOR_CARD_BORDER, 0);
      lv_label_set_text(s_lblMainOta,
                        (hi != 0U) ? "New version  " LV_SYMBOL_UP
                                   : LV_SYMBOL_DOWNLOAD "  Update");
    }
  }
}
