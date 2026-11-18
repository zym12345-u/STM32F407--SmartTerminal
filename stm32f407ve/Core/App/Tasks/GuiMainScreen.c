/* GuiMainScreen.c
 * 主屏：状态栏、传感器卡片、灯控卡片、WiFi/OTA 入口与 1s 刷新。
 * 只由 GuiTask 线程调用 lv_*，控件状态仅在本文件写。
 */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "lvgl.h"
#include "LEDType.h"
#include "SensorData.h"
#include "ClockData.h"
#include "usart.h"
#include "GuiCommon.h"
#include <stdio.h>
#include <string.h>
/* LEDQueue 由 freertos.c 中 CubeMX 生成的代码创建 */
extern osMessageQueueId_t LEDQueueHandle;

/* 传感器数据显示标签 */
static lv_obj_t *labelTemp;
static lv_obj_t *labelHumi;
static lv_obj_t *labelLight;

/* 顶部状态栏 */
static lv_obj_t *labelClock;
static lv_obj_t *labelWifiIcon;
static lv_obj_t *labelFwVer;

/*LED 命令投递*/

/*队列只传指针：这里 pvPortMalloc，LEDTask 负责 vPortFree，投递失败必须自己释放*/
/**
  * 功    能：往 LEDQueue 投一条点灯指令。GUI 按钮点灯也走这条统一通道，不直接碰 GPIO
  * 参    数：color  目标灯，LEDColor_Green / LEDColor_Blue；state  目标状态
  * 返 回 值：无
  */
static void GUI_LedPost(LEDColor color, LEDState state)
{
  LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));
  if(msg != NULL)
  {
    msg->color = color;
    msg->state = state;
    /* 超时 0:界面线程不能被队列卡住,投不进就丢弃 */
    if(osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
      vPortFree(msg);
    }
  }
}

/*传感器卡片*/
/**
  * 功    能：建一张传感器卡片：图标 + 名称 + 数值。返回数值那个 label，
  * 参    数：parent 父容器；x/y 坐标 px；w/h 尺寸 px；accent 色条颜色；labelText 标签；initVal 初值
  * 返 回 值：数值 label 指针，供 1s 定时器刷新
  * 补    充：后续 1s 定时器直接改它的文字来刷新
  */
static lv_obj_t *GUI_CreateSensorCard(lv_obj_t *parent, int16_t x, int16_t y,
                                      int16_t w, int16_t h,
                                      lv_color_t accent,
                                      const char *labelText,
                                      const char *initVal)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_remove_style_all(card);
  lv_obj_set_size(card, w, h);
  lv_obj_set_pos(card, x, y);

  /* 左侧色条 */
  lv_obj_t *bar = lv_obj_create(card);
  lv_obj_remove_style_all(bar);
  lv_obj_add_style(bar, &Gui_Styles()->accentBar, 0);
  lv_obj_set_style_bg_color(bar, accent, 0);
  lv_obj_set_size(bar, 4, h - 8);                    /* 4px 宽色条,上下各留 4px */
  lv_obj_set_pos(bar, 4, 4);

  /* 数值(大) */
  lv_obj_t *val = lv_label_create(card);
  lv_label_set_text(val, initVal);
  lv_obj_add_style(val, &Gui_Styles()->cardLabel, 0);
  lv_obj_set_pos(val, 14, 6);

  /* 标签(小,右侧) */
  lv_obj_t *lbl = lv_label_create(card);
  lv_label_set_text(lbl, labelText);
  lv_obj_add_style(lbl, &Gui_Styles()->cardSub, 0);
  lv_obj_align(lbl, LV_ALIGN_RIGHT_MID, -8, 0);

  return val;
}

/**
  * 功    能：在主屏顶部建状态栏（WiFi 状态、时间之类）
  * 参    数：scr  目标屏幕，主屏对象
  * 返 回 值：无
  */
static void GUI_CreateStatusBar(lv_obj_t *scr)
{
  /* 底部细线分隔 */
  lv_obj_t *line = lv_obj_create(scr);
  lv_obj_remove_style_all(line);
  lv_obj_set_size(line, 304, 1);
  lv_obj_set_pos(line, 8, 22);
  lv_obj_set_style_bg_color(line, COLOR_CARD_BORDER, 0);

  /* WiFi 图标(用文本符号近似) */
  labelWifiIcon = lv_label_create(scr);
  lv_label_set_text(labelWifiIcon, "WiFi");
  lv_obj_add_style(labelWifiIcon, &Gui_Styles()->status, 0);
  lv_obj_set_pos(labelWifiIcon, 8, 5);

  /* 固件版本：放 WiFi 右边，宽度够 "v1.4"；不可信时自动退回编译期版本 */
  {
    uint8_t major;
    uint8_t minor;
    uint8_t fromMeta;
    char    buf[16];

    GUI_GetFwVersion(&major, &minor, &fromMeta);
    snprintf(buf, sizeof(buf), "v%u.%u", (unsigned)major, (unsigned)minor);

    labelFwVer = lv_label_create(scr);
    lv_label_set_text(labelFwVer, buf);
    lv_obj_add_style(labelFwVer, &Gui_Styles()->status, 0);
    lv_obj_set_pos(labelFwVer, 44, 5);

    /*上电日志留证：读完元数据后能看出这个版本号是从哪来的*/
    UART_Printf("[GUI] fw v%u.%u (%s)\n", (unsigned)major, (unsigned)minor,
                (fromMeta != 0U) ? "meta" : "compile");
  }

  /* 时钟(右对齐) */
  labelClock = lv_label_create(scr);
  lv_label_set_text(labelClock, "syncing...");
  lv_obj_add_style(labelClock, &Gui_Styles()->status, 0);
  lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, -8, 5);
}

/**
  * 功    能：灯开关的事件回调：从 user_data 取出是哪个灯，投 LEDQueue，同时更新按钮外观
  * 参    数：event  LVGL 事件对象，user_data 存的是 LEDColor
  * 返 回 值：无
  * 补    充：实际点灯还是 LEDTask 干的，GUI 只负责把意图发出去
  */
static void LedSwitchEventCb(lv_event_t *event)
{
  lv_obj_t *sw = lv_event_get_target(event);
  uint32_t color = (uint32_t)lv_obj_get_user_data(sw);
  bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

  /*板载 LED 低电平点亮：CHECKED=亮*/
  LEDState target = on ? LEDState_On : LEDState_Off;
  GUI_LedPost((LEDColor)color, target);
}

/**
  * 功    能：建一张灯控卡片（开关 + 状态指示）。灯的颜色存在 user_data 里，供回调取用
  * 参    数：parent 父对象；x/y 坐标 px；w/h 尺寸 px；onColor 开态指示色；color 灯枚举，存 user_data
  * 返 回 值：容器 card 指针
  */
static lv_obj_t *GUI_CreateLedCard(lv_obj_t *parent, int16_t x, int16_t y,
                                    int16_t w, int16_t h,
                                    lv_color_t onColor,
                                    LEDColor color)
{
  /*透明容器*/
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_remove_style_all(card);
  lv_obj_set_size(card, w, h);
  lv_obj_set_pos(card, x, y);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  /*右侧开关*/
  lv_obj_t *sw = lv_switch_create(card);
  lv_obj_set_size(sw, 44, 22);                       /* 44×22 开关,容器内居中 */
  lv_obj_center(sw);
  lv_obj_set_user_data(sw, (void *)(uint32_t)color);

  /* 开关样式 */
  lv_obj_remove_style_all(sw);
  lv_obj_add_style(sw, &Gui_Styles()->swBg, LV_PART_MAIN);
  lv_obj_add_style(sw, &Gui_Styles()->swKnob, LV_PART_KNOB);

  /* 开态指示器颜色(绿/蓝) */
  lv_obj_set_style_bg_color(sw, onColor, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_set_style_radius(sw, 20, LV_PART_INDICATOR | LV_STATE_CHECKED);

  /*按引脚电平设初始状态，以引脚为真相*/
  GPIO_PinState pin;
  if (color == LEDColor_Green)
    pin = HAL_GPIO_ReadPin(led_green_GPIO_Port, led_green_Pin);
  else
    pin = HAL_GPIO_ReadPin(led_blue_GPIO_Port, led_blue_Pin);
  /* 引脚低=LED亮=switch开 */
  if (pin == GPIO_PIN_RESET)
    lv_obj_add_state(sw, LV_STATE_CHECKED);

  lv_obj_add_event_cb(sw, LedSwitchEventCb, LV_EVENT_VALUE_CHANGED, NULL);
  return card;
}

/*LED 开关卡片*/
/**
  * 功    能：在主屏上建几个功能按钮（比如进 WiFi 配网页）
  * 参    数：scr  主屏对象
  * 返 回 值：无
  */
static void GUI_CreateButtons(lv_obj_t *scr)
{
  /* LED 开关竖向排列在传感器框右下角 */
  GUI_CreateLedCard(scr, 170, 138, 142, 30,
                    COLOR_ACCENT_GREEN, LEDColor_Green);
  GUI_CreateLedCard(scr, 170, 172, 142, 30,
                    COLOR_ACCENT_BLUE, LEDColor_Blue);
}

/*主屏*/
/**
  * 功    能：把主屏整个搭起来：状态栏 + 传感器卡片 + 灯控卡片 + 功能按钮
  * 参    数：无，内部用 lv_scr_act() 取当前屏
  * 返 回 值：无
  */
void GUI_CreateMainScreen(void)
{
  lv_obj_t *scr = lv_scr_act();
  lv_obj_remove_style_all(scr);
  lv_obj_add_style(scr, &Gui_Styles()->bg, 0);

  GUI_CreateStatusBar(scr);

  /* 传感器滚动容器:三个传感器卡片放在同一个可滚动框架里,上下拖动查看 */
  lv_obj_t *sensorBox = lv_obj_create(scr);
  lv_obj_remove_style_all(sensorBox);
  lv_obj_set_style_bg_opa(sensorBox, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(sensorBox, COLOR_CARD, 0);
  lv_obj_set_style_border_color(sensorBox, COLOR_CARD_BORDER, 0);
  lv_obj_set_style_border_width(sensorBox, 1, 0);
  lv_obj_set_style_radius(sensorBox, 12, 0);
  lv_obj_set_style_pad_all(sensorBox, 6, 0);
  lv_obj_set_style_pad_row(sensorBox, 4, 0);
  lv_obj_set_size(sensorBox, 304, 104);              /* 屏宽 320,左右各留 8px 边距 */
  lv_obj_set_pos(sensorBox, 8, 28);
  lv_obj_set_scroll_dir(sensorBox, LV_DIR_VER);

  labelTemp  = GUI_CreateSensorCard(sensorBox, 0, 0, 288, 30,
                                     COLOR_ACCENT_TEMP,  "TEMP", "--.- C");
  labelHumi  = GUI_CreateSensorCard(sensorBox, 0, 34, 288, 30,
                                     COLOR_ACCENT_HUMI,  "HUMI", "--.- %");
  labelLight = GUI_CreateSensorCard(sensorBox, 0, 68, 288, 30,
                                     COLOR_ACCENT_LIGHT, "LIGHT", "---- lux");

  GUI_CreateButtons(scr);
  GUI_CreateWifiEntry(scr);
  GUI_CreateUpdateEntry(scr);
}

/*传感器刷新*/
/**
  * 功    能：1s 定时器回调：把全局 g_sensorData 的最新值刷到卡片上
  * 参    数：timer  定时器句柄，未使用
  * 返 回 值：无
  * 补    充：只读不写，所以不用加锁（单写多读，各字段单独原子）
  */
void SensorRefreshTimerCb(lv_timer_t *timer)
{
  (void)timer;
  char buf[32];
  int  intPart, decPart;

  if (g_sensorData.temp_valid)
  {
    intPart = (int)g_sensorData.temperature;
    decPart = (int)((g_sensorData.temperature - intPart) * 10.0f);
    if (decPart < 0) decPart = -decPart;
    snprintf(buf, sizeof(buf), "%d.%d C", intPart, decPart);
  }
  else
  {
    snprintf(buf, sizeof(buf), "--.- C");
  }
  lv_label_set_text(labelTemp, buf);

  if (g_sensorData.temp_valid)
  {
    intPart = (int)g_sensorData.humidity;
    decPart = (int)((g_sensorData.humidity - intPart) * 10.0f);
    if (decPart < 0) decPart = -decPart;
    snprintf(buf, sizeof(buf), "%d.%d %%", intPart, decPart);
  }
  else
  {
    snprintf(buf, sizeof(buf), "--.- %%");
  }
  lv_label_set_text(labelHumi, buf);

  if (g_sensorData.light_valid)
    snprintf(buf, sizeof(buf), "%d lux", (int)g_sensorData.light);
  else
    snprintf(buf, sizeof(buf), "---- lux");
  lv_label_set_text(labelLight, buf);

  /* 时钟 */
  {
    ClockData_t clk;
    ClockData_Get(&clk);
    if (clk.synced)
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d  %02d/%02d",
               (int)clk.hour, (int)clk.min, (int)clk.sec,
               (int)clk.month, (int)clk.day);
    else
      snprintf(buf, sizeof(buf), "syncing...");
    lv_label_set_text(labelClock, buf);
  }

  /*主屏 OTA 入口高亮由 OTA 模块维护*/
  GUI_RefreshMainOtaEntry();
}
