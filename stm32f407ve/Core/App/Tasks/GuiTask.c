/* USER CODE BEGIN Header */
/* GuiTask.c
 * LVGL 界面任务：ILI9341(FSMC) + XPT2046(GPIO 模拟 SPI) + LVGL v8.3，320×240 横屏暗色界面。
 * 周期：正常 5ms 一轮 lv_timer_handler()，息屏 50ms 轮询触摸；优先级 Normal，栈 2560 字节。
 * 注意：LVGL 单线程，所有 lv_* 只能在本任务及其定时器回调里调用。
 */
/* USER CODE END Header */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "bsp_LCD_ILI9341.h"
#include "bsp_XPT2046.h"
#include "bsp_IWDG.h"
#include "LEDType.h"
#include "SensorData.h"
#include "ClockData.h"
#include "WifiNet.h"
#include "bsp_font_gbk.h"   /*中文走外挂字库，内部 262KB 字库已关闭*/
#include "usart.h"
#include "PowerMgr.h"
#include <stdio.h>

/* LEDQueue 由 freertos.c 中 CubeMX 生成的代码创建 */
extern osMessageQueueId_t LEDQueueHandle;

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

/* 传感器数据显示标签 */
static lv_obj_t *labelTemp;
static lv_obj_t *labelHumi;
static lv_obj_t *labelLight;

/* 顶部状态栏 */
static lv_obj_t *labelClock;
static lv_obj_t *labelWifiIcon;

/* WiFi 配网页控件 */
static lv_obj_t *s_mainScr;
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

#define GUI_TASK_STACK_SIZE   2560U

/*LED 命令投递*/

/*队列只传指针：这里 pvPortMalloc，LEDTask 负责 vPortFree，投递失败必须自己释放*/
/* 往 LEDQueue 投一条点灯指令。GUI 按钮点灯也走这条统一通道，不直接碰 GPIO。 */
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

/*可复用样式对象*/

static lv_style_t s_styleBg;        /* 全屏渐变背景 */
static lv_style_t s_styleCard;      /* 传感器卡片 */
static lv_style_t s_styleCardLabel; /* 卡片数值文字 */
static lv_style_t s_styleCardSub;   /* 卡片标签文字 */
static lv_style_t s_styleBtnGreen;  /* 绿灯按钮 */
static lv_style_t s_styleBtnBlue;   /* 蓝灯按钮 */
static lv_style_t s_styleBtnWifi;   /* WiFi 按钮 */
static lv_style_t s_styleStatus;    /* 状态栏文字 */
static lv_style_t s_styleAccentBar; /* 卡片左侧色条 */
/* LED 开关样式 */
static lv_style_t s_styleSwBg;      /* 开关背景(关) */
static lv_style_t s_styleSwIndOn;   /* 开关指示器(开) */
static lv_style_t s_styleSwKnob;    /* 开关旋钮 */

/* 初始化界面样式（圆角、配色、阴影等），建屏之前调一次。 */
static void GUI_InitStyles(void)
{
  /* 背景:纵向渐变 */
  lv_style_init(&s_styleBg);
  lv_style_set_bg_opa(&s_styleBg, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleBg, COLOR_BG_TOP);
  lv_style_set_bg_grad_color(&s_styleBg, COLOR_BG_BOT);
  lv_style_set_bg_grad_dir(&s_styleBg, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styleBg, 0);
  lv_style_set_bg_grad_stop(&s_styleBg, 255);
  lv_style_set_pad_all(&s_styleBg, 0);

  /* 卡片:圆角 + 边框 + 内边距 + 不透明背景 */
  lv_style_init(&s_styleCard);
  lv_style_set_bg_opa(&s_styleCard, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleCard, COLOR_CARD);
  lv_style_set_border_color(&s_styleCard, COLOR_CARD_BORDER);
  lv_style_set_border_width(&s_styleCard, 1);
  lv_style_set_radius(&s_styleCard, 12);
  lv_style_set_pad_all(&s_styleCard, 8);

  /* 卡片数值:大号深色(白卡上极清晰) */
  lv_style_init(&s_styleCardLabel);
  lv_style_set_text_color(&s_styleCardLabel, COLOR_TEXT);
  lv_style_set_text_font(&s_styleCardLabel, &lv_font_montserrat_14);

  /* 卡片标签:小号中灰 */
  lv_style_init(&s_styleCardSub);
  lv_style_set_text_color(&s_styleCardSub, COLOR_CARD_TEXT_DARK);
  lv_style_set_text_font(&s_styleCardSub, &lv_font_montserrat_12);

  /* 绿灯按钮:绿色渐变 + 不透明背景 */
  lv_style_init(&s_styleBtnGreen);
  lv_style_set_bg_opa(&s_styleBtnGreen, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleBtnGreen, COLOR_ACCENT_GREEN);
  lv_style_set_bg_grad_color(&s_styleBtnGreen, COLOR_ACCENT_GREEN_D);
  lv_style_set_bg_grad_dir(&s_styleBtnGreen, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styleBtnGreen, 0);
  lv_style_set_bg_grad_stop(&s_styleBtnGreen, 255);
  lv_style_set_radius(&s_styleBtnGreen, 14);
  lv_style_set_border_width(&s_styleBtnGreen, 0);
  lv_style_set_shadow_width(&s_styleBtnGreen, 8);
  lv_style_set_shadow_color(&s_styleBtnGreen, COLOR_ACCENT_GREEN_D);
  lv_style_set_shadow_ofs_y(&s_styleBtnGreen, 2);
  lv_style_set_text_color(&s_styleBtnGreen, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styleBtnGreen, &lv_font_montserrat_14);

  /* 蓝灯按钮:蓝色渐变 + 不透明背景 */
  lv_style_init(&s_styleBtnBlue);
  lv_style_set_bg_opa(&s_styleBtnBlue, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleBtnBlue, COLOR_ACCENT_BLUE);
  lv_style_set_bg_grad_color(&s_styleBtnBlue, COLOR_ACCENT_BLUE_D);
  lv_style_set_bg_grad_dir(&s_styleBtnBlue, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styleBtnBlue, 0);
  lv_style_set_bg_grad_stop(&s_styleBtnBlue, 255);
  lv_style_set_radius(&s_styleBtnBlue, 14);
  lv_style_set_border_width(&s_styleBtnBlue, 0);
  lv_style_set_shadow_width(&s_styleBtnBlue, 8);
  lv_style_set_shadow_color(&s_styleBtnBlue, COLOR_ACCENT_BLUE_D);
  lv_style_set_shadow_ofs_y(&s_styleBtnBlue, 2);
  lv_style_set_text_color(&s_styleBtnBlue, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styleBtnBlue, &lv_font_montserrat_14);

  /* WiFi 按钮:中性灰蓝渐变 + 不透明背景 */
  lv_style_init(&s_styleBtnWifi);
  lv_style_set_bg_opa(&s_styleBtnWifi, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleBtnWifi, lv_color_hex(0x334155));
  lv_style_set_bg_grad_color(&s_styleBtnWifi, lv_color_hex(0x1e293b));
  lv_style_set_bg_grad_dir(&s_styleBtnWifi, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styleBtnWifi, 0);
  lv_style_set_bg_grad_stop(&s_styleBtnWifi, 255);
  lv_style_set_radius(&s_styleBtnWifi, 10);
  lv_style_set_border_color(&s_styleBtnWifi, COLOR_CARD_BORDER);
  lv_style_set_border_width(&s_styleBtnWifi, 1);
  lv_style_set_text_color(&s_styleBtnWifi, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styleBtnWifi, &lv_font_montserrat_14);

  /* 状态栏文字(深色背景上用浅色) */
  lv_style_init(&s_styleStatus);
  lv_style_set_text_color(&s_styleStatus, lv_color_hex(0xcbd5e1));
  lv_style_set_text_font(&s_styleStatus, &lv_font_montserrat_12);

  /* 卡片左侧色条 */
  lv_style_init(&s_styleAccentBar);
  lv_style_set_bg_opa(&s_styleAccentBar, LV_OPA_COVER);
  lv_style_set_radius(&s_styleAccentBar, 2);

  /* LED 开关:背景(关态灰色) */
  lv_style_init(&s_styleSwBg);
  lv_style_set_bg_opa(&s_styleSwBg, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleSwBg, lv_color_hex(0x64748b));
  lv_style_set_radius(&s_styleSwBg, 20);

  /* LED 开关:指示器(开态绿色/蓝色) */
  lv_style_init(&s_styleSwIndOn);
  lv_style_set_bg_opa(&s_styleSwIndOn, LV_OPA_COVER);
  lv_style_set_radius(&s_styleSwIndOn, 20);

  /* LED 开关:旋钮(白色圆形) */
  lv_style_init(&s_styleSwKnob);
  lv_style_set_bg_opa(&s_styleSwKnob, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styleSwKnob, lv_color_hex(0xffffff));
  lv_style_set_radius(&s_styleSwKnob, 20);
}

/*传感器卡片*/
/* 建一张传感器卡片：图标 + 名称 + 数值。返回数值那个 label，
 * 后续 1s 定时器直接改它的文字来刷新。 */
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
  lv_obj_add_style(bar, &s_styleAccentBar, 0);
  lv_obj_set_style_bg_color(bar, accent, 0);
  lv_obj_set_size(bar, 4, h - 8);                    /* 4px 宽色条,上下各留 4px */
  lv_obj_set_pos(bar, 4, 4);

  /* 数值(大) */
  lv_obj_t *val = lv_label_create(card);
  lv_label_set_text(val, initVal);
  lv_obj_add_style(val, &s_styleCardLabel, 0);
  lv_obj_set_pos(val, 14, 6);

  /* 标签(小,右侧) */
  lv_obj_t *lbl = lv_label_create(card);
  lv_label_set_text(lbl, labelText);
  lv_obj_add_style(lbl, &s_styleCardSub, 0);
  lv_obj_align(lbl, LV_ALIGN_RIGHT_MID, -8, 0);

  return val;
}

/*状态栏*/
/* 在主屏顶部建状态栏（WiFi 状态、时间之类）。 */
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
  lv_obj_add_style(labelWifiIcon, &s_styleStatus, 0);
  lv_obj_set_pos(labelWifiIcon, 8, 5);

  /* 时钟(右对齐) */
  labelClock = lv_label_create(scr);
  lv_label_set_text(labelClock, "syncing...");
  lv_obj_add_style(labelClock, &s_styleStatus, 0);
  lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, -8, 5);
}

/* 灯开关的事件回调：从 user_data 取出是哪个灯，投 LEDQueue，同时更新按钮外观。
 * 实际点灯还是 LEDTask 干的，GUI 只负责把意图发出去。 */
static void LedSwitchEventCb(lv_event_t *event)
{
  lv_obj_t *sw = lv_event_get_target(event);
  uint32_t color = (uint32_t)lv_obj_get_user_data(sw);
  bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

  /*板载 LED 低电平点亮：CHECKED=亮*/
  LEDState target = on ? LEDState_On : LEDState_Off;
  GUI_LedPost((LEDColor)color, target);
}

/* 建一张灯控卡片（开关 + 状态指示）。灯的颜色存在 user_data 里，供回调取用。 */
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
  lv_obj_add_style(sw, &s_styleSwBg, LV_PART_MAIN);
  lv_obj_add_style(sw, &s_styleSwKnob, LV_PART_KNOB);

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
/* 在主屏上建几个功能按钮（比如进 WiFi 配网页）。 */
static void GUI_CreateButtons(lv_obj_t *scr)
{
  /* LED 开关竖向排列在传感器框右下角 */
  GUI_CreateLedCard(scr, 170, 138, 142, 30,
                    COLOR_ACCENT_GREEN, LEDColor_Green);
  GUI_CreateLedCard(scr, 170, 172, 142, 30,
                    COLOR_ACCENT_BLUE, LEDColor_Blue);
}

/*WiFi 入口*/
static void WifiEntryEventCb(lv_event_t *event);   /* 前向声明 */

/* 在主屏放一个 WiFi 入口，显示当前状态，点进去是配网页。 */
static void GUI_CreateWifiEntry(lv_obj_t *scr)
{
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &s_styleBtnWifi, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 150, 60);
  lv_obj_set_pos(btn, 8, 138);
  lv_obj_add_event_cb(btn, WifiEntryEventCb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_WIFI "\nWiFi Setup");
  lv_obj_center(lbl);
}

/*主屏*/
/* 把主屏整个搭起来：状态栏 + 传感器卡片 + 灯控卡片 + 功能按钮。 */
static void GUI_CreateMainScreen(void)
{
  lv_obj_t *scr = lv_scr_act();
  lv_obj_remove_style_all(scr);
  lv_obj_add_style(scr, &s_styleBg, 0);

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
}

/*传感器刷新*/
/* 1s 定时器回调：把全局 g_sensorData 的最新值刷到卡片上。
 * 只读不写，所以不用加锁（单写多读，各字段单独原子）。 */
static void SensorRefreshTimerCb(lv_timer_t *timer)
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
}

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
/* 动画回调：按当前动画值设置对象的纵向偏移，用来做滑入滑出效果。 */
static void AnimSetTranslateY(void *var, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)var, v, 0);
}

/* 用最新的扫描结果重建热点列表。 */
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

/* 一次性定时器回调：延时到点后把界面切回去，进函数先把自己删掉。 */
static void WifiReturnTimerCb(lv_timer_t *timer)
{
    lv_timer_del(timer);
    lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
}

/* 提交连接请求：把选中的 SSID 和用户输入的密码投给 WifiTask，不等结果。
 * 开放网络密码传空串。 */
static void WifiSubmitJoin(const char *pwd)
{
    s_wifiBusy = 1U;
    lv_obj_add_state(s_btnRescan, LV_STATE_DISABLED);
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_lblWifiStatus, "Connecting (up to 40s)...");
    Wifi_PostJoin(s_selSsid, pwd);
}

/* 点某个热点的事件回调：记下选中的下标，开放网络直接连，加密的弹密码框。 */
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

/* 点「重新扫描」：投一条扫描命令给 WifiTask。 */
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

/* 点「返回」：从配网页退回主屏。 */
static void WifiBackCb(lv_event_t *event)
{
    (void)event;
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
}

/* 密码框点「确定」：取出输入内容并提交连接。 */
static void WifiPwdOkCb(lv_event_t *event)
{
    const char *pwd;
    (void)event;
    pwd = lv_textarea_get_text(s_pwdTa);
    WifiSubmitJoin((pwd != NULL) ? pwd : "");
}

/* 密码框点「取消」：收起键盘和密码框，不连接。 */
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

/* 密码框隐藏动画结束时的回调，用来做收尾清理（比如把键盘也收掉）。 */
static void WifiPwdHideReady(lv_anim_t *a)
{
    (void)a;
    lv_obj_add_flag(s_pwdPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_translate_y(s_pwdPanel, 0, 0);  /* 复位 translate,下次滑入从 0 开始 */
}

/* 定时器：轮询 WifiTask 的事件队列，处理「扫描完成」「连接成功/失败」。
 * GUI 就是这样拿到 WiFi 的异步结果的。 */
/*不在配网页时事件不取走，会一直堆在 evtQ，故进页时必须先清陈旧事件*/
static void WifiEventTimerCb(lv_timer_t *timer)
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

/* 点主屏的 WiFi 入口：切到配网页，并发起一次扫描。 */
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
/* 把配网页搭起来：热点列表 + 密码输入框 + 软键盘。 */
static void GUI_CreateWifiScreen(void)
{
    lv_obj_t *lblTitle;
    lv_obj_t *btn;
    lv_obj_t *lbl;

    s_wifiScr = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_wifiScr);
    lv_obj_add_style(s_wifiScr, &s_styleBg, 0);

    /* 标题 */
    lblTitle = lv_label_create(s_wifiScr);
    lv_label_set_text(lblTitle, "WiFi Setup");
    lv_obj_set_style_text_color(lblTitle, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lblTitle, 8, 6);

    s_lblWifiStatus = lv_label_create(s_wifiScr);
    lv_label_set_text(s_lblWifiStatus, "Scanning...");
    lv_obj_add_style(s_lblWifiStatus, &s_styleStatus, 0);
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
    lv_obj_add_style(s_btnRescan, &s_styleBtnWifi, 0);
    lv_obj_set_style_bg_opa(s_btnRescan, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(s_btnRescan, 150, 34);
    lv_obj_set_pos(s_btnRescan, 6, 200);
    lv_obj_add_event_cb(s_btnRescan, WifiRescanCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(s_btnRescan);
    lv_label_set_text(lbl, LV_SYMBOL_REFRESH "  Rescan");
    lv_obj_center(lbl);

    btn = lv_btn_create(s_wifiScr);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &s_styleBtnWifi, 0);
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
    lv_obj_add_style(btn, &s_styleBtnWifi, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
    lv_obj_set_size(btn, 56, 28);
    lv_obj_set_pos(btn, 202, 19);
    lv_obj_add_event_cb(btn, WifiPwdCancelCb, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Cancel");
    lv_obj_center(lbl);

    btn = lv_btn_create(s_pwdPanel);
    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &s_styleBtnGreen, 0);
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

/*任务入口*/

/* GUI 主循环：初始化 LVGL → 建主屏 → 每 5ms 跑一次 lv_timer_handler 处理刷新和触摸。
 * 死循环不返回。 */
void StartGuiTask(void *argument)
{
  (void)argument;

  UART_Printf("\r\n[GUI] start\r\n");
  LCD_Init();
  UART_Printf("[GUI] LCD init ok, %dx%d\r\n", LCD_GetWidth(), LCD_GetHeight());
  LCD_SetDir(1U);
  XPT2046_Init(LCD_GetWidth(), LCD_GetHeight(), 1U);
  UART_Printf("[GUI] touch init ok\r\n");

  lv_init();
  UART_Printf("[GUI] lv_init ok\r\n");
  lv_port_disp_init();
  UART_Printf("[GUI] disp port ok\r\n");
  lv_port_indev_init();
  UART_Printf("[GUI] indev port ok\r\n");

  /* 初始化可复用样式 */
  GUI_InitStyles();

  /* 创建界面 */
  s_mainScr = lv_scr_act();
  GUI_CreateMainScreen();
  GUI_CreateWifiScreen();

  /* LVGL 定时器挂在 lv_timer_handler() 上，只有主循环跑到它才执行 */
  /* 传感器刷新:1s */
  lv_timer_create(SensorRefreshTimerCb, 1000, NULL);   /* 1s 刷新,与 CAN/Modbus 传感器帧同频 */

  /* WiFi 配网事件轮询:100ms */
  lv_timer_create(WifiEventTimerCb, 100, NULL);        /* 100ms 轮询 evtQ,非阻塞 */

  UART_Printf("[GUI] modern UI created, entering loop\r\n");

  PowerMgr_Init();

  /* lv_timer_handler 的调用周期决定流畅度：正常 5ms，息屏跳过*/

  /**
   * 主循环双状态，由 PowerMgr_IsSleeping() 选分支
   * 状态     含义    动作                                       周期/超时
   * 正常显示 背光开  PowerMgr_Process() + lv_timer_handler()    5ms 一轮，30000ms 无操作转息屏
   * 息屏     背光关  只查 XPT2046_IsPressed()，有触摸则 Wakeup  50ms 轮询，无超时，必须人为唤醒
   * 两个分支入口都先 Watchdog_Heartbeat(WDT_TASK_GUI)。
   */
  for(;;)
  {
    /*每轮上报，远快于 3s 判死线*/
    Watchdog_Heartbeat(WDT_TASK_GUI);

    if (PowerMgr_IsSleeping() != 0U)
    {
      /*息屏：只查触摸唤醒*/
      if (XPT2046_IsPressed() != 0U)
        PowerMgr_Wakeup();
      osDelay(POWERMGR_SLEEP_POLL_MS);     /* 息屏:50ms 轮询一次触摸,配合 tickless idle 省电 */
    }
    else
    {
      PowerMgr_Process();
      lv_timer_handler();
      osDelay(5);                          /* 5ms 一轮:动画与触摸响应的流畅度基准 */
    }
  }
}

/* 创建 GUI 任务，由 freertos.c 在启动调度器前调一次。 */
void GuiTask_Create(void)
{
  static const osThreadAttr_t guiTaskAttr = {
    .name = "GUITask",
    .stack_size = GUI_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
  };
  osThreadNew(StartGuiTask, NULL, &guiTaskAttr);
}
