/* USER CODE BEGIN Header */
/* GuiTask.c
 * LVGL 界面任务：ILI9341(FSMC) + XPT2046(GPIO 模拟 SPI) + LVGL v8.3，320×240 横屏暗色界面。
 * 周期：正常 5ms 一轮 lv_timer_handler()，息屏 50ms 轮询触摸；优先级 Normal，栈 2560 字节。
 * 注意：LVGL 单线程，所有 lv_* 只能在本任务及其定时器回调里调用。
 *
 * ══════ LVGL 的渲染机制（理解卡顿、画面撕裂都靠这段）══════
 *
 * ── LVGL 是什么 ──
 *   一个纯软件的嵌入式 GUI 库。它维护一棵【对象树】（屏幕 → 容器 → 按钮/标签…），
 *   不直接操作硬件，而是把「要画什么」算好，再通过你提供的 flush 回调交给底层去刷。
 *
 * ── 谁在驱动它 ──
 *   只要一行：定期调用 lv_timer_handler()。它内部会依次处理：
 *     定时器到期 → 动画推进 → 输入设备读数 → 计算哪些区域脏了 → 组织重绘
 *   本任务 5ms 一轮，所以界面上 5ms 内的变化会合并到下一次处理里，不会漏。
 *   【注意】LVGL 本身没有线程，它就靠你这一行来「活」；没人调它，界面就冻住。
 *
 * ── 关键机制：flush 回调 ──
 *   LVGL 把像素画到一块内存缓冲里，然后调用你注册的 flush_cb 说：
 *   「这一块区域(x,y,w,h)我画好了，你负责弄到屏幕上」。
 *   于是我们在这块回调里发 DMA 把数据送到 LCD。
 *
 * ── 为什么 flush 之后还要「等回调」──
 *   LVGL 只有那【一块】缓冲。如果 DMA 还在往屏幕搬、你就告诉 LVGL「画完了」，
 *   它会立刻开始画下一块并复用同一块缓冲 —— 结果 DMA 搬的是被改过的数据，
 *   屏幕上就会出现半张旧图半张新图，也就是【画面撕裂】。
 *   所以必须等 DMA 真搬完（LCD_SetFlushDoneCallback 注册的那个回调被触发），
 *   再调 lv_disp_flush_ready() 通知 LVGL「缓冲可以复用了」。
 *   这和「DMA 发送完成 ≠ 数据发完」是同一类问题，只是对象从串口换成了屏幕。
 *
 * ── 单线程约束 ──
 *   LVGL 内部没有加锁，所有 lv_* 接口都必须在【同一个线程】里调。
 *   所以别的任务（比如 WifiTask）想改界面，只能通过队列/事件把消息传给本任务，
 *   由本任务去调 lv_*。这就是「LED 控制走 LEDQueue、WiFi 结果走事件队列」的原因。
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
#include "bsp_24C02.h"      /*固件升级入口：写 BootLoader 认的升级标志*/
#include "bsp_bootflag.h"   /*OTA-2：下载成功置搬运请求、串口 IAP 前清请求*/
#include "bsp_W25Q128.h"    /*固件备份：把内部 App 区整段写进外部 Flash 槽 A*/
#include "bsp_OTA.h"        /*WiFi OTA：检查更新/当前版本/下载入口*/
#include "bsp_OtaCfg.h"     /*服务器地址只读缓存：本任务不碰 LittleFS*/
#include "OtaData.h"        /*OTA 进度共享区，WifiTask 写、本任务轮询*/
#include "CanData.h"        /*固件版本号的编译期常量，元数据不可信时退回它*/
#include "usart.h"
#include "PowerMgr.h"
#include <stdio.h>
#include <string.h>

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
static lv_obj_t *labelFwVer;

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

/* WiFi OTA 下载页控件（只下载到槽 B，不切启动） */
static lv_obj_t *s_otaScr;
static lv_obj_t *s_lblOtaWifi;
static lv_obj_t *s_lblOtaServer;
static lv_obj_t *s_lblOtaVer;
static lv_obj_t *s_lblOtaState;
static lv_obj_t *s_lblOtaDetail;
static lv_obj_t *s_barOta;
static lv_obj_t *s_btnOtaStart;
static lv_obj_t *s_btnOtaCheck;
static uint8_t   s_otaCurMajor;
static uint8_t   s_otaCurMinor;
static uint8_t   s_otaUiDirty = 1U;   /*进入页面时置 1，让轮询函数丢弃上次的显示缓存*/

/* 主屏 OTA 入口按钮：检查到新版本时这里换成蓝色高亮，避免新开弹窗 */
static lv_obj_t *s_btnMainOta;
static lv_obj_t *s_lblMainOta;

#define GUI_TASK_STACK_SIZE   2560U

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

/**
  * 功    能：初始化界面样式（圆角、配色、阴影等），建屏之前调一次
  * 参    数：无
  * 返 回 值：无
  */
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
/**
  * 功    能：取当前固件版本：元数据可信就用它，否则退回 CanData.h 的编译期版本
  * 参    数：major/minor 输出版本号；fromMeta 输出 1=读自元数据 0=编译期常量
  * 返 回 值：无
  * 补    充：判据已抽到 bsp_OTA.c 的 OTA_GetCurrentVer()，检查更新与这里共用一份，
  *            避免两套判据漂移；本函数只做拆分显示
  */
static void GUI_GetFwVersion(uint8_t *major, uint8_t *minor, uint8_t *fromMeta)
{
  uint16_t ver = 0U;
  uint8_t  from = OTA_GetCurrentVer(&ver);

  *major    = (uint8_t)((ver >> 8U) & 0xFFU);
  *minor    = (uint8_t)(ver & 0xFFU);
  *fromMeta = from;
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
  lv_obj_add_style(labelWifiIcon, &s_styleStatus, 0);
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
    lv_obj_add_style(labelFwVer, &s_styleStatus, 0);
    lv_obj_set_pos(labelFwVer, 44, 5);

    /*上电日志留证：读完元数据后能看出这个版本号是从哪来的*/
    UART_Printf("[GUI] fw v%u.%u (%s)\n", (unsigned)major, (unsigned)minor,
                (fromMeta != 0U) ? "meta" : "compile");
  }

  /* 时钟(右对齐) */
  labelClock = lv_label_create(scr);
  lv_label_set_text(labelClock, "syncing...");
  lv_obj_add_style(labelClock, &s_styleStatus, 0);
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

/*WiFi 入口*/
static void WifiEntryEventCb(lv_event_t *event);   /* 前向声明 */

/**
  * 功    能：在主屏放一个 WiFi 入口，显示当前状态，点进去是配网页
  * 参    数：scr  主屏对象
  * 返 回 值：无
  */
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

/* ══════ 固件备份：内部 App 区整段 → W25Q 槽 A ══════
 *
 * ── 为什么按整区 448KB 备份，而不是只备份"实际镜像大小" ──
 *   回滚时 BootLoader 的做法是「整段擦掉 App 区、整段写回」，
 *   备份也按同样粒度存，两边的长度、偏移、CRC 口径就永远一致，
 *   不用再在槽里维护一份"这份镜像到底多少字节"的额外信息。
 *   多抄的尾部 0xFF 不影响回滚结果，代价只是多花一秒左右。
 *
 * ── 记录为什么最后写，而且记录 CRC 最后写 ──
 *   判断"槽里有可用备份"只看首 4KB 的 magic + 记录 CRC。
 *   如果先写记录再写镜像，中途断电会留下一条"记录说完整、镜像却只写了一半"的假备份，
 *   BootLoader 拿它回滚就把 App 写坏。所以顺序是：擦记录扇区 → 写镜像 → 回读校验 → 才写记录。
 *   记录内部同理：前 5 个字的 CRC 放在第 6 个字，最后落盘，写完这条记录才算数。
 */
#define BK_APP_BASE     0x08010000UL   /*App 区起点，与 BootLoader 的 APP_BASE_ADDR 同值*/
#define BK_APP_SIZE     0x00070000UL   /*448KB，整区备份*/
#define BK_SECTOR       0x1000UL       /*W25Q128 4KB 扇区*/
#define BK_CHUNK        256U           /*W25Q128 页大小*/
#define BK_SECTORS      (BK_APP_SIZE / BK_SECTOR)
#define BK_PAGES        (BK_SECTOR / BK_CHUNK)

/**
  * 功    能：逐字节更新 CRC32（多项式 0xEDB88320，初值/终值 0xFFFFFFFF）
  * 参    数：crc 当前值；buf 数据；n 字节数
  * 返 回 值：更新后的 CRC32
  * 补    充：与 BootLoader 的 crc32_update、iap_upgrade.py 的 crc32 同一算法
  */
static uint32_t BK_Crc32Bytes(uint32_t crc, const uint8_t *buf, uint32_t n)
{
  uint32_t i;
  uint8_t  bit;

  for (i = 0UL; i < n; i++)
  {
    crc ^= (uint32_t)buf[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }
  return crc;
}

/**
  * 功    能：检查 App 区开头是否像一个能跑的程序
  * 参    数：无
  * 返 回 值：1 = 向量表合理；0 = 空白/写了一半
  * 补    充：判据与 BootLoader 跳转前一致：栈顶在 SRAM 且 8 字节对齐，
  *            复位向量落在 App 区内且 bit0=1（Thumb）。备份前先拦一次，
  *            免得把一份坏镜像当宝贝存进槽里。
  */
static uint8_t BK_AppVectorOk(void)
{
  uint32_t sp = *(volatile const uint32_t *)BK_APP_BASE;
  uint32_t pc = *(volatile const uint32_t *)(BK_APP_BASE + 4UL);

  if ((sp < 0x20000000UL) || (sp > 0x20020000UL) || ((sp & 7UL) != 0UL))
  {
    return 0U;
  }
  if ((pc < BK_APP_BASE) || (pc >= (BK_APP_BASE + BK_APP_SIZE)) || ((pc & 1UL) == 0UL))
  {
    return 0U;
  }
  return 1U;
}

/**
  * 功    能：往槽首 4KB 写镜像记录（32B，前 6 字有效）
  * 参    数：slot 槽基址；size 镜像字节数；crc 镜像 CRC32；ver 版本号(主<<8|次)
  * 返 回 值：1 = 写入成功；0 = 失败
  * 补    充：magic 先写、记录自身 CRC 最后写；调用前记录扇区必须已擦成全 0xFF
  */
static uint8_t BK_WriteHeader(uint32_t slot, uint32_t size, uint32_t crc, uint32_t ver)
{
  uint32_t hdr[8];
  uint32_t i;

  for (i = 0UL; i < 8UL; i++)
  {
    hdr[i] = 0xFFFFFFFFUL;
  }
  hdr[0] = W25Q128_FW_MAGIC;
  hdr[1] = 1UL;                       /*记录格式 1：整区备份*/
  hdr[2] = ver;
  hdr[3] = size;
  hdr[4] = crc;
  hdr[5] = BK_Crc32Bytes(0xFFFFFFFFUL, (const uint8_t *)hdr, 20UL) ^ 0xFFFFFFFFUL;

  if (W25Q128_WriteRange(slot, (const uint8_t *)hdr, 32UL) != HAL_OK)
  {
    UART_Printf("[BK] header write FAIL at 0x%08lX\r\n", (unsigned long)slot);
    return 0U;
  }
  UART_Printf("[BK] header written: slot=0x%08lX ver=%lu.%lu size=0x%lX crc=0x%08lX\r\n",
              (unsigned long)slot,
              (unsigned long)((ver >> 8U) & 0xFFUL), (unsigned long)(ver & 0xFFUL),
              (unsigned long)size, (unsigned long)crc);
  return 1U;
}

/**
  * 功    能：把内部 App 区 448KB 整段备份到 W25Q 指定槽，并回读校验
  * 参    数：slot 槽基址（W25Q128_FW_SLOT_A_ADDR / B）；lbl 进度标签；ver 版本号(主<<8|次)
  * 返 回 值：1 = 备份且校验通过；0 = 任一步失败
  * 补    充：逐扇区擦、逐页写；每扇区补报 GUI 心跳（否则 8 秒的长操作会被 MonitorTask 判卡死）；
  *            每 8 个扇区刷一次进度。整段回读 CRC 与写入时累计的内部 CRC 必须一致。
  */
static uint8_t GUI_BackupFwToSlot(uint32_t slot, lv_obj_t *lbl, uint32_t ver)
{
  static uint32_t buf[BK_CHUNK / 4U];   /*256B 页缓冲，放静态区不占任务栈*/
  uint32_t sect;
  uint32_t page;
  uint32_t i;
  uint32_t src;
  uint32_t dst;
  uint32_t imgBase = slot + W25Q128_FW_HDR_SIZE;
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t readCrc;
  uint32_t t0;
  char     txt[32];

  if (BK_AppVectorOk() == 0U)
  {
    UART_Printf("[BK] app vector invalid, abort\r\n");
    return 0U;
  }
  if (W25Q128_ReadJedecId() != W25Q128_JEDEC_ID_EXPECT)
  {
    UART_Printf("[BK] W25Q not ready, abort\r\n");
    return 0U;
  }

  UART_Printf("[BK] start slot=0x%08lX size=0x%lX ver=%lu.%lu\r\n",
              (unsigned long)slot, (unsigned long)BK_APP_SIZE,
              (unsigned long)((ver >> 8U) & 0xFFUL), (unsigned long)(ver & 0xFFUL));
  t0 = HAL_GetTick();

  lv_label_set_text(lbl, "Backup 0%");
  lv_refr_now(NULL);

  /*记录扇区先擦空：镜像全部校验通过后才往里写记录*/
  if (W25Q128_EraseRange(slot, W25Q128_FW_HDR_SIZE) != HAL_OK)
  {
    UART_Printf("[BK] erase header sector FAIL\r\n");
    return 0U;
  }

  for (sect = 0UL; sect < BK_SECTORS; sect++)
  {
    src = BK_APP_BASE + (sect * BK_SECTOR);
    dst = imgBase + (sect * BK_SECTOR);

    if (W25Q128_SectorErase4K(dst) != HAL_OK)
    {
      UART_Printf("[BK] erase FAIL at 0x%08lX\r\n", (unsigned long)dst);
      return 0U;
    }

    for (page = 0UL; page < BK_PAGES; page++)
    {
      uint32_t off = page * BK_CHUNK;
      const volatile uint32_t *s32 = (const volatile uint32_t *)(src + off);

      /*内部 Flash 是内存映射，按字搬比按字节快；同时顺手累计内部镜像的 CRC*/
      for (i = 0UL; i < (BK_CHUNK / 4U); i++)
      {
        buf[i] = s32[i];
      }
      crc = BK_Crc32Bytes(crc, (const uint8_t *)buf, BK_CHUNK);

      if (W25Q128_PageProgram(dst + off, (const uint8_t *)buf, BK_CHUNK) != HAL_OK)
      {
        UART_Printf("[BK] program FAIL at 0x%08lX\r\n", (unsigned long)(dst + off));
        return 0U;
      }
    }

    Watchdog_Heartbeat(WDT_TASK_GUI);
    if (((sect + 1UL) % 8UL) == 0UL)
    {
      snprintf(txt, sizeof(txt), "Backup %lu%%",
               (unsigned long)(((sect + 1UL) * 100UL) / BK_SECTORS));
      lv_label_set_text(lbl, txt);
      lv_refr_now(NULL);
    }
  }
  crc ^= 0xFFFFFFFFUL;

  /*回读整段再算一遍 CRC：擦/写/读三条链路任何一处出错都会在这里暴露*/
  readCrc = W25Q128_Crc32Range(imgBase, BK_APP_SIZE);
  UART_Printf("[BK] crc internal=0x%08lX readback=0x%08lX\r\n",
              (unsigned long)crc, (unsigned long)readCrc);
  if (crc != readCrc)
  {
    UART_Printf("[BK] FAIL: crc mismatch\r\n");
    return 0U;
  }

  if (BK_WriteHeader(slot, BK_APP_SIZE, crc, ver) != 1U)
  {
    return 0U;
  }

  UART_Printf("[BK] PASS %lu ms\r\n", (unsigned long)(HAL_GetTick() - t0));
  lv_label_set_text(lbl, "Backup 100%");
  lv_refr_now(NULL);
  return 1U;
}

/**
  * 功    能：判断 WiFi OTA 是否正在占用 W25Q（擦除/下载/校验任一阶段）
  * 参    数：无
  * 返 回 值：1 = 忙，0 = 空闲
  * 补    充：串口升级前的整区备份也写 W25Q，两个动作同时进行会抢 SPI1 总线，
  *            所以备份入口先看这个标志；反过来 OTA 下载期间点备份会被拒绝
  */
static uint8_t GUI_OtaBusy(void)
{
  uint8_t st = g_otaStatus.state;

  if ((st == OTA_ST_ERASE) || (st == OTA_ST_META) ||
      (st == OTA_ST_DOWNLOAD) || (st == OTA_ST_VERIFY))
  {
    return 1U;
  }
  return 0U;
}

/*固件升级入口：先备份到槽 A，再写 24C02 标志后软复位，交给 BootLoader*/
#define OTA_FLAG_ADDR   0xF0U   /*与 BootLoader 的 EE_OTA_MEM 是同一页*/

/**
  * 功    能：点击「固件升级」：先把当前 App 备份到 W25Q 槽 A，成功后再写标志并复位
  * 参    数：event  LVGL 事件（取按钮对象用来改文字）
  * 返 回 值：无（成功时不返回，直接复位）
  * 补    充：备份失败就停在原地、不写 24C02、不复位——否则进了升级模式又没有可回滚的镜像，
  *            等于把最后一层保险也丢了。写的是 0xF0 页 8 字节：'U''P''G''2' + 已写字节数 0，
  *            magic 与 BootLoader 的 EE_OTA_MAGIC(0x32475055) 是同一个值（小端序）。
  */
static void FwUpdateEventCb(lv_event_t *event)
{
  lv_obj_t *btn = lv_event_get_target(event);
  lv_obj_t *lbl = lv_obj_get_child(btn, 0);
  uint8_t flag[8] = { 'U', 'P', 'G', '2', 0U, 0U, 0U, 0U };
  uint8_t major;
  uint8_t minor;
  uint8_t fromMeta;
  uint32_t ver;

  /*下载期间槽 B 正在被写，此时再做 448KB 整区备份必然抢总线、两边都出错*/
  if (GUI_OtaBusy() != 0U)
  {
    lv_label_set_text(lbl, "OTA busy, retry later");
    lv_refr_now(NULL);
    return;
  }

  GUI_GetFwVersion(&major, &minor, &fromMeta);
  (void)fromMeta;   /*备份记录里只关心版本号本身，来源不影响回滚*/
  ver = (((uint32_t)major & 0xFFUL) << 8U) | ((uint32_t)minor & 0xFFUL);

  if (GUI_BackupFwToSlot(W25Q128_FW_SLOT_A_ADDR, lbl, ver) == 0U)
  {
    lv_label_set_text(lbl, "Backup failed, retry");
    lv_refr_now(NULL);
    return;
  }

  /* 让用户看到反馈再复位：lv_refr_now 直接刷一帧，不等下一轮 5ms 的 handler */
  lv_label_set_text(lbl, "Backup OK, rebooting...");
  lv_refr_now(NULL);

  /*串口 IAP 与槽 B 搬运只能选一个：先清掉 OTA-2 的搬运请求*/
  BootFlag_Clear();

  if (AT24C02_WriteBytes(OTA_FLAG_ADDR, flag, sizeof(flag)) != 0U)
  {
    /* EEPROM 不在位或写失败：原地提示，不改布局也不复位；槽里的备份仍然有效 */
    lv_label_set_text(lbl, "EEPROM error, retry");
    lv_refr_now(NULL);
    return;
  }

  osDelay(300);
  NVIC_SystemReset();
  for (;;)
  {
  }
}

/*════════════ WiFi OTA 下载页（第一阶段：只下载到槽 B）════════════
 * 为什么不复用弹窗：下载要几十秒，必须能看到“擦除 / 取 meta / 下载 / 校验 / 失败原因”；
 * 为什么退出页面还继续下载：下载由 WifiTask 执行，界面只是观察者，关页面不该中断传输。
 */

/**
  * 功    能：把 OtaErr_t 翻译成一句能看懂的中文提示（界面全英文，避免字库依赖）
  * 参    数：err  OtaErr_t
  * 返 回 值：状态文字指针
  * 补    充：不拼字符串、不跨任务传字符串，GUI 只按码查表
  */
static const char *GUI_OtaErrText(uint8_t err)
{
  switch (err)
  {
    case OTA_ERR_WIFI:    return "WiFi not connected - open WiFi Setup";
    case OTA_ERR_FLASH:   return "W25Q not ready";
    case OTA_ERR_ERASE:   return "Erase slot B failed";
    case OTA_ERR_CONNECT: return "Cannot reach PC server";
    case OTA_ERR_HTTP:    return "HTTP error - check server";
    case OTA_ERR_META:    return "app.meta missing or bad";
    case OTA_ERR_SIZE:    return "Size mismatch - regenerate app.meta";
    case OTA_ERR_TIMEOUT: return "Timeout - check WiFi signal";
    case OTA_ERR_CRC:     return "CRC mismatch - retry download";
    case OTA_ERR_RECORD:  return "Slot record write failed";
    case OTA_ERR_QUEUE:   return "Internal queue unavailable";
    default:              return "Unknown error";
  }
}

/**
  * 功    能：OTA 页返回主屏
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：不取消下载；WifiTask 仍在跑，回来还能看到进度
  */
static void OtaBackEventCb(lv_event_t *event)
{
  (void)event;
  lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
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
  * 功    能：点「Check」：只检查版本，不下载
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：检查由 WifiTask 执行；这里只投命令并先给一行即时反馈
  */
static void OtaCheckEventCb(lv_event_t *event)
{
  (void)event;

  /*下载占用 W25Q 期间不允许并发检查，避免两段 TCP 收包互相踩*/
  if (GUI_OtaBusy() != 0U)
  {
    return;
  }
  if (g_wifiScan.status != WIFI_ST_CONNECTED)
  {
    lv_label_set_text(s_lblOtaState, "WiFi not connected - open WiFi Setup");
    return;
  }

  Wifi_PostCheckOta(1U);
  lv_label_set_text(s_lblOtaState, "Checking...");
}

/**
  * 功    能：点「Download」：检查 WiFi 后投递 OTA 命令
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：只投命令、不在这里下载——ESP8266 与 W25Q 都由 WifiTask 独占
  */
static void OtaStartEventCb(lv_event_t *event)
{
  (void)event;

  if (GUI_OtaBusy() != 0U)
  {
    return;
  }
  /*检查还在跑时先别开始下载：WifiTask 串行处理，下载会排在检查后面，
   *不如让用户看清检查结果再点*/
  if (g_otaStatus.checkState == OTA_CHK_CHECKING)
  {
    return;
  }
  if (g_wifiScan.status != WIFI_ST_CONNECTED)
  {
    lv_label_set_text(s_lblOtaState, "WiFi not connected - open WiFi Setup");
    return;
  }

  Wifi_PostOta();
  lv_label_set_text(s_lblOtaState, "Request sent...");
}

/**
  * 功    能：进入 OTA 页时刷新服务器地址与“当前版本”，并清掉上次遗留的显示缓存
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：版本号仍从 GUI_GetFwVersion 取，和状态栏显示保持一致；
  *            服务器地址每次都读缓存，串口改完配置回页面就能看到新值
  */
static void OtaEntryEventCb(lv_event_t *event)
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
}

/**
  * 功    能：150ms 轮询 g_otaStatus，只在值变化时才刷控件
  * 参    数：timer LVGL 定时器
  * 返 回 值：无
  * 补    充：跨任务读 volatile 字段；每次只取一个字段做显示，不做复合判断，
  *            避免读到“上一阶段的 percent + 下一阶段的 state”这种撕裂组合
  */
static void OtaStatusTimerCb(lv_timer_t *timer)
{
  static uint8_t  lastPct    = 0xFFU;
  static uint8_t  lastWifi   = 0xFFU;
  static uint32_t lastKb     = 0xFFFFFFFFUL;
  static uint16_t lastVer    = 0xFFFFU;
  static uint8_t  lastDisDl  = 0xFFU;
  static uint8_t  lastDisChk = 0xFFU;
  static char     lastStateTxt[48] = {0};
  uint8_t  st;
  uint8_t  err;
  uint8_t  chk;
  uint8_t  chkErr;
  uint8_t  disDl;
  uint8_t  disChk;
  char     txt[80];
  char     stateTxt[48];
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
    lastDisDl  = 0xFFU;
    lastDisChk = 0xFFU;
    lastStateTxt[0] = '\0';
  }

  /*WiFi 状态只在变化时刷*/
  if (lastWifi != (uint8_t)g_wifiScan.status)
  {
    lastWifi = (uint8_t)g_wifiScan.status;
    lv_label_set_text(s_lblOtaWifi,
                      (lastWifi == (uint8_t)WIFI_ST_CONNECTED) ? "WiFi: connected"
                                                              : "WiFi: not connected");
  }

  st     = g_otaStatus.state;
  err    = g_otaStatus.err;
  chk    = g_otaStatus.checkState;
  chkErr = g_otaStatus.checkErr;

  /*状态行优先级：下载进行中 > 检查进行中 > 下载结果 > 检查结果 > 空闲。
   *这样检查失败不会覆盖下载失败文案，下载失败也不会被一次检查成功冲掉。*/
  if ((st == OTA_ST_ERASE) || (st == OTA_ST_META) ||
      (st == OTA_ST_DOWNLOAD) || (st == OTA_ST_VERIFY))
  {
    switch (st)
    {
      case OTA_ST_ERASE:    (void)snprintf(stateTxt, sizeof(stateTxt), "Erasing slot B..."); break;
      case OTA_ST_META:     (void)snprintf(stateTxt, sizeof(stateTxt), "Fetching app.meta..."); break;
      case OTA_ST_DOWNLOAD: (void)snprintf(stateTxt, sizeof(stateTxt), "Downloading app.bin..."); break;
      default:              (void)snprintf(stateTxt, sizeof(stateTxt), "Verifying image..."); break;
    }
  }
  else if (chk == OTA_CHK_CHECKING)
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "Checking...");
  }
  else if (st == OTA_ST_DONE)
  {
    /*rebootPending=1 才是真的会自动复位；标志写失败时停在 DONE 等用户手动复位*/
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s",
                   (g_otaStatus.rebootPending != 0U) ? "Done - rebooting..."
                                                     : "Slot B ready - reboot manually");
  }
  else if (st == OTA_ST_FAIL)
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", GUI_OtaErrText(err));
  }
  else if (chk == OTA_CHK_DONE)
  {
    if (g_otaStatus.checkVer != 0U)
    {
      (void)snprintf(stateTxt, sizeof(stateTxt), "New v%u.%u available",
                     (unsigned)((g_otaStatus.checkVer >> 8U) & 0xFFU),
                     (unsigned)(g_otaStatus.checkVer & 0xFFU));
    }
    else
    {
      (void)snprintf(stateTxt, sizeof(stateTxt), "Up to date (v%u.%u)",
                     (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
    }
  }
  else if (chk == OTA_CHK_FAIL)
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "%s", GUI_OtaErrText(chkErr));
  }
  else
  {
    (void)snprintf(stateTxt, sizeof(stateTxt), "Idle - tap Check or Download");
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

  /*忙时禁用按钮，避免连点把命令队列刷满：检查期间锁 Download，下载期间两个都锁*/
  disDl  = ((GUI_OtaBusy() != 0U) || (chk == OTA_CHK_CHECKING)) ? 1U : 0U;
  disChk = (GUI_OtaBusy() != 0U) ? 1U : 0U;

  if (lastDisDl != disDl)
  {
    lastDisDl = disDl;
    if (disDl != 0U)
    {
      lv_obj_add_state(s_btnOtaStart, LV_STATE_DISABLED);
    }
    else
    {
      lv_obj_clear_state(s_btnOtaStart, LV_STATE_DISABLED);
    }
  }
  if (lastDisChk != disChk)
  {
    lastDisChk = disChk;
    if (disChk != 0U)
    {
      lv_obj_add_state(s_btnOtaCheck, LV_STATE_DISABLED);
    }
    else
    {
      lv_obj_clear_state(s_btnOtaCheck, LV_STATE_DISABLED);
    }
  }
}

/**
  * 功    能：搭 WiFi OTA 页：服务器信息、当前/目标版本、阶段、进度条、下载/返回
  * 参    数：无
  * 返 回 值：无
  * 补    充：控件全用现有样式；卡片高 96 是为了让失败原因能折两行不压进度条
  */
static void GUI_CreateOtaScreen(void)
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
  lv_obj_add_style(s_otaScr, &s_styleBg, 0);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "WiFi OTA");
  lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(lbl, 8, 6);

  s_lblOtaWifi = lv_label_create(s_otaScr);
  lv_label_set_text(s_lblOtaWifi, "WiFi: --");
  lv_obj_add_style(s_lblOtaWifi, &s_styleStatus, 0);
  lv_obj_set_pos(s_lblOtaWifi, 110, 8);

  card = lv_obj_create(s_otaScr);
  lv_obj_remove_style_all(card);
  lv_obj_add_style(card, &s_styleCard, 0);
  lv_obj_set_size(card, 304, 96);
  lv_obj_set_pos(card, 8, 28);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(card, 8, 0);

  s_lblOtaServer = lv_label_create(card);
  lv_label_set_text(s_lblOtaServer, "Server  --");
  lv_obj_add_style(s_lblOtaServer, &s_styleCardSub, 0);
  lv_obj_set_pos(s_lblOtaServer, 0, 0);
  GUI_UpdateOtaServer();   /*建页时先按缓存填一次，Init 跑完/改配置后进页再刷*/

  s_lblOtaVer = lv_label_create(card);
  snprintf(txt, sizeof(txt), "Current v%u.%u  ->  New: --",
           (unsigned)s_otaCurMajor, (unsigned)s_otaCurMinor);
  lv_label_set_text(s_lblOtaVer, txt);
  lv_obj_add_style(s_lblOtaVer, &s_styleCardSub, 0);
  lv_obj_set_pos(s_lblOtaVer, 0, 20);

  s_lblOtaState = lv_label_create(card);
  lv_label_set_text(s_lblOtaState, "Idle - tap Check or Download");
  lv_label_set_long_mode(s_lblOtaState, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(s_lblOtaState, 288);
  lv_obj_set_style_text_color(s_lblOtaState, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(s_lblOtaState, &lv_font_montserrat_14, 0);
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
  lv_obj_add_style(s_lblOtaDetail, &s_styleStatus, 0);
  lv_obj_set_pos(s_lblOtaDetail, 8, 148);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "PC: python Tools/ota_serve.py app_image.bin");
  lv_obj_add_style(lbl, &s_styleStatus, 0);
  lv_obj_set_pos(lbl, 8, 166);

  lbl = lv_label_create(s_otaScr);
  lv_label_set_text(lbl, "Check first, then Download (auto reboot 3s)");
  lv_obj_add_style(lbl, &s_styleStatus, 0);
  lv_obj_set_pos(lbl, 8, 182);

  /*三按钮横排：6+96 / 106+100 / 210+100，总宽 304，右留 6px；文案取短词防裁字*/
  s_btnOtaCheck = lv_btn_create(s_otaScr);
  lv_obj_remove_style_all(s_btnOtaCheck);
  lv_obj_add_style(s_btnOtaCheck, &s_styleBtnBlue, 0);
  lv_obj_set_style_bg_opa(s_btnOtaCheck, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(s_btnOtaCheck, 96, 32);
  lv_obj_set_pos(s_btnOtaCheck, 6, 202);
  lv_obj_add_event_cb(s_btnOtaCheck, OtaCheckEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(s_btnOtaCheck);
  lv_label_set_text(lbl, LV_SYMBOL_REFRESH " Check");
  lv_obj_center(lbl);

  s_btnOtaStart = lv_btn_create(s_otaScr);
  lv_obj_remove_style_all(s_btnOtaStart);
  lv_obj_add_style(s_btnOtaStart, &s_styleBtnGreen, 0);
  lv_obj_set_style_bg_opa(s_btnOtaStart, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(s_btnOtaStart, 100, 32);
  lv_obj_set_pos(s_btnOtaStart, 106, 202);
  lv_obj_add_event_cb(s_btnOtaStart, OtaStartEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(s_btnOtaStart);
  lv_label_set_text(lbl, LV_SYMBOL_DOWNLOAD " Download");
  lv_obj_center(lbl);

  lv_obj_t *btn = lv_btn_create(s_otaScr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &s_styleBtnWifi, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 100, 32);
  lv_obj_set_pos(btn, 210, 202);
  lv_obj_add_event_cb(btn, OtaBackEventCb, LV_EVENT_CLICKED, NULL);
  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_LEFT " Back");
  lv_obj_center(lbl);
}

/**
  * 功    能：主屏底部放「串口升级」与「WiFi OTA」两个入口按钮
  * 参    数：scr 主屏对象
  * 返 回 值：无
  * 补    充：原本是一条 304px 通栏按钮，拆成两半后标签要缩短，否则文字会被裁掉
  */
static void GUI_CreateFwEntry(lv_obj_t *scr)
{
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &s_styleBtnWifi, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(btn, 150, 28);   /*屏宽 320，左右各留 8px；底部空区 206~234*/
  lv_obj_set_pos(btn, 8, 206);
  lv_obj_add_event_cb(btn, FwUpdateEventCb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_DOWNLOAD "  FW Update");
  lv_obj_center(lbl);

  s_btnMainOta = lv_btn_create(scr);
  lv_obj_remove_style_all(s_btnMainOta);
  lv_obj_add_style(s_btnMainOta, &s_styleBtnWifi, 0);
  lv_obj_set_style_bg_opa(s_btnMainOta, LV_OPA_60, LV_STATE_PRESSED);
  lv_obj_set_size(s_btnMainOta, 150, 28);
  lv_obj_set_pos(s_btnMainOta, 164, 206);
  lv_obj_add_event_cb(s_btnMainOta, OtaEntryEventCb, LV_EVENT_CLICKED, NULL);

  s_lblMainOta = lv_label_create(s_btnMainOta);
  lv_label_set_text(s_lblMainOta, LV_SYMBOL_WIFI "  OTA");
  lv_obj_center(s_lblMainOta);
}

/*主屏*/
/**
  * 功    能：把主屏整个搭起来：状态栏 + 传感器卡片 + 灯控卡片 + 功能按钮
  * 参    数：无，内部用 lv_scr_act() 取当前屏
  * 返 回 值：无
  */
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
  GUI_CreateFwEntry(scr);
}

/*传感器刷新*/
/**
  * 功    能：1s 定时器回调：把全局 g_sensorData 的最新值刷到卡片上
  * 参    数：timer  定时器句柄，未使用
  * 返 回 值：无
  * 补    充：只读不写，所以不用加锁（单写多读，各字段单独原子）
  */
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

  /*主屏 OTA 入口：检查到新版本就换蓝底 + 上箭头，下载成功（checkVer 清零）后恢复。
   *只改局部样式，不动共享的 s_styleBtnWifi，避免影响其他按钮*/
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
                        (hi != 0U) ? "OTA  " LV_SYMBOL_UP : LV_SYMBOL_WIFI "  OTA");
    }
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
    lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
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
    lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
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

/**
  * 功    能：GUI 主循环：初始化 LVGL → 建主屏 → 每 5ms 跑一次 lv_timer_handler 处理刷新和触摸
  * 参    数：argument  未使用，创建时传 NULL
  * 返 回 值：无，死循环不返回
  */
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
  GUI_CreateOtaScreen();

  /* LVGL 定时器挂在 lv_timer_handler() 上，只有主循环跑到它才执行 */
  /* 传感器刷新:1s */
  lv_timer_create(SensorRefreshTimerCb, 1000, NULL);   /* 1s 刷新,与 CAN/Modbus 传感器帧同频 */

  /* WiFi 配网事件轮询:100ms */
  lv_timer_create(WifiEventTimerCb, 100, NULL);        /* 100ms 轮询 evtQ,非阻塞 */

  /* OTA 进度轮询:150ms；不在 OTA 页时回调直接返回，不产生额外刷新 */
  lv_timer_create(OtaStatusTimerCb, 150, NULL);

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

/**
  * 功    能：创建 GUI 任务，由 freertos.c 在启动调度器前调一次
  * 参    数：无
  * 返 回 值：无
  */
void GuiTask_Create(void)
{
  static const osThreadAttr_t guiTaskAttr = {
    .name = "GUITask",
    .stack_size = GUI_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
  };
  osThreadNew(StartGuiTask, NULL, &guiTaskAttr);
}
