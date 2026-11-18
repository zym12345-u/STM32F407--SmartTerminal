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
#include "main.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "bsp_LCD_ILI9341.h"
#include "bsp_XPT2046.h"
#include "bsp_IWDG.h"
#include "bsp_OTA.h"        /*固件版本号读取：OTA_GetCurrentVer*/
#include "usart.h"
#include "PowerMgr.h"
#include "GuiCommon.h"

/* 主屏对象：WiFi/OTA 页返回时复用 */
static lv_obj_t *s_mainScr;

#define GUI_TASK_STACK_SIZE   2560U


/*可复用样式对象*/

static GuiStyles_t s_styles;        /* 12 个可复用样式，初始化后只读 */

/**
  * 功    能：初始化界面样式（圆角、配色、阴影等），建屏之前调一次
  * 参    数：无
  * 返 回 值：无
  */
static void GUI_InitStyles(void)
{
  /* 背景:纵向渐变 */
  lv_style_init(&s_styles.bg);
  lv_style_set_bg_opa(&s_styles.bg, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.bg, COLOR_BG_TOP);
  lv_style_set_bg_grad_color(&s_styles.bg, COLOR_BG_BOT);
  lv_style_set_bg_grad_dir(&s_styles.bg, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styles.bg, 0);
  lv_style_set_bg_grad_stop(&s_styles.bg, 255);
  lv_style_set_pad_all(&s_styles.bg, 0);

  /* 卡片:圆角 + 边框 + 内边距 + 不透明背景 */
  lv_style_init(&s_styles.card);
  lv_style_set_bg_opa(&s_styles.card, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.card, COLOR_CARD);
  lv_style_set_border_color(&s_styles.card, COLOR_CARD_BORDER);
  lv_style_set_border_width(&s_styles.card, 1);
  lv_style_set_radius(&s_styles.card, 12);
  lv_style_set_pad_all(&s_styles.card, 8);

  /* 卡片数值:大号深色(白卡上极清晰) */
  lv_style_init(&s_styles.cardLabel);
  lv_style_set_text_color(&s_styles.cardLabel, COLOR_TEXT);
  lv_style_set_text_font(&s_styles.cardLabel, &lv_font_montserrat_14);

  /* 卡片标签:小号中灰 */
  lv_style_init(&s_styles.cardSub);
  lv_style_set_text_color(&s_styles.cardSub, COLOR_CARD_TEXT_DARK);
  lv_style_set_text_font(&s_styles.cardSub, &lv_font_montserrat_12);

  /* 绿灯按钮:绿色渐变 + 不透明背景 */
  lv_style_init(&s_styles.btnGreen);
  lv_style_set_bg_opa(&s_styles.btnGreen, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.btnGreen, COLOR_ACCENT_GREEN);
  lv_style_set_bg_grad_color(&s_styles.btnGreen, COLOR_ACCENT_GREEN_D);
  lv_style_set_bg_grad_dir(&s_styles.btnGreen, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styles.btnGreen, 0);
  lv_style_set_bg_grad_stop(&s_styles.btnGreen, 255);
  lv_style_set_radius(&s_styles.btnGreen, 14);
  lv_style_set_border_width(&s_styles.btnGreen, 0);
  lv_style_set_shadow_width(&s_styles.btnGreen, 8);
  lv_style_set_shadow_color(&s_styles.btnGreen, COLOR_ACCENT_GREEN_D);
  lv_style_set_shadow_ofs_y(&s_styles.btnGreen, 2);
  lv_style_set_text_color(&s_styles.btnGreen, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styles.btnGreen, &lv_font_montserrat_14);

  /* 蓝灯按钮:蓝色渐变 + 不透明背景 */
  lv_style_init(&s_styles.btnBlue);
  lv_style_set_bg_opa(&s_styles.btnBlue, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.btnBlue, COLOR_ACCENT_BLUE);
  lv_style_set_bg_grad_color(&s_styles.btnBlue, COLOR_ACCENT_BLUE_D);
  lv_style_set_bg_grad_dir(&s_styles.btnBlue, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styles.btnBlue, 0);
  lv_style_set_bg_grad_stop(&s_styles.btnBlue, 255);
  lv_style_set_radius(&s_styles.btnBlue, 14);
  lv_style_set_border_width(&s_styles.btnBlue, 0);
  lv_style_set_shadow_width(&s_styles.btnBlue, 8);
  lv_style_set_shadow_color(&s_styles.btnBlue, COLOR_ACCENT_BLUE_D);
  lv_style_set_shadow_ofs_y(&s_styles.btnBlue, 2);
  lv_style_set_text_color(&s_styles.btnBlue, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styles.btnBlue, &lv_font_montserrat_14);

  /* WiFi 按钮:中性灰蓝渐变 + 不透明背景 */
  lv_style_init(&s_styles.btnWifi);
  lv_style_set_bg_opa(&s_styles.btnWifi, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.btnWifi, lv_color_hex(0x334155));
  lv_style_set_bg_grad_color(&s_styles.btnWifi, lv_color_hex(0x1e293b));
  lv_style_set_bg_grad_dir(&s_styles.btnWifi, LV_GRAD_DIR_VER);
  lv_style_set_bg_main_stop(&s_styles.btnWifi, 0);
  lv_style_set_bg_grad_stop(&s_styles.btnWifi, 255);
  lv_style_set_radius(&s_styles.btnWifi, 10);
  lv_style_set_border_color(&s_styles.btnWifi, COLOR_CARD_BORDER);
  lv_style_set_border_width(&s_styles.btnWifi, 1);
  lv_style_set_text_color(&s_styles.btnWifi, lv_color_hex(0xffffff));
  lv_style_set_text_font(&s_styles.btnWifi, &lv_font_montserrat_14);

  /* 状态栏文字(深色背景上用浅色) */
  lv_style_init(&s_styles.status);
  lv_style_set_text_color(&s_styles.status, lv_color_hex(0xcbd5e1));
  lv_style_set_text_font(&s_styles.status, &lv_font_montserrat_12);

  /* 卡片左侧色条 */
  lv_style_init(&s_styles.accentBar);
  lv_style_set_bg_opa(&s_styles.accentBar, LV_OPA_COVER);
  lv_style_set_radius(&s_styles.accentBar, 2);

  /* LED 开关:背景(关态灰色) */
  lv_style_init(&s_styles.swBg);
  lv_style_set_bg_opa(&s_styles.swBg, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.swBg, lv_color_hex(0x64748b));
  lv_style_set_radius(&s_styles.swBg, 20);

  /* LED 开关:指示器(开态绿色/蓝色) */
  lv_style_init(&s_styles.swIndOn);
  lv_style_set_bg_opa(&s_styles.swIndOn, LV_OPA_COVER);
  lv_style_set_radius(&s_styles.swIndOn, 20);

  /* LED 开关:旋钮(白色圆形) */
  lv_style_init(&s_styles.swKnob);
  lv_style_set_bg_opa(&s_styles.swKnob, LV_OPA_COVER);
  lv_style_set_bg_color(&s_styles.swKnob, lv_color_hex(0xffffff));
  lv_style_set_radius(&s_styles.swKnob, 20);
}


/*状态栏*/
/**
  * 功    能：取当前固件版本：元数据可信就用它，否则退回 CanData.h 的编译期版本
  * 参    数：major/minor 输出版本号；fromMeta 输出 1=读自元数据 0=编译期常量
  * 返 回 值：无
  * 补    充：判据已抽到 bsp_OTA.c 的 OTA_GetCurrentVer()，检查更新与这里共用一份，
  *            避免两套判据漂移；本函数只做拆分显示
  */
void GUI_GetFwVersion(uint8_t *major, uint8_t *minor, uint8_t *fromMeta)
{
  uint16_t ver = 0U;
  uint8_t  from = OTA_GetCurrentVer(&ver);

  *major    = (uint8_t)((ver >> 8U) & 0xFFU);
  *minor    = (uint8_t)(ver & 0xFFU);
  *fromMeta = from;
}

/**
  * 功    能：返回 12 个可复用样式对象，供拆分后的各 GUI 文件只读使用
  * 参    数：无
  * 返 回 值：样式结构体指针，生命周期同程序
  */
GuiStyles_t *Gui_Styles(void)
{
  return &s_styles;
}

/**
  * 功    能：切回主屏，统一 250ms 淡入
  * 参    数：无
  * 返 回 值：无
  */
void Gui_ShowMainScr(void)
{
  lv_scr_load_anim(s_mainScr, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
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

