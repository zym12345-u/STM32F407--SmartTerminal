/* GuiOtaScreen.c
 * OTA 下载页、串口升级入口与弹窗提示，所有 lv_* 只在 GuiTask 线程调用。
 * 进度数据由 WifiTask 写 g_otaStatus，本文件只读。
 */

#include "cmsis_os2.h"
#include "main.h"
#include "lvgl.h"
#include "bsp_24C02.h"
#include "bsp_bootflag.h"
#include "bsp_IWDG.h"
#include "bsp_OTA.h"
#include "bsp_OtaCfg.h"
#include "bsp_font_gbk.h"
#include "OtaData.h"
#include "WifiNet.h"
#include "usart.h"
#include "GuiCommon.h"
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

/* 主屏 OTA 入口按钮：检查到新版本时这里换成蓝色高亮，避免新开弹窗 */
static lv_obj_t *s_btnMainOta;
static lv_obj_t *s_lblMainOta;

/* ══════ 固件备份：纯逻辑在 bsp_OTA.c（OTA_BackupAppToSlotA），GUI 只管进度显示 ══════
 * GUI 与远程 MQTT 下载共用同一份备份实现；备份的顺序约束（记录最后写）见 bsp_OTA.c。
 */
#define TXT_ST_BKPCT       "\xE5\xA4\x87\xE4\xBB\xBD\x20\x25\x6C\x75\x25\x25"   /* 备份 %lu%% */
#define TXT_ST_BKPCT_PASS  "\xE5\xA4\x87\xE4\xBB\xBD\x20\x31\x30\x30\x25"       /* 备份 100% */

static lv_obj_t *s_bkLbl;   /*备份进度回调要刷新的标签*/

/**
  * 功    能：备份进度回调：OTA 层每完成一个扇区调一次，这里每 8% 刷一次标签
  * 参    数：percent  0~100
  * 返 回 值：无
  * 补    充：OTA 层只补 WIFI 心跳，GUI 任务这边的心跳必须在这里自己补
  */
static void BK_ProgressCb(uint8_t percent)
{
  char txt[32];

  Watchdog_Heartbeat(WDT_TASK_GUI);
  if (((percent % 8U) == 0U) || (percent == 100U))
  {
    (void)snprintf(txt, sizeof(txt), TXT_ST_BKPCT, (unsigned long)percent);
    lv_label_set_text(s_bkLbl, txt);
    lv_refr_now(NULL);
  }
}

/**
  * 功    能：确保槽 A 里存的就是"正在跑的这版"固件
  * 参    数：lbl 进度标签（备份过程要显示百分比）
  * 返 回 值：1 = 槽 A 可用；0 = 备份失败
  * 补    充：版本一致就跳过，避免每次升级都白磨 448KB；
  *            槽 A 是 BootLoader 搬运失败或新固件连不上时的唯一回滚基线，
  *            必须先立住再下载，否则 BootLoader 会直接 apply refused
  */
static uint8_t GUI_EnsureSlotABackup(lv_obj_t *lbl)
{
  uint16_t cur     = 0U;
  uint32_t slotVer = 0U;
  char     txt[32];

  (void)OTA_GetCurrentVer(&cur);
  if ((OTA_ReadSlotABackupVer(&slotVer) != 0U) && (slotVer == (uint32_t)cur))
  {
    UART_Printf("[BK] slot A already v%u.%u, skip\r\n",
                (unsigned)(cur >> 8U), (unsigned)(cur & 0xFFU));
    return 1U;
  }

  (void)snprintf(txt, sizeof(txt), TXT_ST_BKPCT, 0UL);
  lv_label_set_text(lbl, txt);
  lv_refr_now(NULL);
  s_bkLbl = lbl;

  if (OTA_BackupAppToSlotA((uint32_t)cur, BK_ProgressCb) == 0U)
  {
    return 0U;
  }

  lv_label_set_text(lbl, TXT_ST_BKPCT_PASS);
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

  if ((st == OTA_ST_BACKUP) || (st == OTA_ST_ERASE) || (st == OTA_ST_META) ||
      (st == OTA_ST_DOWNLOAD) || (st == OTA_ST_VERIFY))
  {
    return 1U;
  }
  return 0U;
}

/*串口升级入口：先备份到槽 A，再写 24C02 标志后软复位，交给 BootLoader 的 3 秒窗口*/
#define OTA_FLAG_ADDR   0xF0U   /*与 BootLoader 的 EE_OTA_MEM 是同一页*/

/*弹窗确认键的动作：同一个弹窗要服务「下载」和「串口升级」两条路径*/
#define MSG_ACT_DOWNLOAD   1U
#define MSG_ACT_SERIAL     2U

static uint8_t s_msgAct = MSG_ACT_DOWNLOAD;

/* 弹窗中文文案：必须写成 UTF-8 字节转义，不能直接写汉字。
 * armcc 5 按本地代码页解析源码，源码里的 UTF-8 汉字会让字符串提前结束
 * （报 invalid multibyte character sequence / missing closing quote）。
 * 字节仍是 UTF-8，LVGL 侧配 BSP_Font_Gbk16 即可正常显示。 */
#define TXT_SERIAL_CONFIRM   "\xE4\xB8\xB2\xE5\x8F\xA3\xE5\x8D\x87\xE7\xBA\xA7\xEF\xBC\x9F\x0A\xE5\xB1\x8F\xE5\xB9\x95\xE4\xBC\x9A\xE9\xBB\x91\xE5\xB1\x8F\xE7\xBA\xA6\x20\x33\x20\xE7\xA7\x92\xE3\x80\x82"   /* 串口升级？\n屏幕会黑屏约 3 秒。 */
#define TXT_SERIAL_OK        "\xE4\xB8\xB2\xE5\x8F\xA3"                     /* 串口 */
#define TXT_BK_FAIL          "\xE5\xA4\x87\xE4\xBB\xBD\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\x0A\xE6\x9C\xAA\xE5\xBC\x80\xE5\xA7\x8B\xE4\xB8\x8B\xE8\xBD\xBD\xE3\x80\x82"   /* 备份失败，\n未开始下载。 */
#define TXT_NO_WIFI          "\x57\x69\x46\x69\x20\xE6\x9C\xAA\xE8\xBF\x9E\xE6\x8E\xA5\xEF\xBC\x8C\x0A\xE8\xAF\xB7\xE5\x85\x88\xE8\xBF\x9B\xE5\x85\xA5\x20\x57\x69\x46\x69\x20\xE9\x85\x8D\xE7\xBD\x91\xE3\x80\x82"   /* WiFi 未连接，\n请先进入 WiFi 配网。 */
#define TXT_NEW_VER          "\xE5\x8F\x91\xE7\x8E\xB0\xE6\x96\xB0\xE7\x89\x88\xE6\x9C\xAC\x20\x76\x25\x75\x2E\x25\x75\x0A\xE7\x8E\xB0\xE5\x9C\xA8\xE5\x8D\x87\xE7\xBA\xA7\xEF\xBC\x9F"   /* 发现新版本 v%u.%u\n现在升级？ */
#define TXT_UPDATE           "\xE5\x8D\x87\xE7\xBA\xA7"                     /* 升级 */
#define TXT_UP_TO_DATE       "\xE5\xB7\xB2\xE6\x98\xAF\xE6\x9C\x80\xE6\x96\xB0\xE7\x89\x88\xE6\x9C\xAC\x20\x28\x76\x25\x75\x2E\x25\x75\x29\xE3\x80\x82"   /* 已是最新版本 (v%u.%u)。 */
#define TXT_OK               "\xE7\xA1\xAE\xE5\xAE\x9A"                     /* 确定 */
#define TXT_LATER            "\xE7\xA8\x8D\xE5\x90\x8E"                     /* 稍后 */
#define TXT_TIMEOUT          "\xE6\xA3\x80\xE6\x9F\xA5\xE8\xB6\x85\xE6\x97\xB6\xEF\xBC\x8C\x0A\xE8\xAF\xB7\xE7\xA1\xAE\xE8\xAE\xA4\x20\x50\x43\x20\xE6\x9C\x8D\xE5\x8A\xA1\xE5\xB7\xB2\xE5\x90\xAF\xE5\x8A\xA8\xE3\x80\x82"   /* 检查超时，\n请确认 PC 服务已启动。 */

/* OTA 进行中的状态行：擦写 W25Q 时外挂字模读回 0x00/0xFF 会显示全黑/全白，
   这些阶段统一用内部 ASCII 字体；完成/失败后再恢复中文提示 */
#define TXT_ST_BACKUP      "BACKUP SLOT A..."
#define TXT_ST_BK_FAIL     "\xE5\xA4\x87\xE4\xBB\xBD\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE5\xB7\xB2\xE5\x8F\x96\xE6\xB6\x88\xE4\xB8\x8B\xE8\xBD\xBD"   /* 备份失败，已取消下载 */
#define TXT_ST_SENT        "\xE5\xB7\xB2\xE5\x8F\x91\xE9\x80\x81\xEF\xBC\x8C\xE7\xAD\x89\xE5\xBE\x85\xE5\x93\x8D\xE5\xBA\x94\x2E\x2E\x2E"   /* 已发送，等待响应... */
#define TXT_ST_ERASE       "ERASING SLOT B..."
#define TXT_ST_META        "FETCHING app.meta..."
#define TXT_ST_DOWNLOAD    "DOWNLOADING app.bin..."
#define TXT_ST_VERIFY      "VERIFYING IMAGE..."
#define TXT_ST_CHECKING    "\xE6\xAD\xA3\xE5\x9C\xA8\xE6\xA3\x80\xE6\x9F\xA5\xE6\x9B\xB4\xE6\x96\xB0\x2E\x2E\x2E"   /* 正在检查更新... */
#define TXT_ST_REBOOT      "\xE5\xAE\x8C\xE6\x88\x90\xEF\xBC\x8C\xE5\x8D\xB3\xE5\xB0\x86\xE9\x87\x8D\xE5\x90\xAF\x2E\x2E\x2E"   /* 完成，即将重启... */
#define TXT_ST_MANUAL      "\xE6\xA7\xBD\x20\x42\x20\xE5\xB0\xB1\xE7\xBB\xAA\xEF\xBC\x8C\xE8\xAF\xB7\xE6\x89\x8B\xE5\x8A\xA8\xE9\x87\x8D\xE5\x90\xAF"   /* 槽 B 就绪，请手动重启 */
#define TXT_ST_NEWVER      "\xE5\x8F\x91\xE7\x8E\xB0\xE6\x96\xB0\xE7\x89\x88\xE6\x9C\xAC\x20\x76\x25\x75\x2E\x25\x75"   /* 发现新版本 v%u.%u */
#define TXT_ST_UPTODATE    "\xE5\xB7\xB2\xE6\x98\xAF\xE6\x9C\x80\xE6\x96\xB0\x20\x28\x76\x25\x75\x2E\x25\x75\x29"   /* 已是最新 (v%u.%u) */
#define TXT_ST_IDLE        "\xE7\xA9\xBA\xE9\x97\xB2\xEF\xBC\x8C\xE7\x82\xB9\x20\x55\x70\x64\x61\x74\x65\x20\xE6\xA3\x80\xE6\x9F\xA5"   /* 空闲，点 Update 检查 */
#define TXT_ST_RETRY       "\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE7\x82\xB9\x20\x55\x70\x64\x61\x74\x65\x20\xE9\x87\x8D\xE8\xAF\x95"   /* 失败，点 Update 重试 */
#define TXT_ST_NOWIFI      "\x57\x69\x46\x69\x20\xE6\x9C\xAA\xE8\xBF\x9E\xE6\x8E\xA5"   /* WiFi 未连接 */
#define TXT_ST_BUSY        "\xE5\x8D\x87\xE7\xBA\xA7\xE4\xB8\xAD\xEF\xBC\x8C\xE8\xAF\xB7\xE7\xA8\x8D\xE5\x90\x8E\xE5\x86\x8D\xE8\xAF\x95"   /* 升级中，请稍后再试 */
#define TXT_ST_BKFAIL2     "\xE5\xA4\x87\xE4\xBB\xBD\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE8\xAF\xB7\xE9\x87\x8D\xE8\xAF\x95"   /* 备份失败，请重试 */
#define TXT_ST_BKOK        "BACKUP OK, REBOOT..."
#define TXT_ST_EEPROM      "\x45\x45\x50\x52\x4F\x4D\x20\xE5\x86\x99\xE5\x85\xA5\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE8\xAF\xB7\xE9\x87\x8D\xE8\xAF\x95"   /* EEPROM 写入失败，请重试 */

/* OTA 错误码的中文说明（弹窗用） */
#define E_WIFI               "\x57\x69\x46\x69\x20\xE6\x9C\xAA\xE8\xBF\x9E\xE6\x8E\xA5\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x85\x88\xE9\x85\x8D\xE7\xBD\x91"   /* WiFi 未连接，请先配网 */
#define E_FLASH              "\x57\x32\x35\x51\x20\xE6\x9C\xAA\xE5\xB0\xB1\xE7\xBB\xAA"                     /* W25Q 未就绪 */
#define E_ERASE              "\xE6\x93\xA6\xE9\x99\xA4\xE6\xA7\xBD\x20\x42\x20\xE5\xA4\xB1\xE8\xB4\xA5"                     /* 擦除槽 B 失败 */
#define E_BACKUP             "\xE5\xA4\x87\xE4\xBB\xBD\xE6\xA7\xBD\x20\x41\x20\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE6\x9C\xAA\xE5\xBC\x80\xE5\xA7\x8B\xE4\xB8\x8B\xE8\xBD\xBD\xE3\x80\x82"   /* 备份槽 A 失败，未开始下载。 */
#define E_CONNECT            "\xE8\xBF\x9E\xE4\xB8\x8D\xE4\xB8\x8A\x20\x50\x43\x20\xE6\x9C\x8D\xE5\x8A\xA1\xE5\x99\xA8"                     /* 连不上 PC 服务器 */
#define E_HTTP               "\x48\x54\x54\x50\x20\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x8C\xE6\xA3\x80\xE6\x9F\xA5\xE6\x9C\x8D\xE5\x8A\xA1"                     /* HTTP 错误，检查服务 */
#define E_META               "\x61\x70\x70\x2E\x6D\x65\x74\x61\x20\xE7\xBC\xBA\xE5\xA4\xB1\xE6\x88\x96\xE6\x8D\x9F\xE5\x9D\x8F"                     /* app.meta 缺失或损坏 */
#define E_SIZE               "\xE9\x95\xBF\xE5\xBA\xA6\xE4\xB8\x8D\xE7\xAC\xA6\xEF\xBC\x8C\xE8\xAF\xB7\xE9\x87\x8D\xE6\x96\xB0\xE7\x94\x9F\xE6\x88\x90"                     /* 长度不符，请重新生成 */
#define E_TIMEOUT            "\xE8\xB6\x85\xE6\x97\xB6\xEF\xBC\x8C\xE6\xA3\x80\xE6\x9F\xA5\x20\x57\x69\x46\x69\x20\xE4\xBF\xA1\xE5\x8F\xB7"                     /* 超时，检查 WiFi 信号 */
#define E_CRC                "\x43\x52\x43\x20\xE6\xA0\xA1\xE9\xAA\x8C\xE5\xA4\xB1\xE8\xB4\xA5\xEF\xBC\x8C\xE8\xAF\xB7\xE9\x87\x8D\xE8\xAF\x95"                     /* CRC 校验失败，请重试 */
#define E_RECORD             "\xE6\xA7\xBD\xE8\xAE\xB0\xE5\xBD\x95\xE5\x86\x99\xE5\x85\xA5\xE5\xA4\xB1\xE8\xB4\xA5"                     /* 槽记录写入失败 */
#define E_QUEUE              "\xE5\x86\x85\xE9\x83\xA8\xE9\x98\x9F\xE5\x88\x97\xE4\xB8\x8D\xE5\x8F\xAF\xE7\x94\xA8"                     /* 内部队列不可用 */
#define E_UNKNOWN            "\xE6\x9C\xAA\xE7\x9F\xA5\xE9\x94\x99\xE8\xAF\xAF"                     /* 未知错误 */

static void GUI_ShowMsg(const char *txt, uint8_t confirm, const char *okText);

/**
  * 功    能：准备串口 IAP：确保槽 A 有回滚基线，写 0xF0 标志后软复位
  * 参    数：lbl 显示进度与结果的状态标签（不再是小按钮里的文字，避免被裁）
  * 返 回 值：无（成功时不返回，直接复位）
  * 补    充：备份失败就停在原地、不写 24C02、不复位——否则进了升级模式又没有可回滚的镜像，
  *            等于把最后一层保险也丢了。写的是 0xF0 页 8 字节：'U''P''G''2' + 已写字节数 0，
  *            magic 与 BootLoader 的 EE_OTA_MAGIC(0x32475055) 是同一个值（小端序）。
  */
static void GUI_SerialUpdate(lv_obj_t *lbl)
{
  uint8_t flag[8] = { 'U', 'P', 'G', '2', 0U, 0U, 0U, 0U };

  /*下载期间槽 B 正在被写，此时再做 448KB 整区备份必然抢总线、两边都出错*/
  if (GUI_OtaBusy() != 0U)
  {
    lv_label_set_text(lbl, "OTA busy, retry later");
    lv_refr_now(NULL);
    return;
  }

  if (GUI_EnsureSlotABackup(lbl) == 0U)
  {
    lv_label_set_text(lbl, TXT_ST_BKFAIL2);
    lv_refr_now(NULL);
    return;
  }

  /* 让用户看到反馈再复位：lv_refr_now 直接刷一帧，不等下一轮 5ms 的 handler */
  lv_label_set_text(lbl, TXT_ST_BKOK);
  lv_refr_now(NULL);

  /*串口 IAP 与槽 B 搬运只能选一个：先清掉 OTA-2 的搬运请求*/
  BootFlag_Clear();

  if (AT24C02_WriteBytes(OTA_FLAG_ADDR, flag, sizeof(flag)) != 0U)
  {
    /* EEPROM 不在位或写失败：原地提示，不改布局也不复位；槽里的备份仍然有效 */
    lv_label_set_text(lbl, TXT_ST_EEPROM);
    lv_refr_now(NULL);
    return;
  }

  osDelay(300);
  NVIC_SystemReset();
  for (;;)
  {
  }
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

  if (GUI_OtaBusy() != 0U)
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

static void GUI_StartOtaDownload(void);

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
    GUI_SerialUpdate(s_lblOtaState);
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
  if (GUI_OtaBusy() != 0U)
  {
    return;
  }

  lv_label_set_text(s_lblOtaState, TXT_ST_BACKUP);
  lv_refr_now(NULL);

  if (GUI_EnsureSlotABackup(s_lblOtaState) == 0U)
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
  * 功    能：进入 OTA 页：刷新服务器地址/当前版本，并自动发起一次手动检查
  * 参    数：event LVGL 事件
  * 返 回 值：无
  * 补    充：用户只按一个 Update，不区分"检查"和"下载"；
  *            检查结果由 150ms 轮询在 OtaStatusTimerCb 里弹窗，
  *            这里只负责投命令并把 s_otaFlow 立起来
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

  /*下载占用 W25Q 期间不做检查：两段 TCP 收包会互相踩*/
  if (GUI_OtaBusy() != 0U)
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
