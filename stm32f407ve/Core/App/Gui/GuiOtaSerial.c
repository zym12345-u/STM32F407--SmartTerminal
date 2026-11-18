/* GuiOtaSerial.c
 * 串口升级入口与槽 A 备份：备份进度显示、OTA 忙判断、写 24C02 标志后软复位。
 * 所有 lv_* 只在 GuiTask 线程调用。
 */

#include "cmsis_os2.h"
#include "main.h"
#include "lvgl.h"
#include "bsp_24C02.h"
#include "bsp_bootflag.h"
#include "bsp_IWDG.h"
#include "bsp_OTA.h"
#include "OtaData.h"
#include "usart.h"
#include "GuiOtaText.h"
#include "GuiOtaInternal.h"
#include <stdio.h>

/*串口升级入口：先备份到槽 A，再写 24C02 标志后软复位，交给 BootLoader 的 3 秒窗口*/
#define OTA_FLAG_ADDR   0xF0U   /*与 BootLoader 的 EE_OTA_MEM 是同一页*/

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
uint8_t GuiOta_EnsureSlotABackup(lv_obj_t *lbl)
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
uint8_t GuiOta_IsBusy(void)
{
  uint8_t st = g_otaStatus.state;

  if ((st == OTA_ST_BACKUP) || (st == OTA_ST_ERASE) || (st == OTA_ST_META) ||
      (st == OTA_ST_DOWNLOAD) || (st == OTA_ST_VERIFY))
  {
    return 1U;
  }
  return 0U;
}

/**
  * 功    能：准备串口 IAP：确保槽 A 有回滚基线，写 0xF0 标志后软复位
  * 参    数：lbl 显示进度与结果的状态标签（不再是小按钮里的文字，避免被裁）
  * 返 回 值：无（成功时不返回，直接复位）
  * 补    充：备份失败就停在原地、不写 24C02、不复位——否则进了升级模式又没有可回滚的镜像，
  *            等于把最后一层保险也丢了。写的是 0xF0 页 8 字节：'U''P''G''2' + 已写字节数 0，
  *            magic 与 BootLoader 的 EE_OTA_MAGIC(0x32475055) 是同一个值（小端序）。
  */
void GuiOta_SerialUpdate(lv_obj_t *lbl)
{
  uint8_t flag[8] = { 'U', 'P', 'G', '2', 0U, 0U, 0U, 0U };

  /*下载期间槽 B 正在被写，此时再做 448KB 整区备份必然抢总线、两边都出错*/
  if (GuiOta_IsBusy() != 0U)
  {
    lv_label_set_text(lbl, "OTA busy, retry later");
    lv_refr_now(NULL);
    return;
  }

  if (GuiOta_EnsureSlotABackup(lbl) == 0U)
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
