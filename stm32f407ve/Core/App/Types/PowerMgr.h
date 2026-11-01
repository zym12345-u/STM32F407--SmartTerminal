#ifndef __POWER_MGR_H
#define __POWER_MGR_H

#include <stdint.h>

/* PowerMgr.h
 * 低功耗息屏管理：记录最后一次用户活动时刻，超时关闭 LCD 背光，活动到来时唤醒。
 */
/*只关背光(PA15)不睡 ILI9341，唤醒无需重绘，LVGL 对象树保持完整*/

/*息屏超时，单位 ms，建议 1000~600000，只读，Process 用作判据*/
#define POWERMGR_SLEEP_TIMEOUT_MS    30000U   /*30 秒，可调*/

/*息屏期间 GUI 轮询周期，单位 ms，只读；放慢到 50ms 减少唤醒，触摸响应仍 <50ms*/
#define POWERMGR_SLEEP_POLL_MS        50U

/*须在 GuiTask 主循环前调用一次*/
/**
  * 函    数：PowerMgr_Init
  * 参    数：无
  * 返 回 值：无
  */
void PowerMgr_Init(void);

/**
  * 函    数：PowerMgr_Feed
  * 参    数：无
  * 返 回 值：无
  */
void PowerMgr_Feed(void);

/**
  * 函    数：PowerMgr_Wakeup
  * 参    数：无
  * 返 回 值：无
  */
void PowerMgr_Wakeup(void);

/*须在 GuiTask 主循环每轮调用一次*/
/**
  * 函    数：PowerMgr_Process
  * 参    数：无
  * 返 回 值：无
  */
void PowerMgr_Process(void);

/**
  * 函    数：PowerMgr_IsSleeping
  * 参    数：无
  * 返 回 值：1=息屏中(背光关闭)，0=正常显示
  */
uint8_t PowerMgr_IsSleeping(void);

#endif /* __POWER_MGR_H */
