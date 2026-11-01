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
/* 初始化自动息屏模块，程序启动时调一次。 */
void PowerMgr_Init(void);

/* 喂「用户在操作」信号，重置息屏倒计时。按键/触摸/命令都该调它。 */
void PowerMgr_Feed(void);

/* 强制点亮屏幕（息屏中收到触摸或命令时用）。 */
void PowerMgr_Wakeup(void);

/*须在 GuiTask 主循环每轮调用一次*/
/* 周期处理息屏判断，各任务主循环里调。 */
void PowerMgr_Process(void);

/* 当前是否息屏（背光关闭）。返回 1 息屏、0 正常。 */
uint8_t PowerMgr_IsSleeping(void);

#endif /* __POWER_MGR_H */
