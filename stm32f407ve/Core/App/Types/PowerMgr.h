#ifndef __POWER_MGR_H
#define __POWER_MGR_H

#include <stdint.h>

/**
 * @file    PowerMgr.h
 * @brief   低功耗-自动息屏管理模块(跨模块共享接口,放 Types 目录)。
 *          维护"最后一次用户活动时刻",超时后关闭 LCD 背光进入息屏;
 *          触摸/按键等活动触发时退出息屏、点亮背光。
 *          息屏期间 GUI 任务可跳过 lv_timer_handler 渲染并放慢轮询周期,
 *          配合 FreeRTOS tickless idle (configUSE_TICKLESS_IDLE=1)
 *          让 MCU 在空闲时进入 Sleep(__WFI),进一步降低功耗。
 * @note    背光控制使用 bsp_LCD_ILI9341 的 LCD_DisplayOn/Off(实际控制 PA15 背光引脚),
 *          不发送 ILI9341 睡眠命令,唤醒后无需重绘,LVGL 对象树保持完整。
 */

/* 息屏超时(毫秒):无操作超过该时长则自动关闭背光进入息屏 */
#define POWERMGR_SLEEP_TIMEOUT_MS    30000U   /* 30 秒,可按需调整 */

/* 息屏期间 GUI 任务轮询周期(毫秒)。
 * 息屏后无需 5ms 渲染,放慢到 50ms 即可:
 *   - 触摸唤醒响应 < 50ms,用户无感知
 *   - 减少任务唤醒次数,配合 tickless idle 让 MCU 多睡 */
#define POWERMGR_SLEEP_POLL_MS        50U

/**
 * @brief  初始化息屏管理:记录当前 tick 为活动时刻,默认未息屏。
 *         应在 GUITask 进入主循环前调用一次。
 */
void PowerMgr_Init(void);

/**
 * @brief  刷新活动计时。
 *         在触摸按下、按键按下、串口命令处理等"用户活动"点调用,
 *         表示用户仍在操作,推迟息屏时间。
 */
void PowerMgr_Feed(void);

/**
 * @brief  强制唤醒退出息屏(点亮背光 + 刷新活动计时)。
 *         息屏状态下检测到触摸/按键时调用;非息屏时等价于 Feed。
 */
void PowerMgr_Wakeup(void);

/**
 * @brief  周期检查是否超时需进入息屏。
 *         应在 GUITask 主循环中调用(每轮一次)。
 */
void PowerMgr_Process(void);

/**
 * @brief  查询当前是否处于息屏状态。
 * @return 1=息屏中(背光关闭),0=正常显示
 */
uint8_t PowerMgr_IsSleeping(void);

#endif /* __POWER_MGR_H */
