#ifndef LV_TICK_PORT_H
#define LV_TICK_PORT_H

/* LVGL LV_TICK_CUSTOM 只能指定一个头文件,而 FreeRTOS 的 task.h 要求
 * FreeRTOS.h 先被包含,故用此适配头同时引入两者,供 lv_conf.h 使用。 */
#include "FreeRTOS.h"
#include "task.h"

#endif /* LV_TICK_PORT_H */
