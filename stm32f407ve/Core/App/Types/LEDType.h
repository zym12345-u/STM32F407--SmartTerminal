#ifndef APP_LEDTYPE_H
#define APP_LEDTYPE_H

/* LED 控制消息类型(参考 P3_QueueDemo/Core/App/Types/LEDType.h)
 * 本工程只实现绿灯、蓝灯两种颜色 */

typedef enum {
  LEDColor_Green = 0,
  LEDColor_Blue  = 1
} LEDColor;

typedef enum {
  LEDState_Off = 0,
  LEDState_On  = 1
} LEDState;

typedef struct {
  LEDColor color;   /* 要控制的灯(绿/蓝) */
  LEDState state;   /* 目标状态(亮/灭) */
} LEDMessage;

#endif /* APP_LEDTYPE_H */
