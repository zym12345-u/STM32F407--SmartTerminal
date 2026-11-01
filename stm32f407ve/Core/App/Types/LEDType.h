/* LEDType.h
 * LED 控制消息类型：颜色/状态枚举与一条消息结构体，生产者与 LEDTask 共用。
 */
#ifndef APP_LEDTYPE_H
#define APP_LEDTYPE_H

/*枚举值不得重排：CanTask 与 LEDTask 依赖 0=绿 1=蓝*/

typedef enum {
  LEDColor_Green = 0,   /*绿灯 PC5，对应 CAN 灯编号 1*/
  LEDColor_Blue  = 1    /*蓝灯 PB2，对应 CAN 灯编号 2*/
} LEDColor;

typedef enum {
  LEDState_Off = 0,   /*灭，LEDTask 转成引脚 SET*/
  LEDState_On  = 1    /*亮，LEDTask 转成引脚 RESET*/
} LEDState;

/*生产者写，LEDTask 读*/
typedef struct {
  LEDColor color;   /*目标灯，0=绿 1=蓝*/
  LEDState state;   /*目标状态，0=灭 1=亮*/
} LEDMessage;

#endif /* APP_LEDTYPE_H */
