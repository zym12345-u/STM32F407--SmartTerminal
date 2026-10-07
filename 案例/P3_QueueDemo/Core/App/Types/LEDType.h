#ifndef P3_QUEUEDEMO_LEDTYPE_H
#define P3_QUEUEDEMO_LEDTYPE_H

typedef enum {
  LEDColor_Red = 0,
  LEDColor_Green = 1,
  LEDColor_Blue = 2
} LEDColor;

typedef enum {
  LEDState_Off = 0,
  LEDState_On = 1,
} LEDState;

typedef struct {
  LEDColor color;
  LEDState state;
} LEDMessage;

#endif //P3_QUEUEDEMO_LEDTYPE_H