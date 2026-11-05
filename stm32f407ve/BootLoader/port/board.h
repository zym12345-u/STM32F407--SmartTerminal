/* BootLoader/port/board.h
 * 板载 LED、SysTick 毫秒计时和粗延时：全部寄存器操作，不依赖 HAL。
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "stm32f4xx.h"

/* 板载 LED（诊断用）：PC5 低电平点亮 */
#define LED_GPIO_CLK_EN   RCC_AHB1ENR_GPIOCEN
#define LED_PORT          GPIOC
#define LED_PIN           5U
#define LED_PIN_MASK      (1UL << LED_PIN)

void systick_start_1ms(void);
void systick_stop(void);
void led_init(void);
void led_on(void);
void led_off(void);
void delay_loop(uint32_t n);
void rst_cause_dump(void);   /*复位原因解析(RCC->CSR)：器件级, 所以在 port 不在 core*/

#endif /* BOARD_H */
