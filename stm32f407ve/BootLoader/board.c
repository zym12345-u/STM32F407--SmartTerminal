/* BootLoader/board.c
 * 板载 LED、SysTick 毫秒计时和粗延时。
 * 这三个小东西被串口窗口、升级主循环、I2C 总线恢复共用，单独放一处免得各文件抄一遍。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"

/* SysTick 当毫秒计时器用：只计数不产生中断，省掉写中断服务函数和管 NVIC 的麻烦 */
void systick_start_1ms(void)
{
  SysTick->LOAD = 16000UL - 1UL;   /*HCLK=16MHz，数 16000 拍正好 1ms*/
  SysTick->VAL  = 0UL;             /*写 VAL 清零，重装值才会立刻生效*/
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void systick_stop(void)
{
  SysTick->CTRL = 0UL;
}

/* LED 与延时 */
void led_init(void)
{
  RCC->AHB1ENR |= LED_GPIO_CLK_EN;

  LED_PORT->MODER   = (LED_PORT->MODER & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->OTYPER &= ~LED_PIN_MASK;
  LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & ~(3UL << (LED_PIN * 2U))) | (3UL << (LED_PIN * 2U));
  LED_PORT->PUPDR   = (LED_PORT->PUPDR & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->BSRR     = LED_PIN_MASK;   /*输出高，灯灭*/
}

void led_on(void)
{
  LED_PORT->BSRR = (LED_PIN_MASK << 16U);   /*BSRR 高 16 位复位，低电平点亮*/
}

void led_off(void)
{
  LED_PORT->BSRR = LED_PIN_MASK;   /*输出高，灯灭*/
}

/* 跑在 HSI 16MHz 上，只要肉眼可见的闪烁；volatile 防被优化掉 */
void delay_loop(uint32_t n)
{
  volatile uint32_t i;
  for (i = 0UL; i < n; i++)
  {
  }
}
