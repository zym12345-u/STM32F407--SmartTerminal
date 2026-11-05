/* BootLoader/port/board.c
 * 板载 LED、SysTick 毫秒计时、粗延时、复位原因解析。
 * 这些小东西被串口窗口、升级主循环、I2C 总线恢复共用，单独放一处免得各文件抄一遍。
 * 本文件属于 port/：换板/换芯片时【重写】这里的实现，core 不跟着动。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"
#include "bsp_uart1.h"

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

/**
  * 功    能：打印本次复位的原因，然后把标志清掉
  * 参    数：无
  * 返 回 值：无
  * 补    充：这是排查"升级到一半板子自己重启"最省事的证据。
  *            注意本函数在 port/ 而不是 core/：它读的 RCC->CSR 是【器件级外设寄存器】
  *            (RCC 属外设，SCB/NVIC/SysTick 才是内核)——这是 2026-11-05 驻场复核
  *            指出的分层漏洞，core 的寄存器例外名单因此收窄为 SCB/NVIC/SysTick。
  *            RCC->CSR 里每一位对应一种复位源，读出即知：
  *              POR = 上电复位（电源从 0 爬起来）
  *              BOR = 掉电复位（电压跌破 BOR 门限，擦写 Flash 电流冲击最常见）
  *              PIN = NRST 引脚被拉低（按键、外部干扰、复位芯片）
  *              IWDG/WWDG = 看门狗（BootLoader 根本没开，出现就是异常）
  *              SFT = 软件复位（代码里主动 NVIC_SystemReset）
  *              LPWR = 低功耗相关复位
  *            多个位可能同时置 1（比如掉电后再上电），所以全部打印不取第一个。
  *            读完立刻写 RMVF 清除：不清的话下次上电看到的还是"历史上任意一次"的残留，
  *            证据就失效了。
  */
void rst_cause_dump(void)
{
  uint32_t csr = RCC->CSR;

  uart1_send_str("[BOOT] rst:");
  if ((csr & RCC_CSR_LPWRRSTF) != 0UL) { uart1_send_str(" LPWR"); }
  if ((csr & RCC_CSR_WWDGRSTF) != 0UL) { uart1_send_str(" WWDG"); }
  if ((csr & RCC_CSR_IWDGRSTF) != 0UL) { uart1_send_str(" IWDG"); }
  if ((csr & RCC_CSR_SFTRSTF)  != 0UL) { uart1_send_str(" SFT");  }
  if ((csr & RCC_CSR_PORRSTF)  != 0UL) { uart1_send_str(" POR");  }
  if ((csr & RCC_CSR_PINRSTF)  != 0UL) { uart1_send_str(" PIN");  }
  if ((csr & RCC_CSR_BORRSTF)  != 0UL) { uart1_send_str(" BOR");  }
  uart1_send_str("\r\n");

  RCC->CSR |= RCC_CSR_RMVF;
}
