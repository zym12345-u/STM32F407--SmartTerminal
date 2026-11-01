/* BootLoader/main.c
 * 上电校验 App 区向量表，合法则跳到 App，不合法则闪灯提示。
 * 分区：BootLoader 48KB，元数据 16KB，App 448KB（元数据独占整个 sector 3）。
 */

#include <stdint.h>
#include "stm32f4xx.h"

/* 分区常量：IAP 和 App 侧共用同一组地址 */
#define APP_BASE_ADDR     0x08010000UL   /*App 复位向量（sector 4 起点）*/
#define APP_LIMIT_ADDR    0x08080000UL   /*App 区上界（不含）*/
#define META_BASE_ADDR    0x0800C000UL   /*升级元数据（sector 3）*/
#define META_LIMIT_ADDR   0x08010000UL   /*元数据区上界（不含）*/
#define SRAM_BASE_ADDR    0x20000000UL   /*SRAM 起始*/
#define SRAM_LIMIT_ADDR   0x20020000UL   /*SRAM 上界 128KB*/

/* 板载 LED（诊断用）：PC5 低电平点亮 */
#define LED_GPIO_CLK_EN   RCC_AHB1ENR_GPIOCEN
#define LED_PORT          GPIOC
#define LED_PIN           5U
#define LED_PIN_MASK      (1UL << LED_PIN)

static void led_init(void);
static void led_on(void);
static void led_off(void);
static void delay_loop(uint32_t n);
static int  app_is_valid(void);
static void jump_to_app(void);

int main(void)
{
  if (!app_is_valid())
  {
    /* App 无效就闪灯：灯在闪说明 BootLoader 活着，只是 App 区没有固件 */
    led_init();
    for (;;)
    {
      led_on();
      delay_loop(800000UL);
      led_off();
      delay_loop(800000UL);
    }
  }

  jump_to_app();

  /* 正常情况下不会返回 */
  for (;;)
  {
  }
}

/**
  * 函    数：app_is_valid
  * 参    数：无
  * 返 回 值：1 有效，0 无效
  */
static int app_is_valid(void)
{
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  if ((app_msp < SRAM_BASE_ADDR) || (app_msp >= SRAM_LIMIT_ADDR))
  {
    return 0;
  }
  if ((app_msp & 0x7UL) != 0UL)   /*栈必须 8 字节对齐，否则用 64 位变量就 UsageFault*/
  {
    return 0;
  }
  if (((app_reset & 1UL) == 0UL) ||   /*bit0 必须为 1，Thumb 地址，为 0 立刻 HardFault*/
      (app_reset < APP_BASE_ADDR) || (app_reset >= APP_LIMIT_ADDR))
  {
    return 0;
  }
  return 1;
}

/**
  * 函    数：jump_to_app
  * 参    数：无
  * 返 回 值：无
  */
static void jump_to_app(void)
{
  typedef void (*app_entry_t)(void);

  uint32_t i;
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  __disable_irq();             /*1) 跳转过程中不响应任何中断*/

  /*2) 停 SysTick，App 会自己重新配置*/
  SysTick->CTRL = 0UL;
  SysTick->LOAD = 0UL;
  SysTick->VAL  = 0UL;

  /*3) 清 NVIC 使能位和挂起位，否则 App 一开中断就进 BootLoader 遗留的 handler*/
  for (i = 0UL; i < 8UL; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  /*4) 清 PendSV/PendST 挂起，内核异常 ICPR 管不到*/
  SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

  /* 等上面的寄存器写生效再改 VTOR/MSP */
  __DSB();
  __ISB();

  SCB->VTOR = APP_BASE_ADDR;   /*5) App 的异常/中断才能找到自己的入口*/
  __DSB();
  __ISB();

  __set_MSP(app_msp);          /*6) 硬件复位时 MSP 取自向量表第 0 项，软件跳转必须手动赋值*/

  __enable_irq();              /*7) App 期望复位后总中断已打开，带着 PRIMASK=1 跳过去会卡死*/

  ((app_entry_t)app_reset)();  /*8) 向量表第 1 项就是可跳转的 Thumb 地址*/
}

/* LED 与延时 */
static void led_init(void)
{
  RCC->AHB1ENR |= LED_GPIO_CLK_EN;

  LED_PORT->MODER   = (LED_PORT->MODER & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->OTYPER &= ~LED_PIN_MASK;
  LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & ~(3UL << (LED_PIN * 2U))) | (3UL << (LED_PIN * 2U));
  LED_PORT->PUPDR   = (LED_PORT->PUPDR & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->BSRR     = LED_PIN_MASK;   /*输出高，灯灭*/
}

static void led_on(void)
{
  LED_PORT->BSRR = (LED_PIN_MASK << 16U);   /*BSRR 高 16 位复位，低电平点亮*/
}

static void led_off(void)
{
  LED_PORT->BSRR = LED_PIN_MASK;   /*输出高，灯灭*/
}

/* 跑在 HSI 16MHz 上，只要肉眼可见的闪烁；volatile 防被优化掉 */
static void delay_loop(uint32_t n)
{
  volatile uint32_t i;
  for (i = 0UL; i < n; i++)
  {
  }
}
