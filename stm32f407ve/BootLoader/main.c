/* BootLoader/main.c
 * 上电校验 App 区向量表，合法则跳到 App，不合法则闪灯提示。
 * 分区：BootLoader 48KB，元数据 16KB，App 448KB（元数据独占整个 sector 3）。
 *
 * ══════ 分区为什么这么切、跳转为什么要做一串动作 ══════
 *
 * ── 分区位置不是随便定的，得跟着【扇区边界】走 ──
 *   STM32F407 内部 Flash 的扇区大小是【不均匀】的（这点很关键）：
 *     sector 0~3   ：每个 16KB   → 0x08000000 ~ 0x0800FFFF
 *     sector 4     ：64KB        → 0x08010000 ~ 0x0801FFFF
 *     sector 5~11  ：每个 128KB  → 0x08020000 起
 *   而擦除的最小单位就是【整个扇区】，所以：
 *     - 元数据只需要几十字节，但最少也得占一整个扇区 → 塞进 sector 3（16KB）
 *     - App 从 sector 4 开始（0x08010000），正好接在元数据后面，边界天然对齐
 *     - 如果硬要让 App 从 0x0800C000 开始，就跨在 sector 3 中间，
 *       将来单独升级 App 时没法只擦 App 那部分（会连带擦掉元数据）
 *   这就是「分区大小必须等于整数个扇区」的原因。
 *
 * ── 为什么要校验 App 的向量表 ──
 *   App 区可能是空的（全 0xFF，刚出厂或刚被擦掉），也可能只写了一半（升级掉电）。
 *   直接跳过去就是跑飞跑进 HardFault，而且看不出原因。
 *   所以跳之前先看 App 那 8 个字节（向量表开头）合不合理：
 *     第 0~3 字节：初始栈顶 MSP → 必须落在 SRAM 范围内，且 8 字节对齐
 *     第 4~7 字节：复位向量   → bit0 必须为 1（Thumb 地址），且落在 App 区内
 *   这两个条件就是「这段 Flash 里像不像一个能跑的程序」的最低判据。
 *
 * ── 跳转前为什么必须做那一串动作（顺序也有讲究）──
 *   1) 关全局中断    ：跳转过程中不能被打断，此时中断向量还指向 BootLoader
 *   2) 停 SysTick    ：否则每 1ms 触发一次，而对应的 handler 马上就不归我们管了
 *   3) 清 NVIC 的使能与挂起位：App 一开中断，不会突然跳回 BootLoader 里残留的 handler
 *   4) 设 MSP        ：用 App 向量表里的栈顶值，让 App 有自己的栈
 *   5) 改 SCB->VTOR  ：让中断向量表指向 App 的，否则 App 里所有中断都会跑错地方
 *   6) 取复位向量跳过去（函数指针）
 *   顺序错了的典型症状：能跳过去但一开中断就 HardFault，或者中断进了却在执行旧代码。
 *   第 3 步必须在第 5 步之前 —— 先把旧的清干净，再换新的。
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
  * 功    能：判断 App 区有没有能跑的固件：读它向量表的前两个字（栈顶、复位向量）
  * 参    数：无
  * 返 回 值：1 有效，0 无效
  * 补    充：栈顶必须落在 SRAM 范围内且 8 字节对齐（不对齐用 64 位变量就 UsageFault）；
  *            复位向量 bit0 必须为 1（Thumb 地址，为 0 立刻 HardFault）且落在 App 区内
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
  * 功    能：从 BootLoader 跳到 App 执行：先关中断、停 SysTick、清 NVIC 的使能与挂起位，
  * 参    数：无
  * 返 回 值：无
  * 补    充：再把 MSP 设成 App 的栈顶、VTOR 指向 App 的向量表，最后按复位向量跳过去
  *            顺序不能乱 —— 尤其是清 NVIC 必须在改 VTOR 之前，否则 App 一开中断就跳回 BootLoader
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
