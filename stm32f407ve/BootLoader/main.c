/**
  ******************************************************************************
  * @file    main.c
  * @brief   V2.0 阶段 2：最小 BootLoader（校验 App 向量表 + 跳转）
  *
  * 分区规划（内部 Flash）：
  *   0x08000000 - 0x0800BFFF  48KB   BootLoader（sector 0~2）
  *   0x0800C000 - 0x080DFFFF  848KB  App（sector 3~10）
  *   0x080E0000 - 0x080E0FFF  4KB    升级元数据（sector 11 的前 4KB，阶段 3 才用）
  *
  * 本阶段只做两件事：
  *   1) 判断 App 区的向量表（初始 MSP / 复位向量）是否像一个可运行的固件；
  *   2) 是则把 CPU 复位到"App 刚上电"的状态并跳过去；不是则闪灯等待。
  *
  * 为什么不用 HAL / 不用 FreeRTOS：
  *   BootLoader 是断电也不会消失的"救生舱"，代码越少越可靠。这里只用 CMSIS
  *   （直接操作寄存器），没有 HAL 依赖、没有中断、没有 RTOS，编译产物只有几 KB，
  *   后面再往里加 UART + CRC32（阶段 3）也放得进 48KB。
  ******************************************************************************
  */

#include <stdint.h>
#include "stm32f4xx.h"

/* ---------------- 分区常量 ----------------
 * 为什么把地址写成宏：阶段 3 的 IAP 要用同一组地址做擦写范围检查，
 * 阶段 5 的 App 侧也要引用同一组值。集中在一处，改分区只改这里。
 */
#define APP_BASE_ADDR     0x0800C000UL   /* App 复位向量所在地址 */
#define APP_LIMIT_ADDR    0x080E0000UL   /* App 区上界（不含），再往后是元数据区 */
#define SRAM_BASE_ADDR    0x20000000UL   /* SRAM 起始（112KB + 16KB CCM 连续编址） */
#define SRAM_LIMIT_ADDR   0x20020000UL   /* SRAM 上界 = 0x20000000 + 128KB */

/* ---------------- 板载 LED（诊断用） ----------------
 * 绿色 LED = PC5，低电平点亮（灌电流）。用 LED 是因为 BootLoader 阶段还没有
 * 串口/屏，出问题时必须有一个"我还活着"的物理证据。
 */
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
    /* App 无效：闪灯。这里不报错、不试图修复，把"板子为什么没跑起来"这件事
     * 用最直观的方式暴露出来——灯在闪 = BootLoader 正常、App 区没有有效固件。 */
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

  /* 正常情况下 jump_to_app 不会返回；返回说明出错了，落在这里原地循环。 */
  for (;;)
  {
  }
}

/* -------------------------------------------------------------------------
 * App 有效性检查
 * Cortex-M 复位时硬件只做两件事：从向量表第 0 项取 MSP、第 1 项取复位入口。
 * 所以"向量表像不像样"就是"固件能不能跑"的最廉价判据：
 *   - 初始 MSP 必须落在 SRAM 内且 8 字节对齐（AAPCS/EABI 要求栈 8 字节对齐，
 *     不对齐会在 App 里第一次用 double/64 位变量时产生 UsageFault）；
 *   - 复位向量必须落在 App 区内且最低位为 1（Cortex-M 只支持 Thumb 指令集，
 *     bit0=1 是硬件强制要求，为 0 会立刻 HardFault）。
 * 全 0xFF（擦除态）和全 0x00（未写过）都会被上面两条挡掉，不需要额外判空。
 * ------------------------------------------------------------------------- */
static int app_is_valid(void)
{
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  if ((app_msp < SRAM_BASE_ADDR) || (app_msp >= SRAM_LIMIT_ADDR))
  {
    return 0;
  }
  if ((app_msp & 0x7UL) != 0UL)
  {
    return 0;
  }
  if (((app_reset & 1UL) == 0UL) ||
      (app_reset < APP_BASE_ADDR) || (app_reset >= APP_LIMIT_ADDR))
  {
    return 0;
  }
  return 1;
}

/* -------------------------------------------------------------------------
 * 跳转序列
 * 顺序有讲究，逐条说明（以后加 IAP 时这段不能乱序）：
 *
 * 1) 关总中断：跳转过程中不响应任何中断，避免在半初始化状态下进 ISR。
 * 2) 关 SysTick：App 的 SysTick 由它自己的启动代码/HAL 重新配置；如果这里
 *    还跑着 SysTick，跳过去后到 App 重配之间会不断产生 SysTick 中断。
 * 3) 清 NVIC 使能位 + 挂起位：使能位不清，App 里一旦 unmask 同一中断就会
 *    立刻进 BootLoader 遗留的 handler；挂起位不清，App 一开总中断就先跑一次
 *    "幽灵中断"。STM32F407 有 82 个外部中断，8 组 ICER/ICPR 全部清掉最省事。
 * 4) 清 PendSV/PendST 挂起：同上，属于内核异常，ICPR 管不到。
 * 5) 设 VTOR 指向 App 向量表：App 的异常/中断才能找到它自己的入口。
 *    注意——App 的 SystemInit 里还会再写一次 SCB->VTOR，所以 App 侧必须把
 *    VECT_TAB_OFFSET 或显式 VTOR 设成 0x0800C000，否则会被改回 0x08000000
 *    （后果：App 的所有中断都跳到 BootLoader 的 Default_Handler 死循环里）。
 * 6) 设 MSP：硬件复位时 MSP 来自向量表第 0 项，软件跳转必须手动赋同一个值。
 * 7) 开总中断（PRIMASK=0）：App 期望"复位后的状态"就是总中断打开；若带着
 *    PRIMASK=1 跳过去，App 在调度器启动前的 HAL 延时、串口收发都会被拖死。
 *    放在最后，是因为此时 VTOR 和 MSP 都已就绪，即使中断立刻进来也是安全的。
 * 8) 用函数指针跳到 App 复位向量：Cortex-M 的向量表第 1 项本身就是可直接
 *    BLX 的 Thumb 地址（bit0=1），所以强转成函数指针调用即可，不需要汇编。
 * ------------------------------------------------------------------------- */
static void jump_to_app(void)
{
  typedef void (*app_entry_t)(void);

  uint32_t i;
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  __disable_irq();

  /* 2) 停 SysTick */
  SysTick->CTRL = 0UL;
  SysTick->LOAD = 0UL;
  SysTick->VAL  = 0UL;

  /* 3) 清 NVIC 使能与挂起（ICER/ICPR 写 1 有效） */
  for (i = 0UL; i < 8UL; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  /* 4) 清 PendSV / PendST 挂起 */
  SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

  /* 存储屏障：保证上面的寄存器写已经生效，再改 VTOR/MSP */
  __DSB();
  __ISB();

  /* 5) 向量表重定位到 App */
  SCB->VTOR = APP_BASE_ADDR;
  __DSB();
  __ISB();

  /* 6) 栈指针复位到 App 的初始 MSP */
  __set_MSP(app_msp);

  /* 7) 恢复"复位后"的中断状态 */
  __enable_irq();

  /* 8) 跳转（App 的 Reset_Handler → SystemInit → __main → main） */
  ((app_entry_t)app_reset)();
}

/* ---------------- LED 与延时（裸机） ---------------- */
static void led_init(void)
{
  /* 时钟不开则写 GPIO 寄存器无效（写进去也读不回来）——这是新手最常见的坑。 */
  RCC->AHB1ENR |= LED_GPIO_CLK_EN;

  /* MODER 每引脚 2 bit，01 = 通用输出 */
  LED_PORT->MODER   = (LED_PORT->MODER & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->OTYPER &= ~LED_PIN_MASK;                                  /* 推挽 */
  LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & ~(3UL << (LED_PIN * 2U))) | (3UL << (LED_PIN * 2U));
  LED_PORT->PUPDR   = (LED_PORT->PUPDR & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->BSRR     = LED_PIN_MASK;   /* BSRR 写 1 置位 = 输出高 = 灯灭 */
}

static void led_on(void)
{
  LED_PORT->BSRR = (LED_PIN_MASK << 16U);   /* BSRR 高 16 位 = 复位，低电平点亮 */
}

static void led_off(void)
{
  LED_PORT->BSRR = LED_PIN_MASK;
}

/* 不依赖 SysTick 的粗延时：此时系统还跑在 SystemInit 后的 HSI 16MHz 上，
 * 精确值无意义，只需要"肉眼可见的闪烁"。volatile 防止 -O2 把循环优化掉。 */
static void delay_loop(uint32_t n)
{
  volatile uint32_t i;
  for (i = 0UL; i < n; i++)
  {
  }
}
