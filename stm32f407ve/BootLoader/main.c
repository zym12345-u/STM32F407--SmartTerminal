/* BootLoader/main.c
 * 上电先开 3 秒串口升级窗口，窗口结束再校验 App 区向量表，合法则跳到 App，不合法则闪灯提示。
 * 分区：BootLoader 48KB，元数据 16KB，App 448KB（元数据独占整个 sector 3）。
 *
 * ── 为什么用「上电 3 秒窗口」而不是 BOOT0 按键 ──
 *   BOOT0 只在【复位那一刻】被采样，程序跑起来以后再改它没用，必须断电重上电；
 *   而且板子上要为此多留一个跳线帽或按键，升级时还得手动拨，容易拨错。
 *   串口窗口没有这些麻烦：BootLoader 一上电先停 3 秒等握手帧，
 *   收到就进升级流程，收不到就照常校验并跳 App —— 工具端只要打开串口就行。
 *   窗口必须有【明确的长度】，不能一直等，否则没有升级请求时 App 永远起不来。
 *   3 秒是折中：够 PC 端把 USB 虚拟串口枚举完再发帧，又不会让正常上电等太久。
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
 *
 * ── 启动时先查标志再决定干什么 ──
 *   升级中途断电，App 区是个写了一半的镜像 —— 向量表校验"碰巧合法"是可能的
 *   (只要前 8 字节刚好写完)，直接跳过去照样跑飞。
 *   所以 BootLoader 上电先读 24C02 的 magic：在 → 上次升级没走完，
 *   跳过窗口直接进升级模式等 PC 重传；不在 → 照旧窗口 + 校验 + 跳转。
 *   24C02 探测失败(没焊/挂了)就当没有标志走原流程 —— 一片 EEPROM 挂了
 *   不能让整块板子起不来，向量表校验还是最后的底。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "boot_layout.h"
#include "bsp_uart1.h"
#include "bsp_ee.h"
#include "meta.h"
#include "board.h"
#include "upgrade.h"
#include "main.h"

static void rst_cause_dump(void);

int main(void)
{
  /* 窗口内等握手帧：等到就进升级模式，等不到就照旧校验并跳 App */
  uart1_init();
  rst_cause_dump();                            /*先报本次复位原因，排查中途重启*/
  uart1_send_str("\r\n[BOOT] window 3s\r\n");   /*打一行横幅，串口助手能自证"现在跑的是 BootLoader"*/

  ee_init();
  uart1_send_str((g_ee_ok != 0U) ? "[BOOT] eeprom ok\r\n" : "[BOOT] eeprom absent\r\n");

  if (ota_is_marked() != 0)
  {
    /*上次升级中途断电、或 App 里的「固件升级」按钮主动请求：跳过窗口直接进升级模式*/
    uart1_send_str("[BOOT] ota flag found, resume upgrade\r\n");

    if (app_is_valid())
    {
      /* App 区是完整的 —— 说明标志是 App 自己写的，不是断电丢的。
       * 这里必须限时：误点一下按钮却没人发数据，标志就一直留着，
       * 每次上电都进升级模式，按 RST 也出不来，板子等于变砖。
       * 60 秒够 PC 端打开串口并握手；超时就清标志、把旧 App 跑起来。*/
      uart1_send_str("[BOOT] app valid, wait 60s for PC\r\n");
      if (uart1_wait_handshake(60000UL) != 0)
      {
        upgrade_loop();
      }
      ota_clear();
      uart1_send_str("[BOOT] no PC, flag cleared, back to app\r\n");
    }
    else
    {
      /* App 不完整（真断电升级中）：绝不能跳转，无限等 PC 重传*/
      upgrade_loop();
    }
  }

  if (uart1_wait_handshake(UART_WINDOW_MS))
  {
    uart1_send_str("[BOOT] upgrade mode\r\n");
    upgrade_loop();   /*升级模式：等擦除/数据块/结束三条命令*/
  }

  uart1_send_str("[BOOT] window closed\r\n");

  boot_meta_report();   /*把 sector 3 里记的版本/大小打出来，只报告不拦启动*/

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
  * 功    能：打印本次复位的原因，然后把标志清掉
  * 参    数：无
  * 返 回 值：无
  * 补    充：这是排查"升级到一半板子自己重启"最省事的证据。
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
static void rst_cause_dump(void)
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

/**
  * 功    能：判断 App 区有没有能跑的固件：读它向量表的前两个字（栈顶、复位向量）
  * 参    数：无
  * 返 回 值：1 有效，0 无效
  * 补    充：栈顶必须落在 SRAM 范围内且 8 字节对齐（不对齐用 64 位变量就 UsageFault）；
  *            复位向量 bit0 必须为 1（Thumb 地址，为 0 立刻 HardFault）且落在 App 区内
  */
int app_is_valid(void)
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
void jump_to_app(void)
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
