/* BootLoader/port/flash_ops.c
 * 内部 Flash 操作：解锁、上锁、擦扇区、按字编程、扇区号换算。
 * 全部用寄存器，不经过 HAL —— BootLoader 不链接 HAL，越瘦越好。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "flash_ops.h"

/* ── 内部 Flash 操作：全部用寄存器，不经过 HAL ── */

static void flash_clear_errors(void)
{
  /*这些错误标志是"写 1 清零"，不清掉下一次操作会带着旧错误一起失败*/
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR |
              FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
}

void flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0UL)   /*已经解锁还写一次 key 会报错，先看 LOCK 位*/
  {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  flash_clear_errors();
}

void flash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;   /*用完立刻锁回去，防止跑飞误写*/
}

static int flash_busy_wait(void)
{
  uint32_t wait = 0UL;

  while ((FLASH->SR & FLASH_SR_BSY) != 0UL)   /*BSY 是硬件在忙，软件只能干等*/
  {
    wait++;
    if (wait > 20000000UL)   /*16MHz 下大约几秒，128KB 扇区最长 4 秒也够*/
    {
      return 0;
    }
  }
  return 1;
}

/**
  * 功    能：擦除一个扇区
  * 参    数：snb 扇区号 0~11
  * 返 回 值：1 成功，0 失败（超时或写了错误标志）
  * 补    充：必须先把 PSIZE 和 SNB 配好再置 SER，最后用 STRT 触发；
  *            擦完一定要清 SER，否则紧接着的编程操作会当成擦除命令
  */
int flash_erase_sector(uint8_t snb)
{
  flash_clear_errors();

  FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB_Msk | FLASH_CR_PSIZE_Msk | FLASH_CR_PG))
            | ((uint32_t)snb << FLASH_CR_SNB_Pos);
  FLASH->CR |= FLASH_CR_SER;
  FLASH->CR |= FLASH_CR_STRT;

  if (!flash_busy_wait())
  {
    FLASH->CR &= ~FLASH_CR_SER;
    return 0;
  }
  FLASH->CR &= ~FLASH_CR_SER;

  if ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                    FLASH_SR_PGPERR | FLASH_SR_PGSERR)) != 0UL)
  {
    return 0;   /*写保护或对齐错误，这扇区没擦成*/
  }
  return 1;
}

/**
  * 功    能：往内部 Flash 写一个 32 位字
  * 参    数：addr 目标地址（必须 4 字节对齐）；word 要写的值
  * 返 回 值：1 成功，0 失败
  * 补    充：写前先读一遍，不是 0xFFFFFFFF 直接拒写 ——
  *            内部 Flash 只能把 1 变 0，在脏字上硬写会得到一个错值，而且事后看不出来
  */
int flash_program_word(uint32_t addr, uint32_t word)
{
  flash_clear_errors();

  if (*(volatile uint32_t *)addr != 0xFFFFFFFFUL)
  {
    return 0;
  }

  /*PSIZE=10 是 x32 位并行，写一个字一次搞定，比 x8 快得多*/
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_PSIZE_Msk | FLASH_CR_SER | FLASH_CR_SNB_Msk))
            | FLASH_CR_PSIZE_1;
  FLASH->CR |= FLASH_CR_PG;

  *(volatile uint32_t *)addr = word;   /*写这个动作本身触发编程*/

  if (!flash_busy_wait())
  {
    FLASH->CR &= ~FLASH_CR_PG;
    return 0;
  }
  FLASH->CR &= ~FLASH_CR_PG;

  if ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                    FLASH_SR_PGPERR | FLASH_SR_PGSERR)) != 0UL)
  {
    return 0;
  }
  return 1;
}

uint32_t flash_sector_start(uint8_t snb)
{
  if (snb <= 3U)   /*sector 0~3 每个 16KB*/
  {
    return 0x08000000UL + ((uint32_t)snb * 0x4000UL);
  }
  if (snb == 4U)   /*sector 4 是 64KB*/
  {
    return 0x08010000UL;
  }
  return 0x08020000UL + ((uint32_t)(snb - 5U) * 0x20000UL);   /*sector 5~11 每个 128KB*/
}

uint32_t flash_sector_size(uint8_t snb)
{
  if (snb <= 3U)
  {
    return 0x4000UL;
  }
  if (snb == 4U)
  {
    return 0x10000UL;
  }
  return 0x20000UL;
}

/**
  * 功    能：查一个地址落在哪个扇区
  * 参    数：addr 内部 Flash 里的地址
  * 返 回 值：扇区号 0~11；地址不在内部 Flash 范围内返回 0xFF
  * 补    充：扇区大小不均匀，所以只能用分段判断，不能直接除
  */
uint8_t flash_sector_of(uint32_t addr)
{
  if (addr < 0x08010000UL)   /*sector 0~3 每 16KB，直接用偏移右移 14 位*/
  {
    return (uint8_t)((addr - 0x08000000UL) >> 14);
  }
  if (addr < 0x08020000UL)   /*sector 4 独占 0x08010000~0x0801FFFF*/
  {
    return 4U;
  }
  if (addr < 0x08080000UL)   /*sector 5~11 每 128KB*/
  {
    return (uint8_t)(5U + ((addr - 0x08020000UL) >> 17));
  }
  return 0xFFU;
}
