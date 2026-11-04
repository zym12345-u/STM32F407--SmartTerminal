/* BootLoader/port/flash_ops.h
 * STM32F407 内部 Flash 的寄存器级操作：解锁/上锁/擦除/按字编程/扇区换算。
 */
#ifndef FLASH_OPS_H
#define FLASH_OPS_H

#include <stdint.h>

/* 内部 Flash 解锁序列，少写一个字节解不了锁，顺序也不能反 */
#define FLASH_KEY1        0x45670123UL
#define FLASH_KEY2        0xCDEF89ABUL

void flash_unlock(void);
void flash_lock(void);
int  flash_erase_sector(uint8_t snb);
int  flash_program_word(uint32_t addr, uint32_t word);
uint32_t flash_sector_start(uint8_t snb);
uint32_t flash_sector_size(uint8_t snb);
uint8_t  flash_sector_of(uint32_t addr);

#endif /* FLASH_OPS_H */
