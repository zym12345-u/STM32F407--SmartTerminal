/* BootLoader/port/bsp_w25q.h
 * BootLoader 专用的 W25Q128 极简驱动：只做「读数据 + 擦槽记录扇区」两件事。
 * 搬运镜像只需要顺序读；作废槽 B 记录只需要擦 4KB 扇区，所以不实现页编程。
 *
 * 硬件：SPI1 = PA5(SCK)/PA6(MISO)/PA7(MOSI)，CS = PC13，模式 3（CPOL=1/CPHA=1）。
 * 时钟：BootLoader 跑 HSI 16MHz，APB2 也是 16MHz；SPI 分频 /4 = 4MHz，
 *       比 App 的 10.5MHz 保守，只影响搬运耗时，不影响正确性。
 */
#ifndef BSP_W25Q_H
#define BSP_W25Q_H

#include <stdint.h>

#define W25Q_JEDEC_EXPECT   0x00EF4018UL   /*Winbond W25Q128：EF 40 18*/

/**
  * 功    能：初始化 SPI1/CS 并读 JEDEC ID 确认芯片在位
  * 参    数：无
  * 返 回 值：1 = 在位且 ID 正确；0 = 不在位/ID 不对
  */
int bsp_w25q_init(void);

/**
  * 功    能：读 JEDEC ID（命令 0x9F）
  * 参    数：无
  * 返 回 值：0x00EF4018 形式的三字节 ID
  */
uint32_t bsp_w25q_jedec(void);

/**
  * 功    能：从 24 位地址连续读一段数据（命令 0x03）
  * 参    数：addr 起始地址；buf 输出缓冲；len 字节数
  * 返 回 值：无（总线异常由上层用记录 CRC 兜底）
  */
void bsp_w25q_read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
  * 功    能：擦除包含 addr 的 4KB 扇区（用于作废槽 B 记录）
  * 参    数：addr 扇区内任意地址
  * 返 回 值：0 成功；1 写使能失败或擦除超时
  */
int bsp_w25q_erase_4k(uint32_t addr);

#endif /* BSP_W25Q_H */
