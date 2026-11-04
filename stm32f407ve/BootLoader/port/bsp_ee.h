/* BootLoader/port/bsp_ee.h
 * 板载 24C02（I2C1）驱动 + 升级进度标志（断电续传用）。
 */
#ifndef BSP_EE_H
#define BSP_EE_H

#include <stdint.h>

/* 24C02（板载 EEPROM，I2C1 PB6/PB7）：只用来存「升级中标志 + 已写字节数」 */
#define EE_I2C_ADDR_W     0xA0U        /*7 位地址 0x50 左移 1 位 + 写*/
#define EE_I2C_ADDR_R     0xA1U        /*同地址 + 读*/
#define EE_OTA_MEM        0xF0U        /*进度页：magic(4B) + written(4B)，正好一页，不碰自检页 0xF8~0xFF*/
#define EE_OTA_MAGIC      0x32475055UL /*内存里字节序是 'U''P''G''2'，页首是这个值 = 升级中途断电*/
#define I2C_TIMEOUT       100000UL     /*每步最久等几十 ms，总线死了也不能把启动卡死*/

extern uint8_t g_ee_ok;   /*0 = EEPROM 不在位，之后所有 OTA 标志操作静默跳过*/

void ee_init(void);
void ota_mark(uint32_t written);
void ota_clear(void);
int  ota_is_marked(void);
uint32_t ota_read_written(void);

#endif /* BSP_EE_H */
