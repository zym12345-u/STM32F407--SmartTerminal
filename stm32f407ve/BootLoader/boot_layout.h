/* BootLoader/boot_layout.h
 * 分区常量收口：BootLoader / 元数据 / App / SRAM 的地址与大小。
 * IAP 和 App 侧共用同一组地址，改分区只需改这里。
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
 */
#ifndef BOOT_LAYOUT_H
#define BOOT_LAYOUT_H

#include <stdint.h>

/* 分区常量：IAP 和 App 侧共用同一组地址 */
#define APP_BASE_ADDR     0x08010000UL   /*App 复位向量（sector 4 起点）*/
#define APP_LIMIT_ADDR    0x08080000UL   /*App 区上界（不含）*/
#define META_BASE_ADDR    0x0800C000UL   /*升级元数据（sector 3）*/
#define META_LIMIT_ADDR   0x08010000UL   /*元数据区上界（不含）*/
#define SRAM_BASE_ADDR    0x20000000UL   /*SRAM 起始*/
#define SRAM_LIMIT_ADDR   0x20020000UL   /*SRAM 上界 128KB*/

#define APP_SIZE            (APP_LIMIT_ADDR - APP_BASE_ADDR)
#define FW_VER_PACK(maj, min) ((((uint32_t)(maj) & 0xFFUL) << 8U) | ((uint32_t)(min) & 0xFFUL))

#endif /* BOOT_LAYOUT_H */
