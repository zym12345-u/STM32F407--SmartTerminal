/* BootLoader/config/boot_layout.h
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

/* 外部 W25Q128 固件槽：数值必须与 App 侧 bsp_W25Q128.h 保持一致
 * 每槽 448KB，槽首 4KB 是 32B 记录，镜像从 +0x1000 起；A 存整区备份，B 存 OTA 镜像
 * 【槽 B 从 0x00871000 起，不能是 0x00870000】A 的 448KB 镜像从 0x00801000 写到
 * 0x00870FFF；若 B 的 4KB 记录扇区放在 0x00870000，就会压掉 A 备份的最后 4KB，
 * 让 A 的整区 CRC 永远对不上，回滚基线失效。
 *
 * ⚠ 这组宏和 App 侧是【两个工程各存一份】的复制关系（Keil 两个 target 不共享
 *   头文件搜索路径），改地址必须两边同时改。典型事故：App 写 0x00870000、
 *   BootLoader 读 0x00871000，表现为"下载成功、复位永远不搬运"。
 *   改完用 grep 把两个头文件的 0x008 常量逐字节比对一遍，比"记得改过"可靠。
 *
 * 布局（槽内偏移）：
 *   +0x0000  32B 槽记录（magic/fmt/ver/size/crc/记录 CRC，后 8B 保留 0xFF）
 *   +0x1000  镜像本体，最长 0x6F000(444KB)
 *   +0x70000 槽结束（不含） */
#define SLOT_A_BASE_ADDR   0x00800000UL   /*槽 A：串口 IAP 前的整区备份*/
#define SLOT_B_BASE_ADDR   0x00871000UL   /*槽 B：WiFi OTA 下载的新镜像（A 后面紧接）*/
#define SLOT_SIZE          0x00070000UL   /*448KB，与内部 App 区同大小*/
#define SLOT_HDR_SIZE      0x00001000UL   /*4KB 记录扇区（32B 记录 + 擦除最小单位）*/
#define SLOT_IMG_MAX       (SLOT_SIZE - SLOT_HDR_SIZE)   /*444KB：镜像真实上限*/
#define SLOT_MAGIC         0x4657534CUL   /*'F''W''S''L'，与 App 侧记录 magic 同值*/
#define SLOT_FMT_BACKUP    1UL            /*槽 A：整区备份（size 恒为 448KB）*/
#define SLOT_FMT_OTA       2UL            /*槽 B：OTA 镜像（只有前 size 字节有效）*/

#endif /* BOOT_LAYOUT_H */
