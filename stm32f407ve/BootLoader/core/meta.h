/* BootLoader/core/meta.h
 * 升级元数据（sector 3）的布局与读写接口。
 */
#ifndef META_H
#define META_H

#include <stdint.h>
#include "boot_layout.h"

/* 元数据区（sector 3，16KB）用「追加记录」布局，为的是抗掉电：
 * 每次升级成功只往后追加一条 32 字节记录，从不擦除已有记录；
 * 写到一半掉电只会留下一条校验不过的记录，上一条仍然完整可用。
 * 记录布局（8 个字，只用前 6 个，其余保持 0xFF）：
 *   [0] magic      [1] 记录格式=3        [2] 固件版本(主字节 << 8 | 次字节)
 *   [3] 镜像字节数(4 对齐)                [4] 整包 CRC32
 *   [5] 前 5 个字的 CRC32（记录自身校验；最后写，写完这条才算有效）
 * 空槽判据：第 0 个字是 0xFFFFFFFF（整扇区擦完就是它）
 * v2（magic, 2, 版本, size, crc）与 v1（magic, size, crc, 1）旧布局
 * 只读不写，兼容已经升级过的板子 */
#define META_MAGIC          0x4154454DUL  /*'M''E''T''A'：元数据有效的标记*/
#define META_VERSION_REC    3UL           /*追加记录格式：带记录自身 CRC*/
#define META_VERSION        2UL           /*旧 v2 五字布局，仅兼容读*/
#define META_VERSION_V1     1UL           /*旧格式：没有版本号和结束标记*/
#define META_SLOT_SIZE      32UL          /*一条记录占 32 字节，16KB 共 512 条*/
#define META_SLOT_COUNT     ((META_LIMIT_ADDR - META_BASE_ADDR) / META_SLOT_SIZE)
#define APP_END_MAGIC       0x444E454DUL  /*内存里读出 'M''E''N''D'：镜像末尾标记*/

int  app_end_mark_write(uint32_t size);
int  meta_write(uint32_t size, uint32_t crc, uint32_t fw_ver);
void boot_meta_report(void);

#endif /* META_H */
