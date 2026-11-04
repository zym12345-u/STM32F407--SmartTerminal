/* BootLoader/core/meta.c
 * 升级元数据：追加记录式布局的读校验、末尾标记、上电报告和写入。
 *
 * ── 元数据 v2 加了什么，为什么 ──
 *   v1 只有 magic/size/crc，能证明「上次升级完整结束」，但回答不了两个问题：
 *   现在跑的是哪个版本；Flash 里的内容和记录的大小还对不对得上。
 *   v2 在原来 4 个字之外加了两样：
 *     固件版本：PC 在结束帧里带上来（--fw-ver 1.4），App 读出来显示；
 *     镜像末尾标记 'MEND'：写在镜像最后一个字节之后的那个字上，
 *     0xFFFFFFFF 被换成 'MEND'，说明「元数据记的 size 就是 Flash 里这份镜像的真实大小」。
 *   【为什么标记不拦启动】用 Keil 直接烧 App 时元数据不会跟着更新，标记必然对不上；
 *   若据此拒跳，改完代码一烧板子就只会闪灯。所以 BootLoader 只把对不上的记成
 *   stale 照常启动，App 端再据此决定「信元数据里的版本，还是退回编译期版本号」。
 *   【v1 旧格式】只读出来打日志，不做大小校验，避免上一轮升级过的板子变砖。
 */

#include <stdint.h>
#include "boot_layout.h"
#include "flash_ops.h"
#include "bsp_uart1.h"
#include "meta.h"

/**
  * 功    能：检查镜像字节数是否落在合理范围：至少一个向量表、不超过 App 区、4 字节对齐
  * 参    数：size 元数据里记的镜像字节数
  * 返 回 值：1 合理；0 不可能
  * 补    充：这是「大小校验」的第一道：越界的大小说明元数据本身坏了，不能采信；
  *            4 对齐是因为 Flash 按字编程，PC 端送数据前也已补齐到 4 字节
  */
static int meta_size_ok(uint32_t size)
{
  if (size < 8UL)             /*连向量表都不够，不可能是镜像*/
  {
    return 0;
  }
  if (size > APP_SIZE)        /*超出 App 区，地址根本不存在*/
  {
    return 0;
  }
  if ((size & 3UL) != 0UL)    /*按字编程的单位对不上*/
  {
    return 0;
  }
  return 1;
}

/**
  * 功    能：算一条元数据记录的自校验 CRC（只覆盖前 5 个字）
  * 参    数：w 记录首地址
  * 返 回 值：CRC32 值
  * 补    充：与整包 CRC 同一个算法，但输入只有 20 字节，逐位算也不费时。
  *            掉电时记录若只写了一半，这个值必然对不上，那条记录就不算数。
  */
static uint32_t meta_rec_crc(const volatile uint32_t *w)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint8_t  k;

  for (i = 0UL; i < 5UL; i++)
  {
    for (k = 0U; k < 4U; k++)
    {
      crc = crc32_update(crc, (uint8_t)((w[i] >> (8U * k)) & 0xFFUL));
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

/**
  * 功    能：读元数据：先扫追加记录取最新一条，扫不到再认旧布局
  * 参    数：fmt 输出格式版本；size/crc/fw_ver 输出对应字段；slot 输出记录槽号（旧布局为 0）
  * 返 回 值：1 读到可用元数据；0 没有或不可信
  * 补    充：槽号从 0 往后递增，后写的记录一定在更大的槽号里，所以
  *            「最后一个通过校验的」就是最新的；中间某条写坏了不影响上一条。
  */
static int meta_read(uint32_t *fmt, uint32_t *size, uint32_t *crc, uint32_t *fw_ver,
                     uint32_t *slot)
{
  uint32_t i;
  uint32_t addr;
  uint32_t m0;
  uint32_t m1;
  uint32_t m3;
  int      hit = 0;

  *slot = 0UL;

  for (i = 0UL; i < META_SLOT_COUNT; i++)
  {
    addr = META_BASE_ADDR + (i * META_SLOT_SIZE);
    if (*(volatile uint32_t *)addr != META_MAGIC)
    {
      continue;   /*空槽（0xFFFFFFFF）或别的东西，跳过*/
    }
    if (*(volatile uint32_t *)(addr + 4UL) != META_VERSION_REC)
    {
      continue;
    }
    if (meta_size_ok(*(volatile uint32_t *)(addr + 12UL)) == 0)
    {
      continue;
    }
    if (meta_rec_crc((const volatile uint32_t *)addr) != *(volatile uint32_t *)(addr + 20UL))
    {
      continue;   /*记录自身 CRC 不对：这条是掉电写了一半的，不算数*/
    }
    *fmt    = META_VERSION_REC;
    *fw_ver = *(volatile uint32_t *)(addr + 8UL);
    *size   = *(volatile uint32_t *)(addr + 12UL);
    *crc    = *(volatile uint32_t *)(addr + 16UL);
    *slot   = i;
    hit = 1;    /*继续往后扫，保证取到槽号最大的那条*/
  }

  if (hit != 0)
  {
    return 1;
  }

  /*旧布局兼容：0 偏移处的五字结构*/
  m0 = *(volatile uint32_t *)META_BASE_ADDR;
  m1 = *(volatile uint32_t *)(META_BASE_ADDR + 4UL);
  m3 = *(volatile uint32_t *)(META_BASE_ADDR + 12UL);

  if (m0 != META_MAGIC)
  {
    return 0;
  }

  if ((m1 == META_VERSION) && (meta_size_ok(m3) != 0))
  {
    *fmt    = META_VERSION;
    *fw_ver = *(volatile uint32_t *)(META_BASE_ADDR + 8UL);
    *size   = m3;
    *crc    = *(volatile uint32_t *)(META_BASE_ADDR + 16UL);
    return 1;
  }

  if ((m3 == META_VERSION_V1) && (meta_size_ok(m1) != 0))
  {
    *fmt    = META_VERSION_V1;
    *fw_ver = 0UL;
    *size   = m1;
    *crc    = *(volatile uint32_t *)(META_BASE_ADDR + 8UL);
    return 1;
  }

  return 0;
}

/**
  * 功    能：检查镜像末尾标记是否与元数据记录的大小对应
  * 参    数：size 元数据里的镜像字节数
  * 返 回 值：1 标记在位；0 标记不在
  * 补    充：镜像顶满整个 App 区时后面没有空位写标记，按通过处理（这种镜像极少见）
  */
static int app_end_mark_ok(uint32_t size)
{
  if (size >= APP_SIZE)
  {
    return 1;
  }
  return (*(volatile uint32_t *)(APP_BASE_ADDR + size) == APP_END_MAGIC) ? 1 : 0;
}

/**
  * 功    能：在镜像最后一个字节之后写 'MEND' 结束标记
  * 参    数：size 本次实际写入的镜像字节数（已 4 对齐）
  * 返 回 值：1 成功；0 编程失败
  * 补    充：擦除阶段已把 s4~s7 整片清成 0xFF，这个位置必定可写；
  *            写完标记才算「Flash 内容与元数据对得上」，所以必须在 meta_write 之前做
  */
int app_end_mark_write(uint32_t size)
{
  int ok;

  if (size >= APP_SIZE)
  {
    return 1;   /*没有多余的字可写，跳过*/
  }

  flash_unlock();
  ok = flash_program_word(APP_BASE_ADDR + size, APP_END_MAGIC);
  flash_lock();
  return ok;
}

/**
  * 功    能：上电时打印元数据里的固件版本、镜像大小、CRC 和结束标记状态
  * 参    数：无
  * 返 回 值：无
  * 补    充：只报告、不拦启动。结束标记对不上最常见的原因是「用 Keil 直接烧了 App」，
  *            此时旧元数据留在 sector 3、标记已被擦掉；拒绝启动会让开发流程直接卡死。
  *            真正拦启动的仍是 app_is_valid() 的向量表校验和 24C02 的升级中标志
  */
void boot_meta_report(void)
{
  uint32_t fmt;
  uint32_t size;
  uint32_t crc;
  uint32_t fw_ver;
  uint32_t slot;

  if (meta_read(&fmt, &size, &crc, &fw_ver, &slot) == 0)
  {
    uart1_send_str("[BOOT] meta none, vector check only\r\n");
    return;
  }

  uart1_send_str("[BOOT] meta v");
  uart1_send_dec(fmt);
  if ((fmt == META_VERSION_REC) || (fmt == META_VERSION))
  {
    uart1_send_str(" fw=");
    uart1_send_dec((fw_ver >> 8U) & 0xFFUL);
    uart1_send_str(".");
    uart1_send_dec(fw_ver & 0xFFUL);
    uart1_send_str((app_end_mark_ok(size) != 0) ? " mark ok" : " mark stale");
  }
  uart1_send_str(" size=");
  uart1_send_dec(size);
  uart1_send_str(" crc=0x");
  uart1_send_hex32(crc);
  if (fmt == META_VERSION_REC)
  {
    /*槽号是排查掉电时最有用的一列：相邻两次升级的槽号应当递增*/
    uart1_send_str(" slot=");
    uart1_send_dec(slot);
  }
  uart1_send_str("\r\n");
}

/**
  * 功    能：把本次升级结果追加一条记录到元数据区（sector 3）
  * 参    数：size 镜像字节数；crc 整包 CRC32（已核对通过的值）；fw_ver 固件版本（主<<8 | 次）
  * 返 回 值：1 成功；0 编程或擦除失败
  * 补    充：找第一个空槽（首字为 0xFFFFFFFF），只写这一条，不动已有的记录，
  *            所以元数据更新没有「擦掉旧值再写新值」的危险窗口。
  *            写入顺序是 magic 先、校验字最后：中途掉电留下的半条记录
  *            magic 在、CRC 不对，读取时直接跳过，上一条仍然有效。
  *            512 条用满（正常使用到不了）才整扇区擦一次，从 0 号槽重来。
  */
int meta_write(uint32_t size, uint32_t crc, uint32_t fw_ver)
{
  uint32_t buf[6];
  uint32_t i;
  uint32_t addr;
  uint8_t  snb;
  uint8_t  k;

  for (i = 0UL; i < META_SLOT_COUNT; i++)
  {
    if (*(volatile uint32_t *)(META_BASE_ADDR + (i * META_SLOT_SIZE)) == 0xFFFFFFFFUL)
    {
      break;      /*首字为空 ⇒ 整槽都还是擦除态*/
    }
  }

  if (i >= META_SLOT_COUNT)
  {
    /*记录区写满了：整扇区擦一次，从 0 号槽重来（这一步会丢掉全部历史记录）*/
    snb = flash_sector_of(META_BASE_ADDR);
    flash_unlock();
    if (!flash_erase_sector(snb))
    {
      flash_lock();
      return 0;
    }
    flash_lock();
    uart1_send_str("[BOOT] meta slots full, sector erased\r\n");
    i = 0UL;
  }

  buf[0] = META_MAGIC;
  buf[1] = META_VERSION_REC;
  buf[2] = fw_ver;
  buf[3] = size;
  buf[4] = crc;
  buf[5] = meta_rec_crc((const volatile uint32_t *)buf);

  addr = META_BASE_ADDR + (i * META_SLOT_SIZE);

  flash_unlock();
  for (k = 0U; k < 6U; k++)
  {
    if (!flash_program_word(addr + (4UL * (uint32_t)k), buf[k]))
    {
      flash_lock();
      return 0;
    }
  }
  flash_lock();
  return 1;
}
