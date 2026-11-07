/* BootLoader/core/ota2.c
 * OTA-2：把 W25Q 槽 B 的新镜像搬运到内部 App 区；失败或连续未确认时回滚槽 A。
 *
 * ── 为什么搬运前先校验槽 A ──
 *   擦 App 区是「不可逆」的第一步：一旦擦掉，当前 App 就没了。
 *   如果这时槽 A 备份其实坏了，新镜像再搬运失败，板子就没有可运行的固件。
 *   所以搬运前必须先把槽 A 的记录和整区 CRC 都验一遍，
 *   确认「回滚一定有东西可回」，才允许擦 App 区。
 *   多花一次 448KB 读取（4MHz 下约 1 秒），换的是不变砖。
 *
 * ── 为什么搬运后要算两次 CRC ──
 *   第一次边读槽 B 边算：证明 SPI 读出来的数据就是记录里那份；
 *   第二次回读内部 App 区再算：证明写进 Flash 的字节没有丢/错。
 *   两次都对上才算搬运成功。只算一次的话，SPI 读对但 Flash 写坏，
 *   或者反过来，都会被漏掉。
 *
 * ── 启动确认的计数规则 ──
 *   搬运成功当次 count=0（新固件第一次启动），之后每次上电 count+1。
 *   count 到 3 说明新固件连续 3 次都没能跑到「确认」那一步，
 *   下一次上电就回滚槽 A。App 确认后整页清 0，计数自然归零。
 *
 * ── 槽 B 记录为什么失败后要擦掉 ──
 *   记录有效但镜像搬不动，如果不作废，下次上电还会再试一次，
 *   每次都白等几十秒；更糟的是 App 端再次请求时会把坏镜像重新搬进 App 区。
 *   擦掉槽 B 首扇区（4KB）只废记录，不动镜像数据，重新下载即可覆盖。
 */

#include <stdint.h>
#include "boot_layout.h"
#include "bsp_w25q.h"
#include "bsp_ee.h"
#include "bsp_uart1.h"
#include "flash_ops.h"
#include "meta.h"
#include "main.h"
#include "ota2.h"

#define OTA2_MAX_UNCONFIRMED   3U       /*连续 3 次未确认，第 4 次上电回滚*/
#define OTA2_COPY_CHUNK        256U     /*按 256B 读 W25Q，正好 64 个字*/

/* 槽记录 32B：前 20B 参与记录自身 CRC，后 8B 保留 */
typedef struct
{
  uint32_t magic;
  uint32_t fmt;
  uint32_t ver;
  uint32_t size;
  uint32_t crc;
  uint32_t rec_crc;
} slot_rec_t;

static uint8_t s_copy_buf[OTA2_COPY_CHUNK];

/**
  * 功    能：打印一条槽记录的大小/CRC（只用于日志）
  * 参    数：tag 日志前缀；rec 记录
  * 返 回 值：无
  */
static void ota2_log_rec(const char *tag, const slot_rec_t *rec)
{
  uart1_send_str(tag);
  uart1_send_str(" size=");
  uart1_send_dec(rec->size);
  uart1_send_str(" crc=0x");
  uart1_send_hex32(rec->crc);
  uart1_send_str("\r\n");
}

/**
  * 功    能：读 32B 槽记录并校验 magic/fmt/size/记录 CRC
  * 参    数：base 槽基址；fmt 期望格式（1=备份 2=OTA）；rec 输出
  * 返 回 值：1 有效；0 无效
  * 补    充：size 上限按格式区分：
  *            槽 A 整区备份可以是 448KB；槽 B 镜像最多 444KB（槽首 4KB 是记录区）。
  */
static int ota2_rec_read(uint32_t base, uint32_t fmt, slot_rec_t *rec)
{
  uint32_t buf[8];
  uint32_t i;
  uint32_t crc = 0xFFFFFFFFUL;

  bsp_w25q_read(base, (uint8_t *)buf, 32UL);

  if (buf[0] != SLOT_MAGIC)
  {
    return 0;
  }
  if (buf[1] != fmt)
  {
    return 0;
  }
  if (buf[3] < 8UL)
  {
    return 0;
  }
  if ((buf[3] & 3UL) != 0UL)
  {
    return 0;
  }
  if (buf[3] > ((fmt == SLOT_FMT_BACKUP) ? APP_SIZE : SLOT_IMG_MAX))
  {
    return 0;
  }

  for (i = 0UL; i < 5UL; i++)
  {
    uint8_t k;
    for (k = 0U; k < 4U; k++)
    {
      crc = crc32_update(crc, (uint8_t)((buf[i] >> (8U * k)) & 0xFFUL));
    }
  }
  crc ^= 0xFFFFFFFFUL;
  if (crc != buf[5])
  {
    return 0;
  }

  rec->magic   = buf[0];
  rec->fmt     = buf[1];
  rec->ver     = buf[2];
  rec->size    = buf[3];
  rec->crc     = buf[4];
  rec->rec_crc = buf[5];
  return 1;
}

/**
  * 功    能：校验槽内镜像整区 CRC 是否等于记录里的 CRC
  * 参    数：base 槽基址；rec 记录
  * 返 回 值：1 一致；0 不一致或长度非法
  * 补    充：分块读，避免一次性大缓冲；搬运前验槽 A、回滚前再验一次都靠它
  */
static int ota2_img_crc_ok(uint32_t base, const slot_rec_t *rec)
{
  uint32_t off = 0UL;
  uint32_t crc = 0xFFFFFFFFUL;

  while (off < rec->size)
  {
    uint32_t n = rec->size - off;
    uint32_t i;

    if (n > OTA2_COPY_CHUNK)
    {
      n = OTA2_COPY_CHUNK;
    }
    bsp_w25q_read(base + SLOT_HDR_SIZE + off, s_copy_buf, n);
    for (i = 0UL; i < n; i++)
    {
      crc = crc32_update(crc, s_copy_buf[i]);
    }
    off += n;
  }

  return ((crc ^ 0xFFFFFFFFUL) == rec->crc) ? 1 : 0;
}

/**
  * 功    能：校验内部 App 区前 len 字节的 CRC（按小端字节序）
  * 参    数：len 字节数（4 对齐）
  * 返 回 值：CRC32
  */
static uint32_t ota2_app_crc(uint32_t len)
{
  uint32_t addr;
  uint32_t crc = 0xFFFFFFFFUL;

  for (addr = 0UL; addr < len; addr += 4UL)
  {
    uint32_t w = *(volatile uint32_t *)(APP_BASE_ADDR + addr);
    uint8_t  k;

    for (k = 0U; k < 4U; k++)
    {
      crc = crc32_update(crc, (uint8_t)((w >> (8U * k)) & 0xFFUL));
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

/**
  * 功    能：把槽内镜像整段搬到内部 App 区，并做源/目标双重 CRC 校验
  * 参    数：base 槽基址；rec 记录
  * 返 回 值：1 成功；0 任一步失败（擦除/编程/CRC/向量表）
  * 补    充：先擦 sector 4~7（整个 App 区），再按 32 位字编程；
  *            内部 Flash 解锁一次覆盖整个过程，失败路径也必须上锁。
  */
static int ota2_img_copy(uint32_t base, const slot_rec_t *rec)
{
  uint32_t off = 0UL;
  uint32_t crc = 0xFFFFFFFFUL;
  uint8_t  snb;

  flash_unlock();

  for (snb = flash_sector_of(APP_BASE_ADDR);
       snb <= flash_sector_of(APP_LIMIT_ADDR - 1UL);
       snb++)
  {
    if (flash_erase_sector(snb) == 0)
    {
      flash_lock();
      uart1_send_str("[OTA2] erase fail\r\n");
      return 0;
    }
  }

  while (off < rec->size)
  {
    uint32_t n = rec->size - off;
    uint32_t i;

    if (n > OTA2_COPY_CHUNK)
    {
      n = OTA2_COPY_CHUNK;
    }
    bsp_w25q_read(base + SLOT_HDR_SIZE + off, s_copy_buf, n);

    for (i = 0UL; i < n; i += 4UL)
    {
      uint32_t w = (uint32_t)s_copy_buf[i] |
                   ((uint32_t)s_copy_buf[i + 1UL] << 8U) |
                   ((uint32_t)s_copy_buf[i + 2UL] << 16U) |
                   ((uint32_t)s_copy_buf[i + 3UL] << 24U);
      uint8_t  k;

      for (k = 0U; k < 4U; k++)
      {
        crc = crc32_update(crc, s_copy_buf[i + k]);
      }

      if (flash_program_word(APP_BASE_ADDR + off + i, w) == 0)
      {
        flash_lock();
        uart1_send_str("[OTA2] program fail\r\n");
        return 0;
      }
    }
    off += n;
  }

  flash_lock();

  crc ^= 0xFFFFFFFFUL;
  if (crc != rec->crc)
  {
    uart1_send_str("[OTA2] source crc mismatch\r\n");
    return 0;
  }
  if (ota2_app_crc(rec->size) != rec->crc)
  {
    uart1_send_str("[OTA2] flash crc mismatch\r\n");
    return 0;
  }
  if (app_is_valid() == 0)
  {
    uart1_send_str("[OTA2] app vector invalid\r\n");
    return 0;
  }
  return 1;
}

/**
  * 功    能：作废槽 B 记录（擦首 4KB 扇区，不动镜像数据）
  * 参    数：无
  * 返 回 值：无
  */
static void ota2_slot_b_invalidate(void)
{
  if (bsp_w25q_erase_4k(SLOT_B_BASE_ADDR) != 0)
  {
    uart1_send_str("[OTA2] slot B invalidate fail\r\n");
  }
  else
  {
    uart1_send_str("[OTA2] slot B record erased\r\n");
  }
}

/**
  * 功    能：从槽 A 整区备份回滚到 App 区
  * 参    数：无
  * 返 回 值：1 成功；0 槽 A 不可用或搬运失败
  * 补    充：回滚前先验槽 A 的记录 + 整区 CRC，
  *            验不过就直接放弃，绝不在没把握的情况下先擦掉 App 区。
  */
static int ota2_rollback(void)
{
  slot_rec_t rec;

  if (ota2_rec_read(SLOT_A_BASE_ADDR, SLOT_FMT_BACKUP, &rec) == 0)
  {
    uart1_send_str("[OTA2] slot A record invalid\r\n");
    return 0;
  }
  if (ota2_img_crc_ok(SLOT_A_BASE_ADDR, &rec) == 0)
  {
    uart1_send_str("[OTA2] slot A image crc mismatch\r\n");
    return 0;
  }

  ota2_log_rec("[OTA2] rollback slot A", &rec);

  if (ota2_img_copy(SLOT_A_BASE_ADDR, &rec) == 0)
  {
    uart1_send_str("[OTA2] rollback copy fail\r\n");
    return 0;
  }

  (void)app_end_mark_write(rec.size);   /*整区备份 size=APP_SIZE，内部会跳过*/
  if (meta_write(rec.size, rec.crc, rec.ver) == 0)
  {
    uart1_send_str("[OTA2] rollback meta write fail\r\n");
  }

  uart1_send_str("[OTA2] rollback ok\r\n");
  return 1;
}

/**
  * 功    能：处理 state=1：搬运槽 B
  * 参    数：无
  * 返 回 值：无
  * 补    充：搬运成功置 state=2/count=0；失败作废槽 B 记录并回滚槽 A
  */
static void ota2_handle_apply(void)
{
  slot_rec_t rec_a;
  slot_rec_t rec_b;

  uart1_send_str("[OTA2] apply slot B\r\n");

  /*回滚基线先验：槽 A 必须记录有效且整区 CRC 对得上*/
  if (ota2_rec_read(SLOT_A_BASE_ADDR, SLOT_FMT_BACKUP, &rec_a) == 0)
  {
    uart1_send_str("[OTA2] slot A record invalid, apply refused\r\n");
    ota2_state_clear();
    return;
  }
  if (ota2_img_crc_ok(SLOT_A_BASE_ADDR, &rec_a) == 0)
  {
    uart1_send_str("[OTA2] slot A image crc mismatch, apply refused\r\n");
    ota2_state_clear();
    return;
  }
  ota2_log_rec("[OTA2] slot A ok", &rec_a);

  if (ota2_rec_read(SLOT_B_BASE_ADDR, SLOT_FMT_OTA, &rec_b) == 0)
  {
    uart1_send_str("[OTA2] slot B record invalid, apply skipped\r\n");
    ota2_state_clear();
    return;
  }
  ota2_log_rec("[OTA2] slot B ok", &rec_b);

  if (ota2_img_copy(SLOT_B_BASE_ADDR, &rec_b) == 1)
  {
    (void)app_end_mark_write(rec_b.size);
    if (meta_write(rec_b.size, rec_b.crc, rec_b.ver) == 0)
    {
      uart1_send_str("[OTA2] meta write fail\r\n");
    }
    ota2_state_write(OTA2_ST_CONFIRM, 0U);
    uart1_send_str("[OTA2] apply ok, waiting confirm\r\n");
    return;
  }

  /*搬运失败：先废掉坏镜像，避免下次上电再白试一遍，然后回滚槽 A*/
  uart1_send_str("[OTA2] apply fail, rollback slot A\r\n");
  ota2_slot_b_invalidate();
  (void)ota2_rollback();
  ota2_state_clear();
}

/**
  * 功    能：处理 state=2：统计未确认启动次数，到上限回滚
  * 参    数：count 当前计数
  * 返 回 值：无
  */
static void ota2_handle_confirm(uint8_t count)
{
  if (count >= OTA2_MAX_UNCONFIRMED)
  {
    uart1_send_str("[OTA2] unconfirmed x3, rollback slot A\r\n");
    ota2_slot_b_invalidate();
    (void)ota2_rollback();
    ota2_state_clear();
    return;
  }

  count++;
  ota2_state_write(OTA2_ST_CONFIRM, count);
  uart1_send_str("[OTA2] waiting confirm ");
  uart1_send_dec(count);
  uart1_send_str("/3\r\n");
}

void ota2_boot_handle(void)
{
  uint8_t state = OTA2_ST_NONE;
  uint8_t count = 0U;

  if (ota2_state_read(&state, &count) == 0)
  {
    return;
  }
  if (state == OTA2_ST_NONE)
  {
    return;
  }

  if (bsp_w25q_init() == 0)
  {
    uart1_send_str("[OTA2] w25q absent, ota2 skipped\r\n");
    ota2_state_clear();
    return;
  }

  if (state == OTA2_ST_APPLY)
  {
    ota2_handle_apply();
  }
  else if (state == OTA2_ST_CONFIRM)
  {
    ota2_handle_confirm(count);
  }
}
