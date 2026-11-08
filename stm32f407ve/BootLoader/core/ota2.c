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
  uint32_t buf[8];    /*一次读 32B：用 uint32 数组接，省掉手工拼字节*/
  uint32_t i;
  uint32_t crc = 0xFFFFFFFFUL;

  /*读失败在这里没有返回值可查（bsp_w25q_read 无返回值），
   *但 SPI 超时会返回 0xFF，magic 对不上直接判无效——错误被安全地吸收掉了*/
  bsp_w25q_read(base, (uint8_t *)buf, 32UL);

  if (buf[0] != SLOT_MAGIC)        /*magic 不对：空槽(0xFFFFFFFF)、或者压根不是槽记录*/
  {
    return 0;
  }
  if (buf[1] != fmt)               /*格式不对：1=备份、2=OTA，拿错槽就拒绝*/
  {
    return 0;
  }
  if (buf[3] < 8UL)                /*size 太小：连一个向量表都放不下*/
  {
    return 0;
  }
  if ((buf[3] & 3UL) != 0UL)       /*size 非 4 对齐：搬运时按字写会越界半字*/
  {
    return 0;
  }
  /*size 上限按格式区分：槽 A 整区备份 = APP_SIZE(448KB)，
   *槽 B 镜像 = SLOT_IMG_MAX(444KB)。搞反了会把 448KB 的备份当成镜像搬，多写 4KB*/
  if (buf[3] > ((fmt == SLOT_FMT_BACKUP) ? APP_SIZE : SLOT_IMG_MAX))
  {
    return 0;
  }

  /*记录 CRC 只覆盖前 5 字(20B)，与 App 侧 OTA_WriteRecord 完全一致。
   *按小端把每个字拆成 4 个字节再喂 crc32_update：
   *crc32 是"流式"算法，字节顺序必须和写入时一致，不能直接把 uint32 塞进去*/
  for (i = 0UL; i < 5UL; i++)
  {
    uint8_t k;
    for (k = 0U; k < 4U; k++)
    {
      crc = crc32_update(crc, (uint8_t)((buf[i] >> (8U * k)) & 0xFFUL));
    }
  }
  crc ^= 0xFFFFFFFFUL;             /*算法约定：终值异或一次初值*/
  if (crc != buf[5])
  {
    return 0;                      /*记录自身被写坏（掉电写了半条）*/
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

    if (n > OTA2_COPY_CHUNK)     /*每次只读 256B：BootLoader 没有大块 RAM 可用*/
    {
      n = OTA2_COPY_CHUNK;
    }
    /*+SLOT_HDR_SIZE 跳过槽首 4KB 记录区，读到的是镜像本体。
     *注意这里读的长度是 rec->size，不是整槽 448KB——
     *槽 A 的 size 就是 APP_SIZE，槽 B 的是真实镜像长度，两者共用这段逻辑*/
    bsp_w25q_read(base + SLOT_HDR_SIZE + off, s_copy_buf, n);
    for (i = 0UL; i < n; i++)
    {
      crc = crc32_update(crc, s_copy_buf[i]);
    }
    off += n;
  }

  /*这一步是"回滚基线可信"的唯一判据：对不上就绝不允许擦 App 区。
   *代价是搬运前多花约 1 秒读 448KB，换的是"擦完发现备份是坏的"这种死局*/
  return ((crc ^ 0xFFFFFFFFUL) == rec->crc) ? 1 : 0;
}

/**
  * 功    能：校验内部 App 区前 len 字节的 CRC（按小端字节序）
  * 参    数：len 字节数（4 对齐）
  * 返 回 值：CRC32
  * 补    充：搬运写完后再算一次，用来证明"写进 Flash 的字节没错"。
  *            为什么按字读再拆字节：内部 Flash 是内存映射，32 位读一次拿 4 字节，
  *            比按字节读快得多；拆字节是因为 crc32_update 是逐字节接口。
  *            耗时参考（估算）：16MHz 下 448KB 约 2~3 秒。
  */
static uint32_t ota2_app_crc(uint32_t len)
{
  uint32_t addr;
  uint32_t crc = 0xFFFFFFFFUL;

  for (addr = 0UL; addr < len; addr += 4UL)
  {
    /*volatile 不能省：否则编译器可能把循环里的重复读优化掉，
     *而这块内容在搬运过程中是会变的（由 flash_program_word 写进去）*/
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

  /*解锁一次覆盖"擦 + 写"整个过程：每次操作都解锁上锁会多出十几万次
   *寄存器写，而且中间被中断打断时状态更难推理。
   *下面每条失败路径都必须先 flash_lock() 再返回，否则 BootLoader 会带着
   *解锁状态去跳 App——App 里任何一次误写都能把 Flash 改掉。*/
  flash_unlock();

  /*擦的是 App 区整片：sector 4(64KB) + sector 5~7(各 128KB) = 448KB。
   *用 flash_sector_of 换算而不是硬编码 4~7：换芯片后 Flash 扇区表变了，
   *这里会跟着变，这正是 port 层 flash_ops 存在的意义*/
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

  /*擦完才能写：Flash 编程只能把 1 变 0，不先擦成 0xFF 根本写不进新数据。
   *所以不能"边擦边写"——一旦擦除顺序与写入顺序交叉，后面的擦除会把前面
   *写好的数据一起抹掉。顺序必须是"整片擦 → 整片写"*/
  while (off < rec->size)
  {
    uint32_t n = rec->size - off;
    uint32_t i;

    if (n > OTA2_COPY_CHUNK)
    {
      n = OTA2_COPY_CHUNK;
    }
    bsp_w25q_read(base + SLOT_HDR_SIZE + off, s_copy_buf, n);

    for (i = 0UL; i < n; i += 4UL)   /*步长 4：rec->size 在 ota2_rec_read 里已验过 4 对齐*/
    {
      /*手工拼 32 位：W25Q 读出来的是字节流，内部 Flash 只能按字编程。
       *按小端拼，和 PC 端 bin 的字节序一致，也与 ota2_app_crc 的拆字节顺序对称*/
      uint32_t w = (uint32_t)s_copy_buf[i] |
                   ((uint32_t)s_copy_buf[i + 1UL] << 8U) |
                   ((uint32_t)s_copy_buf[i + 2UL] << 16U) |
                   ((uint32_t)s_copy_buf[i + 3UL] << 24U);
      uint8_t  k;

      /*第一次 CRC：对"从槽里读出来的数据"算，证明源数据没坏*/
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

  /*三重验收，缺一不可：
   *  1) 源 CRC —— 从槽里读出来的字节就是记录里那份（SPI 没读错）
   *  2) 目标 CRC —— 这些字节真的落到 App 区了（写操作没静默失效）
   *  3) 向量表 —— 新镜像的栈顶/复位入口合法（不是一份"CRC 对但根本不是程序"的数据）
   *少任何一条，都可能出现"校验通过但一开机就 HardFault"的镜像*/
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
  /*只擦槽首 4KB（记录扇区），镜像本体 444KB 原封不动：
   *记录一没，BootLoader 就当"这个槽里没有可用镜像"，
   *将来重新下载时会从槽基址开始整槽擦写，坏数据自然被覆盖*/
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

  /*整区备份的 size = APP_SIZE(448KB)，镜像正好填满 App 区，
   *没有"镜像末尾之后那个字"可写 MEND，app_end_mark_write 内部会直接跳过。
   *后果是回滚后启动日志会显示 mark stale——那不是回滚失败，是正常现象*/
  (void)app_end_mark_write(rec.size);
  if (meta_write(rec.size, rec.crc, rec.ver) == 0)
  {
    /*元数据写失败不致命：App 区内容已经是好的，只是版本号会报告成旧值。
     *下次成功升级/回滚会再写一次，所以这里只记日志不阻断*/
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

  /*第一道门：回滚基线（槽 A）必须存在且记录自洽。
   *顺序刻意的——先验槽 A 再验槽 B：擦 App 区之前花在槽 A 上的每一毫秒，
   *买的都是"擦完发现回不去"这个死局不会发生。
   *【已知代价】用户如果从没点过「FW Update」做整区备份，这里会拒绝并清掉请求，
   *他刚下完的 300KB 等于白下（要重下）。安全上是对的，流程上确实不够友好。*/
  if (ota2_rec_read(SLOT_A_BASE_ADDR, SLOT_FMT_BACKUP, &rec_a) == 0)
  {
    uart1_send_str("[OTA2] slot A record invalid, apply refused\r\n");
    ota2_state_clear();
    return;
  }
  /*第二道门：槽 A 的整区 CRC。记录可以伪造/残留，数据才是真的。
   *这一步要读 448KB，约 1 秒，是整个启动流程里最贵的一次校验，必须做*/
  if (ota2_img_crc_ok(SLOT_A_BASE_ADDR, &rec_a) == 0)
  {
    uart1_send_str("[OTA2] slot A image crc mismatch, apply refused\r\n");
    ota2_state_clear();
    return;
  }
  ota2_log_rec("[OTA2] slot A ok", &rec_a);

  /*第三道门：槽 B（要搬的新镜像）记录必须有效。
   *这里只验记录不验整区 CRC——整区 CRC 会在 ota2_img_copy 里边读边算，
   *再单独验一遍等于白读 444KB*/
  if (ota2_rec_read(SLOT_B_BASE_ADDR, SLOT_FMT_OTA, &rec_b) == 0)
  {
    uart1_send_str("[OTA2] slot B record invalid, apply skipped\r\n");
    ota2_state_clear();
    return;
  }
  ota2_log_rec("[OTA2] slot B ok", &rec_b);

  if (ota2_img_copy(SLOT_B_BASE_ADDR, &rec_b) == 1)
  {
    /*搬运成功后的顺序不能反：先写 MEND（证明 Flash 内容与 size 对得上），
     *再写元数据（让 App 报告正确的版本），最后改状态页。
     *任何一步掉电，下次上电都会重走 apply——因为 state 还是 APPLY*/
    (void)app_end_mark_write(rec_b.size);
    if (meta_write(rec_b.size, rec_b.crc, rec_b.ver) == 0)
    {
      uart1_send_str("[OTA2] meta write fail\r\n");
    }
    /*state=CONFIRM、count=0：新固件这一次启动算"第 0 次机会"，
     *跑起来确认后 App 会把整页清 0；跑不起来就每次上电 +1，到 3 次回滚*/
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
  /*为什么是 3 次而不是 1 次：新固件第一次启动后 App 要跑满 3 秒才确认，
   *这 3 秒里用户完全可能拔电、按 RST、或者串口正在刷日志。
   *只给 1 次机会，正常升级也会被误判成"跑不起来"而回滚。
   *给 3 次，误判概率降到基本可以忽略；代价是新固件真坏时要多等 3 次上电*/
  if (count >= OTA2_MAX_UNCONFIRMED)
  {
    uart1_send_str("[OTA2] unconfirmed x3, rollback slot A\r\n");
    ota2_slot_b_invalidate();
    (void)ota2_rollback();
    ota2_state_clear();
    return;
  }

  /*计数 +1 立刻写回 EEPROM：写在"这次启动还没证明自己"的时刻，
   *所以哪怕下一秒掉电，这次未确认也已经记账了（宁可多记一次，不能少记）*/
  count++;
  ota2_state_write(OTA2_ST_CONFIRM, count);
  uart1_send_str("[OTA2] waiting confirm ");
  uart1_send_dec(count);
  uart1_send_str("/3\r\n");
}

/**
  * 功    能：OTA-2 总入口，由 main 在串口窗口关闭后、报告元数据之前调用
  * 参    数：无
  * 返 回 值：无
  * 补    充：位置有讲究——
  *            1) 必须在 uart1_wait_handshake 之后：串口 IAP 的 3 秒窗口优先级更高，
  *               不能让一次 20 秒的搬运把升级窗口挤掉
  *            2) 必须在 boot_meta_report 之前：搬运会改写元数据，先搬后报才对得上
  *            【已知代价】bsp_w25q_init 失败（杜邦线接触不良/上电时序抖动）时，
  *            这里会把 APPLY 请求清掉，用户得重新下载一次。改成"保留状态下次再试"
  *            需要额外的失败计数，当前版本选了简单方案。
  */
void ota2_boot_handle(void)
{
  uint8_t state = OTA2_ST_NONE;
  uint8_t count = 0U;

  /*先读状态页：绝大多数启动（没有 OTA 请求）会在这里直接返回，
   *连 W25Q 都不初始化，不给正常启动增加任何延迟*/
  if (ota2_state_read(&state, &count) == 0)
  {
    return;   /*EEPROM 不在位或 magic 不对：当作没有请求*/
  }
  if (state == OTA2_ST_NONE)
  {
    return;
  }

  /*只有真有请求时才初始化外部 Flash：SPI1 配置 + JEDEC 读 ID，
   *正常启动路径上完全不碰它，避免"OTA 功能拖慢每次开机"*/
  if (bsp_w25q_init() == 0)
  {
    uart1_send_str("[OTA2] w25q absent, ota2 skipped\r\n");
    ota2_state_clear();
    return;
  }

  if (state == OTA2_ST_APPLY)
  {
    ota2_handle_apply();      /*用户刚下完新固件：搬进 App 区*/
  }
  else if (state == OTA2_ST_CONFIRM)
  {
    ota2_handle_confirm(count);  /*已经搬过、等新固件自己确认*/
  }
}
