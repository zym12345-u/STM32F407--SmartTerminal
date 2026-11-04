/* BootLoader/upgrade.c
 * 升级模式：擦除 → 数据块 → 结束三条命令的落地动作，外加断电续传的游标恢复。
 *
 * ── 三条升级命令怎么配合 ──
 *   0x04 擦除（带起始/结束地址）→ 0x02 数据块（不带地址）→ 0x03 结束。
 *   数据块故意不带地址：地址由设备自己往后加，PC 只管按顺序发；
 *   每帧都等 ACK 才发下一帧（停等），漏帧会被 CRC 拦下重发，顺序天然不会乱，
 *   PC 端也不用算偏移，少一类出错的可能。
 *   每条命令都回一帧应答：命令号 | 0x80，数据 1 字节状态 —— 0 成功、1 地址/长度不对、2 Flash 出错。
 *   擦除和编程只允许落在 App 区（0x08010000~0x0807FFFF，即 sector 4~7），
 *   而且必须按扇区边界对齐；这样 sector 0~2（BootLoader 自己）和 sector 3（元数据）
 *   永远不会被这条命令碰到 —— 越界的请求直接回 1，不硬写。
 *   写每个字之前先读一遍，不是 0xFFFFFFFF 就报错：说明这扇区没擦干净，
 *   不做「悄悄帮你擦掉」这种事，免得把还在用的数据抹了。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "boot_layout.h"
#include "bsp_uart1.h"
#include "flash_ops.h"
#include "meta.h"
#include "bsp_ee.h"
#include "board.h"
#include "main.h"
#include "upgrade.h"

/* 升级过程的写入游标：擦除成功后从 App 基址开始，按收到的字节顺序往后长 */
static uint32_t g_write_addr;
static uint32_t g_written;
static uint8_t  g_word_buf[4];   /*F4 内部 Flash 只能按字编程，凑够 4 字节才写一次*/
static uint8_t  g_word_n;

/**
  * 功    能：处理擦除命令：只动 App 区，而且必须按扇区边界对齐
  * 参    数：data 8 字节载荷（起始地址 4B + 结束地址 4B，都是小端、结束地址不含）
  * 返 回 值：状态字节 ST_OK / ST_RANGE / ST_FLASH
  * 补    充：为什么要求对齐：擦除最小单位是整个扇区，
  *            传个"半扇区"进来擦不干净，还会让后面写数据的地址判断变得含糊，
  *            干脆一开始就拒绝，让 PC 端正对自己要传的镜像大小算好边界
  */
static uint8_t upgrade_erase(const uint8_t *data, uint8_t len)
{
  uint32_t start;
  uint32_t end;
  uint8_t  s0;
  uint8_t  s1;
  uint8_t  s;

  if (len != 8U)
  {
    return ST_RANGE;
  }

  start = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
          ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
  end   = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) |
          ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);

  if ((start < APP_BASE_ADDR) || (end > APP_LIMIT_ADDR) || (start >= end))
  {
    return ST_RANGE;   /*越界一步都不擦*/
  }

  s0 = flash_sector_of(start);
  s1 = flash_sector_of(end - 1UL);
  if ((s0 > 7U) || (s1 > 7U) || (s0 > s1))
  {
    return ST_RANGE;   /*sector 0~3 里有 BootLoader 自己和元数据，一律不许碰*/
  }
  if ((start != flash_sector_start(s0)) ||
      (end != (flash_sector_start(s1) + flash_sector_size(s1))))
  {
    return ST_RANGE;   /*不是整扇区边界就不擦*/
  }

  flash_unlock();
  for (s = s0; s <= s1; s++)
  {
    if (!flash_erase_sector(s))
    {
      flash_lock();
      return ST_FLASH;
    }
  }
  flash_lock();

  /*擦完把写入游标归零：数据块永远从 App 基址开始长*/
  g_write_addr = APP_BASE_ADDR;
  g_word_n     = 0U;
  g_written    = 0UL;

  /*擦除成功 = 升级正式开始：标志先立起来，从这里开始掉电都能被认出来*/
  ota_mark(0UL);
  return ST_OK;
}

/**
  * 功    能：处理数据块命令：按收到的顺序把字节攒成字，逐个写进 Flash
  * 参    数：data 数据段；len 本帧字节数
  * 返 回 值：状态字节
  * 补    充：地址由游标自己维护，PC 端不带地址；
  *            剩余不足 4 字节的尾巴留到"结束"命令时补齐，所以这里只管整字
  */
static uint8_t upgrade_write(const uint8_t *data, uint8_t len)
{
  uint32_t word;
  uint8_t  i;

  if ((g_write_addr < APP_BASE_ADDR) ||
      ((g_write_addr + (uint32_t)len) > APP_LIMIT_ADDR))
  {
    return ST_RANGE;   /*越界先拦住，别把 sector 3 的元数据写了*/
  }

  flash_unlock();

  for (i = 0U; i < len; i++)
  {
    g_word_buf[g_word_n] = data[i];
    g_word_n++;
    g_written++;

    if (g_word_n >= 4U)
    {
      word = ((uint32_t)g_word_buf[0]) | ((uint32_t)g_word_buf[1] << 8U) |
             ((uint32_t)g_word_buf[2] << 16U) | ((uint32_t)g_word_buf[3] << 24U);

      if (!flash_program_word(g_write_addr, word))
      {
        flash_lock();
        return ST_FLASH;
      }
      g_write_addr += 4UL;
      g_word_n = 0U;
    }
  }

  flash_lock();

  /*进度落盘：先写 Flash 再更新 EEPROM，断电最坏是 EEPROM 比实际少记一帧，
    方向安全 —— 不会把没写进去的字节记成已写*/
  if (len > 0U)
  {
    ota_mark(g_written);
  }
  return ST_OK;
}

/**
  * 功    能：处理结束命令：把不足一字的尾巴补齐写完
  * 参    数：无
  * 返 回 值：状态字节
  * 补    充：尾巴补 0x00，只影响镜像的最后一个字；镜像大小不一定是 4 的倍数，
  *            但不补齐的话这份数据就不是"整字写进去"的状态，读回来会缺字节
  */
static uint8_t upgrade_finish(void)
{
  uint32_t word = 0UL;
  uint8_t  i;

  if (g_word_n == 0U)
  {
    return ST_OK;   /*正好整字收完，没什么可补的*/
  }

  while (g_word_n < 4U)
  {
    g_word_buf[g_word_n] = 0x00U;
    g_word_n++;
  }

  for (i = 0U; i < 4U; i++)
  {
    word |= ((uint32_t)g_word_buf[i]) << (8U * i);
  }

  flash_unlock();
  if (!flash_program_word(g_write_addr, word))
  {
    flash_lock();
    return ST_FLASH;
  }
  flash_lock();

  g_write_addr += 4UL;
  g_word_n = 0U;
  return ST_OK;
}

/**
  * 功    能：对 App 区前 len 字节算 CRC32（算法与帧 CRC 完全一致，初值 0xFFFFFFFF，末尾取反）
  * 参    数：len 字节数——就是 PC 本次实际送来的长度（g_written）
  * 返 回 值：CRC32 值
  * 补    充：只在升级结束时算这一次（约 300KB，逐位算法在 168MHz 下几百毫秒）。
  *            不在每次上电都算：那会把正常启动也拖慢，而断电场景已经由 24C02
  *            标志兜底，重复计算没有额外收益，只是白等。
  */
static uint32_t app_crc32(uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t addr;

  for (addr = APP_BASE_ADDR; addr < (APP_BASE_ADDR + len); addr++)
  {
    crc = crc32_update(crc, *(volatile uint8_t *)addr);
  }
  return crc ^ 0xFFFFFFFFUL;
}

/**
  * 功    能：升级模式主循环：一直收命令，按命令号分发
  * 参    数：无
  * 返 回 值：无（结束命令校验通过就跳 App；没通过就继续留在这儿等重传）
  * 补    充：每条命令都回一帧应答，PC 端拿到状态才发下一帧（停等），
  *            这样丢帧、错帧都能在局部重发，不用整包重来
  */
void upgrade_loop(void)
{
  frame_parser_t p;
  uint8_t ch;
  uint8_t r;
  uint8_t st;

  /*还没擦除就直接发数据的话，地址检查会立刻把这块拦住*/
  g_write_addr = APP_BASE_ADDR;
  g_word_n     = 0U;
  g_written    = 0UL;

  /*断电续传：标志还在就按 24C02 里记的进度把游标接上。
   * 只有 PC 真的从断点开始发数据才用得到；PC 若改发擦除命令，这两个值会被清 0。
   * 要求 written 是 4 的倍数：Flash 只能按字写，不对齐的那几个字节没法复原，
   * 这种情况宁可让 PC 重擦重发（PC 端镜像已经补齐到 4 字节，正常不会触发）。*/
  if (ota_is_marked() != 0)
  {
    uint32_t w = ota_read_written();
    if ((w > 0UL) && (w < (APP_LIMIT_ADDR - APP_BASE_ADDR)) && ((w & 3UL) == 0UL))
    {
      g_written    = w;
      g_write_addr = APP_BASE_ADDR + w;
    }
  }

  led_init();
  led_on();   /*绿灯常亮 = 停在 BootLoader，没跳 App*/

  frame_reset(&p);

  for (;;)
  {
    if ((USART1->SR & USART_SR_RXNE) == 0UL)   /*升级模式不设超时，一直等到有字节*/
    {
      continue;
    }

    ch = (uint8_t)USART1->DR;
    r  = frame_feed(&p, ch);

    if (r == 0U)
    {
      continue;
    }

    if (r == 2U)   /*CRC 不对：不回话，PC 超时后重发这一帧*/
    {
      uart1_send_str("[BOOT] bad frame\r\n");
      continue;
    }

    switch (p.cmd)
    {
    case CMD_HANDSHAKE:
      /*升级模式里再收到握手照样回 ACK：断电续升级时 PC 端还是从握手开始走流程*/
      frame_reply(CMD_HANDSHAKE, ST_OK);
      break;

    case CMD_DATA:
      st = upgrade_write(p.data, p.len);
      frame_reply(CMD_DATA, st);
      break;

    case CMD_ERASE:
      st = upgrade_erase(p.data, p.len);
      frame_reply(CMD_ERASE, st);
      break;

    case CMD_PROGRESS:
    {
      uint8_t buf[4];

      if (p.len != 0U)
      {
        frame_reply(CMD_PROGRESS, ST_RANGE);
        break;
      }
      buf[0] = (uint8_t)(g_written & 0xFFUL);
      buf[1] = (uint8_t)((g_written >> 8U) & 0xFFUL);
      buf[2] = (uint8_t)((g_written >> 16U) & 0xFFUL);
      buf[3] = (uint8_t)((g_written >> 24U) & 0xFFUL);
      /*这条应答的「数据」是进度值而不是状态字节，所以不能走 frame_reply*/
      frame_send(CMD_PROGRESS | CMD_ACK_FLAG, buf, 4U);
      break;
    }

    case CMD_END:
    {
      uint32_t want_crc = 0UL;
      uint32_t want_ver = 0UL;
      uint32_t want_len = g_written;   /*补字会挪游标，但 g_written 始终是 PC 送来的长度*/
      int      check    = 0;

      /* 结束命令带 6 字节 = 整包 CRC32(4) + 固件版本(主,次)；带 0 字节 = 只擦除模式，跳过校验 */
      if (p.len == 6U)
      {
        want_crc = (uint32_t)p.data[0] | ((uint32_t)p.data[1] << 8U) |
                   ((uint32_t)p.data[2] << 16U) | ((uint32_t)p.data[3] << 24U);
        want_ver = FW_VER_PACK(p.data[4], p.data[5]);
        check = 1;
      }
      else if (p.len != 0U)
      {
        frame_reply(CMD_END, ST_RANGE);
        break;
      }

      st = upgrade_finish();

      if ((st == ST_OK) && (check != 0))
      {
        uart1_send_str("[BOOT] crc check ");
        if (app_crc32(want_len) != want_crc)
        {
          uart1_send_str("mismatch, image corrupted\r\n");
          st = ST_CRC;   /*整包不可信：不清标志、不写元数据，让 PC 重传*/
        }
        else
        {
          uart1_send_str("ok\r\n");
        }
      }

      if ((st == ST_OK) && (check != 0))
      {
        /*先写镜像末尾标记，再写元数据：两者都成功才算「内容和记录对得上」*/
        if (app_end_mark_write(want_len) == 0)
        {
          uart1_send_str("[BOOT] end mark write failed\r\n");
          st = ST_FLASH;
        }
        else if (meta_write(want_len, want_crc, want_ver) == 0)
        {
          uart1_send_str("[BOOT] meta write failed\r\n");
          st = ST_FLASH;
        }
        else
        {
          uart1_send_str("[BOOT] meta saved, fw v");
          uart1_send_dec((want_ver >> 8U) & 0xFFUL);
          uart1_send_str(".");
          uart1_send_dec(want_ver & 0xFFUL);
          uart1_send_str("\r\n");
        }
      }

      frame_reply(CMD_END, st);
      if (st == ST_OK)
      {
        ota_clear();   /*升级顺利走完才清标志；清了以后断电上电都走正常跳转*/
        uart1_send_str("[BOOT] written ");
        uart1_send_dec(g_written);
        uart1_send_str(" bytes, head=0x");
        uart1_send_hex32(*(volatile uint32_t *)APP_BASE_ADDR);
        uart1_send_str("\r\n");

        if (app_is_valid())
        {
          uart1_send_str("[BOOT] jump to app\r\n");
          jump_to_app();
        }
        uart1_send_str("[BOOT] app invalid, stay in upgrade mode\r\n");
      }
      break;
    }

    default:
      frame_reply(p.cmd, ST_RANGE);   /*不认识的命令也回一句，PC 不会一直干等*/
      break;
    }
  }
}
