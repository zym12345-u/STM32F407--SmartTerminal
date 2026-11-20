/* bsp_OtaStore.c
 * 槽 B 存储：CRC32、256B 攒页写入、整槽擦除、槽记录写入与回读校验。
 * 只被下载流程调用；大缓冲全部 static，不占 WifiTask 的 3KB 栈。
 */
#include "bsp_OtaInternal.h"
#include "OtaData.h"
#include "bsp_IWDG.h"
#include "usart.h"

#include <string.h>


/*这些缓冲必须是 static：WifiTask 栈只有 3KB，放栈上直接溢出*/
static uint8_t  s_page[W25Q128_PAGE_SIZE];
static uint16_t s_pageFill;
static uint32_t s_written;
static uint32_t s_crc;


/**
  * 功    能：CRC32（逐位，多项式 0xEDB88320），与 W25Q128_Crc32Range / iap_upgrade.py 同算法
  * 参    数：crc  初值传 0xFFFFFFFF；buf  数据；n  字节数
  * 返 回 值：中间值（最终结果要再异或 0xFFFFFFFF）
  * 补    充：边收边算，避免为 300KB 镜像再申请一块缓冲。
  *            为什么不用 STM32 的硬件 CRC 外设：硬件 CRC 的初值、输入/输出反射、
  *            字节序都可配，默认语义与 zlib / binascii.crc32 不一致，PC 端算出的
  *            meta.crc 在这边永远对不上。软件逐位实现虽然慢，但"和谁算都一样"。
  */
uint32_t OtaCrc32(uint32_t crc, const uint8_t *buf, uint32_t n)
{
    uint32_t i;
    uint8_t  bit;

    for (i = 0UL; i < n; i++)
    {
        crc ^= (uint32_t)buf[i];          /*先并进最低字节，再逐位移出（反射算法）*/
        for (bit = 0U; bit < 8U; bit++)
        {
            /*最低位为 1 表示这一位产生了余数，右移后要异或多项式；否则只右移。
             *0xEDB88320 是标准 CRC32 多项式的"位反转"形式，配合右移使用*/
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
  * 功    能：开始一轮下载：重置 CRC 初值、页缓冲和已写字节
  * 参    数：无
  * 返 回 值：无
  */
void OtaStore_Begin(void)
{
    s_crc      = 0xFFFFFFFFUL;   /*CRC 初值按标准固定是这个，不能给 0*/
    s_pageFill = 0U;
    s_written  = 0UL;            /*三个 static 每次下载都重置：它们是"这一轮"的状态*/
}

/**
  * 功    能：把一段收到的正文算 CRC 并按 256B 攒页写进槽 B
  * 参    数：data  数据；n  字节数
  * 返 回 值：1 = 成功，0 = 页编程失败
  * 补    充：攒满一页才写，减少 SPI 事务；最后不足一页由 OtaStore_Finish 收尾。
  *
  * ── 为什么必须按 256B 攒页 ──
  *   W25Q 的"页编程"命令（0x02）一次最多写 256B，而且【不能跨页】：
  *   从地址 0x...F0 写 32 字节，超出的部分会回卷到本页开头（0x...00），
  *   把前面的数据覆盖掉。所以起始地址和长度都必须按页边界安排。
  *   这里让 s_written 只在"整页写成功"时才前进，天然保证下一页起点 256 对齐；
  *   结尾的零头由 OtaStore_Finish 单独写一次（起点仍对齐，页内部分写合法）。
  *
  * ── 为什么在这里喂狗 ──
  *   下载循环里每收一块也喂，但页写本身要 1~2ms，加上 SPI 事务的抖动，
  *   一条 30 秒的下载链路不能只靠外层循环喂。规则：任何可能"闷头干几百毫秒"
  *   的地方自己补报一次心跳，见 stm32-freertos-task skill 的"长流程补喂狗"。
  */
uint8_t OtaStore_Feed(const uint8_t *data, uint16_t n)
{
    s_crc = OtaCrc32(s_crc, data, n);   /*边收边算：收完就不用再读一遍 300KB*/

    while (n > 0U)
    {
        uint16_t room = (uint16_t)(W25Q128_PAGE_SIZE - s_pageFill);   /*本页还能装多少*/
        uint16_t take = (n < room) ? n : room;

        memcpy(&s_page[s_pageFill], data, take);
        s_pageFill = (uint16_t)(s_pageFill + take);
        data += take;
        n = (uint16_t)(n - take);

        if (s_pageFill >= W25Q128_PAGE_SIZE)      /*攒满一页才动一次 SPI*/
        {
            if (W25Q128_PageProgram(OTA_IMG_BASE + s_written, s_page, W25Q128_PAGE_SIZE) != HAL_OK)
            {
                UART_Printf("[OTA] page program fail @0x%06lX\r\n",
                            (unsigned long)(OTA_IMG_BASE + s_written));
                return 0U;
            }
            s_written += W25Q128_PAGE_SIZE;       /*只在整页写成功后前进，保证页对齐*/
            s_pageFill = 0U;
            Watchdog_Heartbeat(WDT_TASK_WIFI);
        }
    }
    return 1U;
}

/**
  * 功    能：收尾：写最后不足一页的零头，并回填本轮已写字节数与整包 CRC
  * 参    数：written  输出已写入槽 B 的字节数；crc  输出整包 CRC32（已做终值异或）
  * 返 回 值：1 = 成功；0 = 尾页编程失败
  */
uint8_t OtaStore_Finish(uint32_t *written, uint32_t *crc)
{
    /*最后不足一页：起始地址是 256 对齐的，页内部分写合法*/
    if (s_pageFill > 0U)
    {
        if (W25Q128_PageProgram(OTA_IMG_BASE + s_written, s_page, s_pageFill) != HAL_OK)
        {
            return 0U;
        }
        s_written += (uint32_t)s_pageFill;
        s_pageFill = 0U;
    }

    if (written != NULL)
    {
        *written = s_written;
    }
    if (crc != NULL)
    {
        /*算法约定：最终结果要再异或一次初值*/
        *crc = s_crc ^ 0xFFFFFFFFUL;
    }
    return 1U;
}

/**
  * 功    能：擦除槽 B 的全部 112 个扇区（含记录扇区）
  * 参    数：无
  * 返 回 值：1 = 全部擦完，0 = 某个扇区失败
  * 补    充：逐扇区调用以便按扇区报进度、补 WIFI 心跳（底层只报 STORAGE 心跳）。
  *
  * ── 112 是怎么来的 ──
  *   槽大小 0x70000(448KB) ÷ 扇区 0x1000(4KB) = 112。
  *   注意这里擦的是【整槽】，包含首扇区里那条 32B 记录：
  *   所以一旦开始下载，槽 B 的旧记录就先没了——下载失败后槽 B 只剩"没记录的垃圾"，
  *   重跑一次成功才能恢复。这是有意为之：不先废掉旧记录，BootLoader 可能在
  *   下载中途复位时把半份镜像当有效。
  *
  * ── 为什么不用"整片擦除"命令 ──
  *   W25Q 有 0xC7 整片擦（十几秒）和 0x52/0xD8 块擦，但只有 4KB 扇区擦能给出
  *   细粒度进度、并且每次擦完能补一次心跳；整片擦期间任务完全失联，看门狗会先动手。
  */
uint8_t OtaStore_EraseSlot(void)
{
    uint32_t i;
    uint32_t sectors = W25Q128_FW_SLOT_SIZE / W25Q128_SECTOR_SIZE;   /*448KB/4KB = 112*/

    for (i = 0UL; i < sectors; i++)
    {
        if (W25Q128_SectorErase4K(W25Q128_FW_SLOT_B_ADDR + (i * W25Q128_SECTOR_SIZE)) != HAL_OK)
        {
            UART_Printf("[OTA] erase fail sector %lu\r\n", (unsigned long)i);
            return 0U;
        }
        Watchdog_Heartbeat(WDT_TASK_WIFI);   /*单扇区擦除几十 ms，112 次累计 4~5s，必须中途补报*/
        g_otaStatus.percent = (uint8_t)(15UL + (5UL * (i + 1UL)) / sectors);   /* 擦除段占 15~20（0~15 是前置的槽 A 备份） */
    }
    return 1U;
}

/**
  * 功    能：在槽 B 首扇区写 32B 记录（格式 2 = OTA 镜像），并回读校验
  * 参    数：size  镜像字节数；crc  整包 CRC；ver  目标版本
  * 返 回 值：1 = 写入且回读一致，0 = 失败
  * 补    充：记录布局与槽 A 备份一致：magic / 格式 / 版本 / 大小 / CRC / 记录自身 CRC
  *            记录最后写，失败随时中止都不会留下“看起来有效”的槽
  */
uint8_t OtaStore_WriteRecord(uint32_t size, uint32_t crc, uint16_t ver)
{
    uint32_t hdr[8];      /*8 字 = 32B，正好一条记录。槽基址是 4KB 对齐的，32B 不会跨页*/
    uint32_t rd[8];       /*回读缓冲，和 hdr 同尺寸，方便逐字段比对*/
    uint8_t  i;
    uint32_t recCrc;

    for (i = 0U; i < 8U; i++)
    {
        hdr[i] = 0xFFFFFFFFUL;   /*保留字节保持擦除态：写 0 反而会和"真实数据"混淆*/
    }
    hdr[0] = W25Q128_FW_MAGIC;   /*'FWSL'：BootLoader 靠它确认"这是一条槽记录"*/
    hdr[1] = OTA_REC_FMT;        /*2 = 槽 B OTA 镜像（槽 A 备份是 1，两者不能混）*/
    hdr[2] = (uint32_t)ver;      /*版本：主×256 + 次，v1.4 = 0x0104*/
    hdr[3] = size;               /*镜像有效字节数，不含末尾补齐的 0xFF*/
    hdr[4] = crc;                /*整包 CRC32，BootLoader 搬完要对这个数*/
    /*记录自身 CRC 只覆盖前 20B：后 8B 保留位是 0xFF，
     *如果算进去，将来改保留位含义时所有旧记录都会校验失败*/
    recCrc = OtaCrc32(0xFFFFFFFFUL, (const uint8_t *)hdr, 20UL) ^ 0xFFFFFFFFUL;
    hdr[5] = recCrc;

    if (W25Q128_WriteRange(W25Q128_FW_SLOT_B_ADDR, (const uint8_t *)hdr, 32UL) != HAL_OK)
    {
        return 0U;
    }
    /*回读五项逐字段比对，而不是只比 CRC：
     *SPI 在供电抖动时可能"写进去了但内容变了"，只算 CRC 也有可能巧合相等；
     *逐字段比对能定位到具体哪个字段坏，日志上更容易判断是写保护还是时序问题*/
    if (W25Q128_ReadData(W25Q128_FW_SLOT_B_ADDR, (uint8_t *)rd, 32UL) != HAL_OK)
    {
        return 0U;
    }
    if ((rd[0] != W25Q128_FW_MAGIC) || (rd[1] != OTA_REC_FMT) ||
        (rd[2] != (uint32_t)ver) || (rd[3] != size) || (rd[4] != crc))
    {
        return 0U;
    }
    if ((OtaCrc32(0xFFFFFFFFUL, (const uint8_t *)rd, 20UL) ^ 0xFFFFFFFFUL) != rd[5])
    {
        return 0U;
    }
    return 1U;
}
