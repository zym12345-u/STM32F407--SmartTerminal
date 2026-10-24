/**
  ******************************************************************************
  * @file    bsp_lfs_port.c
  * @brief   LittleFS 块设备适配层(W25Q128 -> lfs_config 的四个回调)
  *
  *          实现要点:
  *            1. 把 read/prog/erase/sync 四个回调接到 W25Q128 驱动上,
  *               擦/读/写都走同一套底层接口, 换 Flash 只改这一层
  *            2. prog 必须按 256B 页边界切开(W25Q128 不支持跨页编程)
  *            3. 所有缓冲区都放 static 全局(lfs_config.read_buffer/prog_buffer/
  *               lookahead_buffer + 文件 buffer), 不走 FreeRTOS 堆,
  *               这样 boot 阶段仅 1KB 的 MSP 栈也够用
  *            4. 挂载前先探测"Flash 是否全 0xFF(即从未写过)"; 只有确认是空白片
  *               才 format, 否则瞬时 IO 错误会把已有数据格式化掉
  ******************************************************************************
  */
#include "bsp_lfs_port.h"
#include "bsp_W25Q128.h"
#include "usart.h"

#include <string.h>

/* ---- 地址与容量 ---- */
#define LFS_FLASH_BASE   0x00000000UL
/* 2559 块: 0x00000000 ~ 0x009FEFFF, 末块 0x009FF000 起留给 boot 自测扇区,
 * 不越过 0x00A00000 的 GBK 字库区(驱动层写保护) */
#define LFS_BLOCK_COUNT  ((lfs_size_t)((W25Q128_GBK_BASE - W25Q128_SECTOR_SIZE) / W25Q128_SECTOR_SIZE))

/* ---- 缓冲区(全部静态, 共 512*3 + 512 = 2KB .bss) ---- */
#define LFS_CACHE_SIZE       512U
#define LFS_LOOKAHEAD_SIZE   512U   /* 1 bit/块, 512B 可覆盖 4096 块 > 2559 */

static uint8_t s_readBuf[LFS_CACHE_SIZE];
static uint8_t s_progBuf[LFS_CACHE_SIZE];
static uint8_t s_lookBuf[LFS_LOOKAHEAD_SIZE];
static uint8_t s_fileBuf[LFS_CACHE_SIZE];

/* ---- 块设备回调实现 ---- */
static int lfs_bd_read(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t addr = LFS_FLASH_BASE + (uint32_t)block * c->block_size + (uint32_t)off;

    if (W25Q128_ReadData(addr, (uint8_t *)buffer, (uint32_t)size) != HAL_OK)
    {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

static int lfs_bd_prog(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, const void *buffer, lfs_size_t size)
{
    const uint8_t *p   = (const uint8_t *)buffer;
    uint32_t       addr = LFS_FLASH_BASE + (uint32_t)block * c->block_size + (uint32_t)off;
    uint32_t       left = (uint32_t)size;
    uint16_t       chunk;
    uint16_t       pageLeft;

    /* W25Q128 限制: 单次页编程 <=256B 且不跨页, 这里按页边界切开 */
    while (left > 0U)
    {
        chunk    = (uint16_t)((left > W25Q128_PAGE_SIZE) ? W25Q128_PAGE_SIZE : left);
        pageLeft = (uint16_t)(W25Q128_PAGE_SIZE - (addr % W25Q128_PAGE_SIZE));
        if (chunk > pageLeft)
        {
            chunk = pageLeft;
        }

        if (W25Q128_PageProgram(addr, p, chunk) != HAL_OK)
        {
            return LFS_ERR_IO;
        }

        addr += chunk;
        p    += chunk;
        left -= chunk;
    }
    return LFS_ERR_OK;
}

static int lfs_bd_erase(const struct lfs_config *c, lfs_block_t block)
{
    uint32_t addr = LFS_FLASH_BASE + (uint32_t)block * c->block_size;

    /* 写保护: 禁止擦到 GBK 字库区(0x00A00000 起), 越界直接返回 IO 错误 */
    if (addr >= (W25Q128_GBK_BASE - W25Q128_SECTOR_SIZE))
    {
        return LFS_ERR_IO;
    }
    if (W25Q128_SectorErase4K(addr) != HAL_OK)
    {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

static int lfs_bd_sync(const struct lfs_config *c)
{
    (void)c;
    return LFS_ERR_OK;      /* SPI NOR 没有写缓存, 不需要 flush 操作 */
}

/* ---- 配置 ---- */
static const struct lfs_config s_cfg = {
    .context = NULL,
    .read    = lfs_bd_read,
    .prog    = lfs_bd_prog,
    .erase   = lfs_bd_erase,
    .sync    = lfs_bd_sync,

    .read_size      = W25Q128_PAGE_SIZE,      /* 256B, 与页大小一致 */
    .prog_size      = W25Q128_PAGE_SIZE,      /* 256B */
    .block_size     = W25Q128_SECTOR_SIZE,    /* 4096B */
    .block_count    = LFS_BLOCK_COUNT,        /* 2559 */
    .block_cycles   = 500,                    /* 每块约 500 次擦写后触发磨损均衡搬迁 */
    .cache_size     = LFS_CACHE_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,

    .read_buffer      = s_readBuf,
    .prog_buffer      = s_progBuf,
    .lookahead_buffer = s_lookBuf,
};

static lfs_t   s_lfs;
static uint8_t s_mounted = 0U;

lfs_t *LFS_Port_Handle(void)
{
    return &s_lfs;
}

uint32_t LFS_Port_UsedBlocks(void)
{
    lfs_ssize_t used;

    if (s_mounted == 0U)
    {
        return 0xFFFFFFFFUL;
    }
    used = lfs_fs_size(&s_lfs);
    return (used < 0) ? 0xFFFFFFFFUL : (uint32_t)used;
}

/* ---- M3 冒烟测试: 写 /m3.txt, 回读校验内容是否一致 ---- */
static void LFS_Port_SmokeTest(void)
{
    static lfs_file_t            s_file;
    static struct lfs_file_config s_fcfg;   /* 放 static: boot 阶段栈仅 1KB */
    char   rd[32];
    int    err;
    lfs_ssize_t n;

    memset(&s_fcfg, 0, sizeof(s_fcfg));
    s_fcfg.buffer = s_fileBuf;

    /* 1. 先尝试只读打开; 文件不存在则创建并写入 */
    err = lfs_file_opencfg(&s_lfs, &s_file, "/m3.txt", LFS_O_RDONLY, &s_fcfg);
    if (err == LFS_ERR_NOENT)
    {
        err = lfs_file_opencfg(&s_lfs, &s_file, "/m3.txt",
                               LFS_O_WRONLY | LFS_O_CREAT, &s_fcfg);
        if (err != LFS_ERR_OK)
        {
            UART_Printf("[LFS] create /m3.txt FAIL err=%d\r\n", err);
            return;
        }
        n = lfs_file_write(&s_lfs, &s_file, "M3 mount ok", 11);
        (void)lfs_file_close(&s_lfs, &s_file);
        UART_Printf("[LFS] created /m3.txt, wrote %ld B\r\n", (long)n);

        err = lfs_file_opencfg(&s_lfs, &s_file, "/m3.txt", LFS_O_RDONLY, &s_fcfg);
        if (err != LFS_ERR_OK)
        {
            UART_Printf("[LFS] reopen FAIL err=%d\r\n", err);
            return;
        }
    }
    else if (err != LFS_ERR_OK)
    {
        UART_Printf("[LFS] open /m3.txt FAIL err=%d\r\n", err);
        return;
    }

    /* 2. 回读校验 */
    memset(rd, 0, sizeof(rd));
    n = lfs_file_read(&s_lfs, &s_file, rd, 11);
    (void)lfs_file_close(&s_lfs, &s_file);

    if (n == 11 && memcmp(rd, "M3 mount ok", 11) == 0)
    {
        UART_Printf("[LFS] M3 PASS: readback \"%s\"\r\n", rd);
    }
    else
    {
        UART_Printf("[LFS] M3 FAIL: read %ld B \"%s\"\r\n", (long)n, rd);
    }
}

uint8_t LFS_Port_Mount(void)
{
    uint8_t probe[16];
    uint8_t blank = 1U;
    uint8_t i;
    int     err;

    if (s_mounted != 0U)
    {
        return 1U;
    }

    /* 1. 探测块 0 是否空白: 全 0xFF 说明从未写过 */
    (void)W25Q128_ReadData(LFS_FLASH_BASE, probe, (uint32_t)sizeof(probe));
    for (i = 0U; i < (uint8_t)sizeof(probe); i++)
    {
        if (probe[i] != 0xFFU)
        {
            blank = 0U;
            break;
        }
    }

    if (blank != 0U)
    {
        err = lfs_format(&s_lfs, &s_cfg);
        UART_Printf("[LFS] blank flash -> format: %s (err=%d)\r\n",
                    (err == LFS_ERR_OK) ? "OK" : "FAIL", err);
        if (err != LFS_ERR_OK)
        {
            return 0U;
        }
    }

    /* 2. 挂载 */
    err = lfs_mount(&s_lfs, &s_cfg);
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[LFS] mount FAIL err=%d (not formatted or corrupted)\r\n", err);
        return 0U;
    }
    s_mounted = 1U;

    UART_Printf("[LFS] mount OK: %lu blocks x %lu B (~%lu KB), used %lu blocks\r\n",
                (unsigned long)s_cfg.block_count,
                (unsigned long)s_cfg.block_size,
                (unsigned long)((uint32_t)s_cfg.block_count * s_cfg.block_size / 1024U),
                (unsigned long)LFS_Port_UsedBlocks());

    /* 3. M3 冒烟测试 */
    LFS_Port_SmokeTest();
    return 1U;
}
