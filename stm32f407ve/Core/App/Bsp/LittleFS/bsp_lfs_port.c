/**
  ******************************************************************************
  * @file    bsp_lfs_port.c
  * @brief   LittleFS ???υτ?????(W25Q128 -> lfs_config ?????)
  *
  *          ??????:
  *            1. ????? read/prog/erase/sync ?????????? W25Q128 ????,
  *               ???/????/???§Υ????????????????, ??????????
  *            2. prog ???? 256B ??????????(W25Q128 ????????)
  *            3. ????????? static ??(lfs_config.read_buffer/prog_buffer/
  *               lookahead_buffer + ??? buffer), ?????? FreeRTOS ??,
  *               ???? boot ??¦Ν? 1KB ?? MSP ?
  *            4. ????????"Flash ? 0xFF(??¦Δ???)"???; ?????????????,
  *               ??????? format ???? LittleFS ???????????, ?????????????????
  ******************************************************************************
  */
#include "bsp_lfs_port.h"
#include "bsp_W25Q128.h"
#include "usart.h"

#include <string.h>

/* ---- ???? ---- */
#define LFS_FLASH_BASE   0x00000000UL
/* 2559 ??: 0x00000000 ~ 0x009FEFFF, ?? 0x009FF000 ??????????????,
 * ??????? boot ?? SelfTest ???????????????? */
#define LFS_BLOCK_COUNT  ((lfs_size_t)((W25Q128_GBK_BASE - W25Q128_SECTOR_SIZE) / W25Q128_SECTOR_SIZE))

/* ---- ????(??????, ?? 512*3 + 512 = 2KB .bss) ---- */
#define LFS_CACHE_SIZE       512U
#define LFS_LOOKAHEAD_SIZE   512U   /* 1 bit/??, 512B ????? 4096 ?? > 2559 */

static uint8_t s_readBuf[LFS_CACHE_SIZE];
static uint8_t s_progBuf[LFS_CACHE_SIZE];
static uint8_t s_lookBuf[LFS_LOOKAHEAD_SIZE];
static uint8_t s_fileBuf[LFS_CACHE_SIZE];

/* ---- ???υτ????? ---- */
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

    /* W25Q128 ????: ???? <=256B ???????, ???????§Ω? */
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

    /* ?????: ??????????(???/???????)????? */
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
    return LFS_ERR_OK;      /* SPI NOR §Υ?????, ????? flush ???? */
}

/* ---- ???? ---- */
static const struct lfs_config s_cfg = {
    .context = NULL,
    .read    = lfs_bd_read,
    .prog    = lfs_bd_prog,
    .erase   = lfs_bd_erase,
    .sync    = lfs_bd_sync,

    .read_size      = W25Q128_PAGE_SIZE,      /* 256B, ?????????? */
    .prog_size      = W25Q128_PAGE_SIZE,      /* 256B */
    .block_size     = W25Q128_SECTOR_SIZE,    /* 4096B */
    .block_count    = LFS_BLOCK_COUNT,        /* 2559 */
    .block_cycles   = 500,                    /* ??? 500 ?¦Β?§Υ??????????, ????????????? */
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

/* ---- M3 ??????: §Υ??? /m3.txt, ?????¦Ο?????????, ?????????????? ---- */
static void LFS_Port_SmokeTest(void)
{
    static lfs_file_t            s_file;
    static struct lfs_file_config s_fcfg;   /* ???? static: boot ???? 1KB */
    char   rd[32];
    int    err;
    lfs_ssize_t n;

    memset(&s_fcfg, 0, sizeof(s_fcfg));
    s_fcfg.buffer = s_fileBuf;

    /* 1. ??????????; ??????????????, ??????§Υ?? */
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

    /* 2. ?????? */
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

    /* 1. ??? 0 ??????: ? 0xFF ?????¦Δ????? */
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

    /* 2. ???? */
    err = lfs_mount(&s_lfs, &s_cfg);
    if (err != LFS_ERR_OK)
    {
        UART_Printf("[LFS] mount FAIL err=%d (??????????, ????????)\r\n", err);
        return 0U;
    }
    s_mounted = 1U;

    UART_Printf("[LFS] mount OK: %lu blocks x %lu B (~%lu KB), used %lu blocks\r\n",
                (unsigned long)s_cfg.block_count,
                (unsigned long)s_cfg.block_size,
                (unsigned long)((uint32_t)s_cfg.block_count * s_cfg.block_size / 1024U),
                (unsigned long)LFS_Port_UsedBlocks());

    /* 3. M3 ?????? */
    LFS_Port_SmokeTest();
    return 1U;
}
