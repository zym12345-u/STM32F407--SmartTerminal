/**
  ******************************************************************************
  * @file    bsp_WifiStore.c
  * @brief   WiFi 凭据 Flash 存储 —— 实现
  ******************************************************************************
  */
#include "bsp_WifiStore.h"

#include "stm32f4xx_hal.h"
#include "bsp_IWDG.h"
#include <string.h>


/* 结构体原始字节(不含 checksum 字段)的 32 位加和,用于识别擦除态/损坏数据 */
static uint32_t Wifi_CalcChecksum(const WifiCred_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint32_t sum = 0U;
    uint32_t n;
    /* checksum 是最后一个字段,只校验它之前的所有字节 */
    uint32_t bodyLen = (uint32_t)((const uint8_t *)&c->checksum - p);

    for (n = 0U; n < bodyLen; n++)
    {
        sum += (uint32_t)p[n];
    }
    return sum;
}

uint8_t WifiStore_Load(WifiCred_t *out)
{
    const WifiCred_t *stored = (const WifiCred_t *)WIFI_FLASH_BASE;

    memset(out, 0, sizeof(*out));

    if (stored->magic != WIFI_FLASH_MAGIC)
    {
        return 0U;                       /* 擦除态(0xFF...)或没写过 */
    }
    memcpy(out, stored, sizeof(*out));
    if (out->checksum != Wifi_CalcChecksum(out))
    {
        memset(out, 0, sizeof(*out));
        return 0U;
    }
    return 1U;
}

uint8_t WifiStore_Save(const WifiCred_t *in)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t               err = 0U;
    HAL_StatusTypeDef      halSt;
    WifiCred_t             buf;
    const uint32_t        *src;
    volatile uint32_t     *dst;
    uint32_t               words;
    uint32_t               i;

    /* 组帧到 4 字节对齐缓冲:Flash 按 32 位字编程,结构体长度向上取整 */
    memset(&buf, 0, sizeof(buf));
    buf.magic = WIFI_FLASH_MAGIC;
    strncpy(buf.ssid, in->ssid, WIFI_SSID_LEN);
    strncpy(buf.pwd,  in->pwd,  WIFI_PWD_LEN);
    buf.checksum = Wifi_CalcChecksum(&buf);
    words = (uint32_t)((sizeof(buf) + 3U) / 4U);

    /* V1.1.1: 擦除 128KB 扇区期间 CPU 取指停顿(typ 1s/max 2s), 全任务冻结,
     * 1s 日常窗口必咬狗(真机已复现: 配网保存后 IWDG 复位)。
     * 保存前放宽到 4s, 保存结束(无论成败)恢复 1s */
    IWDG_SetReload(IWDG_RELOAD_FLASH_SAVE);
    HAL_FLASH_Unlock();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = FLASH_SECTOR_11;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7~3.6V */
    halSt = HAL_FLASHEx_Erase(&erase, &err);

    if (halSt == HAL_OK)
    {
        src = (const uint32_t *)&buf;
        dst = (volatile uint32_t *)WIFI_FLASH_BASE;
        for (i = 0U; i < words; i++)
        {
            halSt = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                      (uint32_t)(dst + i), src[i]);
            if (halSt != HAL_OK)
            {
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    IWDG_SetReload(IWDG_RELOAD_NORMAL);     /* 恢复 1s 日常窗口 */

    if (halSt != HAL_OK)
    {
        return 0U;
    }

    /* 回读校验,确保真的写进去了 */
    if (memcmp((const void *)WIFI_FLASH_BASE, &buf, sizeof(buf)) != 0)
    {
        return 0U;
    }
    return 1U;
}
