/* bsp_bootflag.c
 * OTA-2 启动标志：24C02 页 0xE8，8 字节一页。
 * 常量必须与 BootLoader/port/bsp_ee.h 的 EE_OTA2_* 保持同值。
 */

#include "bsp_bootflag.h"
#include "bsp_24C02.h"

/**
  * 功    能：把整页写成 magic + state + count
  * 参    数：state 状态；count 未确认次数
  * 返 回 值：0 = 成功；1 = EEPROM 写失败
  */
static uint8_t BootFlag_Write(uint8_t state, uint8_t count)
{
  uint8_t buf[8] = {0U};

  buf[0] = (uint8_t)(BOOTFLAG_MAGIC);
  buf[1] = (uint8_t)(BOOTFLAG_MAGIC >> 8U);
  buf[2] = (uint8_t)(BOOTFLAG_MAGIC >> 16U);
  buf[3] = (uint8_t)(BOOTFLAG_MAGIC >> 24U);
  buf[4] = state;
  buf[5] = count;

  return (AT24C02_WriteBytes(BOOTFLAG_MEM, buf, 8U) == 0U) ? 0U : 1U;
}

uint8_t BootFlag_RequestApply(void)
{
  return BootFlag_Write(BOOTFLAG_ST_APPLY, 0U);
}

uint8_t BootFlag_ConfirmBoot(void)
{
  uint8_t buf[8];
  uint32_t magic;

  if (AT24C02_ReadBytes(BOOTFLAG_MEM, buf, 8U) != 0U)
  {
    return 0U;
  }

  magic = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8U) |
          ((uint32_t)buf[2] << 16U) | ((uint32_t)buf[3] << 24U);
  if (magic != BOOTFLAG_MAGIC)
  {
    return 1U;   /*没有待确认状态：普通启动，不动 EEPROM*/
  }
  if (buf[4] != BOOTFLAG_ST_CONFIRM)
  {
    return 1U;   /*不是等待确认（例如 App 刚写了 APPLY 还没复位）*/
  }

  return (BootFlag_Write(0U, 0U) == 0U) ? 2U : 0U;
}

void BootFlag_Clear(void)
{
  uint8_t buf[8] = {0U};

  (void)AT24C02_WriteBytes(BOOTFLAG_MEM, buf, 8U);
}
