/* bsp_W25Q128.h
 * W25Q128 驱动接口与几何参数：容量 16MB，擦除单位 4KB，页编程上限 256B。
 * 硬件：SPI1 = PA5/PA6/PA7，AF5；CS = PC13 软件 NSS。
 * 注意：写/擦在 >= 0x00A00000 的字库区一律拒绝；页编程不得跨页。
 */
#ifndef __BSP_W25Q128_H
#define __BSP_W25Q128_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/*几何参数：128Mbit = 16MB*/
#define W25Q128_TOTAL_BYTES     0x01000000UL   /* 16MB */
#define W25Q128_SECTOR_SIZE     4096U          /* 4KB 扇区(0x20 擦除粒度) */
#define W25Q128_PAGE_SIZE       256U           /* 页编程单次上限, 不可跨页 */
#define W25Q128_SECTOR_COUNT    (W25Q128_TOTAL_BYTES / W25Q128_SECTOR_SIZE)  /* 4096 */

/*期望 ID：与实读值交叉验证*/
#define W25Q128_JEDEC_ID_EXPECT    0x00EF4018UL  /* 0x9F: EF=Winbond 40=类型 18=2^24B */
#define W25Q128_DEVICE_ID_EXPECT   0xEF17U       /* 0x90: 厂商例程宏 W25Q128=0xEF17 */

/*GBK 字库区：后 6MB，验证前一律写保护*/
#define W25Q128_GBK_BASE        0x00A00000UL   /* 字库起始(后 6MB), 之前 10MB 为用户区 */

/**
  * 函    数：W25Q128_Init
  * 参    数：无
  * 返 回 值：1 = 初始化且 ID 校验通过，0 = 失败
  * 注意事项：失败时会打印实读字节
  */
uint8_t W25Q128_Init(void);

/**
  * 函    数：W25Q128_ReadJedecId
  * 参    数：无
  * 返 回 值：0x9F 读回的 3 字节，0x00EF4018 形式
  */
uint32_t W25Q128_ReadJedecId(void);

/**
  * 函    数：W25Q128_ReadDeviceId
  * 参    数：无
  * 返 回 值：0x90 读回的 2 字节，0xEF17 形式
  */
uint16_t W25Q128_ReadDeviceId(void);

/**
  * 函    数：W25Q128_ReadData
  * 参    数：addr  起始字节地址，24 位
  * 参    数：pData  输出缓冲
  * 参    数：num  读取字节数
  * 返 回 值：HAL_OK = 读完，HAL_ERROR = 未初始化
  */
HAL_StatusTypeDef W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint32_t num);

/**
  * 函    数：W25Q128_SectorErase4K
  * 参    数：addr  字节地址，按 A23~A12 对齐到所在扇区
  * 返 回 值：HAL_OK = 擦完，HAL_TIMEOUT = 超时，HAL_ERROR = 未初始化或字库区
  * 注意事项：含写使能与 BUSY 轮询，实测耗时存于 W25Q128_GetLastBusyMs
  */
HAL_StatusTypeDef W25Q128_SectorErase4K(uint32_t addr);

/**
  * 函    数：W25Q128_PageProgram
  * 参    数：addr  起始地址，必须整段在同一 256B 页内
  * 参    数：pData  数据首地址
  * 参    数：len  字节数，范围 1~256
  * 返 回 值：HAL_OK = 完成，HAL_ERROR = 越界或未初始化
  */
HAL_StatusTypeDef W25Q128_PageProgram(uint32_t addr, const uint8_t *pData, uint16_t len);

/**
  * 函    数：W25Q128_ReadStatus1
  * 参    数：无
  * 返 回 值：状态字节，bit0 = BUSY，bit1 = WEL
  */
uint8_t W25Q128_ReadStatus1(void);

/**
  * 函    数：W25Q128_WaitReady
  * 参    数：timeoutMs  最长等待，单位 ms
  * 返 回 值：HAL_OK = BUSY 已清零，HAL_TIMEOUT = 超时仍未空闲
  * 注意事项：入口与循环内各补报一次 WDT_TASK_STORAGE 心跳，否则长擦除会喂不上狗
  */
HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs);

/**
  * 函    数：W25Q128_GetLastBusyMs
  * 参    数：无
  * 返 回 值：最近一次 WaitReady 实测等待时长，单位 ms
  */
uint32_t W25Q128_GetLastBusyMs(void);

/**
  * 函    数：W25Q128_SelfTest
  * 参    数：无
  * 返 回 值：无
  * 注意事项：仅 W25Q128_Init 成功后调用；测试扇区取用户区尾部，避开 GBK 字库区
  */
void W25Q128_SelfTest(void);

/**
  * 函    数：W25Q128_CheckGBKFont
  * 参    数：无
  * 返 回 值：1 = 字库区结构合理，0 = 未写入或结构异常
  * 注意事项：只做存在性检查，不解析字库格式；首字节非全 0xFF 且 6MB 采样 0xFF 占比 < 80%
  */
uint8_t W25Q128_CheckGBKFont(void);

#endif /* __BSP_W25Q128_H */
