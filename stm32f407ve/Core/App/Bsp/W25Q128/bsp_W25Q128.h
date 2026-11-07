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

/*固件槽：LittleFS 缩到 8MB 后，从 0x00800000 起划两个 448KB 槽（A/B），字库区不动
 * 每槽首 4KB 放镜像记录，镜像从槽基址 +0x1000 开始，最大 0x6F000(444KB)
 * 记录 32B：magic('FWSL') / 格式(1=槽A整区备份 2=槽B OTA镜像) / 版本 / 镜像字节数 / 镜像 CRC32 / 记录自身 CRC32
 * 槽 A 存备份镜像，槽 B 留给 OTA 新镜像
 * 【槽 B 必须从 0x00871000 起，不能是 0x00870000】槽 A 是 448KB 整区备份，
 * 镜像从 0x00801000 一直写到 0x00870FFF；若 B 从 0x00870000 起，B 的记录扇区
 * 正好压在 A 备份的最后 4KB 上，写一次 OTA 记录就会让 A 的整区 CRC 失效。*/
#define W25Q128_FW_SLOT_A_ADDR  0x00800000UL
#define W25Q128_FW_SLOT_B_ADDR  0x00871000UL
#define W25Q128_FW_SLOT_SIZE    0x00070000UL   /* 448KB, 与内部 App 区同大小 */
#define W25Q128_FW_HDR_SIZE     0x00001000UL   /* 每槽首 4KB 记录扇区 */
#define W25Q128_FW_IMG_MAX      (W25Q128_FW_SLOT_SIZE - W25Q128_FW_HDR_SIZE)
#define W25Q128_FW_MAGIC        0x4657534CUL   /*'F''W''S''L'：槽内记录有效*/

/**
  * 功    能：初始化 W25Q128：配 SPI 并读 JEDEC ID 确认芯片在位
  * 参    数：无
  * 返 回 值：1 = 初始化且 ID 校验通过，0 = 失败
  * 补    充：失败时会打印实读字节
  */
uint8_t W25Q128_Init(void);

/**
  * 功    能：读 JEDEC ID（命令 0x9F）。正常应返回 0x00EF4018：EF 是 Winbond，40 表示 128Mb
  * 参    数：无
  * 返 回 值：0x9F 读回的 3 字节，0x00EF4018 形式
  */
uint32_t W25Q128_ReadJedecId(void);

/**
  * 功    能：读 Device ID（命令 0x90）。正常应返回 0xEF17
  * 参    数：无
  * 返 回 值：0x90 读回的 2 字节，0xEF17 形式
  */
uint16_t W25Q128_ReadDeviceId(void);

/**
  * 功    能：从指定地址读一段数据。芯片内部地址会自动增长，可以一口气读完
  * 参    数：addr  起始字节地址，24 位
  *            pData  输出缓冲
  *            num  读取字节数
  * 返 回 值：HAL_OK = 读完，HAL_ERROR = 未初始化
  * 补    充：addr+num 不能越过 16MB 边界，这点由调用方保证
  */
HAL_StatusTypeDef W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint32_t num);

/**
  * 功    能：擦掉 addr 所在的 4KB 扇区（按 A23~A12 对齐到扇区起点）
  * 参    数：addr  字节地址，按 A23~A12 对齐到所在扇区
  * 返 回 值：HAL_OK = 擦完，HAL_TIMEOUT = 超时，HAL_ERROR = 未初始化或字库区
  * 补    充：含写使能与 BUSY 轮询，实测耗时存于 W25Q128_GetLastBusyMs
  */
HAL_StatusTypeDef W25Q128_SectorErase4K(uint32_t addr);

/**
  * 功    能：往一页里写数据（一页 256 字节）
  * 参    数：addr  起始地址，必须整段在同一 256B 页内
  *            pData  数据首地址
  *            len  字节数，范围 1~256
  * 返 回 值：HAL_OK = 完成，HAL_ERROR = 越界或未初始化
  * 补    充：【整段必须落在同一页内】—— 跨页会绕回页首覆盖已写内容，这是 Flash 的硬件行为
  */
HAL_StatusTypeDef W25Q128_PageProgram(uint32_t addr, const uint8_t *pData, uint16_t len);

/**
  * 功    能：读状态寄存器 1：bit0 = BUSY，bit1 = 写使能 WEL。本函数只是读，不等待
  * 参    数：无
  * 返 回 值：状态字节，bit0 = BUSY，bit1 = WEL
  */
uint8_t W25Q128_ReadStatus1(void);

/**
  * 功    能：轮询等芯片空闲（BUSY 清零）。擦除给 400ms，编程给 10ms
  * 参    数：timeoutMs  最长等待，单位 ms
  * 返 回 值：HAL_OK = BUSY 已清零，HAL_TIMEOUT = 超时仍未空闲
  * 补    充：入口与循环内各补报一次 WDT_TASK_STORAGE 心跳，否则长擦除会喂不上狗
  *            等待期间会持续补报 STORAGE 心跳，否则长擦除会把看门狗饿死
  */
HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs);

/**
  * 功    能：取最近一次等待实际花了多少毫秒，用来判断芯片是不是越擦越慢
  * 参    数：无
  * 返 回 值：最近一次 WaitReady 实测等待时长，单位 ms
  */
uint32_t W25Q128_GetLastBusyMs(void);

/**
  * 功    能：自检：在用户区尾部找一块扇区做「擦 → 写图案 → 读回比对」
  * 参    数：无
  * 返 回 值：无
  * 补    充：仅 W25Q128_Init 成功后调用；测试扇区取用户区尾部，避开 GBK 字库区
  *            刻意避开字库区，不会破坏字库。只能在 Init 成功之后调
  */
void W25Q128_SelfTest(void);

/**
  * 功    能：查字库有没有烧进去：看首字节、分区边界、6MB 范围内几处采样点的 0xFF 占比
  * 参    数：无
  * 返 回 值：1 = 字库区结构合理，0 = 未写入或结构异常
  * 补    充：只做存在性检查，不解析字库格式；首字节非全 0xFF 且 6MB 采样 0xFF 占比 < 80%
  */
uint8_t W25Q128_CheckGBKFont(void);

/**
  * 功    能：擦除一段连续扇区（起始 4KB 对齐、长度 4KB 整数倍）
  * 参    数：addr  起始地址；size  字节数
  * 返 回 值：HAL_OK = 全部擦完；HAL_TIMEOUT = 某个扇区超时；HAL_ERROR = 未初始化/不对齐/越界
  * 补    充：逐扇区调 SectorErase4K，中途失败立即返回，不继续往后擦
  */
HAL_StatusTypeDef W25Q128_EraseRange(uint32_t addr, uint32_t size);

/**
  * 功    能：从 addr 起连续写入一段数据，内部按 256B 页边界切片
  * 参    数：addr  起始地址，可非页对齐；pData  数据首地址；size  字节数
  * 返 回 值：HAL_OK = 写完；HAL_ERROR = 未初始化/参数为空/越过字库区/底层写失败
  * 补    充：写入区必须先擦过；跨页时自动切开，避免页内回绕覆盖页首
  */
HAL_StatusTypeDef W25Q128_WriteRange(uint32_t addr, const uint8_t *pData, uint32_t size);

/**
  * 功    能：对一段区域做 CRC32（逐位，多项式 0xEDB88320，初值/终值 0xFFFFFFFF）
  * 参    数：addr  起始地址；size  字节数
  * 返 回 值：CRC32；未初始化或越界返回 0
  * 补    充：static 256B 缓冲分块读，不占栈；与 BootLoader/iap_upgrade.py 算法一致
  */
uint32_t W25Q128_Crc32Range(uint32_t addr, uint32_t size);

#endif /* __BSP_W25Q128_H */
