/**
  ******************************************************************************
  * @file    bsp_W25Q128.h
  * @brief   W25Q128 串行 NOR Flash(16MB, SPI1)驱动 —— V1.2 存储阶段
  *
  *          硬件连接(厂商例程 20-1 定论, 与本工程现有占用零冲突):
  *            SCK=PA5 / MISO=PA6 / MOSI=PA7 (AF5), CS=PC13 (GPIO 输出)
  *
  *          与厂商例程的差异(巡检草案第 7 节"不可照抄"清单):
  *            1. JEDEC ID 用 0x9F 命令读 3 字节(0xEF4018),
  *               厂商 readID 的 0x90 只能读到 0xEF17 —— 两者都读, 交叉验证
  *            2. WaitReady 带超时 + 心跳上报(厂商是空 while, 会死循环咬狗)
  *            3. 统一走 HAL_SPI(厂商直写 SPI1->CR1, 与 HAL 初始化会打架)
  *            4. 删除 4KB 全局缓存(当前 RAM 预算承受不起, LittleFS 自带 cache)
  *            5. 里程碑阶段速率 10.5MHz(APB2=84MHz, 预分频 8), 链路通后再提
  *
  *          心跳约定: 等待循环里上报 WDT_TASK_STORAGE 槽位。
  *            本驱动只允许在两类上下文调用:
  *            a) 调度器启动前的 boot 阶段(IWDG 尚未启动, 心跳写入 0 无副作用)
  *            b) 将来的 StorageTask(单任务串行访问 SPI1, 与 LVGL 单线程风格一致)
  *
  *          字库写保护: 板载 Flash 尾部 0x00A00000 起疑似烧有 GBK 字库(M2 验证)。
  *            在 M2 证实前, 本驱动对 >= W25Q128_GBK_BASE 的写/擦一律拒绝。
  ******************************************************************************
  */
#ifndef __BSP_W25Q128_H
#define __BSP_W25Q128_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ---- 几何参数(W25Q128: 128Mbit = 16MB) ---- */
#define W25Q128_TOTAL_BYTES     0x01000000UL   /* 16MB */
#define W25Q128_SECTOR_SIZE     4096U          /* 4KB 扇区(0x20 擦除粒度) */
#define W25Q128_PAGE_SIZE       256U           /* 页编程单次上限, 不可跨页 */
#define W25Q128_SECTOR_COUNT    (W25Q128_TOTAL_BYTES / W25Q128_SECTOR_SIZE)  /* 4096 */

/* ---- 期望值(交叉验证用) ---- */
#define W25Q128_JEDEC_ID_EXPECT    0x00EF4018UL  /* 0x9F: EF=Winbond 40=类型 18=2^24B */
#define W25Q128_DEVICE_ID_EXPECT   0xEF17U       /* 0x90: 厂商例程宏 W25Q128=0xEF17 */

/* ---- GBK 字库区(厂商约定, M2 待验证; 验证前一律写保护) ---- */
#define W25Q128_GBK_BASE        0x00A00000UL   /* 字库起始(后 6MB), 之前 10MB 为用户区 */

/** 初始化 GPIO+SPI1 并读 ID 交叉验证; 通过返回 1, 失败返回 0(并打印原始字节) */
uint8_t W25Q128_Init(void);

/** 0x9F 读 JEDEC ID(3 字节, 返回 0x00EF4018 形式) */
uint32_t W25Q128_ReadJedecId(void);

/** 0x90 读厂商/设备 ID(2 字节, 返回 0xEF17 形式) */
uint16_t W25Q128_ReadDeviceId(void);

/** 0x03 读数据(任意地址/长度) */
HAL_StatusTypeDef W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint32_t num);

/** 0x20 扇区擦除 4KB; addr 为字节地址(芯片按 A23-A12 对齐到所在扇区)。
 *  含写使能 + BUSY 轮询(带超时), 实测耗时存于 W25Q128_GetLastBusyMs() */
HAL_StatusTypeDef W25Q128_SectorErase4K(uint32_t addr);

/** 0x02 页编程: len <= 256 且不得跨 256B 页边界(越界返回 HAL_ERROR 不动作) */
HAL_StatusTypeDef W25Q128_PageProgram(uint32_t addr, const uint8_t *pData, uint16_t len);

/** 读状态寄存器 1(0x05), bit0=BUSY */
uint8_t W25Q128_ReadStatus1(void);

/** BUSY 轮询: 入口必报心跳 + 循环内节流补报 + 超时(巡检草案 4.2 三要素) */
HAL_StatusTypeDef W25Q128_WaitReady(uint32_t timeoutMs);

/** 最近一次 WaitReady 实测等待时长(ms), 用于回填时序表(替代缺失的手册值) */
uint32_t W25Q128_GetLastBusyMs(void);

/** M1.5 自测: 用户区尾部扇区做 擦除->页编程->回读校验, 打印实测耗时。
 *  仅 Init 成功后调用; 测试地址避开 GBK 字库区。 */
void W25Q128_SelfTest(void);

#endif /* __BSP_W25Q128_H */
