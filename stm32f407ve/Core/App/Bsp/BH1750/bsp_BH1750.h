/**
  ******************************************************************************
  * @file    bsp_BH1750.h
  * @brief   BH1750 环境光强度传感器驱动(硬件 I2C1 接口)
  *
  *          硬件连接:
  *            VCC  -> 3.3V
  *            GND  -> GND
  *            SCL  -> PB6 (I2C1_SCL)
  *            SDA  -> PB7 (I2C1_SDA)
  *            ADDR -> GND  (I2C 地址 0x23; 接 VCC 时为 0x5C)
  *
  *          说明:
  *            1. 本驱动使用硬件 I2C1,引脚(PB6/PB7)在 BH1750_Init 内部完成
  *               GPIO 复用与 I2C 外设初始化,不依赖 CubeMX 生成的 MX_I2C1_Init。
  *            2. 测量精度: 高分辨率模式(0x10),分辨率 1 lux,转换时间 ≤120ms。
  *            3. 光照强度 lux = (DataH<<8 | DataL) / 1.2 。
  ******************************************************************************
  */
#ifndef __BSP_BH1750_H
#define __BSP_BH1750_H

#include "stm32f4xx_hal.h"


/* ====================== I2C 地址定义 ====================== */
#define BH1750_ADDR_GND        0x23        /* ADDR 接 GND 时的 7 位地址 */
#define BH1750_ADDR_VCC        0x5C        /* ADDR 接 VCC 时的 7 位地址 */
#define BH1750_I2C_ADDR        (BH1750_ADDR_GND << 1)   /* 左移 1 位的 8 位地址(含 R/W 位) */


/* ====================== BH1750 命令字 ====================== */
#define BH1750_CMD_POWER_DOWN          0x00   /* 掉电 */
#define BH1750_CMD_POWER_ON            0x01   /* 上电 */
#define BH1750_CMD_RESET               0x07   /* 复位数据寄存器(仅在上电后有效) */
#define BH1750_CMD_CONT_H_MODE         0x10   /* 连续测量-高分辨率(1lux)    */
#define BH1750_CMD_CONT_H_MODE2        0x11   /* 连续测量-高分辨率2(0.5lux) */
#define BH1750_CMD_CONT_L_MODE         0x13   /* 连续测量-低分辨率(4lux)    */
#define BH1750_CMD_ONCE_H_MODE         0x20   /* 单次测量-高分辨率(1lux)    */
#define BH1750_CMD_ONCE_H_MODE2        0x21   /* 单次测量-高分辨率2(0.5lux) */
#define BH1750_CMD_ONCE_L_MODE         0x23   /* 单次测量-低分辨率(4lux)    */


/* ====================== 容错参数 ====================== */
#define BH1750_RETRY_CNT           3        /* 初始化/读取失败时的重试次数 */


/* ====================== 状态码 ====================== */
typedef enum
{
    BH1750_OK      = 0,
    BH1750_ERROR   = 1,
    BH1750_TIMEOUT = 2
} BH1750_Status_t;


/* ====================== 数据结构 ====================== */
typedef struct
{
    float     lux;          /* 光照强度,单位 lux */
    uint8_t   is_valid;     /* 数据是否有效:0-无效 1-有效 */
} BH1750_Data_t;


/* ====================== 对外接口 ====================== */

/**
  * @brief  初始化 BH1750(含 I2C1 硬件初始化与连续高分辨率测量模式配置)
  * @retval BH1750_OK-成功;其它-失败
  */
BH1750_Status_t BH1750_Init(void);

/**
  * @brief  读取一次光照强度
  * @param  data  输出参数,存放光照数据
  * @retval BH1750_OK-成功;其它-失败
  */
BH1750_Status_t BH1750_Read(BH1750_Data_t *data);


#endif /* __BSP_BH1750_H */
