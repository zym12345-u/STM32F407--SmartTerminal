/* bsp_BH1750.h
 * BH1750 地址、命令字、状态码与数据结构声明。
 * 硬件：I2C1 = PB6(SCL) / PB7(SDA)，7 位地址 0x23(ADDR 接 GND)。
 * 注意：当前只用上电 0x01、复位 0x07 与连续高分辨率 0x10 三条命令。
 */
#ifndef __BSP_BH1750_H
#define __BSP_BH1750_H

#include "stm32f4xx_hal.h"


/*I2C 地址*/
#define BH1750_ADDR_GND        0x23        /* ADDR 接 GND 时的 7 位地址 */
#define BH1750_ADDR_VCC        0x5C        /* ADDR 接 VCC 时的 7 位地址 */
#define BH1750_I2C_ADDR        (BH1750_ADDR_GND << 1)   /* 左移 1 位的 8 位地址(含 R/W 位) */


/*BH1750 命令字*/
#define BH1750_CMD_POWER_DOWN          0x00   /* 掉电 */
#define BH1750_CMD_POWER_ON            0x01   /* 上电 */
#define BH1750_CMD_RESET               0x07   /* 复位数据寄存器(仅在上电后有效) */
#define BH1750_CMD_CONT_H_MODE         0x10   /* 连续测量-高分辨率(1lux)    */
#define BH1750_CMD_CONT_H_MODE2        0x11   /* 连续测量-高分辨率2(0.5lux) */
#define BH1750_CMD_CONT_L_MODE         0x13   /* 连续测量-低分辨率(4lux)    */
#define BH1750_CMD_ONCE_H_MODE         0x20   /* 单次测量-高分辨率(1lux)    */
#define BH1750_CMD_ONCE_H_MODE2        0x21   /* 单次测量-高分辨率2(0.5lux) */
#define BH1750_CMD_ONCE_L_MODE         0x23   /* 单次测量-低分辨率(4lux)    */


/*容错参数*/
#define BH1750_RETRY_CNT           3        /* 初始化/读取失败时的重试次数 */


/*状态码*/
/* 0=成功; 1=通信超时/参数非法; 2=BH1750_TIMEOUT 保留, 当前实现不会返回 */
typedef enum
{
    BH1750_OK      = 0,
    BH1750_ERROR   = 1,
    BH1750_TIMEOUT = 2
} BH1750_Status_t;


/*数据结构*/
typedef struct
{
    float     lux;          /*光照强度，单位 lux，本驱动算/调用方读*/
    uint8_t   is_valid;     /*数据有效标志，0-无效 1-有效，本驱动写/调用方读*/
} BH1750_Data_t;


/*对外接口*/

/**
  * 函    数：BH1750_Init
  * 参    数：无
  * 返 回 值：BH1750_OK=成功，其它=失败(重试 3 次仍不通)
  */
BH1750_Status_t BH1750_Init(void);

/**
  * 函    数：BH1750_Read
  * 参    数：data  输出，lux 单位 lux(0~54612)，is_valid 由本函数置 1
  * 返 回 值：BH1750_OK=成功，其它=失败
  */
BH1750_Status_t BH1750_Read(BH1750_Data_t *data);


#endif /* __BSP_BH1750_H */
