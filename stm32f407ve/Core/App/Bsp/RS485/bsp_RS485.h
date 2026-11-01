/* bsp_RS485.h
 * RS485 驱动接口：初始化、收发、缓冲清空、IDLE 中断与任务通知。
 * 硬件：USART3 = PB10(TX)/PB11(RX)，AF7；板载收发器硬件自动方向，无 DE/RE 引脚。
 * 注意：只允许 ModbusTask 单任务串行调用；RS485_SetNotifyTask 必须先于 RS485_EnableIdleIT。
 */
#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/*手动方向开关：本板硬件自动方向，默认关闭；若换批次后发送完总线被拉死，打开此宏并在 .c 补 DE 引脚*/
#define RS485_MANUAL_DE         0       /* 0 = 硬件自动方向(本板默认); 1 = 手动 DE，引脚见 .c */

#define RS485_BAUDRATE          9600U   /* 单位 bit/s，与常见 Modbus 从站出厂值一致，可按主站调整 */

/*Modbus 一帧最长 256 字节*/
#define RS485_RX_BUF_SIZE       256U    /* 单位 byte；实际可缓存 255，留 1 字节用于区分空/满 */


/*对外接口*/

/**
  * 函    数：RS485_Init
  * 参    数：无
  * 返 回 值：无
  * 注意事项：依赖 usart.c 中 MX_USART3_UART_Init 已完成首次初始化
  */
void RS485_Init(void);

/**
  * 函    数：RS485_RxCount
  * 参    数：无
  * 返 回 值：当前缓冲待读字节数，范围 0~255
  */
uint16_t RS485_RxCount(void);

/**
  * 函    数：RS485_ReadByte
  * 参    数：byte  输出参数，存放读出的字节
  * 返 回 值：1 = 读到，0 = 缓冲空
  */
uint8_t RS485_ReadByte(uint8_t *byte);

/**
  * 函    数：RS485_RxFlush
  * 参    数：无
  * 返 回 值：无
  * 注意事项：发送前/后调用，丢弃自发自收回环与历史噪声
  */
void RS485_RxFlush(void);

/**
  * 函    数：RS485_Send
  * 参    数：data  待发送数据
  * 参    数：len  字节数，范围 1~256
  * 返 回 值：无
  * 注意事项：仅任务上下文调用，内部处理方向与回环
  */
void RS485_Send(const uint8_t *data, uint16_t len);

/**
  * 函    数：RS485_RxByteISR
  * 参    数：byte  本次收到的字节
  * 返 回 值：无
  * 注意事项：仅供 usart.c 的 HAL_UART_RxCpltCallback 调用；ISR 上下文，不调 RTOS API
  */
void RS485_RxByteISR(uint8_t byte);

/*HAL 单字节接收缓冲，在 bsp_RS485.c 定义；回调必须读它*/
extern uint8_t s_rxByte;

/*IDLE 中断唤醒 ModbusTask，替代 10ms 轮询*/

/**
  * 函    数：RS485_SetNotifyTask
  * 参    数：task  目标任务句柄
  * 返 回 值：无
  * 注意事项：必须先于 RS485_EnableIdleIT 调用
  */
void RS485_SetNotifyTask(TaskHandle_t task);

/**
  * 函    数：RS485_EnableIdleIT
  * 参    数：无
  * 返 回 值：无
  * 注意事项：在 RS485_Init 与任务通知注册之后调用
  */
void RS485_EnableIdleIT(void);

/**
  * 函    数：RS485_IdleISR
  * 参    数：无
  * 返 回 值：无
  * 注意事项：ISR 上下文，在 stm32f4xx_it.c 的 USART3_IRQHandler 里调用
  */
void RS485_IdleISR(void);


#endif /* __BSP_RS485_H */
