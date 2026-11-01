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
  * 功    能：初始化 RS485 收发（复用 USART3）。依赖 usart.c 里的 MX_USART3_UART_Init 先跑过一次；
  * 参    数：无
  * 返 回 值：无
  * 补    充：依赖 usart.c 中 MX_USART3_UART_Init 已完成首次初始化
  *            重复调用没问题，内部会把 MspInit 再走一遍
  */
void RS485_Init(void);

/**
  * 功    能：看接收缓冲里还有多少字节没读走（0~255）
  * 参    数：无
  * 返 回 值：当前缓冲待读字节数，范围 0~255
  */
uint16_t RS485_RxCount(void);

/**
  * 功    能：从接收缓冲取一个字节出来。返回 1 读到，0 缓冲已空（这时 *byte 不动）
  * 参    数：byte  输出参数，存放读出的字节
  * 返 回 值：1 = 读到，0 = 缓冲空
  * 补    充：只有 ModbusTask 一个消费者，多任务同时读会破坏 tail 的单写者约定
  */
uint8_t RS485_ReadByte(uint8_t *byte);

/**
  * 功    能：清空接收缓冲。发送前后各调一下，丢掉自发自收回来的回环和线路噪声
  * 参    数：无
  * 返 回 值：无
  * 补    充：发送前/后调用，丢弃自发自收回环与历史噪声
  */
void RS485_RxFlush(void);

/**
  * 功    能：发一串数据，内部自己切收发方向、处理回环。仅任务上下文调用
  * 参    数：data  待发送数据
  *            len  字节数，范围 1~256
  * 返 回 值：无
  * 补    充：仅任务上下文调用，内部处理方向与回环
  *            9600 波特率下发 256 字节约 267ms，这段时间会一直占着本任务
  */
void RS485_Send(const uint8_t *data, uint16_t len);

/**
  * 功    能：收字节的中断处理：塞进环形缓冲。中断里不能调 RTOS API，
  * 参    数：byte  本次收到的字节
  * 返 回 值：无
  * 补    充：仅供 usart.c 的 HAL_UART_RxCpltCallback 调用；ISR 上下文，不调 RTOS API
  *            由 HAL 的接收完成回调转调；缓冲满了就丢掉这个字节
  */
void RS485_RxByteISR(uint8_t byte);

/*HAL 单字节接收缓冲，在 bsp_RS485.c 定义；回调必须读它*/
extern uint8_t s_rxByte;

/*IDLE 中断唤醒 ModbusTask，替代 10ms 轮询*/

/**
  * 功    能：指定「收完一整帧后通知哪个任务」。必须在开空闲中断（RS485_EnableIdleIT）之前调
  * 参    数：task  目标任务句柄
  * 返 回 值：无
  * 补    充：必须先于 RS485_EnableIdleIT 调用
  */
void RS485_SetNotifyTask(TaskHandle_t task);

/**
  * 功    能：打开空闲线检测中断：总线上一段空闲就会触发，用它判定一帧收完了
  * 参    数：无
  * 返 回 值：无
  * 补    充：在 RS485_Init 与任务通知注册之后调用
  *            调用顺序是 RS485_Init → SetNotifyTask → 本函数
  */
void RS485_EnableIdleIT(void);

/**
  * 功    能：空闲中断处理：通知上面注册的任务「有新帧了」。在 it.c 的 USART3_IRQHandler 里调
  * 参    数：无
  * 返 回 值：无
  * 补    充：ISR 上下文，在 stm32f4xx_it.c 的 USART3_IRQHandler 里调用
  */
void RS485_IdleISR(void);


#endif /* __BSP_RS485_H */
