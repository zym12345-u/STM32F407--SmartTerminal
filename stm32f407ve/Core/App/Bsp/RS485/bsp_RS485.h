/**
  ******************************************************************************
  * @file    bsp_RS485.h
  * @brief   板载 RS485 收发器驱动(USART2 接口, 硬件自动收发方向)
  *
  *          硬件连接(重要, 见厂商例程 14-2 main.c 说明):
  *            板载 RS485 电路出厂默认用跳线帽接到 USART3(PB10/PB11),
  *            但本工程 USART3 已被 ESP8266 占用, 因此:
  *              - 用杜邦线把 RS485 电路的 TX/RX 跳接到 USART2: PA2(TX)/PA3(RX)
  *              - RS485 收发器为硬件自动方向控制(厂商例程发送时未操作任何
  *                方向引脚), 无需 DE/RE 引脚; 若某些批次需要手动方向控制,
  *                打开 RS485_MANUAL_DE 宏并在 RS485_DE_Set 中补上引脚
  *
  *          说明:
  *            1. 复用 CubeMX 生成的 huart2(usart.c), 本驱动内重新配置波特率
  *            2. RX: 单字节中断 -> usart.c 的 HAL_UART_RxCpltCallback 中
  *               USART2 分支调用 RS485_RxByteISR() -> 单生产者/单消费者环形缓冲
  *            3. TX: 任务上下文轮询发送(响应帧最长 256B @9600 ≈ 267ms,
  *               只阻塞本任务, 不影响其它任务); 发送完毕后清空自发自收的回环字节
  *            4. 半双工特性: 发送时本板 RX 会收到自己发出的字节(自发自收),
  *               调用方无需关心, RS485_Send 内部已处理
  ******************************************************************************
  */
#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx_hal.h"

/* 手动方向控制开关: 本板 RS485 为硬件自动方向, 默认关闭。
 * 若实测发现发送后总线被拉死(收不到主站下一帧), 说明该批次收发器
 * 需要手动 DE, 打开此宏并实现 RS485_DE_Set 的引脚操作 */
#define RS485_MANUAL_DE         0

/* 默认通信参数(与常见 Modbus 设备出厂值一致, 可按主站软件调整) */
#define RS485_BAUDRATE          9600U

/* 接收环形缓冲大小: Modbus 一帧最长 256 字节, 缓冲留一帧余量 */
#define RS485_RX_BUF_SIZE       256U


/* ====================== 对外接口 ====================== */

/**
  * @brief  初始化 RS485(重配 huart2 波特率, 清空环形缓冲, 启动接收中断)
  * @note   依赖 usart.c 中 MX_USART2_UART_Init 已完成首次初始化
  */
void RS485_Init(void);

/**
  * @brief  查询接收环形缓冲中的字节数
  * @retval 当前缓冲的待读字节数
  */
uint16_t RS485_RxCount(void);

/**
  * @brief  从接收环形缓冲读出一个字节
  * @param  byte  输出参数, 存放读出的字节
  * @retval 1-读到; 0-缓冲空
  */
uint8_t RS485_ReadByte(uint8_t *byte);

/**
  * @brief  清空接收环形缓冲(发送前/后调用, 丢弃自发自收回环与历史噪声)
  */
void RS485_RxFlush(void);

/**
  * @brief  阻塞发送一帧(任务上下文调用, 内部处理方向/回环)
  * @param  data  待发送数据
  * @param  len   字节数
  */
void RS485_Send(const uint8_t *data, uint16_t len);

/* 仅供 usart.c 的 HAL_UART_RxCpltCallback(USART2 分支)调用: 把 ISR 收到的
 * 字节推入环形缓冲。ISR 上下文, 内部不调用任何 RTOS API */
void RS485_RxByteISR(uint8_t byte);


#endif /* __BSP_RS485_H */
