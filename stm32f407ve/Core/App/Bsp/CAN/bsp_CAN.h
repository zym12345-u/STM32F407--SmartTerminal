/**
  ******************************************************************************
  * @file    bsp_CAN.h
  * @brief   板载 CAN 收发器驱动(CAN1, PB8/PB9 AF9)
  *
  *          硬件连接:
  *            PB8 = CAN1_RX, PB9 = CAN1_TX (AF9),板载收发器已焊好。
  *            注意 CubeMX 默认给的是 PA11/PA12,必须改成 PB8/PB9,
  *            这两个脚在本工程另有用途风险,且板载收发器不接那里。
  *
  *          设计要点(与派活单 §6 铁律一一对应):
  *            1. ISR 优先级 CAN1_RX0_IRQn = 5/0 —— ISR 内要调
  *               xQueueSendToBackFromISR,优先级数值小于 5 会触发 FreeRTOS
  *               断言(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5)。
  *               CAN_BspInit 会打印实测值自证(派活单 R4)。
  *            2. 位时序在 CAN_BspInit 里按 CAN_BAUDRATE 覆写 hcan1.Init
  *               后再 HAL_CAN_Init(与 RS485_Init 重配 huart3 同一套路)。
  *            3. 过滤器/通知/启动的顺序固定:
  *               ConfigFilter -> ActivateNotification -> Start,不能换。
  *            4. ISR 只搬运:排空 FIFO、转 CanMsg_t 入队;不打印、不判断业务。
  *            5. 发送绝不阻塞:邮箱满直接 tx_drop++ 并返回 1,下一轮再试;
  *               禁止 HAL_Delay / while(等空闲邮箱) / while(!TXOK)。
  *            6. 运行期改过滤器会置 FMR.FINIT 重新初始化过滤组,期间到期的帧
  *               会丢 —— 这是 HAL 的固有行为,不是 bug(派活单 R6)。
  *
  *          队列句柄 static,不对外暴露;CAN_Send 可在任意任务上下文调用
  *          (禁止在 ISR 中调用,它不是 FromISR 版)。
  ******************************************************************************
  */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "CanData.h"

/* ====================== 可配置项 ====================== */

/**
  * @brief  回环自测开关
  * @note   用 #if 而非运行期变量(BUG 预防:避免有人忘了改回来还以为是总线问题)。
  *         1 = 环回自测,不接任何线即可验证控制器 + 队列 + 任务链路;
  *         0 = 正常总线模式。
  *         注意:环回模式下 RX 引脚与总线断开、ACK 由自己产生,
  *         只能证明芯片内部链路通,【证明不了收发器到 CANH/CANL 的物理层】。
  */
#define CAN_TEST_LOOPBACK   0U

/* 波特率: 500000 / 250000 / 125000 / 1000000(位时序表见派活单 §7) */
#define CAN_BAUDRATE        500000U

/* 队列深度: 内存预算 16×sizeof(CanMsg_t) + 8×sizeof(CanMsg_t) <= 1KB */
#define CAN_RX_QUEUE_DEPTH  16U
#define CAN_TX_QUEUE_DEPTH  8U

/* ====================== 对外接口 ====================== */

/**
  * @brief  初始化 CAN1(位时序覆写、过滤器、启动接收中断、创建队列)
  * @note   必须在任务上下文调用一次(内部会 osMessageQueueNew,ISR 不可用)
  * @retval 0=成功; 非 0=失败(1=位时序参数不支持, 2=HAL_CAN_Init 失败,
  *         3=过滤器配置失败, 4=启动失败, 5=队列创建失败, 6=通知使能失败)
  */
uint8_t CAN_BspInit(void);

/**
  * @brief  发送一帧(生产者:任意任务上下文,禁止 ISR 调用)
  * @param  msg  待发送帧
  * @retval 0=已入 TX 队列; 1=队列满丢弃(调用方可稍后重试)
  */
uint8_t CAN_Send(const CanMsg_t *msg);

/**
  * @brief  从 TX 队列取一帧(仅供 CanTask)
  * @param  msg        输出帧
  * @param  timeoutMs  阻塞超时(主循环节奏由此兜底,建议 100ms)
  * @retval 0=取到一帧; 非 0=超时
  */
uint8_t CAN_TxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * @brief  把一帧送到硬件邮箱(仅供 CanTask)
  * @note   绝不阻塞:邮箱满即丢弃并返回 1,由调用方决定后续
  * @retval 0=成功入邮箱; 1=邮箱满丢弃; 2=硬件参数错误
  */
uint8_t CAN_TxPump(const CanMsg_t *msg);

/**
  * @brief  从 RX 队列取一帧(仅供 CanTask)
  * @param  msg        输出帧
  * @param  timeoutMs  阻塞超时(0 = 非阻塞排空)
  * @retval 0=取到一帧; 非 0=空
  */
uint8_t CAN_RxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * @brief  取统计与状态快照
  * @note   跨字段快照允许轻微不一致(见 CanData.h 的单写者约定)
  */
void CAN_GetStats(CanStats_t *stats);

/**
  * @brief  运行期重配过滤器
  * @param  stdId  期望的标准帧 ID
  * @param  mask   掩码,位=1 表示该位必须匹配; 0x7FF = 精确匹配
  * @note   内部会置 FMR.FINIT,期间到期的帧会丢(见文件头第 6 条)
  * @retval 0=成功; 非 0=失败
  */
uint8_t CAN_FilterSet(uint32_t stdId, uint32_t mask);

/**
  * @brief  读 ESR 更新内部总线状态(仅供 CanTask 周期调用)
  * @note   R5: 判定顺序 BOFF > EPVF > EWGF;更新后的值由 CAN_GetStats 读出
  */
void CAN_UpdateBusState(void);

#endif /* __BSP_CAN_H */
