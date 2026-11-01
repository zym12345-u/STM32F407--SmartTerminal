/* bsp_CAN.h
 * CAN1 驱动接口：初始化、发送、收发队列、统计快照与过滤器配置。
 * 硬件：CAN1 = PB8(RX)/PB9(TX)，AF9，接板载收发器。
 * 注意：CAN1_RX0_IRQn 优先级数值必须 >= 5；CAN_Send 不是 FromISR 版，禁止在 ISR 中调用。
 */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "CanData.h"

/*可配置项*/

/*回环自测开关：1 = 环回，不接线即可验证控制器+队列+任务链路；0 = 正常总线
 * 环回模式 RX 与总线断开、ACK 自己产生，验不了收发器到 CANH/CANL 的物理层
 * 用 #if 而非运行期变量，避免忘了改回来还以为是总线问题*/
#define CAN_TEST_LOOPBACK   0U          /* 0 = 正常总线模式(默认); 1 = 环回自测 */

/*精确过滤验收开关：1 = 收端只放行 CAN_ID_TEST(0x123)，用于验证 CAN_FilterSet
 * 验收完改回 0，管理方式同回环开关*/
#define CAN_TEST_FILTER_EXACT 0U        /* 0 = 全通(默认); 1 = 只放行 CAN_ID_TEST(0x123) */

/*波特率：1M/500K/250K/125K，位时序表在 bsp_CAN.c*/
#define CAN_BAUDRATE        500000U     /* 42MHz/6/(1+11+2)=500K，14tq，采样点 85.7% */

/*队列深度：受 RAM 预算约束，加大前先核算 CanMsg_t 大小*/
#define CAN_RX_QUEUE_DEPTH  16U         /* RX 队列可缓存 16 帧，满则由 ISR 累加 rx_drop */
#define CAN_TX_QUEUE_DEPTH  8U          /* TX 队列可缓存 8 帧，满则 CAN_Send 立即返回 1 */

/*对外接口*/

/**
  * 函    数：CAN_BspInit
  * 参    数：无
  * 返 回 值：0 = 成功；非 0 = 失败（1 位时序不支持、2 HAL_CAN_Init、3 过滤器、4 启动、5 队列、6 通知）
  * 注意事项：必须在任务上下文调用一次，内部会 osMessageQueueNew
  */
uint8_t CAN_BspInit(void);

/**
  * 函    数：CAN_Send
  * 参    数：msg  待发送帧
  * 返 回 值：0 = 已入 TX 队列，1 = 队列满丢弃，可稍后重试
  * 注意事项：任意任务上下文可调，禁止在 ISR 中调用
  */
uint8_t CAN_Send(const CanMsg_t *msg);

/**
  * 函    数：CAN_TxPop
  * 参    数：msg  输出帧
  * 参    数：timeoutMs  阻塞超时，单位 ms，建议 100
  * 返 回 值：0 = 取到一帧，非 0 = 超时
  * 注意事项：仅供 CanTask 调用；超时是主循环正常节奏
  */
uint8_t CAN_TxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * 函    数：CAN_TxPump
  * 参    数：msg  待发送帧
  * 返 回 值：0 = 成功入邮箱，1 = 邮箱满丢弃，2 = 硬件参数错误
  * 注意事项：仅供 CanTask 调用；邮箱满即丢弃，不阻塞
  */
uint8_t CAN_TxPump(const CanMsg_t *msg);

/**
  * 函    数：CAN_RxPop
  * 参    数：msg  输出帧
  * 参    数：timeoutMs  阻塞超时，单位 ms，0 = 非阻塞排空
  * 返 回 值：0 = 取到一帧，非 0 = 队列空
  * 注意事项：仅供 CanTask 调用
  */
uint8_t CAN_RxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * 函    数：CAN_GetStats
  * 参    数：stats  输出快照
  * 返 回 值：无
  * 注意事项：不关中断、不加锁，跨字段允许轻微不一致
  */
void CAN_GetStats(CanStats_t *stats);

/**
  * 函    数：CAN_FilterSet
  * 参    数：stdId  标准帧 ID，11 位，范围 0x000~0x7FF
  * 参    数：mask  掩码，位 = 1 表示该位必须匹配；0x7FF = 精确匹配
  * 返 回 值：0 = 成功，非 0 = 失败
  * 注意事项：重配会置 FMR.FINIT，期间到期的帧会丢
  */
uint8_t CAN_FilterSet(uint32_t stdId, uint32_t mask);

/**
  * 函    数：CAN_UpdateBusState
  * 参    数：无
  * 返 回 值：无，结果由 CAN_GetStats 读出
  * 注意事项：仅供 CanTask 周期调用；判定顺序 BOFF > EPVF > EWGF
  */
void CAN_UpdateBusState(void);

#endif /* __BSP_CAN_H */
