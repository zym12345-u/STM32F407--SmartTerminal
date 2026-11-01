/* CAN1 驱动：初始化、收发的队列与邮箱、统计快照、过滤器。
 * 硬件：CAN1 = PB8(RX)/PB9(TX)，AF9，接板载收发器。
 * 注意：CAN1_RX0_IRQn 的优先级数值必须 >= 5；CAN_Send 不是 FromISR 版本，禁止在中断里调。
 */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "CanData.h"

/*可配置项*/

/* 回环自测开关：1 = 控制器自发自收，不接线就能验证「队列 + 任务」链路通不通。
 * 环回模式下 RX 不接总线、ACK 由自己产生，所以【验不了】收发器到 CANH/CANL 的物理层。
 * 用 #if 而非常量，是为了让编译器直接断开这段代码，也提醒你别忘了改回来。 */
#define CAN_TEST_LOOPBACK   0U          /* 0 = 正常总线(默认); 1 = 环回自测 */

/* 精确过滤验收开关：1 = 只放行 CAN_ID_TEST(0x123)，用来证明 CAN_FilterSet 真的生效。
 * 它是验收专用，跑完必须改回 0 —— 否则 0x201 命令帧全被硬件挡掉。 */
#define CAN_TEST_FILTER_EXACT 0U        /* 0 = 全通(默认); 1 = 只放行 0x123 */

/* 位时序按此波特率反算，支持 1M/500K/250K/125K（换算表在 bsp_CAN.c） */
#define CAN_BAUDRATE        500000U     /* 42MHz/6/(1+11+2)=500K，14tq，采样点 85.7% */

/* 队列深度：吃 RAM 的大头（一帧 24B），加大前先算预算 */
#define CAN_RX_QUEUE_DEPTH  16U         /* 可缓存 16 帧，塞满后 ISR 丢弃并累加 rx_drop */
#define CAN_TX_QUEUE_DEPTH  8U          /* 可缓存 8 帧，塞满后 CAN_Send 直接返回 1 */

/*对外接口*/

/**
  * 功    能：功    能：把 CAN1 拉起来 —— 配位时序、装过滤器、启动、建收发队列、开 FIFO0 通知
  * 参    数：无
  * 返 回 值：0 = 成功；非 0 = 失败（1 位时序不支持、2 HAL_CAN_Init、3 过滤器、4 启动、5 队列、6 通知）
  * 补    充：必须在任务上下文调用一次，内部会 osMessageQueueNew
  *            参    数：无
  *            返 回 值：0 = 成功；非 0 = 卡在哪一步
  *            1 位时序不支持、2 HAL_CAN_Init、3 过滤器、4 启动、5 队列、6 通知使能
  *            补    充：必须在任务上下文调用一次，内部要建队列，中断里调会失败
  */
uint8_t CAN_BspInit(void);

/**
  * 功    能：功    能：把一帧放进 TX 队列后立即返回，不碰硬件、不等待
  * 参    数：msg  待发送帧
  * 返 回 值：0 = 已入 TX 队列，1 = 队列满丢弃，可稍后重试
  * 补    充：任意任务上下文可调，禁止在 ISR 中调用
  *            参    数：msg  待发送帧，不能为 NULL
  *            返 回 值：0 = 已入队；1 = 参数非法或队列满
  *            补    充：真正发到总线由 CAN_TxPump 做；队列满就直接丢掉，由调用方下次周期再发
  *            只在任务里调，这是非 FromISR 版本
  */
uint8_t CAN_Send(const CanMsg_t *msg);

/**
  * 功    能：功    能：从 TX 队列取一帧出来准备发；队列空就阻塞到超时
  * 参    数：msg  输出帧
  *            timeoutMs  阻塞超时，单位 ms，建议 100
  * 返 回 值：0 = 取到一帧，非 0 = 超时
  * 补    充：仅供 CanTask 调用；超时是主循环正常节奏
  *            参    数：msg        输出帧，不能为 NULL
  *            timeoutMs  阻塞超时，单位 ms；0 = 不阻塞
  *            返 回 值：0 = 取到一帧；1 = 参数非法或超时
  *            补    充：超时属 CanTask 主循环的正常节奏，不是故障
  */
uint8_t CAN_TxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * 功    能：功    能：真正写进硬件邮箱、发到总线上
  * 参    数：msg  待发送帧
  * 返 回 值：0 = 成功入邮箱，1 = 邮箱满丢弃，2 = 硬件参数错误
  * 补    充：仅供 CanTask 调用；邮箱满即丢弃，不阻塞
  *            参    数：msg  待发送帧；ide=0 走标准帧；dlc 超过 8 按 8 截断
  *            返 回 值：0 = 已进邮箱；1 = 三个邮箱全满、本帧丢弃；2 = 参数非法或 HAL 发送失败
  *            补    充：全程不阻塞，失败只记账（tx_drop / tx_error）。tx_total、tx_drop、tx_error
  *            这三个计数只有这里会写
  */
uint8_t CAN_TxPump(const CanMsg_t *msg);

/**
  * 功    能：功    能：从 RX 队列取一帧收到的数据（中断回调里已经把帧投进队列了）
  * 参    数：msg  输出帧
  *            timeoutMs  阻塞超时，单位 ms，0 = 非阻塞排空
  * 返 回 值：0 = 取到一帧，非 0 = 队列空
  * 补    充：仅供 CanTask 调用
  *            参    数：msg        输出帧，不能为 NULL
  *            timeoutMs  阻塞超时，单位 ms；0 = 不等待，用于一次性排空
  *            返 回 值：0 = 取到一帧；1 = 参数非法或队列空
  *            补    充：仅供 CanTask 调用
  */
uint8_t CAN_RxPop(CanMsg_t *msg, uint32_t timeoutMs);

/**
  * 功    能：功    能：拷一份统计快照出来 —— 收发计数、TEC/REC、当前总线状态
  * 参    数：stats  输出快照
  * 返 回 值：无
  * 补    充：不关中断、不加锁，跨字段允许轻微不一致
  *            参    数：stats  输出快照，不能为 NULL
  *            返 回 值：无
  *            补    充：不关中断也不加锁，字段之间可能轻微对不上（快照本身非原子）
  *            这是有意的取舍 —— 看日志够用，别为它加锁
  */
void CAN_GetStats(CanStats_t *stats);

/**
  * 功    能：功    能：重设过滤器，用来做「只收某个 ID」的验收
  * 参    数：stdId  标准帧 ID，11 位，范围 0x000~0x7FF
  *            mask  掩码，位 = 1 表示该位必须匹配；0x7FF = 精确匹配
  * 返 回 值：0 = 成功，非 0 = 失败
  * 补    充：重配会置 FMR.FINIT，期间到期的帧会丢
  *            参    数：stdId  标准帧 ID，11 位，范围 0x000~0x7FF
  *            mask   掩码，位为 1 表示该位必须匹配：0x7FF 精确匹配，0x000 全放行
  *            返 回 值：0 = 配置成功；1 = 失败
  *            补    充：重配会让硬件重建整个过滤组，这一瞬间到期的帧会被丢掉，属硬件行为
  */
uint8_t CAN_FilterSet(uint32_t stdId, uint32_t mask);

/**
  * 功    能：功    能：读一次错误状态寄存器（ESR），更新内部记的总线状态
  * 参    数：无
  * 返 回 值：无，结果由 CAN_GetStats 读出
  * 补    充：仅供 CanTask 周期调用；判定顺序 BOFF > EPVF > EWGF
  *            参    数：无
  *            返 回 值：无，结果写进 s_stats 的 tec / rec / state，由 CAN_GetStats 读出
  *            补    充：判定顺序必须是 BOFF > EPVF > EWGF（先判总线关闭，再看被动，最后警告），
  *            写反了会误报，比如已经 bus-off 却还报 warning。由 CanTask 每秒调一次
  */
void CAN_UpdateBusState(void);

#endif /* __BSP_CAN_H */
