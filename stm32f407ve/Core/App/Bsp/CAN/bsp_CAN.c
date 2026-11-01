/* bsp_CAN.c
 * CAN1 板级驱动：位时序覆写、过滤器配置、收发队列、总线状态统计。
 * 硬件：CAN1 = PB8(RX)/PB9(TX)，AF9，接板载收发器；中断 CAN1_RX0_IRQn。
 * 注意：ISR 只用 FromISR 接口，中断优先级数值必须 >= 5；发送不阻塞，邮箱满即丢。
 */

#include "bsp_CAN.h"
#include "can.h"          /* hcan1: CubeMX 生成 */
#include "usart.h"        /* UART_Printf */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*内部状态*/

static osMessageQueueId_t s_rxQueue = NULL;
static osMessageQueueId_t s_txQueue = NULL;
static CanStats_t         s_stats;          /* rx 由 ISR 写，tx 由 CanTask 写 */

/*内部工具*/

/* 按目标波特率算出位时序写进 hcan1.Init（APB1 = 42MHz 为基准）。
 * 只认 1M/500K/250K/125K 四档；不认识的返回 1 且不动原值。仅 CAN_BspInit 调。 */
static uint8_t CAN_ApplyBaudrate(uint32_t baud)
{
    switch (baud)
    {
        case 1000000U:
            hcan1.Init.Prescaler = 3U;           /* 42M / 3 / (1+11+2) = 1Mbps */
            hcan1.Init.TimeSeg1  = CAN_BS1_11TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_2TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        case 500000U:
            hcan1.Init.Prescaler = 6U;           /* 42M / 6 / (1+11+2) = 500K */
            hcan1.Init.TimeSeg1  = CAN_BS1_11TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_2TQ;  /* 采样点 (1+11)/14 = 85.7% */
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        case 250000U:
            hcan1.Init.Prescaler = 21U;          /* 42M / 21 / (1+6+1) = 250Kbps，采样点 87.5% */
            hcan1.Init.TimeSeg1  = CAN_BS1_6TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_1TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        case 125000U:
            hcan1.Init.Prescaler = 42U;          /* 42M / 42 / (1+6+1) = 125Kbps，采样点 87.5% */
            hcan1.Init.TimeSeg1  = CAN_BS1_6TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_1TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        default:
            return 1U;
    }
    return 0U;
}

/*对外接口实现*/

/* 把 CAN1 拉起来：配位时序 → 装过滤器 → 启动 → 建收发队列 → 开 FIFO0 通知。
 * 必须在任务里调（内部要建队列）。返回 0 成功，非 0 是卡在哪一步：
 * 1 波特率不支持、2 HAL_CAN_Init、3 过滤器、4 启动、5 队列、6 通知使能。 */
uint8_t CAN_BspInit(void)
{
    CAN_FilterTypeDef filter;
    uint32_t          baud = CAN_BAUDRATE;

    /*覆写 CubeMX 的默认位时序*/
    if (CAN_ApplyBaudrate(baud) != 0U)
    {
        UART_Printf("[CAN] init FAIL: unsupported baud=%lu\r\n", (unsigned long)baud);
        return 1U;
    }

#if CAN_TEST_LOOPBACK
    /*环回模式 RX 断开总线，只验证片内链路，验不了 CANH/CANL*/
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
#else
    hcan1.Init.Mode = CAN_MODE_NORMAL;
#endif

    /*CubeMX 没应用 .ioc 的 ABOM，这里补上自动总线关闭恢复*/
    hcan1.Init.AutoBusOff          = ENABLE;
    hcan1.Init.AutoWakeUp          = DISABLE;
    hcan1.Init.AutoRetransmission  = ENABLE;   /* 自动重传: NART=0 */
    hcan1.Init.ReceiveFifoLocked   = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    hcan1.Init.TimeTriggeredMode   = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        UART_Printf("[CAN] init FAIL: HAL_CAN_Init\r\n");
        return 2U;
    }

    /*打印实测优先级，必须 >= 5*/
    UART_Printf("[CAN] nvic prio=%u (must >=5)\r\n",
                (unsigned)NVIC_GetPriority(CAN1_RX0_IRQn));

    /*过滤器：32 位掩码模式，掩码全 0 = 全通*/
    filter.FilterBank           = 0U;                        /* 过滤组 0；CAN1 可用 0~13，CAN2 用 14~27 */
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;     /* 32 位模式：一次比较完整的 11/29 位 ID */
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;                   /* 掩码全 0 = 所有位都不比较，即全通 */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;              /* 挂 FIFO0，对应 RxFifo0MsgPendingCallback */
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;                       /* CAN2 起始过滤组固定为 14 */

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        UART_Printf("[CAN] init FAIL: ConfigFilter\r\n");
        return 3U;
    }

    /*创建收发队列*/
    s_rxQueue = osMessageQueueNew(CAN_RX_QUEUE_DEPTH, sizeof(CanMsg_t), NULL);
    s_txQueue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH, sizeof(CanMsg_t), NULL);
    if ((s_rxQueue == NULL) || (s_txQueue == NULL))
    {
        UART_Printf("[CAN] init FAIL: queue create\r\n");
        return 5U;
    }

    /*先开通知再启动，顺序不能换*/
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        UART_Printf("[CAN] init FAIL: ActivateNotification\r\n");
        return 6U;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        UART_Printf("[CAN] init FAIL: Start\r\n");
        return 4U;
    }

    UART_Printf("[CAN] init ok mode=%s baud=%luK filter=accept-all\r\n",
#if CAN_TEST_LOOPBACK
                "loopback",
#else
                "normal",
#endif
                (unsigned long)(baud / 1000U));

#if CAN_TEST_LOOPBACK
    UART_Printf("[CAN] loopback mode: physical layer NOT verified\r\n");
#endif

    return 0U;
}

/* 把一帧放进 TX 队列就返回 —— 不碰硬件、不等待，真正发出去是 CAN_TxPump 的事。
 * 返回 0 已入队；1 参数非法或队列满（满了直接丢，由调用方下次周期再发）。
 * 只在任务里调，这是非 FromISR 版本。 */
uint8_t CAN_Send(const CanMsg_t *msg)
{
    if ((s_txQueue == NULL) || (msg == NULL))
    {
        return 1U;
    }
    /*队列满即丢弃，不阻塞等待*/
    if (osMessageQueuePut(s_txQueue, msg, 0U, 0U) != osOK)
    {
        return 1U;
    }
    return 0U;
}

/* 从 TX 队列取一帧出来准备发送；队列空就阻塞到 timeoutMs（0 = 不阻塞）。
 * 返回 0 取到一帧，1 参数非法或超时。超时属 CanTask 主循环的正常节奏，不是故障。 */
uint8_t CAN_TxPop(CanMsg_t *msg, uint32_t timeoutMs)
{
    if ((s_txQueue == NULL) || (msg == NULL))
    {
        return 1U;
    }
    if (osMessageQueueGet(s_txQueue, msg, NULL, timeoutMs) != osOK)
    {
        return 1U;   /* 超时让出 CPU */
    }
    return 0U;
}

/* 真正写进硬件邮箱、发到总线上。ide=0 走标准帧；dlc 超过 8 按 8 截断。
 * 返回 0 已进邮箱；1 三个邮箱全满、本帧丢弃；2 参数非法或 HAL 发送失败。
 * tx_total / tx_drop / tx_error 三个计数只在这里写。 */
uint8_t CAN_TxPump(const CanMsg_t *msg)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t            mailbox;

    if (msg == NULL)
    {
        return 2U;
    }

    /*无空闲邮箱立即丢弃，下一轮再试*/
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)     /* 3 个发送邮箱全被占满 */
    {
        s_stats.tx_drop++;          /* 仅 CanTask 写 */
        return 1U;
    }

    if (msg->ide == 0U)
    {
        hdr.IDE   = CAN_ID_STD;
        hdr.StdId = msg->id & 0x7FFU;        /* 标准帧 ID 只有 11 位，高位直接截掉 */
        hdr.ExtId = 0U;
    }
    else
    {
        hdr.IDE   = CAN_ID_EXT;
        hdr.ExtId = msg->id & 0x1FFFFFFFU;   /* 扩展帧 ID 29 位，0x00000000~0x1FFFFFFF */
        hdr.StdId = 0U;
    }
    hdr.RTR = (msg->rtr == 0U) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    hdr.DLC = (msg->dlc > 8U) ? 8U : msg->dlc;   /* CAN 单帧数据最长 8 字节，超长按 8 发 */
    hdr.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)msg->data, &mailbox) != HAL_OK)
    {
        s_stats.tx_error++;         /* 仅 CanTask 写 */
        return 2U;
    }

    s_stats.tx_total++;             /* 仅 CanTask 写 */
    return 0U;
}

/* 从 RX 队列取一帧收到的数据（中断回调里已经投进队列）。
 * timeoutMs = 0 表示不等待，用于一次性把队列排空。返回 0 取到，1 参数非法或队列空。 */
uint8_t CAN_RxPop(CanMsg_t *msg, uint32_t timeoutMs)
{
    if ((s_rxQueue == NULL) || (msg == NULL))
    {
        return 1U;
    }
    if (osMessageQueueGet(s_rxQueue, msg, NULL, timeoutMs) != osOK)
    {
        return 1U;
    }
    return 0U;
}

/* 拷一份统计快照：收发计数、TEC/REC、当前总线状态。
 * 不关中断也不加锁，字段之间可能轻微对不上（快照本身非原子）—— 看日志够用，别为它加锁。 */
void CAN_GetStats(CanStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }
    /*跨字段快照不关中断、不加锁*/
    stats->rx_total = s_stats.rx_total;
    stats->rx_drop  = s_stats.rx_drop;
    stats->tx_total = s_stats.tx_total;
    stats->tx_drop  = s_stats.tx_drop;
    stats->tx_error = s_stats.tx_error;
    stats->state    = s_stats.state;
    stats->tec      = s_stats.tec;
    stats->rec      = s_stats.rec;
}

/* 重设过滤器。mask 位为 1 表示该位必须匹配：0x7FF 精确匹配一个 ID，0x000 全放行。
 * 返回 0 成功，1 配置失败。重配会让硬件重建整个过滤组，这一瞬间到期的帧会丢。 */
uint8_t CAN_FilterSet(uint32_t stdId, uint32_t mask)
{
    CAN_FilterTypeDef filter;

    /*重配置 FMR.FINIT，期间到期的帧会丢*/
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((stdId & 0x7FFU) << 5);   /* 11 位 ID 在寄存器 [31:21]，左移 5 落位 */
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = (uint16_t)((mask & 0x7FFU) << 5);
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return (HAL_CAN_ConfigFilter(&hcan1, &filter) == HAL_OK) ? 0U : 1U;
}

/*状态更新*/

/* 读错误状态寄存器更新内部总线状态（写进 s_stats 的 tec / rec / state）。
 * 判定顺序必须是 BOFF > EPVF > EWGF，写反会误报（已 bus-off 却报 warning）。
 * 由 CanTask 每秒调一次。 */
void CAN_UpdateBusState(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    /*ESR 位段：[23:16] REC、[15:8] TEC；低 8 位是标志和 LEC，不是计数器*/
    uint8_t  tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);   /* 发送错误计数 0~255，>255 饱和 */
    uint8_t  rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);   /* 接收错误计数 0~255，>255 饱和 */

    /*先判 BOFF，再判 EPVF/EWGF*/
    CanBusState_t st;
    if ((esr & CAN_ESR_BOFF) != 0U)
    {
        st = CAN_BUS_OFF;
    }
    else if ((esr & CAN_ESR_EPVF) != 0U)
    {
        st = CAN_BUS_PASSIVE;
    }
    else if ((esr & CAN_ESR_EWGF) != 0U)
    {
        st = CAN_BUS_WARNING;
    }
    else
    {
        st = CAN_BUS_OK;
    }

    s_stats.tec   = tec;    /* 仅 CanTask 写 */
    s_stats.rec   = rec;
    s_stats.state = st;
}

/*接收中断*/

/* 收到帧的中断回调：把帧从硬件 FIFO 搬进 RX 队列就返回。
 * 中断里只搬运 —— 不打印、不判业务、不喂狗。队列满时只累加 rx_drop 然后丢掉。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((hcan == NULL) || (hcan->Instance != CAN1) || (s_rxQueue == NULL))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef hdr;
        /*必须整体清零：DLC<8 时剩余 data[] 是栈垃圾，会对端读出随机参数*/
        CanMsg_t            msg = {0};

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, msg.data) != HAL_OK)
        {
            continue;
        }

        msg.id        = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
        msg.ide       = (hdr.IDE == CAN_ID_STD) ? 0U : 1U;
        msg.rtr       = (hdr.RTR == CAN_RTR_DATA) ? 0U : 1U;
        msg.dlc       = hdr.DLC;
        msg.rsv       = 0U;
        msg.timestamp = xTaskGetTickCountFromISR();

        s_stats.rx_total++;                 /* 仅 ISR 写 */

        /*osMessageQueueId_t 与 QueueHandle_t 在 FreeRTOS 上等价*/
        if (xQueueSendToBackFromISR((QueueHandle_t)s_rxQueue, &msg,
                                    &xHigherPriorityTaskWoken) != pdTRUE)
        {
            s_stats.rx_drop++;              /* 队列满只加计数,不做其它 */
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
