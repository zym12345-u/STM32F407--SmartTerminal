/**
  ******************************************************************************
  * @file    bsp_CAN.c
  * @brief   板载 CAN 收发器驱动实现(CAN1, PB8/PB9 AF9)
  *
  *          实现要点(对应派活单 §6 八条铁律 + 驻场回信 R1~R8):
  *
  *            ① ISR 优先级: CAN1_RX0_IRQn = 5/0,由 CubeMX 生成(.ioc 配置)。
  *               CAN_BspInit 里用 NVIC_GetPriority 打印实测值自证(R4):
  *               必须 >= 5,否则 xQueueSendToBackFromISR 会触发 FreeRTOS 断言。
  *
  *            ② 位时序覆写: 按 CAN_BAUDRATE 重配 hcan1.Init 后 HAL_CAN_Init。
  *               CubeMX 生成的 can.c 给的是默认值(脚本模式下未应用 .ioc 参数),
  *               真实波特率以本文件为准(派活单 §6-2 本就要求 BSP 覆写)。
  *
  *            ③ 顺序固定: ConfigFilter -> ActivateNotification -> Start。
  *
  *            ④ ISR 只搬运: 排空 FIFO、转 CanMsg_t 入队;不打印、不判断业务、
  *               不喂狗、不调非 FromISR 版 RTOS API。
  *
  *            ⑤ 发送绝不阻塞: 邮箱满即 tx_drop++ 返回 1,无 HAL_Delay、
  *               无 while(等邮箱)、无 while(!TXOK)。
  *
  *            ⑥ ESR 判定顺序: BOFF 优先,其次 EPVF,再次 EWGF(R5)。
  *               写反会出现"bus-off 却报 warning"。
  *
  *            ⑦ 统计单写者: rx_total/rx_drop 只由 ISR 写;
  *               tx_total/tx_drop/tx_error 只由 CanTask 写。32 位对齐写原子,
  *               不加锁;跨字段快照允许轻微不一致(R7)。
  *
  *            ⑧ 队列用 osMessageQueueNew 创建(CMSIS 封装),ISR 侧需要
  *               FromISR 版 API,故强转为 FreeRTOS 的 QueueHandle_t 使用 ——
  *               在 FreeRTOS 上 osMessageQueueId_t 与 QueueHandle_t 等价。
  ******************************************************************************
  */

#include "bsp_CAN.h"
#include "can.h"          /* hcan1: CubeMX 生成 */
#include "usart.h"        /* UART_Printf */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* ====================== 内部状态 ====================== */

static osMessageQueueId_t s_rxQueue = NULL;
static osMessageQueueId_t s_txQueue = NULL;
static CanStats_t         s_stats;          /* 单写者约定见文件头 ⑦ */

/* ====================== 内部工具 ====================== */

/**
  * @brief  按目标波特率覆写位时序(APB1 = 42MHz)
  * @note   派活单 §7 参数定档表,采样点均约 85%~87.5%
  * @retval 0=支持; 1=不支持的波特率
  */
static uint8_t CAN_ApplyBaudrate(uint32_t baud)
{
    switch (baud)
    {
        case 1000000U:
            hcan1.Init.Prescaler = 3U;
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
            hcan1.Init.Prescaler = 21U;
            hcan1.Init.TimeSeg1  = CAN_BS1_6TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_1TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        case 125000U:
            hcan1.Init.Prescaler = 42U;
            hcan1.Init.TimeSeg1  = CAN_BS1_6TQ;
            hcan1.Init.TimeSeg2  = CAN_BS2_1TQ;
            hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;
        default:
            return 1U;
    }
    return 0U;
}

/* ====================== 对外接口实现 ====================== */

uint8_t CAN_BspInit(void)
{
    CAN_FilterTypeDef filter;
    uint32_t          baud = CAN_BAUDRATE;

    /* ---- ① 位时序覆写(CubeMX 生成值是默认的,这里才是真实波特率) ---- */
    if (CAN_ApplyBaudrate(baud) != 0U)
    {
        UART_Printf("[CAN] init FAIL: unsupported baud=%lu\r\n", (unsigned long)baud);
        return 1U;
    }

#if CAN_TEST_LOOPBACK
    /* 环回自测: 芯片内部闭环,ACK 由自己产生。
     * 注意(R2): 此模式下 RX 引脚与总线断开,只能证明控制器+队列+任务链路通,
     * 证明不了 PB8/PB9 -> 收发器 -> CANH/CANL 这段物理层。 */
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
#else
    hcan1.Init.Mode = CAN_MODE_NORMAL;
#endif

    /* ABOM(自动总线关闭恢复): 派活单 §3 要求 Enable。
     * CubeMX 脚本模式未把 .ioc 的 ABOM=ENABLE 应用到生成代码,这里显式补上。 */
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

    /* ---- ④ NVIC 优先级自证(R4): 必须 >= 5 ---- */
    UART_Printf("[CAN] nvic prio=%u (must >=5)\r\n",
                (unsigned)NVIC_GetPriority(CAN1_RX0_IRQn));

    /* ---- ③ 过滤器: 默认全通(32 位掩码模式,掩码全 0 = 不筛选) ---- */
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        UART_Printf("[CAN] init FAIL: ConfigFilter\r\n");
        return 3U;
    }

    /* ---- 队列创建(任务上下文;ISR 里不创建) ---- */
    s_rxQueue = osMessageQueueNew(CAN_RX_QUEUE_DEPTH, sizeof(CanMsg_t), NULL);
    s_txQueue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH, sizeof(CanMsg_t), NULL);
    if ((s_rxQueue == NULL) || (s_txQueue == NULL))
    {
        UART_Printf("[CAN] init FAIL: queue create\r\n");
        return 5U;
    }

    /* ---- 顺序: ActivateNotification -> Start(不能换) ---- */
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
    /* R2: 明确提示物理层未验证,避免以后看日志时误读 */
    UART_Printf("[CAN] loopback mode: physical layer NOT verified\r\n");
#endif

    return 0U;
}

uint8_t CAN_Send(const CanMsg_t *msg)
{
    if ((s_txQueue == NULL) || (msg == NULL))
    {
        return 1U;
    }
    /* 任务上下文;队列满即丢弃,由调用方决定重试 —— 绝不阻塞 */
    if (osMessageQueuePut(s_txQueue, msg, 0U, 0U) != osOK)
    {
        return 1U;
    }
    return 0U;
}

uint8_t CAN_TxPop(CanMsg_t *msg, uint32_t timeoutMs)
{
    if ((s_txQueue == NULL) || (msg == NULL))
    {
        return 1U;
    }
    if (osMessageQueueGet(s_txQueue, msg, NULL, timeoutMs) != osOK)
    {
        return 1U;   /* 超时: 主循环节奏由此兜底 */
    }
    return 0U;
}

uint8_t CAN_TxPump(const CanMsg_t *msg)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t            mailbox;

    if (msg == NULL)
    {
        return 2U;
    }

    /* ⑤ 绝不阻塞: 无空闲邮箱立即丢弃,下一轮再试 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        s_stats.tx_drop++;          /* 仅 CanTask 写 */
        return 1U;
    }

    if (msg->ide == 0U)
    {
        hdr.IDE   = CAN_ID_STD;
        hdr.StdId = msg->id & 0x7FFU;
        hdr.ExtId = 0U;
    }
    else
    {
        hdr.IDE   = CAN_ID_EXT;
        hdr.ExtId = msg->id & 0x1FFFFFFFU;
        hdr.StdId = 0U;
    }
    hdr.RTR = (msg->rtr == 0U) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    hdr.DLC = (msg->dlc > 8U) ? 8U : msg->dlc;
    hdr.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)msg->data, &mailbox) != HAL_OK)
    {
        s_stats.tx_error++;         /* 仅 CanTask 写 */
        return 2U;
    }

    s_stats.tx_total++;             /* 仅 CanTask 写 */
    return 0U;
}

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

void CAN_GetStats(CanStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }
    /* 跨字段快照允许轻微不一致(见文件头 ⑦),故不关闭中断、不加锁 */
    stats->rx_total = s_stats.rx_total;
    stats->rx_drop  = s_stats.rx_drop;
    stats->tx_total = s_stats.tx_total;
    stats->tx_drop  = s_stats.tx_drop;
    stats->tx_error = s_stats.tx_error;
    stats->state    = s_stats.state;
    stats->tec      = s_stats.tec;
    stats->rec      = s_stats.rec;
}

uint8_t CAN_FilterSet(uint32_t stdId, uint32_t mask)
{
    CAN_FilterTypeDef filter;

    /* R6: HAL_CAN_ConfigFilter 内部会置 FMR.FINIT 重新初始化过滤组,
     * 期间到期的帧会丢 —— 这是 HAL 固有行为,不是 bug。 */
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((stdId & 0x7FFU) << 5);
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = (uint16_t)((mask & 0x7FFU) << 5);
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return (HAL_CAN_ConfigFilter(&hcan1, &filter) == HAL_OK) ? 0U : 1U;
}

/* ====================== 状态轮询(仅 CanTask 调用) ====================== */

/**
  * @brief  读 ESR 更新总线状态
  * @note   R5: 判定顺序 BOFF > EPVF > EWGF,顺序写反会误报
  */
void CAN_UpdateBusState(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    /* CAN_ESR 位段: [0]EWGF [1]EPVF [2]BOFF [6:4]LEC [15:8]TEC [23:16]REC。
     * 别再把 TEC/REC 读成低字节 —— 低 8 位是标志+LEC,不是计数器。 */
    uint8_t  tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    uint8_t  rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);

    /* ⑥ BOFF 优先 */
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

/* ====================== ISR 回调(只搬运) ====================== */

/**
  * @brief  FIFO0 有挂起报文(HAL_CAN_IRQHandler 分发)
  * @note   ④ 只做搬运: 排空 FIFO -> 入队。禁止打印/业务判断/喂狗。
  *         ⑦ rx_total / rx_drop 仅在此处写。
  */
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
        CanMsg_t            msg;

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

        /* ⑧ osMessageQueueId_t 与 QueueHandle_t 在 FreeRTOS 上等价 */
        if (xQueueSendToBackFromISR((QueueHandle_t)s_rxQueue, &msg,
                                    &xHigherPriorityTaskWoken) != pdTRUE)
        {
            s_stats.rx_drop++;              /* 队列满只加计数,不做其它 */
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
