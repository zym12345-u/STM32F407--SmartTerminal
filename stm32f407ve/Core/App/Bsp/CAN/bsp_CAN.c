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

/**
  * 功    能：按目标波特率算出位时序，写进 hcan1.Init（APB1 = 42MHz 为基准）
  * 参    数：baud  目标波特率，单位 bit/s，仅支持 1M/500K/250K/125K
  * 返 回 值：0 = 已写入；1 = 不支持该波特率，hcan1.Init 保持原值
  * 补    充：仅 CAN_BspInit 调用。
  *
  * ── 下面这几个寄存器到底在干嘛（看视频容易漏掉的部分）──
  *
  * CAN 总线上一个「位」不是简单的高低电平，而是被切成若干「时间份额 TQ」。
  * TQ 有多长，由 Prescaler 决定：
  *     TQ = Prescaler / APB1时钟        例：500K 档 → 6 / 42MHz = 142.86ns
  * 一个位 = 1 个同步段 + BS1 + BS2 个 TQ：
  *     位时间 = (1 + BS1 + BS2) × TQ    例：1+11+2 = 14 TQ = 2us → 500Kbps
  *
  *   Prescaler       分频器，把 42MHz 压成 TQ 的长度
  *   TimeSeg1 = BS1  相位缓冲段1，位于采样点【之前】
  *   TimeSeg2 = BS2  相位缓冲段2，位于采样点【之后】，用来吸收各节点间的时钟偏差
  *   SyncJumpWidth   重同步时一次最多能调整几个 TQ，补偿晶振误差用。这里取 1TQ 最保守。
  *
  * 采样点 = (1 + BS1) / (1 + BS1 + BS2)
  *   500K 档 = 12/14 = 85.7%。这个位置很关键：太靠前抗干扰差，太靠后留给
  *   误差补偿的余量不够。CiA（CAN 应用层标准组织）推荐 87.5%，85.7% 是常用的折中。
  *
  * 一句话记忆：Prescaler 管「一个 TQ 多长」，BS1/BS2 管「一个位切成几段、采样点在哪」。
  */
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

/**
  * 功    能：把 CAN1 拉起来：配位时序 → 装过滤器 → 启动 → 建收发队列 → 开 FIFO0 通知
  * 参    数：无
  * 返 回 值：0 = 成功；1 = 波特率不支持；2 = HAL_CAN_Init 失败；3 = 过滤器配置失败；
  * 补    充：必须在任务上下文调用，内部会创建队列
  *            1 波特率不支持、2 HAL_CAN_Init、3 过滤器、4 启动、5 队列、6 通知使能
  */
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

    /* ── 过滤器在干嘛：决定「哪些帧能进 FIFO」，被拦掉的帧硬件直接丢弃，CPU 根本不知道 ──
     *
     * 工作原理是「ID 与 mask 逐位比较」：
     *   mask 的某一位 = 1 → 这一位必须和 ID 完全一致才算通过
     *   mask 的某一位 = 0 → 这一位不关心（0/1 都放行）
     * 所以 mask 全 0 = 全通（本工程默认），mask 全 1 = 精确匹配一个 ID。
     *
     * STM32 的寄存器排布有个坑：标准 ID（11 位）在 32 位过滤器寄存器里
     * 是存在【最高 11 位】(bit31~21) 的，低 5 位留给扩展帧的标志位。
     * 所以填标准 ID 时必须左移 5 位：
     *     FilterIdHigh = (stdId & 0x7FF) << 5
     * 这就是下面 CAN_FilterSet 里那个 << 5 的由来，忘了移就会「明明填对了却收不到」。
     */
    filter.FilterBank           = 0U;                        /* 过滤组 0；CAN1 可用 0~13，CAN2 用 14~27 */
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;     /* 掩码模式（另一种 IDLIST 是精确列表） */
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;     /* 32 位模式：一次放下完整 ID + 掩码 */
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;                   /* 掩码全 0 = 所有位都不比较，即全通 */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;              /* 挂 FIFO0，对应 RxFifo0MsgPendingCallback */
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;                       /* CAN2 起始过滤组固定为 14，CAN1 用不到 */

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

/**
  * 功    能：把一帧放进 TX 队列就返回 —— 不碰硬件、不等待，真正发出去是 CAN_TxPump 的事
  * 参    数：msg  待发送帧，不允许 NULL
  * 返 回 值：0 = 已入队，1 = 参数非法或队列满
  * 补    充：只在任务上下文调用，不是 FromISR 版本；队列满立即返回不等待
  *            只在任务里调，这是非 FromISR 版本
  */
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

/**
  * 功    能：从 TX 队列取一帧出来准备发送；队列空就阻塞到 timeoutMs（0 = 不阻塞）
  * 参    数：msg  输出帧，不允许 NULL
  *            timeoutMs  阻塞超时，单位 ms，建议 100，0 表示不阻塞
  * 返 回 值：0 = 取到一帧，1 = 参数非法或超时
  * 补    充：仅供 CanTask 调用，超时是正常路径
  */
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

/**
  * 功    能：真正写进硬件邮箱、发到总线上。ide=0 走标准帧；dlc 超过 8 按 8 截断
  * 参    数：msg  待发送帧；ide=0 标准帧，ide!=0 扩展帧；dlc 超过 8 按 8 处理
  * 返 回 值：0 = 已进邮箱，1 = 3 个邮箱全满本帧丢弃，2 = 参数非法或 HAL 发送失败
  * 补    充：仅供 CanTask 调用；tx_total / tx_drop / tx_error 三个计数只在这里写。
  *
  * ── 为什么发送要绕两级（TX 队列 → 硬件邮箱）──
  *
  *   TX 队列  ：软件队列，8 帧。任务把帧丢进去就返回，不等发送完成。
  *   硬件邮箱 ：CAN 外设自带 3 个，是真正的发送缓冲，帧写进去硬件才往总线上推。
  *
  * 分两级的理由：500Kbps 下发一帧 8 字节要 200us 上下，任务不可能站在那等。
  * 队列负责「接住」突发流量并让任务立刻回去干活，邮箱负责「真正发出去」。
  * 邮箱只有 3 个，满了就丢（返回 1），这是设计上的取舍 —— 宁可丢也不阻塞。
  *
  * ── 顺带说清 ACK：这是理解「为什么没接总线就一直重传」的关键 ──
  *
  * 一帧发出去后，接收方会在 ACK 槽里回一个【显性】位，表示「我收到了」。
  * 总线上只要有【任意一个】节点回了 ACK，发送方就认为成功。
  * 如果没有任何节点回（比如分析仪没接、总线断了），发送方判定「ACK 错误」，
  * 自动重传，而且每错一次 TEC +8。
  * 一帧 TEC +8，96 就是警告、128 变被动、256 直接 bus-off ——
  * 所以调试时常见「拔掉分析仪跑一会儿，板子自己下线了」，这不是 bug，是协议在自保。
  * 这也是 CAN_HB_PERIOD_MS 的注释里提醒「调试期可以临时改大到 5000」的原因。
  */
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

/**
  * 功    能：从 RX 队列取一帧收到的数据（中断回调里已经投进队列）
  * 参    数：msg  输出帧，不允许 NULL
  *            timeoutMs  阻塞超时，单位 ms，0 = 非阻塞排空
  * 返 回 值：0 = 取到一帧，1 = 参数非法或队列为空
  * 补    充：仅供 CanTask 调用；ISR 在 HAL_CAN_RxFifo0MsgPendingCallback 里入队
  */
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

/**
  * 功    能：拷一份统计快照：收发计数、TEC/REC、当前总线状态
  * 参    数：stats  输出快照，不允许 NULL
  * 返 回 值：无
  * 补    充：不关中断、不加锁，跨字段允许轻微不一致
  *            不关中断也不加锁，字段之间可能轻微对不上（快照本身非原子）—— 看日志够用，别为它加锁
  */
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

/**
  * 功    能：重设过滤器。mask 位为 1 表示该位必须匹配：0x7FF 精确匹配一个 ID，0x000 全放行
  * 参    数：stdId  标准帧 ID，11 位，范围 0x000~0x7FF
  *            mask  掩码，位 = 1 表示该位必须匹配；0x7FF 精确匹配，0x000 全放行
  * 返 回 值：0 = 配置成功，1 = HAL_CAN_ConfigFilter 失败
  * 补    充：重配会置 FMR.FINIT，期间到期的帧会丢，属 HAL 固有行为
  */
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

/**
  * 功    能：读错误状态寄存器更新内部总线状态（写进 s_stats 的 tec / rec / state）
  * 参    数：无
  * 返 回 值：无，结果写入 s_stats.tec / s_stats.rec / s_stats.state
  * 补    充：仅供 CanTask 周期调用；判定顺序 BOFF > EPVF > EWGF，写反会误报
  *            （已 bus-off 却报 warning）。由 CanTask 每秒调一次。
  *
  * ── TEC / REC 是什么，那几个阈值 96/128/256 为什么这么定 ──
  *
  * TEC = 发送错误计数，REC = 接收错误计数，都由硬件自动加减：
  *   发送出错 TEC +8、发送成功 TEC -1；接收出错 REC +1、接收成功 REC -1。
  *   直觉是「错得越多涨得越快，只有连续正确才慢慢降回来」。
  *
  * 硬件按 TEC/REC 把节点分成四个状态：
  *   错误主动（< 96）      正常状态，能正常收发，出错时发主动错误帧去打断别人。
  *   错误警告（≥ 96）      只是提醒，行为不变，通常说明线路开始有干扰了。
  *   错误被动（≥ 128）     出错时只能发被动错误帧，打不断别人；而且发送时
  *                        只能发隐性 —— 等于自动放弃仲裁权，影响很大。
  *   总线关闭（TEC ≥ 256） 节点主动下线，停止一切收发。恢复要检测到 128 次
  *                        「连续 11 个隐性位」，或者断电重启。这就是 AutoBusOff 要开的原因。
  *
  * 一句话记忆：96 警告 / 128 被动 / 256 下线，全由 TEC 主导。
  * 所以「总线没人 ACK」是最容易把节点打到 bus-off 的场景 —— 每个无 ACK 的帧 TEC 就 +8。
  */
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

/**
  * 功    能：收到帧的中断回调：把帧从硬件 FIFO 搬进 RX 队列就返回
  * 参    数：hcan  触发回调的 CAN 句柄，只处理 Instance == CAN1
  * 返 回 值：无
  * 补    充：ISR 上下文，只搬运不打印、不判业务、不喂狗；队列满只累加 rx_drop
  *            中断里只搬运 —— 不打印、不判业务、不喂狗。队列满时只累加 rx_drop 然后丢掉
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
