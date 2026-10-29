/**
  ******************************************************************************
  * @file    CanTask.c
  * @brief   CAN 通信任务:收帧分发、周期心跳、总线状态轮询、IWDG 心跳上报
  *
  *          任务行为(派活单 §8,单任务轮转,不做双任务):
  *            for (;;) {
  *                心跳上报;
  *                while (RX 队列非空) 分发打印;
  *                if (到心跳周期) 组心跳帧 -> CAN_Send();
  *                if (TX 队列取到帧) CAN_TxPump();
  *                if (到状态轮询周期) 读 ESR,按规则打印;
  *            }
  *          主循环节奏由 CAN_TxPop 的 100ms 超时兜底:无消息时任务阻塞在队列上,
  *          CPU 占用为 0。
  *
  *          心跳(V1.1 真机教训):
  *            每轮循环入口无条件上报;阻塞等待一律用有限超时,禁止 portMAX_DELAY ——
  *            外部无流量时死等会让心跳停摆,进而被 IWDG 咬复位。
  *
  *          总线状态打印采用【非对称去抖】(驻场回信 §1.3):
  *            - 恶化(OK->warning->passive->off):第一次采样立即打印,
  *              安全信息不能延迟;
  *            - 恢复:需连续 2 次稳定采样才打印回迁。
  *            对称去抖会把真实的短时错误吞掉,非对称才对。
  *            另加【限流】(R3):同一状态持续时,距上次打印 >= 2s 才允许再打,
  *            避免 warn<->passive 来回抖动时绕过"变化才打印"刷屏。
  ******************************************************************************
  */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>          /* snprintf: 整行日志先在局部缓冲拼好再发出 */
#include "usart.h"          /* UART_Printf */
#include "bsp_IWDG.h"       /* Watchdog_Heartbeat / WDT_TASK_CAN */
#include "bsp_CAN.h"
#include "CanData.h"

/* 状态轮询与打印限流 */
#define CAN_STAT_POLL_MS        1000U
#define CAN_STAT_PRINT_THROTTLE 2000U   /* R3: 同一状态至少间隔 2s 再打 */

/* 恢复方向去抖所需连续稳定次数 */
#define CAN_RECOVER_STABLE_CNT  2U

/* ====================== 内部状态 ====================== */

static uint16_t      s_txCount   = 0U;
static CanBusState_t s_lastState = CAN_BUS_OK;
static uint8_t       s_stableCnt = 0U;
static uint32_t      s_lastPrint = 0U;

/* ====================== 内部工具 ====================== */

static const char *CAN_StateName(CanBusState_t st)
{
    switch (st)
    {
        case CAN_BUS_WARNING: return "warning";
        case CAN_BUS_PASSIVE: return "passive";
        case CAN_BUS_OFF:     return "off";
        case CAN_BUS_OK:
        default:               return "ok";
    }
}

/**
  * @brief  组心跳帧(大端,8 字节)
  * @note   非 CANopen 兼容,理由见 CanData.h 心跳帧布局注释
  */
static void CAN_BuildHeartbeat(CanMsg_t *msg, uint32_t uptimeSec, const CanStats_t *st)
{
    msg->id        = CAN_ID_HEARTBEAT;
    msg->ide       = 0U;                 /* 标准帧 */
    msg->rtr       = 0U;                 /* 数据帧 */
    msg->dlc       = 8U;
    msg->rsv       = 0U;
    msg->timestamp = xTaskGetTickCount();

    /* 大端: 高字节在前 */
    msg->data[CAN_HB_OFF_CNT_MSB] = (uint8_t)((s_txCount   >> 8) & 0xFFU);
    msg->data[CAN_HB_OFF_CNT_LSB] = (uint8_t)( s_txCount         & 0xFFU);
    msg->data[CAN_HB_OFF_SEC_MSB] = (uint8_t)((uptimeSec   >> 8) & 0xFFU);
    msg->data[CAN_HB_OFF_SEC_LSB] = (uint8_t)( uptimeSec         & 0xFFU);
    msg->data[CAN_HB_OFF_TEC]     = st->tec;
    msg->data[CAN_HB_OFF_REC]     = st->rec;
    msg->data[CAN_HB_OFF_STATE]   = (uint8_t)st->state;
    msg->data[CAN_HB_OFF_RSV]     = 0U;
}

/**
  * @brief  非对称去抖 + 限流的状态打印
  */
static void CAN_PrintStateIfNeeded(CanBusState_t cur, uint8_t tec, uint8_t rec, uint32_t nowMs)
{
    if (cur > s_lastState)
    {
        /* 恶化: 立即上报,不等待稳定 */
        UART_Printf("[CAN] bus=%s tec=%u rec=%u\r\n",
                    CAN_StateName(cur), (unsigned)tec, (unsigned)rec);
        s_lastState = cur;
        s_stableCnt = 0U;
        s_lastPrint = nowMs;
    }
    else if (cur < s_lastState)
    {
        /* 恢复: 需连续 CAN_RECOVER_STABLE_CNT 次稳定采样 */
        s_stableCnt++;
        if (s_stableCnt >= CAN_RECOVER_STABLE_CNT)
        {
            UART_Printf("[CAN] bus=%s tec=%u rec=%u\r\n",
                        CAN_StateName(cur), (unsigned)tec, (unsigned)rec);
            s_lastState = cur;
            s_stableCnt = 0U;
            s_lastPrint = nowMs;
        }
    }
    else
    {
        /* 状态未变: 限流,避免 TEC 在阈值附近抖动时刷屏(R3) */
        if ((nowMs - s_lastPrint) >= CAN_STAT_PRINT_THROTTLE)
        {
            UART_Printf("[CAN] bus=%s tec=%u rec=%u\r\n",
                        CAN_StateName(cur), (unsigned)tec, (unsigned)rec);
            s_lastPrint = nowMs;
        }
        s_stableCnt = 0U;
    }
}

/* ====================== 任务主体 ====================== */

void StartCanTask(void *argument)
{
    (void)argument;

    CanMsg_t   msg;
    CanStats_t stats;
    uint32_t   lastHb   = 0U;
    uint32_t   lastStat = 0U;

    /* BSP 初始化必须在任务上下文(内部创建队列) */
    if (CAN_BspInit() != 0U)
    {
        UART_Printf("[CAN] task abort: BspInit failed\r\n");
    }

#if CAN_TEST_FILTER_EXACT
    /* T4 验收: 精确匹配 0x123,分析仪发其它 ID 应一律不打印。
     * 注意(R6): 运行期改过滤器会置 FMR.FINIT,期间到期的帧会丢,属 HAL 固有行为。 */
    if (CAN_FilterSet(CAN_ID_TEST, 0x7FFU) == 0U)
    {
        UART_Printf("[CAN] T4: filter exact 0x%03lX only\r\n", (unsigned long)CAN_ID_TEST);
    }
#endif

    for (;;)
    {
        Watchdog_Heartbeat(WDT_TASK_CAN);      /* 每轮入口无条件上报 */

        uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* ---- 1. 收帧分发: 非阻塞排空 ---- */
        while (CAN_RxPop(&msg, 0U) == 0U)
        {
            /* 整行先拼进局部缓冲, 再一次 UART_Printf 发出。
             * UART_Printf 保证的是【单次调用】原子: 此前拆成
             * "前缀 + 8 个 %02X + 换行" 共 10 次调用, 中间任何一次被别的
             * 任务插入, 这一行就断成两截 —— 现场 [CANSOR] 之类花屏的直接来源。
             * 边界: 每字节占 3 字符("XX "), 8 字节共 24, 缓冲 32 足够;
             * 仍用 p < sizeof-4 卡住, 防 snprintf 写越界。 */
            char hex[32];
            int  p = 0;
            for (uint8_t i = 0U; i < msg.dlc && p < (int)sizeof(hex) - 4; i++)
            {
                p += snprintf(&hex[p], sizeof(hex) - (size_t)p, "%02X ",
                              (unsigned)msg.data[i]);
            }
            UART_Printf("[CAN] rx id=0x%03lX dlc=%u data=%s\r\n",
                        (unsigned long)msg.id, (unsigned)msg.dlc, hex);
        }

        /* ---- 2. 周期心跳帧 ---- */
        if ((nowMs - lastHb) >= CAN_HB_PERIOD_MS)
        {
            lastHb = nowMs;
            CAN_GetStats(&stats);
            CAN_BuildHeartbeat(&msg, nowMs / 1000U, &stats);
            if (CAN_Send(&msg) == 0U)
            {
                s_txCount++;
            }
            else
            {
                UART_Printf("[CAN] tx queue full, heartbeat dropped\r\n");
            }
        }

        /* ---- 3. 发送: 100ms 超时兜底主循环节奏 ---- */
        if (CAN_TxPop(&msg, 100U) == 0U)
        {
            if (CAN_TxPump(&msg) == 0U)
            {
                UART_Printf("[CAN] tx id=0x%03lX dlc=%u ok\r\n",
                            (unsigned long)msg.id, (unsigned)msg.dlc);
            }
            /* 邮箱满/失败时由 BSP 累计 tx_drop / tx_error,这里不打印,避免刷屏 */
        }

        /* ---- 4. 状态轮询(1s) ---- */
        if ((nowMs - lastStat) >= CAN_STAT_POLL_MS)
        {
            lastStat = nowMs;
            CAN_UpdateBusState();
            CAN_GetStats(&stats);
            CAN_PrintStateIfNeeded(stats.state, stats.tec, stats.rec, nowMs);
        }
    }
}

/* ====================== 对外创建接口 ====================== */

void CanTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name       = "CanTask",
        .stack_size = 256 * 4,                          /* 派活单 §7 定档, 验收时看栈水位 */
        .priority   = (osPriority_t)osPriorityNormal,   /* 与 ModbusTask 同级 */
    };
    (void)osThreadNew(StartCanTask, NULL, &attr);
}
