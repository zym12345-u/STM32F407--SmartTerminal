/* CanTask.c
 * CAN 通信任务：收帧分发、1s 心跳 0x701、1s 传感器帧 0x181、1s 总线状态轮询。
 * 周期：主循环节奏由 CAN_TxPop 的 100ms 超时决定；优先级 Normal，栈 1024 字节。
 * 注意：所有阻塞等待必须有界，禁止 portMAX_DELAY，否则外部无流量时心跳停摆被 IWDG 复位。
 */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>          /* snprintf: 整行日志先在局部缓冲拼好再发出 */
#include "usart.h"          /* UART_Printf */
#include "bsp_IWDG.h"       /* Watchdog_Heartbeat / WDT_TASK_CAN */
#include "bsp_CAN.h"
#include "CanData.h"
#include "SensorData.h"     /*g_sensorData 是唯一数据源，不另采不另存*/
#include "LEDType.h"        /* LEDMessage: 命令下行复用既有 LED 通道 */

/* 状态轮询与打印限流 */
#define CAN_STAT_POLL_MS        1000U
#define CAN_STAT_PRINT_THROTTLE 2000U   /*同一状态至少间隔 2s 再打*/

/* 恢复方向去抖所需连续稳定次数 */
#define CAN_RECOVER_STABLE_CNT  2U

/*内部状态*/

static uint16_t      s_txCount   = 0U;
static CanBusState_t s_lastState = CAN_BUS_OK;
static uint8_t       s_stableCnt = 0U;
static uint32_t      s_lastPrint = 0U;

/*总线状态机：由 ESR 的 EWGF/EPVF/BOFF 判定，BOFF 优先，采样周期 1000ms
 * 状态            含义                        进入条件         超时/恢复
 * CAN_BUS_OK      正常收发                    TEC/REC < 96     无
 * CAN_BUS_WARNING 重传/错误帧变多              TEC 或 REC >= 96 回落连续 2 次采样回 OK
 * CAN_BUS_PASSIVE 只能被动应答                计数继续上升     恶化立即打印
 * CAN_BUS_OFF     总线关闭，停收发             TEC >= 256       需复位 CAN 外设
 * 打印：同一状态持续限流 2000ms。
 */

/*内部工具*/

/* 把总线状态枚举翻成日志里用的短名：ok / warning / passive / off。
 * 返回的是静态字符串，调用方别去改它。 */
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

/* 组心跳帧（0x701）：发送计数、运行秒数、TEC/REC、总线状态，8 字节全覆写。
 * 格式不是 CANopen 心跳（data[0] 放的是发送计数而非 NMT 状态），理由见 CanData.h。 */
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

/* 总线状态变了才打一行日志，用非对称去抖防止刷屏：
 * 恶化（ok→warning→passive→off）立刻报；恢复要连续 2 次稳定采样才报；
 * 状态没变则 2s 限流 —— TEC 在 96/128 阈值附近抖动时不会把日志刷爆。 */
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

/*LEDQueue 元素是指针：LEDTask 负责点灯并 vPortFree，本任务只当生产者*/
extern osMessageQueueId_t LEDQueueHandle;

/* 往 LEDQueue 投一条点灯指令，由 LEDTask 真正执行。
 * 队列元素是 LEDMessage 指针，所以要 malloc；堆耗尽或队列满就静默丢弃，
 * 不能让一次点灯失败拖累 CAN 应答。 */
static void CAN_LedPost(LEDColor color, LEDState state)
{
    LEDMessage *msg = (LEDMessage *)pvPortMalloc(sizeof(LEDMessage));
    if (msg == NULL)
    {
        return;                            /* 堆耗尽: 丢弃本次控制, 不影响 CAN 应答 */
    }
    msg->color = color;
    msg->state = state;
    if (osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
        vPortFree(msg);                    /* 队列满: 释放并丢弃 */
    }
}

/* 组传感器帧（0x181）：读全局 g_sensorData，8 字节先清零再逐字段填。
 * 编码与 Modbus 的 MB_RegRead 完全一致（温湿度光照均 x10，无效填 0xFFFF），
 * 这样拿 Modbus Poll 和 PCAN-View 对照打时两边能一字不差。 */
static void CAN_BuildSensorFrame(CanMsg_t *msg)
{
    uint16_t temp10;
    uint16_t hum10;
    uint16_t lux10;
    uint8_t  valid = 0U;

    /*字段全填：rsv/timestamp 不填会带栈垃圾进队列*/
    msg->id        = CAN_ID_SENSOR;
    msg->ide       = 0U;
    msg->rtr       = 0U;
    msg->dlc       = 8U;
    msg->rsv       = 0U;
    msg->timestamp = xTaskGetTickCount();
    for (uint8_t i = 0U; i < 8U; i++)
    {
        msg->data[i] = 0U;                 /* 先全清, 再逐字段写 */
    }

    /*温湿度共用 temp_valid，故 valid 的 bit0/bit1 恒同值*/
    if (g_sensorData.temp_valid)
    {
        temp10 = (uint16_t)(int16_t)(g_sensorData.temperature * 10.0f);
        hum10  = (uint16_t)(int16_t)(g_sensorData.humidity    * 10.0f);
        valid |= (CAN_SENSOR_VALID_TEMP | CAN_SENSOR_VALID_HUM);
    }
    else
    {
        temp10 = 0xFFFFU;
        hum10  = 0xFFFFU;
    }

    /*光照超量程钳到 0xFFFF，与 Modbus 编码一致*/
    if (g_sensorData.light_valid)
    {
        uint32_t lux32 = (uint32_t)(g_sensorData.light * 10.0f);
        lux10 = (lux32 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)lux32;
        valid |= CAN_SENSOR_VALID_LIGHT;
    }
    else
    {
        lux10 = 0xFFFFU;
    }

    /*大端写入*/
    msg->data[CAN_SENSOR_OFF_TEMP_MSB] = (uint8_t)((temp10 >> 8) & 0xFFU);
    msg->data[CAN_SENSOR_OFF_TEMP_LSB] = (uint8_t)( temp10       & 0xFFU);
    msg->data[CAN_SENSOR_OFF_HUM_MSB]  = (uint8_t)((hum10  >> 8) & 0xFFU);
    msg->data[CAN_SENSOR_OFF_HUM_LSB]  = (uint8_t)( hum10        & 0xFFU);
    msg->data[CAN_SENSOR_OFF_LUX_MSB]  = (uint8_t)((lux10  >> 8) & 0xFFU);
    msg->data[CAN_SENSOR_OFF_LUX_LSB]  = (uint8_t)( lux10        & 0xFFU);
    msg->data[CAN_SENSOR_OFF_VALID]    = valid;
    msg->data[CAN_SENSOR_OFF_DEVID]    = CAN_SENSOR_DEVID;
}

/*命令/应答协议：0x201 下行 → 0x281 上行
 * 0x201  data[0] 命令码 0x01 开灯 / 0x02 关灯 / 0x03 读版本，其余回 NAK
 * 0x201  data[1] 灯编号 1=绿 2=蓝，内部枚举 0=绿 1=蓝，需映射
 * 0x201  data[2..7] 预留填 0
 * 0x281  data[0] 回显命令码，data[1] 结果 0 成功 / 1 未知命令 / 2 参数错误
 * 0x281  data[2..3] 版本号，DLC 固定 4 但 data[4..7] 也必须清零
 * 时序：收到即处理并回帧，不重传、不超时判定
 */

/* 执行一条下行命令（0x201），并把应答帧（0x281）组好。
 * rx 只读；ack 全部字段覆写，其中 data[1] 是结果码（0 成功 / 1 不支持 / 2 参数错）。
 * 注意 ack 是 DLC=4 但缓冲 8 字节，data[4..7] 必须清零，否则会露出上一帧尾巴。 */
static void CAN_HandleCommand(const CanMsg_t *rx, CanMsg_t *ack)
{
    uint8_t code  = rx->data[CAN_CMD_OFF_CODE];
    uint8_t param = rx->data[CAN_CMD_OFF_PARAM];
    uint8_t result;

    ack->id        = CAN_ID_CMD_ACK;
    ack->ide       = 0U;
    ack->rtr       = 0U;
    ack->dlc       = CAN_ACK_DLC;                    /* 应答固定 DLC=4 */
    ack->rsv       = 0U;
    ack->timestamp = xTaskGetTickCount();
    for (uint8_t i = 0U; i < 8U; i++)
    {
        ack->data[i] = 0U;
    }

    switch (code)
    {
        case CAN_CMD_LED_ON:
        case CAN_CMD_LED_OFF:
        {
            /* 灯编号 1=绿 2=蓝; LEDType.h 的枚举是 0=绿 1=蓝, 故做一次映射 */
            LEDColor color;
            if (param == CAN_LED_GREEN)      { color = LEDColor_Green; }
            else if (param == CAN_LED_BLUE)  { color = LEDColor_Blue;  }
            else
            {
                result = CAN_ACK_PARAM;
                break;
            }
            CAN_LedPost(color, (code == CAN_CMD_LED_ON) ? LEDState_On : LEDState_Off);
            result = CAN_ACK_OK;
            break;
        }
        case CAN_CMD_GET_VERSION:
        {
            result = CAN_ACK_OK;             /* 版本号在下面统一回填 */
            break;
        }
        default:
        {
            result = CAN_ACK_NAK;            /* 不支持的命令码 */
            break;
        }
    }

    ack->data[CAN_ACK_OFF_CODE]   = code;
    ack->data[CAN_ACK_OFF_RESULT] = result;
    ack->data[CAN_ACK_OFF_VER_H]  = CAN_FW_VER_MAJOR;
    ack->data[CAN_ACK_OFF_VER_L]  = CAN_FW_VER_MINOR;
    /*DLC=4 但缓冲有 8 字节，data[4..7] 不清会露出上一帧尾巴*/
}

/*任务主体*/

/* CAN 任务主循环，每轮做五件事：排空收到的帧 → 该发心跳就发 →
 * 该发传感器就发 → 从 TX 队列取一帧交给硬件 → 每秒轮询一次总线状态。
 * argument 是 RTOS 传进来的，用不上。 */
void StartCanTask(void *argument)
{
    (void)argument;

    CanMsg_t   msg;
    CanStats_t stats;
    uint32_t   lastHb     = 0U;
    uint32_t   lastStat   = 0U;
    uint32_t   lastSensor = 0U;

    /*BSP 初始化须在任务上下文：内部要创建 RX/TX 队列*/
    if (CAN_BspInit() != 0U)
    {
        UART_Printf("[CAN] task abort: BspInit failed\r\n");
    }

#if CAN_TEST_FILTER_EXACT
    /*只在 0x123 通过；运行期改过滤器置 FMR.FINIT，其间到期的帧会丢*/
    if (CAN_FilterSet(CAN_ID_TEST, 0x7FFU) == 0U)
    {
        UART_Printf("[CAN] T4: filter exact 0x%03lX only\r\n", (unsigned long)CAN_ID_TEST);
    }
#endif

    for (;;)
    {
        /*WDT 槽位 9(CAN)*/
        Watchdog_Heartbeat(WDT_TASK_CAN);      /*每轮入口无条件上报*/

        uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;   /*tick 折算成 ms*/

        /*收帧分发：超时 0 一次排空*/
        while (CAN_RxPop(&msg, 0U) == 0U)                  /* 超时 0:一次排空,不阻塞 */
        {
            /*整行拼好一次发出：UART_Printf 只保证单次调用原子，分多次打会被别的任务插进半行*/
            char hex[32] = {0};   /*必须清零：dlc=0 时循环不执行，未初始化栈内容会被 %s 读到随机 0x00*/
            int  p = 0;
            for (uint8_t i = 0U; i < msg.dlc && p < (int)sizeof(hex) - 4; i++)
            {
                p += snprintf(&hex[p], sizeof(hex) - (size_t)p, "%02X ",
                              (unsigned)msg.data[i]);
            }
            UART_Printf("[CAN] rx id=0x%03lX dlc=%u data=%s\r\n",
                        (unsigned long)msg.id, (unsigned)msg.dlc, hex);

            /*按 ID 分发：0x201 走命令处理并回 0x281，其余只打印*/
            if (msg.id == CAN_ID_CMD)
            {
                CanMsg_t ack;
                CAN_HandleCommand(&msg, &ack);
                if (CAN_Send(&ack) == 0U)
                {
                    s_txCount++;
                    UART_Printf("[CAN] cmd 0x%02X -> ack result=%u\r\n",
                                (unsigned)ack.data[CAN_ACK_OFF_CODE],
                                (unsigned)ack.data[CAN_ACK_OFF_RESULT]);
                }
                else
                {
                    UART_Printf("[CAN] tx queue full, cmd ack dropped\r\n");
                }
            }
        }

        /*周期心跳帧*/
        if ((nowMs - lastHb) >= CAN_HB_PERIOD_MS)          /*1s 心跳帧*/
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

        /*与心跳同为 1s，两帧背靠背约占 0.5ms；错相会让 lastSensor 初值大于 nowMs 时 uint32 减法下溢误发*/
        if ((nowMs - lastSensor) >= CAN_SENSOR_PERIOD_MS)  /*1s 传感器帧*/
        {
            lastSensor = nowMs;
            CAN_BuildSensorFrame(&msg);
            if (CAN_Send(&msg) != 0U)
            {
                UART_Printf("[CAN] tx queue full, sensor dropped\r\n");
            }
        }

        if (CAN_TxPop(&msg, 100U) == 0U)                   /*100ms：出队超时也是主循环节奏*/
        {
            if (CAN_TxPump(&msg) == 0U)
            {
                UART_Printf("[CAN] tx id=0x%03lX dlc=%u ok\r\n",
                            (unsigned long)msg.id, (unsigned)msg.dlc);
            }
            /*邮箱满/失败由 BSP 累计 tx_drop / tx_error，不打印避免刷屏*/
        }

        /*状态轮询：1000ms 读 ESR*/
        if ((nowMs - lastStat) >= CAN_STAT_POLL_MS)
        {
            lastStat = nowMs;
            CAN_UpdateBusState();
            CAN_GetStats(&stats);
            CAN_PrintStateIfNeeded(stats.state, stats.tec, stats.rec, nowMs);
        }
    }
}

/*对外创建接口*/

/* 创建 CAN 任务，由 freertos.c 在启动调度器前调一次。 */
void CanTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name       = "CanTask",
        .stack_size = 256 * 4,                          /*栈以字为单位：256 字 = 1024 字节*/
        .priority   = (osPriority_t)osPriorityNormal,   /*与 ModbusTask 同级*/
    };
    (void)osThreadNew(StartCanTask, NULL, &attr);
}
