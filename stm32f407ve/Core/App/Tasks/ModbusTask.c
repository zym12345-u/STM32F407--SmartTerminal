/**
  ******************************************************************************
  * @file    ModbusTask.c
  * @brief   Modbus RTU 从站任务(功能码 03/06/16, 异常响应, CRC16 校验)
  *
  *          架构说明:
  *            bsp_RS485(USART2 中断收) -> 环形缓冲
  *            -> 本任务每 10ms 轮询: 字节数稳定(帧间隔 >= 3.5T)即取出一帧
  *            -> 校验从站地址/CRC -> 按功能码分发 -> 组响应帧 -> RS485_Send
  *
  *          保持寄存器映射(从站地址 1, 9600-8-N-1):
  *            0x0000  温度 x10    (RO, 传感器无效时读到 0xFFFF)
  *            0x0001  湿度 x10    (RO, 同上)
  *            0x0002  光照 x10    (RO, 同上)
  *            0x0003~0x000F       保留, 读 0xFFFF, 写返回非法地址异常
  *            0x0010  绿灯状态    (RW, 1=亮 0=灭, 写后经 LEDQueue 生效)
  *            0x0011  蓝灯状态    (RW, 同上)
  *
  *          已知限制(v1, 刻意简化):
  *            - 帧提取按"空闲整缓冲"一次取完, 依赖主站遵守帧间隔 >= 3.5T。
  *              若未来要支持主站背靠背请求, 应改为按功能码推算期望帧长、
  *              每次只消费一帧(改进点已在面试可讲)
  *            - 暂未实现线圈(01/05)与输入寄存器(04), LED 用保持寄存器代替
  ******************************************************************************
  */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"

#include "bsp_RS485.h"
#include "SensorData.h"
#include "LEDType.h"

/* LEDQueue 由 freertos.c 创建, 消费者是 LEDTask(收到指针后 vPortFree) */
extern osMessageQueueId_t LEDQueueHandle;

/* ====================== 参数定义 ====================== */
#define MB_SLAVE_ADDR           1U      /* 本从站地址 */
#define MB_FRAME_MAX            256U    /* Modbus RTU 一帧最长 256 字节 */
#define MB_POLL_INTERVAL_MS     10U     /* 轮询周期; 帧内字节间隔 3.5T@9600≈4ms,
                                           连续两次轮询计数不变即认为整帧到达 */
#define MB_REG_MAP_END          0x0012U /* 有效寄存器区间 [0, 0x0012) */
#define MB_REG_GREEN            0x0010U
#define MB_REG_BLUE             0x0011U

/* 功能码 */
#define MB_FUNC_READ_HOLDING    0x03U
#define MB_FUNC_WRITE_SINGLE    0x06U
#define MB_FUNC_WRITE_MULTI     0x10U

/* 异常码 */
#define MB_EX_ILLEGAL_FUNCTION  0x01U
#define MB_EX_ILLEGAL_ADDRESS   0x02U
#define MB_EX_ILLEGAL_VALUE     0x03U

/* ====================== 内部状态 ====================== */
/* LED 影子寄存器: 写操作经 LEDQueue 异步生效, 读操作直接返回影子值 */
static volatile uint16_t s_regGreen = 0U;
static volatile uint16_t s_regBlue  = 0U;

/* ====================== CRC16(Modbus) ======================
 * 多项式 0x8005 反射为 0xA001, 初值 0xFFFF, 结果低字节在前发往总线。
 * 此处用位迭代法: 每字节 8 次循环, 256 字节帧约 2k 次循环, 微秒级,
 * 换查表法仅省 ~100us 却多占 512B Flash, 从站负载下无必要。 */
static uint16_t MB_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  j;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ====================== 寄存器读取 ====================== */
static uint16_t MB_RegRead(uint16_t reg)
{
    switch (reg)
    {
        case 0x0000U:   /* 温度 x10, 无效返回 0xFFFF */
            return g_sensorData.temp_valid ? (uint16_t)(int16_t)(g_sensorData.temperature * 10.0f) : 0xFFFFU;
        case 0x0001U:   /* 湿度 x10 */
            return g_sensorData.temp_valid ? (uint16_t)(int16_t)(g_sensorData.humidity * 10.0f) : 0xFFFFU;
        case 0x0002U:   /* 光照 x10, 上限 6553.5 lux */
            if (!g_sensorData.light_valid) return 0xFFFFU;
        {
            uint32_t lux10 = (uint32_t)(g_sensorData.light * 10.0f);
            return (lux10 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)lux10;
        }
        case MB_REG_GREEN:
            return s_regGreen;
        case MB_REG_BLUE:
            return s_regBlue;
        default:        /* 保留区间 */
            return 0xFFFFU;
    }
}

/* ====================== LED 写入(经队列, 生产者分配/消费者释放) ====================== */
static void MB_LedWrite(uint16_t reg, uint16_t value)
{
    LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));
    if (msg == NULL)
    {
        return;                         /* 堆耗尽: 丢弃本次控制, 不影响 Modbus 响应 */
    }

    if (reg == MB_REG_GREEN) { msg->color = LEDColor_Green; s_regGreen = value; }
    else                     { msg->color = LEDColor_Blue;  s_regBlue  = value; }
    msg->state = (value != 0U) ? LEDState_On : LEDState_Off;

    if (osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
        vPortFree(msg);                 /* 队列满: 释放并丢弃 */
    }
}

/* ====================== 各功能码处理 ======================
 * 统一约定: 入参为整帧(含地址与CRC), 输出响应到 resp, 返回响应长度;
 * 广播帧(地址0)只执行写操作并返回 0(不回应答) */
static uint16_t MB_OnReadHolding(const uint8_t *req, uint16_t reqLen, uint8_t *resp)
{
    uint16_t start, qty, i;

    if (reqLen != 8U) return 0U;                        /* 帧长固定 8 */
    start = (uint16_t)((uint16_t)req[2] << 8 | req[3]);
    qty   = (uint16_t)((uint16_t)req[4] << 8 | req[5]);

    if (qty < 1U || qty > 125U || (uint32_t)start + qty > MB_REG_MAP_END)
    {
        resp[1] = (uint8_t)(MB_FUNC_READ_HOLDING | 0x80U);
        resp[2] = (start + qty > MB_REG_MAP_END) ? MB_EX_ILLEGAL_ADDRESS
                                                 : MB_EX_ILLEGAL_VALUE;
        return 3U;                                      /* 地址+异常码, CRC 由调用方补 */
    }

    resp[1] = MB_FUNC_READ_HOLDING;
    resp[2] = (uint8_t)(qty * 2U);
    for (i = 0U; i < qty; i++)
    {
        uint16_t v = MB_RegRead((uint16_t)(start + i));
        resp[3U + i * 2U]     = (uint8_t)(v >> 8);
        resp[3U + i * 2U + 1U] = (uint8_t)(v & 0xFFU);
    }
    return (uint16_t)(3U + qty * 2U);
}

static uint16_t MB_OnWriteSingle(const uint8_t *req, uint16_t reqLen,
                                 uint8_t *resp, uint8_t isBroadcast)
{
    uint16_t reg, val;

    if (reqLen != 8U) return 0U;
    reg = (uint16_t)((uint16_t)req[2] << 8 | req[3]);
    val = (uint16_t)((uint16_t)req[4] << 8 | req[5]);

    if (reg != MB_REG_GREEN && reg != MB_REG_BLUE)
    {
        if (!isBroadcast)
        {
            resp[1] = (uint8_t)(MB_FUNC_WRITE_SINGLE | 0x80U);
            resp[2] = MB_EX_ILLEGAL_ADDRESS;
            return 3U;
        }
        return 0U;
    }

    if (!isBroadcast)
    {
        MB_LedWrite(reg, val);
        (void)memcpy(resp, req, 8U);                    /* 06 正常响应=原样回显请求 */
        return 8U;
    }

    MB_LedWrite(reg, val);
    return 0U;
}

static uint16_t MB_OnWriteMulti(const uint8_t *req, uint16_t reqLen,
                                uint8_t *resp, uint8_t isBroadcast)
{
    uint16_t start, qty, byteCnt, i;

    if (reqLen < 9U) return 0U;
    start   = (uint16_t)((uint16_t)req[2] << 8 | req[3]);
    qty     = (uint16_t)((uint16_t)req[4] << 8 | req[5]);
    byteCnt = req[6];

    if (qty < 1U || qty > 123U || byteCnt != qty * 2U ||
        (uint32_t)start + qty > MB_REG_MAP_END || (uint32_t)reqLen != 9U + byteCnt)
    {
        if (!isBroadcast)
        {
            resp[1] = (uint8_t)(MB_FUNC_WRITE_MULTI | 0x80U);
            resp[2] = ((uint32_t)start + qty > MB_REG_MAP_END) ? MB_EX_ILLEGAL_ADDRESS
                                                               : MB_EX_ILLEGAL_VALUE;
            return 3U;
        }
        return 0U;
    }

    for (i = 0U; i < qty; i++)
    {
        uint16_t reg = (uint16_t)(start + i);
        uint16_t val = (uint16_t)((uint16_t)req[7U + i * 2U] << 8 | req[8U + i * 2U]);
        if (reg == MB_REG_GREEN || reg == MB_REG_BLUE)
        {
            MB_LedWrite(reg, val);
        }
        /* 保留寄存器(RO): 静默忽略, 保证批量写整体成功(与常见工业设备行为一致) */
    }

    if (!isBroadcast)
    {
        resp[1] = MB_FUNC_WRITE_MULTI;
        resp[2] = req[2];
        resp[3] = req[3];
        resp[4] = req[4];
        resp[5] = req[5];
        return 6U;                                      /* 地址+功能码+起始+数量 */
    }
    return 0U;
}

/* ====================== 帧处理入口 ====================== */
static void MB_ProcessFrame(const uint8_t *frame, uint16_t len)
{
    uint8_t  resp[MB_FRAME_MAX];
    uint16_t respLen = 0U;
    uint16_t crc;
    uint8_t  isBroadcast = (frame[0] == 0U);

    /* 从站地址过滤: 非本机且非广播, 直接丢弃 */
    if (!isBroadcast && frame[0] != MB_SLAVE_ADDR)
    {
        UART_Printf("[MB] addr=%u not mine, drop\r\n", (unsigned)frame[0]);
        return;
    }

    switch (frame[1])
    {
        case MB_FUNC_READ_HOLDING:
            respLen = MB_OnReadHolding(frame, len, resp);
            break;
        case MB_FUNC_WRITE_SINGLE:
            respLen = MB_OnWriteSingle(frame, len, resp, isBroadcast);
            break;
        case MB_FUNC_WRITE_MULTI:
            respLen = MB_OnWriteMulti(frame, len, resp, isBroadcast);
            break;
        default:                                        /* 不支持的功能码 */
            if (!isBroadcast)
            {
                resp[1] = (uint8_t)(frame[1] | 0x80U);
                resp[2] = MB_EX_ILLEGAL_FUNCTION;
                respLen = 3U;
            }
            break;
    }

    if (respLen == 0U)                                  /* 广播或异常丢弃, 不应答 */
    {
        return;
    }

    resp[0] = MB_SLAVE_ADDR;                            /* 应答永远用本站地址 */
    crc = MB_Crc16(resp, respLen);
    resp[respLen++] = (uint8_t)(crc & 0xFFU);           /* CRC 低字节在前 */
    resp[respLen++] = (uint8_t)(crc >> 8);

    RS485_Send(resp, respLen);
}

/* ====================== 任务主体 ====================== */
void StartModbusTask(void *argument)
{
    (void)argument;
    uint16_t lastCnt = 0U;
    uint8_t  frame[MB_FRAME_MAX];

    RS485_Init();
    UART_Printf("[MODBUS] RS485 slave ok (USART2 PA2/PA3, %u-8-N-1, addr=%u)\r\n",
                (unsigned)RS485_BAUDRATE, (unsigned)MB_SLAVE_ADDR);

    for (;;)
    {
        uint16_t cnt = RS485_RxCount();

        if (cnt == 0U)
        {
            lastCnt = 0U;
        }
        else if (cnt == lastCnt)
        {
            /* 连续两次轮询(>=10ms)字节数不变 => 帧间隔已过(3.5T@9600=4ms),
             * 整帧到达; 超过单帧上限视为噪声直接冲掉 */
            if (cnt > MB_FRAME_MAX)
            {
                RS485_RxFlush();
                lastCnt = 0U;
            }
            else
            {
                uint16_t i;
                static char dbg[48];                        /* 诊断用 static, 不占栈 */
                int dbgLen = 0;

                for (i = 0U; i < cnt; i++)
                {
                    (void)RS485_ReadByte(&frame[i]);
                }

                /* 诊断打印(临时): 从站听到的原始帧(hex, 最多 12 字节) */
                for (i = 0U; i < cnt && i < 12U; i++)
                {
                    dbgLen += snprintf(&dbg[dbgLen], sizeof(dbg) - (size_t)dbgLen,
                                       "%02X ", frame[i]);
                }
                UART_Printf("[MB] rx %uB: %s%s\r\n",
                            (unsigned)cnt, dbg, (cnt > 12U) ? "..." : "");

                /* 最短合法帧 = 地址+功能码+CRC(2) = 4 字节; CRC 错整帧丢弃 */
                if (cnt >= 4U)
                {
                    uint16_t crcCalc = MB_Crc16(frame, (uint16_t)(cnt - 2U));
                    uint16_t crcRecv = (uint16_t)((uint16_t)frame[cnt - 1U] << 8 | frame[cnt - 2U]);
                    if (crcCalc == crcRecv)
                    {
                        MB_ProcessFrame(frame, cnt);
                    }
                    else
                    {
                        UART_Printf("[MB] CRC bad calc=%04X recv=%04X\r\n",
                                    (unsigned)crcCalc, (unsigned)crcRecv);
                    }
                }
                else
                {
                    UART_Printf("[MB] frame too short: %uB\r\n", (unsigned)cnt);
                }
                lastCnt = 0U;
            }
        }
        else
        {
            lastCnt = cnt;                              /* 还在收, 继续等帧间隔 */
        }

        (void)osDelay(MB_POLL_INTERVAL_MS);
    }
}

/* ====================== 对外创建接口 ====================== */
void ModbusTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name = "ModbusTask",
        .stack_size = 512 * 4,                          /* 帧缓冲在静态区, 栈给足裕量 */
        .priority = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(StartModbusTask, NULL, &attr);
}
