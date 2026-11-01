/* ModbusTask.c
 * Modbus RTU 从站：USART3(RS485) 收帧，校验地址/CRC 后按功能码 03/06/16 应答；
 * 帧结束由 IDLE 中断通知本任务，无请求时任务挂起。
 */
/* 保持寄存器映射(从站地址 1，9600-8-N-1)
 * 地址         内容      读写  取值
 * 0x0000       温度 x10  RO    传感器无效时 0xFFFF
 * 0x0001       湿度 x10  RO    同上
 * 0x0002       光照 x10  RO    同上
 * 0x0003~0x000F         保留  读 0xFFFF，写回非法地址异常
 * 0x0010       绿灯状态  RW    1=亮 0=灭
 * 0x0011       蓝灯状态  RW    同上
 */
/*按空闲整缓冲一次取完，需主站遵守帧间隔 >= 3.5T；01/05 线圈与 04 输入寄存器未实现，LED 用保持寄存器*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"

#include "bsp_RS485.h"
#include "bsp_IWDG.h"
#include "SensorData.h"
#include "LEDType.h"

/*队列由 freertos.c 创建，LEDTask 消费后释放*/
extern osMessageQueueId_t LEDQueueHandle;

/*参数定义*/
#define MB_SLAVE_ADDR           1U      /*本从站地址*/
#define MB_FRAME_MAX            256U    /*一帧最长 256 字节*/
#define MB_REG_MAP_END          0x0012U /*有效寄存器区间 [0, 0x0012)*/
#define MB_REG_GREEN            0x0010U /*绿灯寄存器地址*/
#define MB_REG_BLUE             0x0011U /*蓝灯寄存器地址*/

/*功能码*/
#define MB_FUNC_READ_HOLDING    0x03U   /*03 读保持寄存器*/
#define MB_FUNC_WRITE_SINGLE    0x06U   /*06 写单寄存器*/
#define MB_FUNC_WRITE_MULTI     0x10U   /*16 写多寄存器*/

/*异常码*/
#define MB_EX_ILLEGAL_FUNCTION  0x01U   /*01 非法功能码*/
#define MB_EX_ILLEGAL_ADDRESS   0x02U   /*02 非法地址*/
#define MB_EX_ILLEGAL_VALUE     0x03U   /*03 非法数值*/

/*内部状态*/
/*LED 影子寄存器：写经 LEDQueue 异步生效，读直接返回影子值*/
static volatile uint16_t s_regGreen = 0U;   /*绿灯光态影子，本任务写，读寄存器时取*/
static volatile uint16_t s_regBlue  = 0U;   /*蓝灯光态影子，本任务写，读寄存器时取*/

/*CRC16(Modbus)：多项式 0x8005 反射为 0xA001，初值 0xFFFF，发送时低字节在前*/
/*位迭代法微秒级；查表法省 ~100us 但多占 512B Flash，不划算*/
/**
  * 函    数：MB_Crc16
  * 参    数：data  待校验数据首地址(不含 CRC)；len  字节数，0~256
  * 返 回 值：16 位 CRC，发往总线时低字节先发
  */
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

/*寄存器读取*/
/**
  * 函    数：MB_RegRead
  * 参    数：reg  寄存器地址，0x0000~0x0011
  * 返 回 值：温湿度/光照为 ×10 定点，无效 0xFFFF；0x0010/0x0011 返回影子值
  */
static uint16_t MB_RegRead(uint16_t reg)
{
    switch (reg)
    {
        case 0x0000U:   /*温度 x10，无效 0xFFFF*/
            return g_sensorData.temp_valid ? (uint16_t)(int16_t)(g_sensorData.temperature * 10.0f) : 0xFFFFU;
        case 0x0001U:   /*湿度 x10，与温度共用 temp_valid*/
            return g_sensorData.temp_valid ? (uint16_t)(int16_t)(g_sensorData.humidity * 10.0f) : 0xFFFFU;
        case 0x0002U:   /*光照 x10，上限 6553.5 lux*/
            if (!g_sensorData.light_valid) return 0xFFFFU;
        {
            uint32_t lux10 = (uint32_t)(g_sensorData.light * 10.0f);
            return (lux10 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)lux10;
        }
        case MB_REG_GREEN:
            return s_regGreen;
        case MB_REG_BLUE:
            return s_regBlue;
        default:        /*保留区间，读 0xFFFF*/
            return 0xFFFFU;
    }
}

/*LED 写入：经队列，生产者分配、消费者释放*/
/**
  * 函    数：MB_LedWrite
  * 参    数：reg  只接受 0x0010/0x0011；value  非 0=亮，0=灭
  * 返 回 值：无
  */
static void MB_LedWrite(uint16_t reg, uint16_t value)
{
    LEDMessage *msg = pvPortMalloc(sizeof(LEDMessage));
    if (msg == NULL)
    {
        return;                         /*堆耗尽丢弃控制，写仍算成功*/
    }

    if (reg == MB_REG_GREEN) { msg->color = LEDColor_Green; s_regGreen = value; }
    else                     { msg->color = LEDColor_Blue;  s_regBlue  = value; }
    msg->state = (value != 0U) ? LEDState_On : LEDState_Off;

    if (osMessageQueuePut(LEDQueueHandle, &msg, 0U, 0U) != osOK)
    {
        vPortFree(msg);                 /*队列满释放丢弃，写仍算成功*/
    }
}

/*各功能码处理：入参为整帧(含地址与 CRC)，响应写 resp 并返回长度；广播只写不应答*/
/**
  * 函    数：MB_OnReadHolding
  * 参    数：req  请求整帧；reqLen  固定 8；resp  响应缓冲(CRC 由调用方补)
  * 返 回 值：3+2×qty(qty 1~125 且 start+qty 不越 0x0012)；非法返回 3；帧长错返回 0
  */
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
        return 3U;                                      /*地址+异常码，CRC 由调用方补*/
    }

    resp[1] = MB_FUNC_READ_HOLDING;
    resp[2] = (uint8_t)(qty * 2U);                      /*字节数＝寄存器数 × 2*/
    for (i = 0U; i < qty; i++)
    {
        /*寄存器大端，高字节在前，与 CRC 的低字节在前相反*/
        uint16_t v = MB_RegRead((uint16_t)(start + i));
        resp[3U + i * 2U]     = (uint8_t)(v >> 8);
        resp[3U + i * 2U + 1U] = (uint8_t)(v & 0xFFU);
    }
    return (uint16_t)(3U + qty * 2U);
}

/**
  * 函    数：MB_OnWriteSingle
  * 参    数：req  请求整帧；reqLen  必须 8；resp  响应缓冲；isBroadcast  1=广播
  * 返 回 值：8=回显请求；3=非法地址异常；0=广播或帧长错
  */
static uint16_t MB_OnWriteSingle(const uint8_t *req, uint16_t reqLen,
                                 uint8_t *resp, uint8_t isBroadcast)
{
    uint16_t reg, val;

    if (reqLen != 8U) return 0U;
    reg = (uint16_t)((uint16_t)req[2] << 8 | req[3]);
    val = (uint16_t)((uint16_t)req[4] << 8 | req[5]);

    if (reg != MB_REG_GREEN && reg != MB_REG_BLUE)
    {
        /*保留寄存器一律回 02，与 16 的静默忽略不同*/
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
        (void)memcpy(resp, req, 8U);                    /*06 正常响应＝原样回显请求*/
        return 8U;
    }

    MB_LedWrite(reg, val);
    return 0U;
}

/**
  * 函    数：MB_OnWriteMulti
  * 参    数：req  请求整帧；reqLen  必须 9+byteCnt；resp  响应缓冲；isBroadcast  1=广播
  * 返 回 值：6=正常响应；3=异常；0=广播或帧长错
  */
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
        /*保留寄存器静默忽略，保证整批写整体成功*/
    }

    if (!isBroadcast)
    {
        resp[1] = MB_FUNC_WRITE_MULTI;
        resp[2] = req[2];
        resp[3] = req[3];
        resp[4] = req[4];
        resp[5] = req[5];
        return 6U;                                      /*地址+功能码+起始+数量*/
    }
    return 0U;
}

/*帧处理入口*/
/**
  * 函    数：MB_ProcessFrame
  * 参    数：frame  已通过 CRC 校验的整帧；len  帧长度，>= 4
  * 返 回 值：无，应答经 RS485_Send 发出
  */
static void MB_ProcessFrame(const uint8_t *frame, uint16_t len)
{
    uint8_t  resp[MB_FRAME_MAX];
    uint16_t respLen = 0U;
    uint16_t crc;
    /*广播地址 0：所有从站执行写但不回应答，多站同时发会撞总线*/
    uint8_t  isBroadcast = (frame[0] == 0U);

    /*非本机且非广播，丢弃*/
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
        default:                                        /*不支持的功能码*/
            if (!isBroadcast)
            {
                /*异常应答：功能码最高位置 1，后跟异常码*/
                resp[1] = (uint8_t)(frame[1] | 0x80U);
                resp[2] = MB_EX_ILLEGAL_FUNCTION;
                respLen = 3U;
            }
            break;
    }

    if (respLen == 0U)                                  /*广播或异常，不应答*/
    {
        return;
    }

    resp[0] = MB_SLAVE_ADDR;                            /*应答用本站地址*/
    crc = MB_Crc16(resp, respLen);
    resp[respLen++] = (uint8_t)(crc & 0xFFU);           /*CRC 低字节在前*/
    resp[respLen++] = (uint8_t)(crc >> 8);

    RS485_Send(resp, respLen);
}

/*任务主体*/
/**
  * 函    数：StartModbusTask
  * 参    数：argument  未使用，任务参数
  * 返 回 值：无
  */
void StartModbusTask(void *argument)
{
    (void)argument;
    uint8_t  frame[MB_FRAME_MAX];

    RS485_Init();

    /*注册任务句柄并开 IDLE 中断：一帧结束时 ISR 通知本任务*/
    RS485_SetNotifyTask(xTaskGetCurrentTaskHandle());
    RS485_EnableIdleIT();

    UART_Printf("[MODBUS] RS485 slave ok (USART3 PB10/PB11, %u-8-N-1, addr=%u)\r\n",
                (unsigned)RS485_BAUDRATE, (unsigned)MB_SLAVE_ADDR);

    for (;;)
    {
        uint16_t cnt;
        uint16_t i;
        static char dbg[48];                        /*诊断缓冲 static，不占栈*/
        int dbgLen = 0;

        /*必须带 1s 超时：死等时主站停轮询，心跳停发会被判卡死复位(真机复现)*/
        (void)xTaskNotifyWait(0U, 0U, NULL, pdMS_TO_TICKS(1000U));

        /*醒来先喂心跳，再取环形缓冲*/
        Watchdog_Heartbeat(WDT_TASK_MODBUS);
        cnt = RS485_RxCount();
        if (cnt == 0U)
        {
            continue;                               /*噪声或自发自收已清空*/
        }
        if (cnt > MB_FRAME_MAX)                     /*超长帧按噪声丢弃*/
        {
            RS485_RxFlush();
            continue;
        }

        for (i = 0U; i < cnt; i++)
        {
            (void)RS485_ReadByte(&frame[i]);
        }

        /*诊断打印原始帧(hex，最多 12 字节)*/
        for (i = 0U; i < cnt && i < 12U; i++)
        {
            dbgLen += snprintf(&dbg[dbgLen], sizeof(dbg) - (size_t)dbgLen,
                               "%02X ", frame[i]);
        }
        UART_Printf("[MB] rx %uB: %s%s\r\n",
                    (unsigned)cnt, dbg, (cnt > 12U) ? "..." : "");

        /*最短合法帧 4 字节，CRC 错整帧丢弃*/
        if (cnt >= 4U)
        {
            uint16_t crcCalc = MB_Crc16(frame, (uint16_t)(cnt - 2U));
            uint16_t crcRecv = (uint16_t)((uint16_t)frame[cnt - 1U] << 8 | frame[cnt - 2U]);   /*低字节在前，cnt-2 是低字节*/
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
    }
}

/*对外创建接口*/
/**
  * 函    数：ModbusTask_Create
  * 参    数：无
  * 返 回 值：无，句柄不保存
  */
void ModbusTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name = "ModbusTask",
        .stack_size = 512 * 4,                          /*帧缓冲 256B 在栈上，栈给足裕量*/
        .priority = (osPriority_t)osPriorityNormal,
    };
    (void)osThreadNew(StartModbusTask, NULL, &attr);
}
