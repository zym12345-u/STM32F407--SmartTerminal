/**
  ******************************************************************************
  * @file    CanData.h
  * @brief   CAN 通信框架(V1.3)共享数据结构与帧格式定义
  *
  *          架构说明:
  *            bsp_CAN.c 负责位时序覆写、过滤器、启动、ISR 入队、邮箱发送、错误状态;
  *            CanTask   负责收帧分发、周期心跳、状态轮询、IWDG 心跳上报;
  *            两者通过 RX/TX 队列解耦(队列句柄不对外暴露,见 bsp_CAN.h),
  *            ISR 只做搬运,不做任何业务判断与打印。
  *
  *          统计计数器的单写者约定(派活单 §6 R7):
  *            rx_total / rx_drop           —— 只由 ISR 写;
  *            tx_total / tx_drop / tx_error —— 只由 CanTask 写。
  *            32 位对齐写在 Cortex-M4 上原子,故不加互斥锁;打印时允许跨字段
  *            轻微不一致(快照本身非原子),这是有意的取舍,不要为此加锁。
  *
  *          字节序: 与本项目 Modbus 习惯一致,多字节字段采用大端(高字节在前)。
  ******************************************************************************
  */
#ifndef __CAN_DATA_H
#define __CAN_DATA_H

#include <stdint.h>

/* ====================== 帧数据结构 ====================== */

/**
  * @brief  CAN 帧(接收与发送共用)
  * @note   rsv 是为了让 timestamp 保持 4 字节对齐而显式保留的填充位,
  *         不要挪作他用;若调整结构体布局,请先核对下面的静态断言。
  */
typedef struct
{
    uint32_t id;        /* 标准帧 11 位 / 扩展帧 29 位      */
    uint8_t  ide;       /* 0=标准帧 1=扩展帧                */
    uint8_t  rtr;       /* 0=数据帧 1=遥控帧                */
    uint8_t  dlc;       /* 0~8                               */
    uint8_t  rsv;       /* 保留(对齐用)                     */
    uint32_t timestamp; /* 接收时刻 xTaskGetTickCount()      */
    uint8_t  data[8];
} CanMsg_t;

/* 静态断言: sizeof(CanMsg_t) 必须 <= 24 字节。
 * 队列内存预算 = 16 帧 RX + 8 帧 TX = 24 × sizeof(CanMsg_t),需 <= 1KB。
 * 用 typedef char[条件 ? 1 : -1] 技巧而非 _Static_assert,
 * 以兼容 ARMCC5 的 C99(C11 的 _Static_assert 不可用)。 */
typedef char can_msg_size_check_t[(sizeof(CanMsg_t) <= 24U) ? 1 : -1];

/**
  * @brief  总线状态(由 ESR 的 EWGF/EPVF/BOFF 位判定,BOFF 优先)
  */
typedef enum
{
    CAN_BUS_OK      = 0,   /* 错误主动(正常)                     */
    CAN_BUS_WARNING,       /* 错误警告: TEC 或 REC >= 96          */
    CAN_BUS_PASSIVE,       /* 错误被动: TEC 或 REC >= 128         */
    CAN_BUS_OFF            /* 总线关闭: TEC >= 256                */
} CanBusState_t;

/**
  * @brief  统计与状态快照
  */
typedef struct
{
    uint32_t      rx_total;   /* 收到并成功入队帧数(仅 ISR 写)   */
    uint32_t      rx_drop;    /* 队列满丢弃帧数(仅 ISR 写)       */
    uint32_t      tx_total;   /* 成功送入邮箱帧数(仅 CanTask 写) */
    uint32_t      tx_drop;    /* 邮箱满丢弃帧数(仅 CanTask 写)   */
    uint32_t      tx_error;   /* 硬件发送失败次数(仅 CanTask 写) */
    CanBusState_t state;      /* 当前总线状态(仅 CanTask 写)     */
    uint8_t       tec;        /* 发送错误计数(ESR[7:0])          */
    uint8_t       rec;        /* 接收错误计数(ESR[15:8])         */
} CanStats_t;

/* ====================== ID 约定(V1.4 定档) ====================== */
#define CAN_ID_HEARTBEAT   0x701U   /* 心跳,     TX, 周期 CAN_HB_PERIOD_MS     */
#define CAN_ID_TEST        0x123U   /* 分析仪手发 / 回环测试                    */
#define CAN_ID_SENSOR      0x181U   /* 传感器,   TX, 周期 CAN_SENSOR_PERIOD_MS */
#define CAN_ID_CMD         0x201U   /* 命令下行, RX, 事件触发                   */
#define CAN_ID_CMD_ACK     0x281U   /* 命令应答, TX, 事件触发(DLC=4)           */

/**
  * @brief  心跳发送周期(ms)
  * @note   派活单 R3: AutoRetransmission=Enable 且总线无人 ACK 时,
  *         每次重传 TEC +8,1s 周期会在几十帧内进 error-passive、上百帧内
  *         bus-off,日志刷成一片状态迁移。调试期可临时改大到 5000 观察。
  */
#ifndef CAN_HB_PERIOD_MS
#define CAN_HB_PERIOD_MS    1000U
#endif

/**
  * @brief  传感器帧发送周期(ms)
  * @note   与心跳同为 1s, 但【错开半周期】由 CanTask 用不同相位启动,
  *         避免同一轮循环里连发两帧(心跳在前、传感器紧随), 造成周期性
  *         的瞬时总线负载尖峰。R3 同样适用: 总线无人 ACK 时周期越短,
  *         TEC 涨得越快, 调试期可一并改大。
  */
#ifndef CAN_SENSOR_PERIOD_MS
#define CAN_SENSOR_PERIOD_MS 1000U
#endif

/* ====================== 心跳帧(0x701)载荷布局 ======================
 * 大端,8 字节:
 *   [0..1] 发送计数低 16 位
 *   [2..3] 运行秒数低 16 位
 *   [4]    TEC
 *   [5]    REC
 *   [6]    总线状态枚举(CanBusState_t)
 *   [7]    0(预留)
 *
 * 本帧【非 CANopen 兼容】。若将来要对接第三方 CANopen 主站,标准心跳应改为:
 *   ID = 0x700 + 节点号, data[0] = NMT 状态(0x05=Operational / 0x7F=Pre-operational),
 *   data[1] = 节点号。
 * 届时需改动 CanTask 的组帧函数与下面这组宏,其余代码不受影响 ——
 * 这正是把字节偏移写成具名宏的原因(派活单 §1.2 要求)。
 */
/* 大端: 高字节(MSB)在前,低字节(LSB)在后 */
#define CAN_HB_OFF_CNT_MSB  0U      /* 发送计数 bit15..8             */
#define CAN_HB_OFF_CNT_LSB  1U      /* 发送计数 bit7..0              */
#define CAN_HB_OFF_SEC_MSB  2U      /* 运行秒数 bit15..8             */
#define CAN_HB_OFF_SEC_LSB  3U      /* 运行秒数 bit7..0              */
#define CAN_HB_OFF_TEC      4U
#define CAN_HB_OFF_REC      5U
#define CAN_HB_OFF_STATE    6U
#define CAN_HB_OFF_RSV      7U

/* ====================== 传感器帧(0x181)载荷布局 ======================
 * 大端,8 字节。温湿度用定点整数(0.1 单位)而非 float:
 * 既省空间,也避开 %f 的栈与 CPU 开销(与 SensorTask 现有打印风格一致)。
 *   [0..1] 温度 int16, x10        (例: 236 = 23.6℃)
 *   [2..3] 湿度 int16, x10        (例: 455 = 45.5%RH)
 *   [4..5] 光照 uint16, x10       (上限 65535 = 6553.5 lux)
 *   [6]    有效位: bit0=温度 bit1=湿度 bit2=光照
 *   [7]    设备序号(本节点固定 0x01)
 *
 * 无效值统一填 0xFFFF。注意这与 Modbus 的 MB_RegRead(ModbusTask.c:101-110)
 * 是【同一套编码】, 便于 T7 用 Modbus Poll 与 PCAN-View 对照打。
 *
 * 两个从数据源继承而来的既有行为, V1.4 刻意不改(改了两边就对不上了):
 *   ① 温度和湿度共用 g_sensorData.temp_valid —— SensorData.h 里没有独立的
 *      湿度有效标志, 故 bit0 与 bit1 恒为同值;
 *   ② 光照"超量程"与"无效"都返回 0xFFFF —— 0xFFFF 因此有二义性, 这是
 *      Modbus 侧的既有取舍, CAN 侧照抄以保持一致。
 *
 * V1.4 单帧直发(8 字节足够);
 * 若将来要发历史曲线或日志,再引入分包(自定义序号+长度即可,不必上 ISO-TP)。
 */
/* 大端: 高字节(MSB)在前。沿用 CAN_HB_OFF_* 的 MSB/LSB 命名 ——
 * 早先这里写成 TEMP_L/TEMP_H, 在大端下极易误读成"TEMP_L 就是低字节",
 * 而它实际占据偏移 0(即高字节), 故统一改名消除歧义。 */
#define CAN_SENSOR_OFF_TEMP_MSB 0U
#define CAN_SENSOR_OFF_TEMP_LSB 1U
#define CAN_SENSOR_OFF_HUM_MSB  2U
#define CAN_SENSOR_OFF_HUM_LSB  3U
#define CAN_SENSOR_OFF_LUX_MSB  4U
#define CAN_SENSOR_OFF_LUX_LSB  5U
#define CAN_SENSOR_OFF_VALID    6U
#define CAN_SENSOR_OFF_DEVID    7U

#define CAN_SENSOR_VALID_TEMP   0x01U   /* bit0: 温度有效 */
#define CAN_SENSOR_VALID_HUM    0x02U   /* bit1: 湿度有效 */
#define CAN_SENSOR_VALID_LIGHT  0x04U   /* bit2: 光照有效 */

#define CAN_SENSOR_DEVID        0x01U   /* 本节点设备序号(为将来多节点留位) */

/* ====================== 命令帧(0x201)载荷布局 ======================
 * 大端,8 字节:
 *   [0]    命令码(CAN_CMD_*)
 *   [1]    参数(LED 命令时为灯编号 CAN_LED_*)
 *   [2..7] 预留填 0
 *
 * 命令码一栏只有三个, 其余一律回 NAK —— 这是有意的"窄接口":
 * V1.4 只验证业务数据能上下行, 不引入配置类复杂语义。
 */
#define CAN_CMD_OFF_CODE    0U
#define CAN_CMD_OFF_PARAM   1U

#define CAN_CMD_LED_ON      0x01U   /* 开灯, 参数=灯编号 */
#define CAN_CMD_LED_OFF     0x02U   /* 关灯, 参数=灯编号 */
#define CAN_CMD_GET_VERSION 0x03U   /* 读版本, 参数忽略 */

/* 灯编号: 与 LEDType.h 的 LEDColor 枚举对齐(0=绿 1=蓝),
 * 但对外用 1/2 编号(更符合"第几盏灯"的直觉), 由 CanTask 做映射 */
#define CAN_LED_GREEN       0x01U
#define CAN_LED_BLUE        0x02U

/* ====================== 应答帧(0x281)载荷布局 ======================
 * 大端,DLC=4:
 *   [0] 回显的命令码
 *   [1] 结果(CAN_ACK_*)
 *   [2] 版本高位
 *   [3] 版本低位
 *
 * !! DLC=4 但 data[] 有 8 字节: data[4..7] 必须在组帧时清零,
 *    否则总线上会露出上一帧的残留尾巴(派活单 §4 第 2 条)。
 */
#define CAN_ACK_OFF_CODE    0U
#define CAN_ACK_OFF_RESULT  1U
#define CAN_ACK_OFF_VER_H   2U
#define CAN_ACK_OFF_VER_L   3U
#define CAN_ACK_DLC         4U

#define CAN_ACK_OK          0x00U   /* 执行成功                     */
#define CAN_ACK_NAK         0x01U   /* 不支持的命令码               */
#define CAN_ACK_PARAM       0x02U   /* 参数错误(如灯编号非法)       */

/* 版本号: 主版本.次版本, 与 V1.4 对应 */
#define CAN_FW_VER_MAJOR    0x01U
#define CAN_FW_VER_MINOR    0x04U

#endif /* __CAN_DATA_H */
