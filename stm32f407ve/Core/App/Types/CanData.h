/* CanData.h
 * CAN 共享数据类型与帧格式常量：心跳 0x701、传感器 0x181、命令 0x201、应答 0x281 的 ID、字节偏移与取值，
 * BSP 与业务层共用；多字节字段一律大端(高字节在前)。
 */
/*统计计数单写者：rx_total/rx_drop 只由 ISR 写，tx_total/tx_drop/tx_error 只由 CanTask 写；32 位对齐写在 M4 上原子，不加锁*/
#ifndef __CAN_DATA_H
#define __CAN_DATA_H

#include <stdint.h>

/* ====================== 帧数据结构 ====================== */

/* CAN 帧结构，接收与发送共用 */
typedef struct
{
    /* id 帧标识，标准帧 0x000~0x7FF(11 位)，扩展帧 0~0x1FFFFFFF(29 位)；组帧方写，CanTask 按 ID 分发读 */
    uint32_t id;
    /* ide —— 帧【格式】，说的是 ID 有多长：
     *   0 = 标准帧，ID 11 位，范围 0x000~0x7FF（本工程全用这个）
     *   1 = 扩展帧，ID 29 位，范围 0~0x1FFFFFFF
     *
     * 注意：ide 和下面的 rtr 是【两个独立维度】，别混：
     *   「标准/扩展」= ID 多长   「数据/遥控」= 这帧带不带数据
     * 所以四种组合都存在：标准数据帧、标准遥控帧、扩展数据帧、扩展遥控帧。
     * 组帧方写，接收方靠这一位决定按 11 位还是 29 位去解释 id。 */
    uint8_t  ide;
    /* rtr —— 帧【类型】，说的是这帧有没有数据段：
     *   0 = 数据帧，携带 0~8 字节数据（本工程只发这个）
     *   1 = 遥控帧，不携带数据，作用是「请求别人把数据发出来」；
     *       收到遥控帧时 dlc 表示的是对方希望回几个字节，不是自己带了多少字节。
     * 组帧方写，接收方据此决定要不要读 data。 */
    uint8_t  rtr;
    /* dlc 数据长度，0~8 字节；组帧方写，接收方只读 data[0..dlc-1] */
    uint8_t  dlc;
    /* rsv 对齐填充，固定 0 不得挪用；组帧方清零，谁都不读 */
    uint8_t  rsv;
    /* timestamp 接收时刻，单位 tick(默认 1ms/tick)；ISR 写，CanTask 读 */
    uint32_t timestamp;
    /* data 载荷，0~8 字节，各字节含义见 CAN_HB_/CAN_SENSOR_/CAN_CMD_/CAN_ACK_ 偏移宏；组帧方写，接收方读 */
    uint8_t  data[8];
} CanMsg_t;

/* 静态断言 sizeof(CanMsg_t) <= 24 字节：16 RX + 8 TX 共 24 帧须 <= 1KB；ARMCC5 不支持 C11 _Static_assert，故用 typedef char[] */
typedef char can_msg_size_check_t[(sizeof(CanMsg_t) <= 24U) ? 1 : -1];

/* 总线状态 0~3，CanTask 按 ESR 的 EWGF/EPVF/BOFF 位判定(BOFF 优先)后写入 CanStats_t.state，心跳帧 [6] 与诊断读 */
typedef enum
{
    CAN_BUS_OK      = 0,   /* 错误主动 */
    CAN_BUS_WARNING,       /* 错误警告 TEC 或 REC >= 96 */
    CAN_BUS_PASSIVE,       /* 错误被动 TEC 或 REC >= 128 */
    CAN_BUS_OFF            /* 总线关闭 TEC >= 256 */
} CanBusState_t;

/* 统计与状态快照：计数单位均为帧/次，0~0xFFFFFFFF 自然回绕；写者见行尾，跨字段允许轻微不一致(有意取舍) */
typedef struct
{
    uint32_t      rx_total;   /* 收到并入队帧数，ISR 写 */
    uint32_t      rx_drop;    /* RX 队列满丢弃，ISR 写 */
    uint32_t      tx_total;   /* 送入邮箱帧数，CanTask 写 */
    uint32_t      tx_drop;    /* 邮箱满丢弃，CanTask 写 */
    uint32_t      tx_error;   /* 硬件发送失败次数，CanTask 写 */
    CanBusState_t state;      /* 总线状态 0~3，CanTask 写 */
    uint8_t       tec;        /* 发送错误计数 0~255(>255 即 bus-off)，CanTask 从 ESR 读 */
    uint8_t       rec;        /* 接收错误计数 0~255，CanTask 从 ESR 读 */
} CanStats_t;

/* ====================== ID 约定 ====================== */
/* 全为 11 位标准帧 0x000~0x7FF；TX 由 CanTask 组帧发送，RX 由 CanTask 分发 */
#define CAN_ID_HEARTBEAT   0x701U   /* 心跳 TX 周期 CAN_HB_PERIOD_MS */
#define CAN_ID_TEST        0x123U   /* 分析仪手发 / 回环测试 */
#define CAN_ID_SENSOR      0x181U   /* 传感器 TX 周期 CAN_SENSOR_PERIOD_MS */
#define CAN_ID_CMD         0x201U   /* 命令下行 RX 事件触发 */
#define CAN_ID_CMD_ACK     0x281U   /* 命令应答 TX 事件触发 DLC=4 */

/* 心跳周期 ms，100~0xFFFFFFFF，编译期可覆盖；CanTask 用作发送周期 */
/*总线无人 ACK 时每次重传 TEC +8，1s 周期几十帧即进 error-passive、上百帧 bus-off，调试可临时改大到 5000*/
#ifndef CAN_HB_PERIOD_MS
#define CAN_HB_PERIOD_MS    1000U
#endif

/* 传感器帧周期 ms，编译期可覆盖；CanTask 用作发送周期，与心跳错开半周期避免同轮连发两帧 */
#ifndef CAN_SENSOR_PERIOD_MS
#define CAN_SENSOR_PERIOD_MS 1000U
#endif

/* ====================== 心跳帧(0x701)载荷布局 ======================
 * 大端 8 字节，data[] 下标 0~7，无单位；CanTask 组帧写，接收方按同偏移解析
 * [0..1] 发送计数低 16 位，高字节在前
 * [2..3] 运行秒数低 16 位
 * [4]    TEC 发送错误计数 0~255
 * [5]    REC 接收错误计数 0~255
 * [6]    总线状态 0~3(CanBusState_t)
 * [7]    预留填 0
 */
/*本帧非 CANopen 兼容；若对接第三方 CANopen 主站，ID 改 0x700+节点号、data[0] 填 NMT 状态、data[1] 填节点号，只改组帧与下列宏*/
#define CAN_HB_OFF_CNT_MSB  0U      /* 发送计数高字节 */
#define CAN_HB_OFF_CNT_LSB  1U      /* 发送计数低字节 */
#define CAN_HB_OFF_SEC_MSB  2U      /* 运行秒数高字节 */
#define CAN_HB_OFF_SEC_LSB  3U      /* 运行秒数低字节 */
/* [4] TEC / [5] REC / [6] 总线状态 / [7] 预留填 0；CanTask 写，诊断工具读 */
#define CAN_HB_OFF_TEC      4U
#define CAN_HB_OFF_REC      5U
#define CAN_HB_OFF_STATE    6U
#define CAN_HB_OFF_RSV      7U

/* ====================== 传感器帧(0x181)载荷布局 ======================
 * 大端 8 字节，data[] 下标 0~7，无单位；CanTask 组帧写，接收方按偏移解析
 * [0..1] 温度 int16，x10(236 = 23.6℃)
 * [2..3] 湿度 int16，x10(455 = 45.5%RH)
 * [4..5] 光照 uint16，x10(上限 6553.5 lux)
 * [6]    有效位 bit0 温度 bit1 湿度 bit2 光照
 * [7]    设备序号(本节点固定 0x01)
 */
/*温湿度用 x10 定点而非 float：省空间，也避开 %f 的栈与 CPU 开销*/
/*无效值统一填 0xFFFF，与 Modbus MB_RegRead 同一套编码，便于 Modbus Poll 与 PCAN-View 对照*/
/*温湿度共用 g_sensorData.temp_valid，SensorData.h 无独立湿度标志，故 bit0 与 bit1 恒同值；光照超量程与无效都回 0xFFFF，二义性沿袭 Modbus，两边须一致*/
/*单帧直发即可；若将来发历史曲线再引入分包(序号+长度)，不必上 ISO-TP*/
/*偏移宏统一 MSB/LSB 命名：TEMP_L 在大端下易被误读为低字节，而它实际占偏移 0(高字节)*/
#define CAN_SENSOR_OFF_TEMP_MSB 0U
#define CAN_SENSOR_OFF_TEMP_LSB 1U
#define CAN_SENSOR_OFF_HUM_MSB  2U
#define CAN_SENSOR_OFF_HUM_LSB  3U
#define CAN_SENSOR_OFF_LUX_MSB  4U
#define CAN_SENSOR_OFF_LUX_LSB  5U
#define CAN_SENSOR_OFF_VALID    6U
#define CAN_SENSOR_OFF_DEVID    7U

/* 有效位 bit0~bit2，1=有效 0=无效；CanTask 按 g_sensorData 的 _valid 换算写入 */
#define CAN_SENSOR_VALID_TEMP   0x01U   /* bit0 温度 */
#define CAN_SENSOR_VALID_HUM    0x02U   /* bit1 湿度 */
#define CAN_SENSOR_VALID_LIGHT  0x04U   /* bit2 光照 */

/* 设备序号 0x01~0xFF；本节点固定 0x01，CanTask 写，主站按此区分节点 */
#define CAN_SENSOR_DEVID        0x01U   /* 为将来多节点留位 */

/* ====================== 命令帧(0x201)载荷布局 ======================
 * 大端 8 字节，data[] 下标 0~7，无单位；主站发，CanTask 解析
 * [0]    命令码(CAN_CMD_*)
 * [1]    参数(LED 命令时为灯编号 CAN_LED_*)
 * [2..7] 预留填 0
 */
#define CAN_CMD_OFF_CODE    0U
#define CAN_CMD_OFF_PARAM   1U

/* 命令码 0x01~0x03，其余一律回 NAK(1)；主站发，CanTask 读 */
#define CAN_CMD_LED_ON      0x01U   /* 开灯，参数=灯编号 */
#define CAN_CMD_LED_OFF     0x02U   /* 关灯，参数=灯编号 */
#define CAN_CMD_GET_VERSION 0x03U   /* 读版本，参数忽略 */

/* 灯编号 1~2；0 及其它值视为参数错误(应答码 2)；主站填，CanTask 映射到 LEDColor(0=绿 1=蓝) */
#define CAN_LED_GREEN       0x01U
#define CAN_LED_BLUE        0x02U

/* ====================== 应答帧(0x281)载荷布局 ======================
 * 大端 DLC=4，data[] 下标 0~3，无单位；CanTask 组帧写，主站解析
 * [0] 回显命令码
 * [1] 结果(CAN_ACK_*)
 * [2] 版本高位
 * [3] 版本低位
 */
/*DLC=4 而 data[] 有 8 字节，data[4..7] 必须在组帧时清零，否则总线露出上一帧残留*/
#define CAN_ACK_OFF_CODE    0U
#define CAN_ACK_OFF_RESULT  1U
#define CAN_ACK_OFF_VER_H   2U
#define CAN_ACK_OFF_VER_L   3U
#define CAN_ACK_DLC         4U

/* 结果码 0~2；CanTask 填，主站读 */
#define CAN_ACK_OK          0x00U   /* 执行成功 */
#define CAN_ACK_NAK         0x01U   /* 不支持的命令码 */
#define CAN_ACK_PARAM       0x02U   /* 参数错误(如灯编号非法) */

/* 版本号高低字节 0x00~0xFF；编译期常量，CanTask 填入应答帧 [2..3] */
#define CAN_FW_VER_MAJOR    0x01U
#define CAN_FW_VER_MINOR    0x06U

#endif /* __CAN_DATA_H */
