/* bsp_RS485.c
 * RS485 驱动：USART3 中断收字节、环形缓冲、阻塞发送、IDLE 中断唤醒任务。
 * 硬件：USART3 = PB10(TX)/PB11(RX)，AF7；板载收发器硬件自动方向，无 DE/RE 引脚。
 * 注意：只允许 ModbusTask 单消费者调用；发送前后各清一次缓冲，丢掉自发自收字节。
 *
 * ══════ RS485 的基础（拿 CAN 对比着记最快）══════
 *
 * ── 物理层：差分，A / B 两根线 ──
 *   和 CAN 一样靠【压差】判 0/1，不靠对地电平：
 *     A 比 B 高约 200mV 以上 → 一种状态
 *     B 比 A 高约 200mV 以上 → 另一种状态
 *   好处：共模干扰同时叠加在两根线上，相减就抵消了 —— 所以能传 1200 米。
 *
 * ── 半双工：同一时刻只能一个人说话 ──
 *   RS485 的两根线【收发共用】，发的时候自己也在听（会听到自己发的）。
 *   所以必须有个「方向控制」：发之前切到发送，发完切回接收。
 *   本板子的收发器是【硬件自动方向】（没有 DE/RE 引脚要管），省了这事；
 *   如果你的板子需要手动控制，打开 RS485_MANUAL_DE 并填好引脚。
 *   注意切换时机：必须等最后一个字节【完全移出移位寄存器】才能切回接收，
 *   切早了会把这一帧的尾巴截掉。
 *
 * ── 【关键】RS485 只规定了物理层，没有协议 ──
 *   这一点最容易和 CAN 搞混：
 *
 *     | 能力            | CAN          | RS485                  |
 *     |-----------------|--------------|------------------------|
 *     | 差分传输/抗干扰 | 有（硬件）    | 有（硬件）              |
 *     | 多节点          | 有            | 有（电气上可挂 32 节点）|
 *     | 仲裁（同时发）  | 有（硬件）    | 【没有】—— 同时发就撞车  |
 *     | CRC 校验        | 有（硬件）    | 【没有】—— 靠上层协议补  |
 *     | 自动重传        | 有（硬件）    | 【没有】—— 靠上层决定    |
 *     | 错误计数/隔离   | 有（硬件）    | 【没有】                |
 *
 *   所以 RS485 本身只是「能挂多个设备的差分串口」，
 *   上面那堆可靠性能力要靠 Modbus 协议在【软件层】自己实现：
 *   CRC16 校验、主从轮询避免撞车、超时重发 —— 这些都在 ModbusTask.c 里。
 *   这就是为什么 CAN 节点可以随时主动上报，而 RS485 从站必须等主机来问。
 */
#include "bsp_RS485.h"

/* huart3 由 CubeMX 生成的 usart.c 定义 */
extern UART_HandleTypeDef huart3;

#if RS485_MANUAL_DE
/* 若该批次板子需要手动方向控制, 在此填写实际 DE 引脚(按原理图确认) */
#define RS485_DE_PORT       GPIOB
#define RS485_DE_PIN        GPIO_PIN_12
#define RS485_DE_TX()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_RX()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)
#endif

/*接收环形缓冲：ISR 只写 head，ModbusTask 只写 tail，均 volatile，无需关中断*/
static uint8_t  s_rxBuf[RS485_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;

/*HAL 单字节接收缓冲；usart.c 回调必须读它，不能用各自的 static*/
uint8_t s_rxByte;

/* IDLE 中断唤醒的任务句柄；NULL 时不发通知 */
static TaskHandle_t s_notifyTask = NULL;

/**
  * 功    能：收一个字节进环形缓冲。中断里只写缓冲，满了就丢，不做任何判断
  * 参    数：byte  本次收到的字节，范围 0x00~0xFF
  * 返 回 值：无
  * 补    充：ISR 上下文，由 usart.c 的 HAL_UART_RxCpltCallback 调用；缓冲满则丢弃本字节
  */
void RS485_RxByteISR(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rxHead + 1U) % RS485_RX_BUF_SIZE);   /* 环形索引，255 后回卷到 0 */
    if (next != s_rxTail)               /* 满则丢弃新字节(Modbus 主站会重试) */
    {
        s_rxBuf[s_rxHead] = byte;
        s_rxHead = next;
    }
    /* 重新挂起下一字节接收 */
    (void)HAL_UART_Receive_IT(&huart3, &s_rxByte, 1U);   /* 单字节模式：每收 1 字节重挂一次，避免定长接收把一帧切断 */
}

/**
  * 功    能：初始化 RS485 的收发控制（方向引脚、缓冲清零）。依赖 USART3 已经初始化过
  * 参    数：无
  * 返 回 值：无，HAL_UART_Init 失败时静默返回
  * 补    充：依赖 usart.c 已完成 MX_USART3_UART_Init；重新 Init 会再调 MspInit，幂等
  */
void RS485_Init(void)
{
    /*复用 huart3，改波特率后重新初始化*/
    huart3.Init.BaudRate = RS485_BAUDRATE;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        return;
    }

#if RS485_MANUAL_DE
    RS485_DE_RX();                      /* 默认处于接收状态 */
#endif

    RS485_RxFlush();

    /*启动首字节接收，后续在 RxCpltCallback 中自续*/
    (void)HAL_UART_Receive_IT(&huart3, &s_rxByte, 1U);
}

/**
  * 功    能：返回缓冲里待读的字节数。head/tail 是单写者 volatile，读到的值可能略过时，够当判据用
  * 参    数：无
  * 返 回 值：待读字节数，范围 0~255
  * 补    充：head/tail 为单写者 volatile，读瞬间可能略过时，仅作判据
  */
uint16_t RS485_RxCount(void)
{
    return (uint16_t)((s_rxHead + RS485_RX_BUF_SIZE - s_rxTail) % RS485_RX_BUF_SIZE);
}

/**
  * 功    能：取一个字节。1 读到、0 缓冲空。只允许 ModbusTask 一个消费者调
  * 参    数：byte  输出参数，缓冲非空时写入；不允许 NULL
  * 返 回 值：1 = 读到 1 字节，0 = 缓冲空，*byte 未改
  * 补    充：仅 ModbusTask 单消费者调用，多任务并发会破坏 tail 单写者约定
  */
uint8_t RS485_ReadByte(uint8_t *byte)
{
    if (s_rxTail == s_rxHead)
    {
        return 0U;
    }
    *byte = s_rxBuf[s_rxTail];
    s_rxTail = (uint16_t)((s_rxTail + 1U) % RS485_RX_BUF_SIZE);   /* 读指针 255 后回卷到 0 */
    return 1U;
}

/**
  * 功    能：清空接收缓冲：只把 tail 拨到 head，不用关中断（ISR 那边同时写属正常竞态）
  * 参    数：无
  * 返 回 值：无
  * 补    充：只动 tail，不需关中断；ISR 同时写入属正常竞态
  */
void RS485_RxFlush(void)
{
    s_rxTail = s_rxHead;
}

/**
  * 功    能：发一串数据：切到发送方向 → 阻塞发完 → 切回接收方向
  * 参    数：data  待发送数据首地址，NULL 时直接返回
  *            len  字节数，范围 1~256，0 时直接返回
  * 返 回 值：无
  * 补    充：仅 ModbusTask 调用；9600 下 256B 约 267ms，会阻塞本任务
  *            9600 下发 256 字节约 267ms，期间本任务被占住
  */
void RS485_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }

    RS485_RxFlush();                    /* 发送前丢弃残留字节 */

#if RS485_MANUAL_DE
    RS485_DE_TX();
#endif

    /*阻塞发送，超时 4 倍理论时间 + 50ms*/
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, (uint32_t)len * 4U + 50U);   /* 超时 = 4 倍理论时间 + 50ms；9600 下 256B 约 267ms */

#if RS485_MANUAL_DE
    /* TC 已保证最后字节移位完成, 此刻拉低不会截断帧尾 */
    RS485_DE_RX();
#endif

    /*半双工自发自收：丢弃发送期间收到的自己字节*/
    RS485_RxFlush();
}

/*IDLE 中断*/

/**
  * 功    能：记下「收完一帧后要通知谁」的任务句柄，空闲中断里用它 xTaskNotifyFromISR
  * 参    数：task  目标任务句柄，通常为 ModbusTask；NULL 关闭通知
  * 返 回 值：无
  * 补    充：必须在 RS485_EnableIdleIT 之前调用，否则首帧 IDLE 不发通知
  */
void RS485_SetNotifyTask(TaskHandle_t task)
{
    s_notifyTask = task;
}

/**
  * 功    能：使能 USART3 的空闲线中断，用「总线空闲」来判定一帧结束
  * 参    数：无
  * 返 回 值：无
  * 补    充：在 RS485_Init 与 RS485_SetNotifyTask 之后调用；IDLE 标志需先读 SR 再读 DR 清除
  */
void RS485_EnableIdleIT(void)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);   /* 只开 IDLE；RXNE 已由 HAL_UART_Receive_IT 内部使能 */
}

/**
  * 功    能：空闲中断里通知注册过的任务来取帧
  * 参    数：无
  * 返 回 值：无
  * 补    充：ISR 上下文，由 stm32f4xx_it.c 的 USART3_IRQHandler 调用；eNoAction 合并多次唤醒
  */
void RS485_IdleISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    /*清 IDLE 标志，读 SR 再读 DR*/
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    }

    /*eNoAction 不覆盖通知值，多次 IDLE 合并为一次唤醒*/
    if (s_notifyTask != NULL)
    {
        (void)xTaskNotifyFromISR(s_notifyTask, 0U, eNoAction,
                                 &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}
