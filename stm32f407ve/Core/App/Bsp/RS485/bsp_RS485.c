/* bsp_RS485.c
 * RS485 驱动：USART3 中断收字节、环形缓冲、阻塞发送、IDLE 中断唤醒任务。
 * 硬件：USART3 = PB10(TX)/PB11(RX)，AF7；板载收发器硬件自动方向，无 DE/RE 引脚。
 * 注意：只允许 ModbusTask 单消费者调用；发送前后各清一次缓冲，丢掉自发自收字节。
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
  * 函    数：RS485_RxByteISR
  * 参    数：byte  本次收到的字节，范围 0x00~0xFF
  * 返 回 值：无
  * 注意事项：ISR 上下文，由 usart.c 的 HAL_UART_RxCpltCallback 调用；缓冲满则丢弃本字节
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
  * 函    数：RS485_Init
  * 参    数：无
  * 返 回 值：无，HAL_UART_Init 失败时静默返回
  * 注意事项：依赖 usart.c 已完成 MX_USART3_UART_Init；重新 Init 会再调 MspInit，幂等
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
  * 函    数：RS485_RxCount
  * 参    数：无
  * 返 回 值：待读字节数，范围 0~255
  * 注意事项：head/tail 为单写者 volatile，读瞬间可能略过时，仅作判据
  */
uint16_t RS485_RxCount(void)
{
    return (uint16_t)((s_rxHead + RS485_RX_BUF_SIZE - s_rxTail) % RS485_RX_BUF_SIZE);
}

/**
  * 函    数：RS485_ReadByte
  * 参    数：byte  输出参数，缓冲非空时写入；不允许 NULL
  * 返 回 值：1 = 读到 1 字节，0 = 缓冲空，*byte 未改
  * 注意事项：仅 ModbusTask 单消费者调用，多任务并发会破坏 tail 单写者约定
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
  * 函    数：RS485_RxFlush
  * 参    数：无
  * 返 回 值：无
  * 注意事项：只动 tail，不需关中断；ISR 同时写入属正常竞态
  */
void RS485_RxFlush(void)
{
    s_rxTail = s_rxHead;
}

/**
  * 函    数：RS485_Send
  * 参    数：data  待发送数据首地址，NULL 时直接返回
  * 参    数：len  字节数，范围 1~256，0 时直接返回
  * 返 回 值：无
  * 注意事项：仅 ModbusTask 调用；9600 下 256B 约 267ms，会阻塞本任务
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
  * 函    数：RS485_SetNotifyTask
  * 参    数：task  目标任务句柄，通常为 ModbusTask；NULL 关闭通知
  * 返 回 值：无
  * 注意事项：必须在 RS485_EnableIdleIT 之前调用，否则首帧 IDLE 不发通知
  */
void RS485_SetNotifyTask(TaskHandle_t task)
{
    s_notifyTask = task;
}

/**
  * 函    数：RS485_EnableIdleIT
  * 参    数：无
  * 返 回 值：无
  * 注意事项：在 RS485_Init 与 RS485_SetNotifyTask 之后调用；IDLE 标志需先读 SR 再读 DR 清除
  */
void RS485_EnableIdleIT(void)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);   /* 只开 IDLE；RXNE 已由 HAL_UART_Receive_IT 内部使能 */
}

/**
  * 函    数：RS485_IdleISR
  * 参    数：无
  * 返 回 值：无
  * 注意事项：ISR 上下文，由 stm32f4xx_it.c 的 USART3_IRQHandler 调用；eNoAction 合并多次唤醒
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
