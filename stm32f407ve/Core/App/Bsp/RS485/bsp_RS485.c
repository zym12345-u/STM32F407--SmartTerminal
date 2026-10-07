/**
  ******************************************************************************
  * @file    bsp_RS485.c
  * @brief   板载 RS485 收发器驱动实现(USART2, 详见 bsp_RS485.h 头部说明)
  ******************************************************************************
  */
#include "bsp_RS485.h"

/* huart2 由 CubeMX 生成的 usart.c 定义 */
extern UART_HandleTypeDef huart2;

#if RS485_MANUAL_DE
/* 若该批次板子需要手动方向控制, 在此填写实际 DE 引脚(按原理图确认) */
#define RS485_DE_PORT       GPIOB
#define RS485_DE_PIN        GPIO_PIN_12
#define RS485_DE_TX()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_RX()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)
#endif

/* ====================== 接收环形缓冲(SPSC) ======================
 * 生产者: USART2 接收中断(usart.c 回调 -> RS485_RxByteISR), 只写 head
 * 消费者: ModbusTask, 只写 tail
 * head/tail 均 volatile 单写者, 无需关中断或加锁 */
static uint8_t  s_rxBuf[RS485_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;

/* ISR 接收字节暂存(HAL_UART_Receive_IT 要求的缓冲) */
static uint8_t s_rxByte;

void RS485_RxByteISR(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rxHead + 1U) % RS485_RX_BUF_SIZE);
    if (next != s_rxTail)               /* 满则丢弃新字节(Modbus 主站会重试) */
    {
        s_rxBuf[s_rxHead] = byte;
        s_rxHead = next;
    }
    /* 重新挂起下一字节接收 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);
}

void RS485_Init(void)
{
    /* 复用 CubeMX 的 huart2, 改为 Modbus 常用波特率后重新初始化。
     * HAL_UART_Init 会再次调用 MspInit(重复配置 GPIO/NVIC, 幂等无害) */
    huart2.Init.BaudRate = RS485_BAUDRATE;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return;
    }

#if RS485_MANUAL_DE
    RS485_DE_RX();                      /* 默认处于接收状态 */
#endif

    RS485_RxFlush();

    /* 启动首字节接收(后续在 RxCpltCallback -> RS485_RxByteISR 中自续) */
    (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);
}

uint16_t RS485_RxCount(void)
{
    return (uint16_t)((s_rxHead + RS485_RX_BUF_SIZE - s_rxTail) % RS485_RX_BUF_SIZE);
}

uint8_t RS485_ReadByte(uint8_t *byte)
{
    if (s_rxTail == s_rxHead)
    {
        return 0U;
    }
    *byte = s_rxBuf[s_rxTail];
    s_rxTail = (uint16_t)((s_rxTail + 1U) % RS485_RX_BUF_SIZE);
    return 1U;
}

void RS485_RxFlush(void)
{
    s_rxTail = s_rxHead;
}

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

    /* 轮询发送: 9600 波特下 256 字节最长约 267ms, 仅阻塞 Modbus 任务自身。
     * HAL_UART_Transmit 返回时已等待 TC, 字节完整移出移位寄存器 */
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, (uint32_t)len * 4U + 50U);

#if RS485_MANUAL_DE
    /* TC 已保证最后字节移位完成, 此刻拉低不会截断帧尾 */
    RS485_DE_RX();
#endif

    /* 半双工自发自收: 发送期间本板 RX 收到自己发出的字节, 全部丢弃,
     * 避免被误当成主站的下一帧请求 */
    RS485_RxFlush();
}
