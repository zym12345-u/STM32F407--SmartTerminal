/* BootLoader/bsp_uart1.h
 * 升级口 USART1 的裸机驱动 + 升级帧协议（帧头/命令号/状态码/解析器）。
 */
#ifndef BSP_UART1_H
#define BSP_UART1_H

#include <stdint.h>
#include "stm32f4xx.h"

/* 升级口 USART1：PA9=TX、PA10=RX，115200 8N1 */
#define UART_GPIO_CLK_EN  RCC_AHB1ENR_GPIOAEN
#define UART_PORT         GPIOA
#define UART_AF           7UL        /*PA9/PA10 的复用功能号都是 AF7（USART1）*/
#define UART_BRR          (0x8BUL)   /*16MHz / 115200 = 138.9 → 取 139，误差 0.08%*/
#define UART_WINDOW_MS    3000UL     /*升级窗口长度，3 秒*/

/* 升级帧格式：PC 端脚本按同一套定义发包 */
#define FRAME_SYNC0       0xA5U      /*帧头第 1 字节*/
#define FRAME_SYNC1       0x5AU      /*帧头第 2 字节*/
#define CMD_HANDSHAKE     0x01U      /*握手，长度 0*/
#define CMD_DATA          0x02U      /*数据块，设备按顺序往后追加写入*/
#define CMD_END           0x03U      /*结束，长度 6：整包 CRC32 + 固件版本(主,次)；长度 0 = 只擦除模式*/
#define CMD_ERASE         0x04U      /*擦除，数据 8 字节：起始地址 + 结束地址(不含)*/
#define CMD_PROGRESS      0x05U      /*查询已写字节数，长度 0；应答带 4 字节数据（断电续传用）*/
#define CMD_ACK_FLAG      0x80U      /*应答帧的命令号 = 原命令号 | 0x80*/
#define FRAME_DATA_MAX    128U       /*一帧最多 128 字节数据，正好 32 个字*/

/* 应答帧里那 1 个状态字节的取值 */
#define ST_OK             0x00U      /*成功*/
#define ST_RANGE          0x01U      /*地址越界 / 长度不对 / 没按扇区对齐*/
#define ST_FLASH          0x02U      /*擦除或编程失败（写前发现没擦干净也算）*/
#define ST_CRC            0x03U      /*整包 CRC 校验不通过：写入内容与 PC 送的不一致*/

/* 一条帧从帧头到 CRC 的收帧状态：窗口和升级模式共用，握手和写数据都走它 */
typedef struct
{
  uint8_t  state;                  /*0 等 A5 / 1 等 5A / 2 等命令 / 3 等长度 / 4 收数据 / 5~8 收 CRC*/
  uint8_t  cmd;                    /*本帧命令号*/
  uint8_t  len;                    /*本帧数据长度*/
  uint8_t  got;                    /*已收数据字节数*/
  uint32_t crc;                    /*CRC 累加器*/
  uint32_t crc_recv;               /*帧尾带来的 CRC 值*/
  uint8_t  data[FRAME_DATA_MAX];   /*数据段，CRC 对上了才交给命令处理函数*/
} frame_parser_t;

void uart1_init(void);
void uart1_send_str(const char *s);
void uart1_send_dec(uint32_t v);
void uart1_send_hex32(uint32_t v);
uint32_t crc32_update(uint32_t crc, uint8_t b);
void frame_send(uint8_t cmd, const uint8_t *data, uint8_t len);
void frame_reply(uint8_t cmd, uint8_t st);
void frame_reset(frame_parser_t *p);
uint8_t frame_feed(frame_parser_t *p, uint8_t ch);
int  uart1_wait_handshake(uint32_t ms);

#endif /* BSP_UART1_H */
