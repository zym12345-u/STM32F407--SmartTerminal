/* BootLoader/bsp_uart1.c
 * 升级口 USART1：裸机寄存器版驱动 + 升级帧的收发状态机。
 *
 * ── 升级帧为什么这么定（本步只实现握手）──
 *   帧格式：A5 5A | 命令 | 长度 | 数据(0~128B) | CRC32(4B)
 *   A5 5A 是帧头：串口上随时可能有噪声或上位机在发别的东西，
 *   只有这两个字节按顺序出现，才认「一帧开始」，后面的命令和长度才可信。
 *   CRC32 只覆盖【命令 + 长度 + 数据】三段，不包含 A5 5A：
 *   帧头本来就是用来对齐的，把它也算进 CRC 并不增加检错能力，只会让算法更绕。
 *   CRC 用逐位算法、不建 256 项查表：查表白占 1KB Flash，BootLoader 越瘦越好；
 *   115200 波特率下一帧几十字节，逐位算 8 次循环对时间毫无压力。
 *   握手命令 0x01 的数据长度为 0，作用只是「我在这儿，可以开始升级了」；
 *   BootLoader 回一帧 ACK（命令号 |= 0x80）就进升级模式，
 *   没收到就照旧 3 秒后跳 App —— 两条路径都不阻塞，窗口不会卡死。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "bsp_uart1.h"
#include "board.h"

static void uart1_send_byte(uint8_t b);

/**
  * 功    能：初始化升级口 USART1（裸机寄存器版，不依赖 HAL）
  * 参    数：无
  * 返 回 值：无
  * 补    充：BootLoader 只用寄存器操作，这样它不占 HAL 的代码空间，也不会和 App 的 HAL 抢状态；
  *            BRR 按 HSI 16MHz 算，改主频必须同步改 UART_BRR，否则波特率偏差超出 3% 就全是乱码
  */
void uart1_init(void)
{
  RCC->AHB1ENR |= UART_GPIO_CLK_EN;   /*先给 GPIOA 供时钟，时钟没开写配置寄存器全是白写*/
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   /*USART1 挂在 APB2 上*/

  /*PA9/PA10 设成复用模式(10)，并把两个引脚的复用号都填成 AF7 落到 USART1*/
  UART_PORT->MODER = (UART_PORT->MODER & ~(0xFUL << 18U)) | (0xAUL << 18U);
  UART_PORT->AFR[1] = (UART_PORT->AFR[1] & ~(0xFFUL << 4U))
                    | ((UART_AF | (UART_AF << 4U)) << 4U);

  /*两个脚都开内部上拉：TX 空闲本该是高电平，RX 悬空时引脚电平乱跳会被当成起始位*/
  UART_PORT->PUPDR = (UART_PORT->PUPDR & ~(0xFUL << 18U)) | (0x5UL << 18U);

  USART1->BRR = UART_BRR;
  USART1->CR2 = 0UL;                                  /*1 位停止位*/
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   /*使能串口、发送、接收；M=0 即 8 位数据*/
}

/**
  * 功    能：往 USART1 发一串字符
  * 参    数：s 以 '\0' 结尾的字符串
  * 返 回 值：无
  * 补    充：打诊断字符串（开机横幅、状态提示、坏帧提示）；每发一个字节都要等 TXE，阻塞式发送
  */
void uart1_send_str(const char *s)
{
  while (*s != '\0')
  {
    uart1_send_byte((uint8_t)(*s));
    s++;
  }
}

/**
  * 功    能：往 USART1 发一个字节
  * 参    数：b 要发送的字节
  * 返 回 值：无
  * 补    充：等 TXE 置 1 才写 DR —— TXE=1 表示数据寄存器空了，可以塞下一个；
  *            阻塞式发送，收发帧时才用，横幅打印也走这里
  */
static void uart1_send_byte(uint8_t b)
{
  while ((USART1->SR & USART_SR_TXE) == 0UL)   /*上一个字节挪走了才写下一个*/
  {
  }
  USART1->DR = (uint16_t)b;
}

/**
  * 功    能：把 1 个字节喂进 CRC32（IEEE 802.3 反射多项式 0xEDB88320）累加器
  * 参    数：crc 上一次的中间值（第一字节前给 0xFFFFFFFF）；b 本字节
  * 返 回 值：累加后的中间值，整帧收完再 ^0xFFFFFFFF 才是最终校验值
  * 补    充：逐位算，每字节 8 次循环；省掉 1KB 查表，115200 下这点耗时感觉不到
  */
uint32_t crc32_update(uint32_t crc, uint8_t b)
{
  uint32_t i;

  crc ^= (uint32_t)b;
  for (i = 0UL; i < 8UL; i++)
  {
    if ((crc & 1UL) != 0UL)
    {
      crc = (crc >> 1) ^ 0xEDB88320UL;   /*最低位为 1 才异或多项式，这是反射算法的固定动作*/
    }
    else
    {
      crc >>= 1;
    }
  }
  return crc;
}

/**
  * 功    能：按帧格式发一帧：A5 5A | 命令 | 长度 | 数据 | CRC32
  * 参    数：cmd 命令号；data 数据指针（长度为 0 时可传 0）；len 数据长度
  * 返 回 值：无
  * 补    充：ACK 也是用这个函数发，只是命令号要 |= 0x80，这样才能和"发命令"区分开
  */
void frame_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint8_t  i;

  uart1_send_byte(FRAME_SYNC0);
  uart1_send_byte(FRAME_SYNC1);
  uart1_send_byte(cmd);
  uart1_send_byte(len);

  crc = crc32_update(crc, cmd);   /*CRC 从命令字节开始算，不含两个帧头字节*/
  crc = crc32_update(crc, len);
  for (i = 0U; i < len; i++)
  {
    uart1_send_byte(data[i]);
    crc = crc32_update(crc, data[i]);
  }
  crc ^= 0xFFFFFFFFUL;

  for (i = 0U; i < 4U; i++)   /*低字节在前，PC 端按同样顺序拼*/
  {
    uart1_send_byte((uint8_t)(crc >> (8U * i)));
  }
}

/**
  * 功    能：把解析器复位到"等帧头"的状态
  * 参    数：p 解析器
  * 返 回 值：无
  * 补    充：CRC 先塞初值 0xFFFFFFFF，等 A5 5A 收齐时还会再塞一次，反正那时候还没开始算
  */
void frame_reset(frame_parser_t *p)
{
  p->state    = 0U;
  p->cmd      = 0U;
  p->len      = 0U;
  p->got      = 0U;
  p->crc      = 0xFFFFFFFFUL;
  p->crc_recv = 0UL;
}

/**
  * 功    能：往解析器喂一个字节，推进状态机
  * 参    数：p 解析器；ch 刚收到的字节
  * 返 回 值：0 还没收完；1 收完且 CRC 对；2 收完但 CRC 错（状态已自动复位）
  * 补    充：不认识的字节只在"等帧头"状态下被忽略，半帧里对不上帧头就退回重找 ——
  *            这样无论前面混了什么噪声，下一帧总能重新对齐
  */
uint8_t frame_feed(frame_parser_t *p, uint8_t ch)
{
  switch (p->state)
  {
  case 0U:
    if (ch == FRAME_SYNC0)   /*先把第一个帧头字节对上*/
    {
      p->state = 1U;
    }
    break;

  case 1U:
    if (ch == FRAME_SYNC1)   /*两个帧头字节齐了，正式开始收一帧*/
    {
      p->crc   = 0xFFFFFFFFUL;
      p->state = 2U;
    }
    else if (ch != FRAME_SYNC0)   /*既不是 5A 也不是连着来的 A5，退回去重新找帧头*/
    {
      p->state = 0U;
    }
    break;

  case 2U:
    p->cmd   = ch;
    p->crc   = crc32_update(p->crc, ch);
    p->state = 3U;
    break;

  case 3U:
    p->len = ch;
    p->crc = crc32_update(p->crc, ch);
    p->got = 0U;
    if (p->len > FRAME_DATA_MAX)   /*长度离谱当噪声，直接丢掉重新找帧头*/
    {
      p->state = 0U;
    }
    else
    {
      p->state = (p->len == 0U) ? 5U : 4U;   /*长度为 0 就没有数据段*/
    }
    break;

  case 4U:
    p->data[p->got] = ch;
    p->got++;
    p->crc = crc32_update(p->crc, ch);
    if (p->got >= p->len)
    {
      p->state = 5U;
    }
    break;

  case 5U:
    p->crc_recv = (uint32_t)ch;
    p->state    = 6U;
    break;

  case 6U:
    p->crc_recv |= (uint32_t)ch << 8U;
    p->state     = 7U;
    break;

  case 7U:
    p->crc_recv |= (uint32_t)ch << 16U;
    p->state     = 8U;
    break;

  case 8U:
    p->crc_recv |= (uint32_t)ch << 24U;
    p->state = 0U;   /*帧收完就复位，下一帧重新找帧头*/
    if ((p->crc ^ 0xFFFFFFFFUL) == p->crc_recv)
    {
      return 1U;
    }
    return 2U;

  default:   /*状态值不该越界，真越界了就当噪声重置，保证不会死锁*/
    p->state = 0U;
    break;
  }

  return 0U;
}

/**
  * 功    能：回一帧应答：命令号 | 0x80，带 1 字节状态
  * 参    数：cmd 原命令号；st 状态（ST_OK / ST_RANGE / ST_FLASH）
  * 返 回 值：无
  * 补    充：握手、擦除、数据块、结束都用这一种应答格式，PC 端只看状态字节决定下一步
  */
void frame_reply(uint8_t cmd, uint8_t st)
{
  frame_send((uint8_t)(cmd | CMD_ACK_FLAG), &st, 1U);
}

/**
  * 功    能：守住升级窗口若干毫秒，期间识别握手帧
  * 参    数：ms 窗口长度
  * 返 回 值：1 收到合法握手帧并已回 ACK；0 窗口走完没等到
  * 补    充：解析用「状态机 + 窗口总超时」，不是死等某个字节 ——
  *            半帧卡住最多耗到窗口结束，绝不会让 App 永远起不来
  */
int uart1_wait_handshake(uint32_t ms)
{
  uint32_t elapsed = 0UL;
  frame_parser_t p;
  uint8_t ch;
  uint8_t r;

  frame_reset(&p);
  systick_start_1ms();

  while (elapsed < ms)
  {
    if ((USART1->SR & USART_SR_RXNE) != 0UL)   /*RXNE=1 说明接收寄存器里有新字节*/
    {
      ch = (uint8_t)USART1->DR;   /*读 DR 同时把 RXNE 清掉，不读就会一直重复进这个分支*/
      r  = frame_feed(&p, ch);

      if (r == 1U)
      {
        if (p.cmd == CMD_HANDSHAKE)
        {
          frame_reply(CMD_HANDSHAKE, ST_OK);   /*握手成功，告诉 PC 可以开始擦写了*/
          systick_stop();
          return 1;
        }
        /*窗口里只认握手，别的命令等进了升级模式再说*/
      }
      else if (r == 2U)
      {
        uart1_send_str("[BOOT] bad frame\r\n");   /*打一行，方便上位机核对 CRC 算法有没有对*/
      }
    }

    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0UL)   /*读 CTRL 顺带清 COUNTFLAG，即满 1ms*/
    {
      elapsed++;
    }
  }

  systick_stop();
  return 0;
}

/**
  * 功    能：串口打印一个十进制无符号数
  * 参    数：v 要打印的数
  * 返 回 值：无
  * 补    充：先取余倒着存进数组再正着发，因为除法的结果是从低位开始出的
  */
void uart1_send_dec(uint32_t v)
{
  uint8_t buf[10];
  uint8_t n = 0U;

  if (v == 0UL)
  {
    uart1_send_byte('0');
    return;
  }
  while (v > 0UL)
  {
    buf[n] = (uint8_t)(v % 10UL);
    v /= 10UL;
    n++;
  }
  while (n > 0U)
  {
    n--;
    uart1_send_byte((uint8_t)('0' + buf[n]));
  }
}

/**
  * 功    能：串口打印一个 32 位数，补足 8 位十六进制
  * 参    数：v 要打印的数
  * 返 回 值：无
  * 补    充：只用于把写进 Flash 的内容读回来核对，不做通用格式化
  */
void uart1_send_hex32(uint32_t v)
{
  uint8_t i;
  uint8_t d;

  for (i = 0U; i < 8U; i++)
  {
    d = (uint8_t)((v >> (28U - 4U * i)) & 0xFUL);
    uart1_send_byte((uint8_t)((d < 10U) ? ('0' + d) : ('A' + d - 10U)));
  }
}
