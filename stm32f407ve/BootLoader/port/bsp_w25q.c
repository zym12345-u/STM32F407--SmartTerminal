/* BootLoader/port/bsp_w25q.c
 * W25Q128 极简驱动：寄存器级 SPI1，无 HAL/RTOS/中断/DMA。
 *
 * ── 为什么 BootLoader 里要另写一份，而不是复用 App 的 bsp_W25Q128.c ──
 *   App 那份长在 HAL_SPI 上，BootLoader 的规矩是无 HAL、纯寄存器；
 *   而且 BootLoader 只需要读和擦 4KB，不需要页编程、不需要高带宽。
 *   两份驱动互不影响，App 怎么改都不会动到 BootLoader 的启动路径。
 *
 * ── 为什么只开 4MHz ──
 *   BootLoader 跑 HSI 16MHz，APB2 固定 16MHz；SPI 分频 /4 得 4MHz。
 *   W25Q 的 0x03 读命令本身最高支持 50MHz，4MHz 只是慢，不会读错。
 *   搬运 448KB 时这点速度差可以接受，换来的是不依赖 PLL 配置。
 *
 * ── 超时为什么用循环计数 ──
 *   BootLoader 启动阶段没有 SysTick（main 也没启动 1ms 时基），
 *   所以所有等待都用递减计数。宁可超时返回失败，也不能卡死在启动里。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"
#include "bsp_w25q.h"

#define W25Q_CS_LOW()    (GPIOC->BSRR = (1UL << (13U + 16U)))
#define W25Q_CS_HIGH()   (GPIOC->BSRR = (1UL << 13U))
#define W25Q_SPI_TIMEOUT 200000UL
#define W25Q_BUSY_TIMEOUT 3000000UL

/* SPI1 状态位：TXE/RXNE/BSY */
#define W25Q_SR_TXE      SPI_SR_TXE
#define W25Q_SR_RXNE     SPI_SR_RXNE

/**
  * 功    能：初始化 GPIO 与 SPI1：PA5/6/7 复用 AF5，PC13 推挽输出做 CS
  * 参    数：无
  * 返 回 值：无
  * 补    充：先把 CS 拉高再配引脚，避免上电瞬间选中 Flash 产生杂散时序
  */
static void w25q_hw_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

  /*CS 先写高（此时还是输入模式，写 ODR 只锁存电平），再切输出，
   *否则切输出瞬间 ODR=0 会把 CS 拉低，Flash 被误选中一段杂散时钟*/
  GPIOC->BSRR = (1UL << 13U);

  /*PC13：通用推挽输出，高速*/
  GPIOC->MODER   = (GPIOC->MODER & ~(3UL << 26U)) | (1UL << 26U);
  GPIOC->OTYPER &= ~(1UL << 13U);
  GPIOC->OSPEEDR |= (3UL << 26U);

  /*PA5/PA6/PA7：复用模式，AF5(SPI1)*/
  GPIOA->MODER   = (GPIOA->MODER & ~(0x3FUL << 10U)) | (0x2AUL << 10U);
  GPIOA->OSPEEDR |= (0x3FUL << 10U);
  GPIOA->PUPDR   = (GPIOA->PUPDR & ~(0x3FUL << 10U)) | (0x04UL << 10U);  /*MISO 上拉*/
  GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~(0xFFFUL << 20U)) | (0x555UL << 20U);

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI1->CR1 = 0UL;   /*先关 SPI 再配置*/
  SPI1->CR2 = 0UL;
  (void)SPI1->SR;
  (void)SPI1->DR;

  /*主机 + 软件片选 + 空闲高(CPOL=1) + 第二边沿采样(CPHA=1) + 分频/4*/
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
              SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_0;
  SPI1->CR1 |= SPI_CR1_SPE;

  delay_loop(20000UL);   /*上电后给 Flash 一点准备时间，约几毫秒*/
}

/**
  * 功    能：收发一个字节（全双工，读时发 0xFF）
  * 参    数：tx 要发出的字节
  * 返 回 值：收到的字节；超时返回 0xFF
  */
static uint8_t w25q_xfer(uint8_t tx)
{
  uint32_t t = W25Q_SPI_TIMEOUT;

  while ((SPI1->SR & W25Q_SR_TXE) == 0UL)
  {
    if (--t == 0UL)
    {
      return 0xFFU;
    }
  }
  SPI1->DR = tx;

  t = W25Q_SPI_TIMEOUT;
  while ((SPI1->SR & W25Q_SR_RXNE) == 0UL)
  {
    if (--t == 0UL)
    {
      return 0xFFU;
    }
  }
  return (uint8_t)SPI1->DR;
}

/**
  * 功    能：读状态寄存器 1
  * 参    数：无
  * 返 回 值：SR1 值
  */
static uint8_t w25q_read_sr1(void)
{
  uint8_t sr;

  W25Q_CS_LOW();
  (void)w25q_xfer(0x05U);
  sr = w25q_xfer(0xFFU);
  W25Q_CS_HIGH();
  return sr;
}

/**
  * 功    能：等 Flash 内部操作结束（SR1 bit0 BUSY=0）
  * 参    数：无
  * 返 回 值：0 空闲；1 超时
  * 补    充：4KB 扇区擦除手册最大值约 400ms；3M 次状态轮询在 4MHz 下远超这个时间
  */
static int w25q_wait_busy(void)
{
  uint32_t n = W25Q_BUSY_TIMEOUT;

  while ((w25q_read_sr1() & 0x01U) != 0U)
  {
    if (--n == 0UL)
    {
      return 1;
    }
  }
  return 0;
}

/**
  * 功    能：发写使能命令 0x06，并确认 WEL 位已置起
  * 参    数：无
  * 返 回 值：0 成功；1 失败
  */
static int w25q_write_enable(void)
{
  uint8_t sr;

  W25Q_CS_LOW();
  (void)w25q_xfer(0x06U);
  W25Q_CS_HIGH();

  sr = w25q_read_sr1();
  return ((sr & 0x02U) != 0U) ? 0 : 1;
}

uint32_t bsp_w25q_jedec(void)
{
  uint32_t id = 0UL;

  W25Q_CS_LOW();
  (void)w25q_xfer(0x9FU);
  id  = (uint32_t)w25q_xfer(0xFFU) << 16U;
  id |= (uint32_t)w25q_xfer(0xFFU) << 8U;
  id |= (uint32_t)w25q_xfer(0xFFU);
  W25Q_CS_HIGH();
  return id;
}

int bsp_w25q_init(void)
{
  w25q_hw_init();
  return (bsp_w25q_jedec() == W25Q_JEDEC_EXPECT) ? 1 : 0;
}

void bsp_w25q_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  W25Q_CS_LOW();
  (void)w25q_xfer(0x03U);
  (void)w25q_xfer((uint8_t)(addr >> 16U));
  (void)w25q_xfer((uint8_t)(addr >> 8U));
  (void)w25q_xfer((uint8_t)addr);

  while (len > 0UL)
  {
    *buf = w25q_xfer(0xFFU);
    buf++;
    len--;
  }
  W25Q_CS_HIGH();
}

int bsp_w25q_erase_4k(uint32_t addr)
{
  if (w25q_wait_busy() != 0)
  {
    return 1;
  }
  if (w25q_write_enable() != 0)
  {
    return 1;
  }

  W25Q_CS_LOW();
  (void)w25q_xfer(0x20U);   /*4KB 扇区擦除*/
  (void)w25q_xfer((uint8_t)(addr >> 16U));
  (void)w25q_xfer((uint8_t)(addr >> 8U));
  (void)w25q_xfer((uint8_t)addr);
  W25Q_CS_HIGH();

  return w25q_wait_busy();
}
