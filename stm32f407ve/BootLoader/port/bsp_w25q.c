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
 *
 * ── 为什么用 0x03（Read Data）而不是 0x0B（Fast Read）──
 *   0x03 是 24 位地址 + 直接出数据，时序最简单：发完命令和 3 字节地址，
 *   后面每个时钟都对应一个数据字节。
 *   0x0B 要多发 1 个 dummy 字节、还常配合 0x3B 的 8 位地址模式省 2 字节；
 *   在 4MHz 下那点速度差毫无意义，多一个"dummy 字节个数"的分支反而更容易写错。
 *   同理不用 Quad/DMA：搬运只在 OTA 复位那一次发生，慢一点没人感知。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"
#include "bsp_w25q.h"

/*片选：PC13 低有效。BSRR 高 16 位写 1 = 复位该引脚（输出低），
 *低 16 位写 1 = 置位（输出高）。用 BSRR 而不是位带/读改写，
 *是因为它是单次原子写，不会被中断打断成"先读、再改、再写"的竞态*/
#define W25Q_CS_LOW()    (GPIOC->BSRR = (1UL << (13U + 16U)))
#define W25Q_CS_HIGH()   (GPIOC->BSRR = (1UL << 13U))
/*单字节 SPI 事务的超时：200000 次递减 @16MHz 约 37ms，
 *正常一次收发只要几微秒，37ms 已经等于"卡死了"*/
#define W25Q_SPI_TIMEOUT 200000UL
/*BUSY 轮询上限：3000000 次读状态 ≈ 十几秒。
 *W25Q 手册给 4KB 扇区擦除最大 400ms（典型 45ms），3M 次远超这个量级，
 *真的超时只可能是芯片没接好，此时返回失败比死等正确*/
#define W25Q_BUSY_TIMEOUT 3000000UL

/* SPI1 状态位：TXE=发送缓冲空、RXNE=接收缓冲非空 */
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

  /*PC13：通用推挽输出，高速。
   *MODER 每位段 2 bit：00=输入 01=输出 10=复用 11=模拟。
   *PC13 对应 bit26~27，先清成 00 再置 01。
   *OTYPER 0=推挽、1=开漏：片选必须推挽，开漏要靠外部上拉才能拉高，边沿会变慢*/
  GPIOC->MODER   = (GPIOC->MODER & ~(3UL << 26U)) | (1UL << 26U);
  GPIOC->OTYPER &= ~(1UL << 13U);
  GPIOC->OSPEEDR |= (3UL << 26U);      /*最高速度档：片选翻转要快*/

  /*PA5(SCK)/PA6(MISO)/PA7(MOSI)：复用模式，AF5(SPI1)。
   *0x2A = 二进制 101010，正好是三个 "10"(复用)；
   *0x555 = 0101 0101 0101，三个半字节都是 5 → PA5/6/7 全选 AF5*/
  GPIOA->MODER   = (GPIOA->MODER & ~(0x3FUL << 10U)) | (0x2AUL << 10U);
  GPIOA->OSPEEDR |= (0x3FUL << 10U);
  /*0x04 落在 PA6 的位段上（每引脚 2 bit，PA6 是 bit12~13）：
   *MISO 是输入，上拉一下，避免 Flash 未选中时引脚悬空、读回随机噪声*/
  GPIOA->PUPDR   = (GPIOA->PUPDR & ~(0x3FUL << 10U)) | (0x04UL << 10U);  /*MISO 上拉*/
  GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~(0xFFFUL << 20U)) | (0x555UL << 20U);

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /*先关 SPI 再配置：SPE=0 时改 CR1 才安全，运行中改分频/模式会让时序错位*/
  SPI1->CR1 = 0UL;
  SPI1->CR2 = 0UL;
  /*空读 SR 和 DR 把上电残留的 OVR/MODF 标志清掉；
   *否则第一次收发可能立刻进错误状态*/
  (void)SPI1->SR;
  (void)SPI1->DR;

  /*主机 + 软件片选 + 空闲高(CPOL=1) + 第二边沿采样(CPHA=1) + 分频/4
   *  MSTR  : 主机模式
   *  SSM+SSI: 软件管理片选。SSM=1 时 SSI=1 表示"没有其他主设备"，
   *           否则 SPI 会以为总线被抢占而自动退成从机（MODF）
   *  CPOL=1 / CPHA=1 : 时钟空闲高、第二个边沿采样 = SPI Mode 3。
   *           W25Q 支持 Mode 0 和 Mode 3，选 Mode 3 是为了和 App 侧 bsp_W25Q128.c 行为一致
   *  BR_0  : BR[2:0]=001 → PCLK/4。BootLoader 的 PCLK2 = HSI 16MHz，
   *          16/4 = 4MHz（见文件头"为什么只开 4MHz"）*/
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
              SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_0;
  SPI1->CR1 |= SPI_CR1_SPE;

  /*Flash 上电内部需要 tPUW（约 5ms，手册最大 20ms）才能接受命令。
   *BootLoader 到这里可能刚上电几百微秒，不等一下 JEDEC 会读到 0*/
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

  /*SPI 是全双工：想收一个字节，就必须发一个字节把时钟打出来。
   *所以"读"不是特殊路径，只是发的是 0xFF（哑元）而已*/
  while ((SPI1->SR & W25Q_SR_TXE) == 0UL)    /*等发送缓冲空，才能写 DR*/
  {
    if (--t == 0UL)
    {
      return 0xFFU;      /*超时：返回 0xFF，与"Flash 里就是 0xFF"同值，
                          *上层只能靠 CRC/magic 兜底——BootLoader 不做日志级区分*/
    }
  }
  SPI1->DR = tx;

  t = W25Q_SPI_TIMEOUT;
  while ((SPI1->SR & W25Q_SR_RXNE) == 0UL)   /*等接收缓冲非空，才能读 DR*/
  {
    if (--t == 0UL)
    {
      return 0xFFU;
    }
  }
  return (uint8_t)SPI1->DR;   /*读 DR 同时会清 RXNE，不需要额外操作*/
}

/**
  * 功    能：读状态寄存器 1
  * 参    数：无
  * 返 回 值：SR1 值
  */
static uint8_t w25q_read_sr1(void)
{
  uint8_t sr;

  /*0x05 = Read Status Register-1。SR1 的位：
   *  bit0 BUSY  : 内部擦/写进行中，此时只认 0x05 和 0x04，别的命令会被忽略
   *  bit1 WEL   : 写使能锁存，每次擦/写后芯片自动清零，必须每次重新发 0x06
   *  bit2~6 BP  : 块保护，被保护的区域拒绝擦写（现象是"写命令成功但数据没变"）
   *  bit7 SRWD  : 状态寄存器写保护开关*/
  W25Q_CS_LOW();
  (void)w25q_xfer(0x05U);
  sr = w25q_xfer(0xFFU);     /*发哑元把时钟打出来，才收得到 SR1*/
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

  /*忙等 BUSY 位。这不是"没必要的轮询"——擦除期间芯片只认 0x05/0x04，
   *不等它变 0 就发下一条命令，下一条会被静默丢弃，
   *现象就是"命令都发了、数据却没变"，最难查的一类问题*/
  while ((w25q_read_sr1() & 0x01U) != 0U)
  {
    if (--n == 0UL)
    {
      return 1;     /*超时：芯片没接好或供电异常，交回上层决定放弃*/
    }
  }
  return 0;
}

/**
  * 功    能：发写使能命令 0x06，并确认 WEL 位已置起
  * 参    数：无
  * 返 回 值：0 成功；1 失败
  * 补    充：WEL 是"一次性"的——芯片每执行完一次擦或写就自动把它清 0，
  *            所以每次擦/写之前都要重新发 0x06。发完必须读回 SR1 确认，
  *            只发不验会在写保护生效时误以为已经使能。
  */
static int w25q_write_enable(void)
{
  uint8_t sr;

  W25Q_CS_LOW();
  (void)w25q_xfer(0x06U);
  W25Q_CS_HIGH();          /*CS 拉高命令才真正生效，命令是"片选期间的一帧"*/

  sr = w25q_read_sr1();
  return ((sr & 0x02U) != 0U) ? 0 : 1;   /*0=成功，1=WEL 没置起来*/
}

uint32_t bsp_w25q_jedec(void)
{
  uint32_t id = 0UL;

  /*0x9F = Read JEDEC ID，返回 3 字节厂商+容量：
   *W25Q128 是 EF 40 18（EF=Winbond，40=SPI NOR，18=128Mbit=16MB）。
   *用它而不是 0x90(读厂商/设备 ID) 的原因：0x90 要 3 字节地址才能拿到第二字节，
   *且不同容量芯片的设备 ID 容易撞；JEDEC 是唯一标识，一步到位。
   *（这一条来自 W25Q128 驱动规格草案里的实测对比）*/
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
  /*返回值语义：1 = 芯片在位且 ID 对得上，0 = 不在位/型号不符。
   *上层据此决定"OTA 跳过"还是"继续搬运"，所以这里不能省略 ID 校验——
   *接触不良时读到的常常是全 0x00 或全 0xFF，不校验会拿垃圾数据当真*/
  return (bsp_w25q_jedec() == W25Q_JEDEC_EXPECT) ? 1 : 0;
}

void bsp_w25q_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  W25Q_CS_LOW();
  (void)w25q_xfer(0x03U);                   /*0x03 = Read Data，无 dummy 字节*/
  (void)w25q_xfer((uint8_t)(addr >> 16U));  /*24 位地址，高字节先发*/
  (void)w25q_xfer((uint8_t)(addr >> 8U));
  (void)w25q_xfer((uint8_t)addr);

  /*片选保持低电平时地址会自动递增，所以一次可以连续读很长一段，
   *中间不需要重发命令——448KB 的整区校验就是这样一遍读完的*/
  while (len > 0UL)
  {
    *buf = w25q_xfer(0xFFU);    /*发哑元产生时钟，收一个字节*/
    buf++;
    len--;
  }
  W25Q_CS_HIGH();
}

int bsp_w25q_erase_4k(uint32_t addr)
{
  /*顺序固定：先确认上一次操作已结束，再发写使能，最后发擦除命令。
   *反过来（先写使能再等 BUSY）会把使能位等丢——芯片在忙时会忽略 0x06*/
  if (w25q_wait_busy() != 0)
  {
    return 1;
  }
  if (w25q_write_enable() != 0)
  {
    return 1;
  }

  W25Q_CS_LOW();
  (void)w25q_xfer(0x20U);   /*4KB 扇区擦除（0xD8 是 64KB 块擦，0xC7 是整片擦）*/
  (void)w25q_xfer((uint8_t)(addr >> 16U));
  (void)w25q_xfer((uint8_t)(addr >> 8U));
  (void)w25q_xfer((uint8_t)addr);
  W25Q_CS_HIGH();           /*拉高片选才真正开始擦，典型 45ms、最大 400ms*/

  return w25q_wait_busy();  /*等擦完再返回，调用方拿到 0 才算擦成功*/
}
