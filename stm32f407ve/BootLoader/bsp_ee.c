/* BootLoader/bsp_ee.c
 * 24C02（I2C1）极简轮询驱动 + 升级进度标志。
 *
 * ── 升级断电保护：标志为什么放 24C02 而不是 sector 3 ──
 *   升级过程要随时记下「我在升级、写到哪里了」，这个标志的要求是：
 *   随时可改、改了立刻生效、断电不丢。
 *   内部 Flash 做不到第一点：写之前必须先擦整个扇区(16KB)，擦一次 CPU 停摆 1~2 秒，
 *   每帧数据都擦一次 sector 3 会把升级拖慢几百倍。
 *   24C02 正好合适：字节级直写(0 能改回 1，不用先擦)、写周期 ~5ms、
 *   写的时候 CPU 只在 I2C 上发个地址做 ACK 轮询，不影响内部 Flash 那边的写入。
 *   所以分工是：「升级中标志 + 已写字节数」放 24C02(随时变)；
 *   版本/大小/CRC 这类升级成功后才写一次的元数据，将来放 sector 3。
 *
 * ── BootLoader 里的 I2C 为什么又写了一遍 ──
 *   App 侧有现成的 bsp_I2CBus + bsp_24C02，但它们长在 HAL 和 FreeRTOS 上；
 *   BootLoader 的规矩是无 HAL/RTOS、纯寄存器，所以这里另写一份极简轮询版：
 *   只做 100kHz 标准模式、单页写、8 字节连读，没有中断/DMA/锁。
 *   两份驱动互不影响，App 怎么改都不会动到 BootLoader。
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"
#include "bsp_uart1.h"
#include "bsp_ee.h"

/* 24C02 是否在位；main 上电要打一行 eeprom ok/absent，所以导出给外部读 */
uint8_t g_ee_ok;

/* ── 24C02 / I2C1：极简轮询版，只够读写 OTA 进度页 ── */

/**
  * 功    能：总线恢复：把 SCL 当普通输出口手动打 9 个拍，再补一个 STOP
  * 参    数：无
  * 返 回 值：无
  * 补    充：调试器热复位可能正好打在 App 用 I2C 的半中间，
  *            从机以为自己还在被读、一直拽着 SDA 不放，主机发什么都没 ACK。
  *            9 个 SCL 拍能把从机"没发完的字节"顶完，再给一个 STOP 让它回空闲；
  *            这是带 I2C 的 BootLoader 的标准动作，成本十几行，省一次"玄学"排查
  */
static void i2c1_bus_recover(void)
{
  uint8_t i;

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /*PB6(SCL)/PB7(SDA) 临时配成开漏通用输出：写 1 = 放开(外部上拉拉高)，写 0 = 拉低*/
  GPIOB->MODER   = (GPIOB->MODER & ~(0xFUL << 12U)) | (0x5UL << 12U);
  GPIOB->OTYPER |= (3UL << 6U);

  for (i = 0U; i < 9U; i++)
  {
    GPIOB->BSRR = (1UL << (6U + 16U));   /*SCL 拉低*/
    delay_loop(800UL);
    GPIOB->BSRR = (1UL << 6U);           /*SCL 放开*/
    delay_loop(800UL);
  }

  /*补一个 STOP：SCL 高电平期间 SDA 由低变高*/
  GPIOB->BSRR = (1UL << (7U + 16U));   /*SDA 拉低*/
  delay_loop(800UL);
  GPIOB->BSRR = (1UL << 6U);           /*SCL 放开*/
  delay_loop(800UL);
  GPIOB->BSRR = (1UL << 7U);           /*SDA 放开 → 上升沿即 STOP*/
  delay_loop(800UL);
}

/**
  * 功    能：初始化 I2C1：100kHz 标准模式，PB6=SCL / PB7=SDA
  * 参    数：无
  * 返 回 值：无
  * 补    充：BootLoader 跑 HSI 16MHz、APB1 不分频，所以 FREQ=16；
  *            CCR=80 来自 16MHz/(2×100kHz)，TRISE=FREQ+1（标准模式上升沿 ≤1000ns）；
  *            板上有 R18/R32 两颗 10K 上拉，内部上拉就不开了
  */
static void i2c1_init(void)
{
  i2c1_bus_recover();   /*先把总线顶回空闲，再谈配置*/

  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  /*PB6/PB7 交给 I2C1：复用模式 + 开漏 + AF4*/
  GPIOB->MODER   = (GPIOB->MODER & ~(0xFUL << 12U)) | (0xAUL << 12U);
  GPIOB->OTYPER |= (3UL << 6U);
  GPIOB->OSPEEDR |= (0xFUL << 12U);
  GPIOB->AFR[0]   = (GPIOB->AFR[0] & ~(0xFFUL << 24U)) | (0x44UL << 24U);

  I2C1->CR1 = I2C_CR1_SWRST;   /*软复位把状态机清干净再配*/
  I2C1->CR1 = 0UL;
  I2C1->CR2   = 16UL;          /*PCLK1 = 16MHz*/
  I2C1->CCR   = 80UL;          /*100kHz：16M/(2×100k)=80*/
  I2C1->TRISE = 17UL;
  I2C1->CR1   = I2C_CR1_PE;
}

/**
  * 功    能：发 START + 器件地址字节，等对方应答
  * 参    数：addr 总线地址字节（bit0 区分读/写）
  * 返 回 值：0 收到 ACK；1 NACK 或超时（已补 STOP）
  * 补    充：重复 START 也走这里（主机没释放总线时置 START 就是重复 START）；
  *            NACK 不算故障：EEPROM 内部写周期内连自己的地址都不应答，
  *            ACK 轮询就是靠"它什么时候答了"判断写没写完
  */
static int i2c1_start_addr(uint8_t addr)
{
  uint32_t t;

  I2C1->CR1 |= I2C_CR1_START;
  t = I2C_TIMEOUT;
  while ((I2C1->SR1 & I2C_SR1_SB) == 0UL)
  {
    if (--t == 0UL)
    {
      return 1;
    }
  }

  I2C1->DR = addr;
  t = I2C_TIMEOUT;
  for (;;)
  {
    if ((I2C1->SR1 & I2C_SR1_ADDR) != 0UL)
    {
      break;
    }
    if ((I2C1->SR1 & I2C_SR1_AF) != 0UL)   /*地址没人应：清 AF、补 STOP 收工*/
    {
      I2C1->SR1 &= ~I2C_SR1_AF;
      I2C1->CR1 |= I2C_CR1_STOP;
      return 1;
    }
    if (--t == 0UL)
    {
      I2C1->CR1 |= I2C_CR1_STOP;
      return 1;
    }
  }

  (void)I2C1->SR1;
  (void)I2C1->SR2;   /*读 SR1 再读 SR2，ADDR 标志才会清掉，不清后面全乱*/
  return 0;
}

/**
  * 功    能：写一个字节并等它真正出完（BTF）
  * 参    数：b 要写的字节
  * 返 回 值：0 成功；1 NACK 或超时
  * 补    充：TXE 只说明数据寄存器空了，字节可能还在移位寄存器里；
  *            等 BTF 才是"这一字节完整发出且收到 ACK"，页写最后一个字节尤其要这个
  */
static int i2c1_write_byte(uint8_t b)
{
  uint32_t t;

  t = I2C_TIMEOUT;
  while ((I2C1->SR1 & I2C_SR1_TXE) == 0UL)
  {
    if (--t == 0UL)
    {
      return 1;
    }
  }
  I2C1->DR = b;

  t = I2C_TIMEOUT;
  for (;;)
  {
    if ((I2C1->SR1 & I2C_SR1_AF) != 0UL)   /*从机 NACK 了这个数据字节*/
    {
      I2C1->SR1 &= ~I2C_SR1_AF;
      return 1;
    }
    if ((I2C1->SR1 & I2C_SR1_BTF) != 0UL)
    {
      return 0;
    }
    if (--t == 0UL)
    {
      return 1;
    }
  }
}

static void i2c1_stop(void)
{
  I2C1->CR1 |= I2C_CR1_STOP;
}

/**
  * 功    能：探测 24C02 在不在（只发地址字节，不产生任何写入）
  * 参    数：无
  * 返 回 值：0 在；1 不在
  * 补    充：ACK 轮询也复用它：写周期内器件不应答，应了 = 写完了
  */
static int ee_probe(void)
{
  if (i2c1_start_addr(EE_I2C_ADDR_W) != 0)
  {
    return 1;
  }
  i2c1_stop();
  return 0;
}

/**
  * 功    能：ACK 轮询，等 EEPROM 内部写周期结束（手册 max 5ms）
  * 参    数：无
  * 返 回 值：0 写完；1 一直不应答（超时）
  * 补    充：2000 次探测 × 每次几十 µs ≈ 几十 ms 上限，远超 5ms
  */
static int ee_wait_ready(void)
{
  uint32_t n = 2000UL;

  while (n > 0UL)
  {
    if (ee_probe() == 0)
    {
      return 0;
    }
    n--;
  }
  return 1;
}

/**
  * 功    能：写一整页 8 字节（24C02 页写不能跨页，OTA 进度正好一页）
  * 参    数：mem 页对齐的内存地址；data 8 字节
  * 返 回 值：0 成功；1 失败
  * 补    充：STOP 之后写周期才开始，所以末尾必须 ACK 轮询确认写完，
  *            紧接着的下一次读/写才不会吃到 NACK
  */
static int ee_write_page(uint8_t mem, const uint8_t *data)
{
  uint8_t i;

  if (i2c1_start_addr(EE_I2C_ADDR_W) != 0)
  {
    return 1;
  }
  if (i2c1_write_byte(mem) != 0)
  {
    i2c1_stop();
    return 1;
  }
  for (i = 0U; i < 8U; i++)
  {
    if (i2c1_write_byte(data[i]) != 0)
    {
      i2c1_stop();
      return 1;
    }
  }
  i2c1_stop();

  return ee_wait_ready();
}

/**
  * 功    能：从指定内存地址连读 8 字节
  * 参    数：mem 起始内存地址；buf 读出的 8 字节放这里
  * 返 回 值：0 成功；1 失败
  * 补    充：F4 主机接收 N>2 的标准收法：前 N-3 字节 RXNE 逐个读；
  *            剩 3 个时等 BTF → 关 ACK → 读一个 → 摆 STOP → 再读两个。
  *            顺序乱一点就会多读/少读/不出 STOP，这段别随手改
  */
static int ee_read8(uint8_t mem, uint8_t *buf)
{
  uint32_t t;
  uint8_t  i;

  /*伪写：先写内存地址，把 EEPROM 内部的地址计数器指到 mem*/
  if (i2c1_start_addr(EE_I2C_ADDR_W) != 0)
  {
    return 1;
  }
  if (i2c1_write_byte(mem) != 0)
  {
    i2c1_stop();
    return 1;
  }

  /*重复 START 换读方向；ACK 必须在这之前打开，前 7 个字节都要回 ACK*/
  I2C1->CR1 |= I2C_CR1_ACK;
  if (i2c1_start_addr(EE_I2C_ADDR_R) != 0)
  {
    return 1;
  }

  for (i = 0U; i < 5U; i++)   /*前 5 个：RXNE 一个读一个*/
  {
    t = I2C_TIMEOUT;
    while ((I2C1->SR1 & I2C_SR1_RXNE) == 0UL)
    {
      if (--t == 0UL)
      {
        i2c1_stop();
        return 1;
      }
    }
    buf[i] = (uint8_t)I2C1->DR;
  }

  t = I2C_TIMEOUT;   /*BTF=1：第 6 字节在 DR、第 7 字节在移位寄存器，时钟已被拉伸暂停*/
  while ((I2C1->SR1 & I2C_SR1_BTF) == 0UL)
  {
    if (--t == 0UL)
    {
      i2c1_stop();
      return 1;
    }
  }
  I2C1->CR1 &= ~I2C_CR1_ACK;   /*最后一字节收完要回 NACK，告诉从机"就到这"*/
  buf[5] = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_STOP;   /*STOP 要在读最后两字节之前摆好*/

  t = I2C_TIMEOUT;
  while ((I2C1->SR1 & I2C_SR1_RXNE) == 0UL)
  {
    if (--t == 0UL)
    {
      return 1;
    }
  }
  buf[6] = (uint8_t)I2C1->DR;

  t = I2C_TIMEOUT;
  while ((I2C1->SR1 & I2C_SR1_RXNE) == 0UL)
  {
    if (--t == 0UL)
    {
      return 1;
    }
  }
  buf[7] = (uint8_t)I2C1->DR;

  return 0;
}

/* ── OTA 进度：magic + 已写字节数，一页 8 字节 ── */

/**
  * 功    能：初始化 I2C1 并探测 24C02 在不在
  * 参    数：无
  * 返 回 值：无
  * 补    充：探测失败置 g_ee_ok=0，之后所有标志操作静默跳过：
  *            EEPROM 挂了只是丢了断电保护，不能连累整块板子起不来
  */
void ee_init(void)
{
  i2c1_init();
  g_ee_ok = (ee_probe() == 0) ? 1U : 0U;
}

/**
  * 功    能：置升级中标志 / 更新已写字节数
  * 参    数：written 当前已写入 Flash 的字节总数
  * 返 回 值：无
  * 补    充：写失败说明 EEPROM 中途掉了，关掉 g_ee_ok 不再每帧重试拖慢升级；
  *            代价是这次升级失去断电保护，串口会打一行提示
  */
void ota_mark(uint32_t written)
{
  uint8_t buf[8];

  if (g_ee_ok == 0U)
  {
    return;
  }

  buf[0] = (uint8_t)(EE_OTA_MAGIC);
  buf[1] = (uint8_t)(EE_OTA_MAGIC >> 8U);
  buf[2] = (uint8_t)(EE_OTA_MAGIC >> 16U);
  buf[3] = (uint8_t)(EE_OTA_MAGIC >> 24U);
  buf[4] = (uint8_t)(written);
  buf[5] = (uint8_t)(written >> 8U);
  buf[6] = (uint8_t)(written >> 16U);
  buf[7] = (uint8_t)(written >> 24U);

  if (ee_write_page(EE_OTA_MEM, buf) != 0)
  {
    g_ee_ok = 0U;
    uart1_send_str("[BOOT] ee write fail, ota flag off\r\n");
  }
}

/**
  * 功    能：清升级中标志（升级顺利走完才调）
  * 参    数：无
  * 返 回 值：无
  * 补    充：magic 写 0 就算清掉，written 一并归零，下次升级从干净状态开始；
  *            万一清不掉，下次启动还会进升级模式，PC 端重走一遍即可恢复
  */
void ota_clear(void)
{
  uint8_t buf[8] = {0U};

  if (g_ee_ok == 0U)
  {
    return;
  }
  if (ee_write_page(EE_OTA_MEM, buf) != 0)
  {
    uart1_send_str("[BOOT] ee clear fail\r\n");
  }
}

/**
  * 功    能：查升级中标志在不在
  * 参    数：无
  * 返 回 值：1 在（上次升级中途断电）；0 不在
  * 补    充：读失败一律当"不在"处理，回到窗口 + 向量表校验的老路
  */
int ota_is_marked(void)
{
  uint8_t  buf[8];
  uint32_t magic;

  if (g_ee_ok == 0U)
  {
    return 0;
  }
  if (ee_read8(EE_OTA_MEM, buf) != 0)
  {
    g_ee_ok = 0U;
    return 0;
  }

  magic = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8U) |
          ((uint32_t)buf[2] << 16U) | ((uint32_t)buf[3] << 24U);
  return (magic == EE_OTA_MAGIC) ? 1 : 0;
}

/**
  * 功    能：从 24C02 读回「已写字节数」，断电续传用
  * 参    数：无
  * 返 回 值：已写字节数；标志不存在或 EEPROM 不在位时返回 0
  * 补    充：magic 不核对就不能信后面 4 字节——自检页残留或总线噪声都可能
  *            拼出一个像模像样的数字，拿它当偏移会把数据写到错误位置。
  *            代价只是多读一次同一个 8 字节页。
  */
uint32_t ota_read_written(void)
{
  uint8_t  buf[8];
  uint32_t magic;

  if (g_ee_ok == 0U)
  {
    return 0UL;
  }
  if (ee_read8(EE_OTA_MEM, buf) != 0)
  {
    g_ee_ok = 0U;
    return 0UL;
  }
  magic = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8U) |
          ((uint32_t)buf[2] << 16U) | ((uint32_t)buf[3] << 24U);
  if (magic != EE_OTA_MAGIC)
  {
    return 0UL;
  }
  return (uint32_t)buf[4] | ((uint32_t)buf[5] << 8U) |
         ((uint32_t)buf[6] << 16U) | ((uint32_t)buf[7] << 24U);
}
