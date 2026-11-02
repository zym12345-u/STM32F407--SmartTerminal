/* BootLoader/main.c
 * 上电先开 3 秒串口升级窗口，窗口结束再校验 App 区向量表，合法则跳到 App，不合法则闪灯提示。
 * 分区：BootLoader 48KB，元数据 16KB，App 448KB（元数据独占整个 sector 3）。
 *
 * ══════ 分区怎么切、升级窗口怎么开、跳转为什么要做一串动作 ══════
 *
 * ── 分区位置不是随便定的，得跟着【扇区边界】走 ──
 *   STM32F407 内部 Flash 的扇区大小是【不均匀】的（这点很关键）：
 *     sector 0~3   ：每个 16KB   → 0x08000000 ~ 0x0800FFFF
 *     sector 4     ：64KB        → 0x08010000 ~ 0x0801FFFF
 *     sector 5~11  ：每个 128KB  → 0x08020000 起
 *   而擦除的最小单位就是【整个扇区】，所以：
 *     - 元数据只需要几十字节，但最少也得占一整个扇区 → 塞进 sector 3（16KB）
 *     - App 从 sector 4 开始（0x08010000），正好接在元数据后面，边界天然对齐
 *     - 如果硬要让 App 从 0x0800C000 开始，就跨在 sector 3 中间，
 *       将来单独升级 App 时没法只擦 App 那部分（会连带擦掉元数据）
 *   这就是「分区大小必须等于整数个扇区」的原因。
 *
 * ── 为什么用「上电 3 秒窗口」而不是 BOOT0 按键 ──
 *   BOOT0 只在【复位那一刻】被采样，程序跑起来以后再改它没用，必须断电重上电；
 *   而且板子上要为此多留一个跳线帽或按键，升级时还得手动拨，容易拨错。
 *   串口窗口没有这些麻烦：BootLoader 一上电先停 3 秒等握手帧，
 *   收到就进升级流程，收不到就照常校验并跳 App —— 工具端只要打开串口就行。
 *   窗口必须有【明确的长度】，不能一直等，否则没有升级请求时 App 永远起不来。
 *   3 秒是折中：够 PC 端把 USB 虚拟串口枚举完再发帧，又不会让正常上电等太久。
 *
 * ── 为什么要校验 App 的向量表 ──
 *   App 区可能是空的（全 0xFF，刚出厂或刚被擦掉），也可能只写了一半（升级掉电）。
 *   直接跳过去就是跑飞跑进 HardFault，而且看不出原因。
 *   所以跳之前先看 App 那 8 个字节（向量表开头）合不合理：
 *     第 0~3 字节：初始栈顶 MSP → 必须落在 SRAM 范围内，且 8 字节对齐
 *     第 4~7 字节：复位向量   → bit0 必须为 1（Thumb 地址），且落在 App 区内
 *   这两个条件就是「这段 Flash 里像不像一个能跑的程序」的最低判据。
 *
 * ── 跳转前为什么必须做那一串动作（顺序也有讲究）──
 *   1) 关全局中断    ：跳转过程中不能被打断，此时中断向量还指向 BootLoader
 *   2) 停 SysTick    ：否则每 1ms 触发一次，而对应的 handler 马上就不归我们管了
 *   3) 清 NVIC 的使能与挂起位：App 一开中断，不会突然跳回 BootLoader 里残留的 handler
 *   4) 设 MSP        ：用 App 向量表里的栈顶值，让 App 有自己的栈
 *   5) 改 SCB->VTOR  ：让中断向量表指向 App 的，否则 App 里所有中断都会跑错地方
 *   6) 取复位向量跳过去（函数指针）
 *   顺序错了的典型症状：能跳过去但一开中断就 HardFault，或者中断进了却在执行旧代码。
 *   第 3 步必须在第 5 步之前 —— 先把旧的清干净，再换新的。
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
 *
 * ── 三条升级命令怎么配合 ──
 *   0x04 擦除（带起始/结束地址）→ 0x02 数据块（不带地址）→ 0x03 结束。
 *   数据块故意不带地址：地址由设备自己往后加，PC 只管按顺序发；
 *   每帧都等 ACK 才发下一帧（停等），漏帧会被 CRC 拦下重发，顺序天然不会乱，
 *   PC 端也不用算偏移，少一类出错的可能。
 *   每条命令都回一帧应答：命令号 | 0x80，数据 1 字节状态 —— 0 成功、1 地址/长度不对、2 Flash 出错。
 *   擦除和编程只允许落在 App 区（0x08010000~0x0807FFFF，即 sector 4~7），
 *   而且必须按扇区边界对齐；这样 sector 0~2（BootLoader 自己）和 sector 3（元数据）
 *   永远不会被这条命令碰到 —— 越界的请求直接回 1，不硬写。
 *   写每个字之前先读一遍，不是 0xFFFFFFFF 就报错：说明这扇区没擦干净，
 *   不做「悄悄帮你擦掉」这种事，免得把还在用的数据抹了。
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
 * ── 启动时先查标志再决定干什么 ──
 *   升级中途断电，App 区是个写了一半的镜像 —— 向量表校验"碰巧合法"是可能的
 *   (只要前 8 字节刚好写完)，直接跳过去照样跑飞。
 *   所以 BootLoader 上电先读 24C02 的 magic：在 → 上次升级没走完，
 *   跳过窗口直接进升级模式等 PC 重传；不在 → 照旧窗口 + 校验 + 跳转。
 *   24C02 探测失败(没焊/挂了)就当没有标志走原流程 —— 一片 EEPROM 挂了
 *   不能让整块板子起不来，向量表校验还是最后的底。
 *
 * ── BootLoader 里的 I2C 为什么又写了一遍 ──
 *   App 侧有现成的 bsp_I2CBus + bsp_24C02，但它们长在 HAL 和 FreeRTOS 上；
 *   BootLoader 的规矩是无 HAL/RTOS、纯寄存器，所以这里另写一份极简轮询版：
 *   只做 100kHz 标准模式、单页写、8 字节连读，没有中断/DMA/锁。
 *   两份驱动互不影响，App 怎么改都不会动到 BootLoader。
 */

#include <stdint.h>
#include "stm32f4xx.h"

/* 分区常量：IAP 和 App 侧共用同一组地址 */
#define APP_BASE_ADDR     0x08010000UL   /*App 复位向量（sector 4 起点）*/
#define APP_LIMIT_ADDR    0x08080000UL   /*App 区上界（不含）*/
#define META_BASE_ADDR    0x0800C000UL   /*升级元数据（sector 3）*/
#define META_LIMIT_ADDR   0x08010000UL   /*元数据区上界（不含）*/
#define SRAM_BASE_ADDR    0x20000000UL   /*SRAM 起始*/
#define SRAM_LIMIT_ADDR   0x20020000UL   /*SRAM 上界 128KB*/

/* 板载 LED（诊断用）：PC5 低电平点亮 */
#define LED_GPIO_CLK_EN   RCC_AHB1ENR_GPIOCEN
#define LED_PORT          GPIOC
#define LED_PIN           5U
#define LED_PIN_MASK      (1UL << LED_PIN)

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
#define CMD_END           0x03U      /*结束，长度 0*/
#define CMD_ERASE         0x04U      /*擦除，数据 8 字节：起始地址 + 结束地址(不含)*/
#define CMD_ACK_FLAG      0x80U      /*应答帧的命令号 = 原命令号 | 0x80*/
#define FRAME_DATA_MAX    128U       /*一帧最多 128 字节数据，正好 32 个字*/

/* 应答帧里那 1 个状态字节的取值 */
#define ST_OK             0x00U      /*成功*/
#define ST_RANGE          0x01U      /*地址越界 / 长度不对 / 没按扇区对齐*/
#define ST_FLASH          0x02U      /*擦除或编程失败（写前发现没擦干净也算）*/

/* 内部 Flash 解锁序列，少写一个字节解不了锁，顺序也不能反 */
#define FLASH_KEY1        0x45670123UL
#define FLASH_KEY2        0xCDEF89ABUL

/* 24C02（板载 EEPROM，I2C1 PB6/PB7）：只用来存「升级中标志 + 已写字节数」 */
#define EE_I2C_ADDR_W     0xA0U        /*7 位地址 0x50 左移 1 位 + 写*/
#define EE_I2C_ADDR_R     0xA1U        /*同地址 + 读*/
#define EE_OTA_MEM        0xF0U        /*进度页：magic(4B) + written(4B)，正好一页，不碰自检页 0xF8~0xFF*/
#define EE_OTA_MAGIC      0x32475055UL /*内存里字节序是 'U''P''G''2'，页首是这个值 = 升级中途断电*/
#define I2C_TIMEOUT       100000UL     /*每步最久等几十 ms，总线死了也不能把启动卡死*/

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

/* 升级过程的写入游标：擦除成功后从 App 基址开始，按收到的字节顺序往后长 */
static uint32_t g_write_addr;
static uint32_t g_written;
static uint8_t  g_word_buf[4];   /*F4 内部 Flash 只能按字编程，凑够 4 字节才写一次*/
static uint8_t  g_word_n;
static uint8_t  g_ee_ok;         /*0 = EEPROM 不在位，之后所有 OTA 标志操作静默跳过*/

static void uart1_init(void);
static void uart1_send_str(const char *s);
static void uart1_send_byte(uint8_t b);
static void uart1_send_dec(uint32_t v);
static void uart1_send_hex32(uint32_t v);
static uint32_t crc32_update(uint32_t crc, uint8_t b);
static void frame_send(uint8_t cmd, const uint8_t *data, uint8_t len);
static void frame_reply(uint8_t cmd, uint8_t st);
static void frame_reset(frame_parser_t *p);
static uint8_t frame_feed(frame_parser_t *p, uint8_t ch);
static int  uart1_wait_handshake(uint32_t ms);
static void upgrade_loop(void);
static void flash_clear_errors(void);
static void flash_unlock(void);
static void flash_lock(void);
static int  flash_busy_wait(void);
static int  flash_erase_sector(uint8_t snb);
static int  flash_program_word(uint32_t addr, uint32_t word);
static uint32_t flash_sector_start(uint8_t snb);
static uint32_t flash_sector_size(uint8_t snb);
static uint8_t  flash_sector_of(uint32_t addr);
static uint8_t  upgrade_erase(const uint8_t *data, uint8_t len);
static uint8_t  upgrade_write(const uint8_t *data, uint8_t len);
static uint8_t  upgrade_finish(void);
static void systick_start_1ms(void);
static void systick_stop(void);
static void led_init(void);
static void led_on(void);
static void led_off(void);
static void delay_loop(uint32_t n);
static int  app_is_valid(void);
static void jump_to_app(void);
static void i2c1_bus_recover(void);
static void i2c1_init(void);
static int  i2c1_start_addr(uint8_t addr);
static int  i2c1_write_byte(uint8_t b);
static void i2c1_stop(void);
static void ee_init(void);
static int  ee_probe(void);
static int  ee_wait_ready(void);
static int  ee_write_page(uint8_t mem, const uint8_t *data);
static int  ee_read8(uint8_t mem, uint8_t *buf);
static void ota_mark(uint32_t written);
static void ota_clear(void);
static int  ota_is_marked(void);
static void rst_cause_dump(void);

int main(void)
{
  /* 窗口内等握手帧：等到就进升级模式，等不到就照旧校验并跳 App */
  uart1_init();
  rst_cause_dump();                            /*先报本次复位原因，排查中途重启*/
  uart1_send_str("\r\n[BOOT] window 3s\r\n");   /*打一行横幅，串口助手能自证"现在跑的是 BootLoader"*/

  ee_init();
  uart1_send_str((g_ee_ok != 0U) ? "[BOOT] eeprom ok\r\n" : "[BOOT] eeprom absent\r\n");

  if (ota_is_marked() != 0)
  {
    /*上次升级中途断电：App 区不可信，跳过窗口直接进升级模式等 PC 重传*/
    uart1_send_str("[BOOT] ota flag found, resume upgrade\r\n");
    upgrade_loop();
  }

  if (uart1_wait_handshake(UART_WINDOW_MS))
  {
    uart1_send_str("[BOOT] upgrade mode\r\n");
    upgrade_loop();   /*升级模式：等擦除/数据块/结束三条命令*/
  }

  uart1_send_str("[BOOT] window closed\r\n");

  if (!app_is_valid())
  {
    /* App 无效就闪灯：灯在闪说明 BootLoader 活着，只是 App 区没有固件 */
    led_init();
    for (;;)
    {
      led_on();
      delay_loop(800000UL);
      led_off();
      delay_loop(800000UL);
    }
  }

  jump_to_app();

  /* 正常情况下不会返回 */
  for (;;)
  {
  }
}

/**
  * 功    能：打印本次复位的原因，然后把标志清掉
  * 参    数：无
  * 返 回 值：无
  * 补    充：这是排查"升级到一半板子自己重启"最省事的证据。
  *            RCC->CSR 里每一位对应一种复位源，读出即知：
  *              POR = 上电复位（电源从 0 爬起来）
  *              BOR = 掉电复位（电压跌破 BOR 门限，擦写 Flash 电流冲击最常见）
  *              PIN = NRST 引脚被拉低（按键、外部干扰、复位芯片）
  *              IWDG/WWDG = 看门狗（BootLoader 根本没开，出现就是异常）
  *              SFT = 软件复位（代码里主动 NVIC_SystemReset）
  *              LPWR = 低功耗相关复位
  *            多个位可能同时置 1（比如掉电后再上电），所以全部打印不取第一个。
  *            读完立刻写 RMVF 清除：不清的话下次上电看到的还是"历史上任意一次"的残留，
  *            证据就失效了。
  */
static void rst_cause_dump(void)
{
  uint32_t csr = RCC->CSR;

  uart1_send_str("[BOOT] rst:");
  if ((csr & RCC_CSR_LPWRRSTF) != 0UL) { uart1_send_str(" LPWR"); }
  if ((csr & RCC_CSR_WWDGRSTF) != 0UL) { uart1_send_str(" WWDG"); }
  if ((csr & RCC_CSR_IWDGRSTF) != 0UL) { uart1_send_str(" IWDG"); }
  if ((csr & RCC_CSR_SFTRSTF)  != 0UL) { uart1_send_str(" SFT");  }
  if ((csr & RCC_CSR_PORRSTF)  != 0UL) { uart1_send_str(" POR");  }
  if ((csr & RCC_CSR_PINRSTF)  != 0UL) { uart1_send_str(" PIN");  }
  if ((csr & RCC_CSR_BORRSTF)  != 0UL) { uart1_send_str(" BOR");  }
  uart1_send_str("\r\n");

  RCC->CSR |= RCC_CSR_RMVF;
}

/**
  * 功    能：判断 App 区有没有能跑的固件：读它向量表的前两个字（栈顶、复位向量）
  * 参    数：无
  * 返 回 值：1 有效，0 无效
  * 补    充：栈顶必须落在 SRAM 范围内且 8 字节对齐（不对齐用 64 位变量就 UsageFault）；
  *            复位向量 bit0 必须为 1（Thumb 地址，为 0 立刻 HardFault）且落在 App 区内
  */
static int app_is_valid(void)
{
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  if ((app_msp < SRAM_BASE_ADDR) || (app_msp >= SRAM_LIMIT_ADDR))
  {
    return 0;
  }
  if ((app_msp & 0x7UL) != 0UL)   /*栈必须 8 字节对齐，否则用 64 位变量就 UsageFault*/
  {
    return 0;
  }
  if (((app_reset & 1UL) == 0UL) ||   /*bit0 必须为 1，Thumb 地址，为 0 立刻 HardFault*/
      (app_reset < APP_BASE_ADDR) || (app_reset >= APP_LIMIT_ADDR))
  {
    return 0;
  }
  return 1;
}

/**
  * 功    能：从 BootLoader 跳到 App 执行：先关中断、停 SysTick、清 NVIC 的使能与挂起位，
  * 参    数：无
  * 返 回 值：无
  * 补    充：再把 MSP 设成 App 的栈顶、VTOR 指向 App 的向量表，最后按复位向量跳过去
  *            顺序不能乱 —— 尤其是清 NVIC 必须在改 VTOR 之前，否则 App 一开中断就跳回 BootLoader
  */
static void jump_to_app(void)
{
  typedef void (*app_entry_t)(void);

  uint32_t i;
  uint32_t app_msp   = *(volatile uint32_t *)(APP_BASE_ADDR);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4UL);

  __disable_irq();             /*1) 跳转过程中不响应任何中断*/

  /*2) 停 SysTick，App 会自己重新配置*/
  SysTick->CTRL = 0UL;
  SysTick->LOAD = 0UL;
  SysTick->VAL  = 0UL;

  /*3) 清 NVIC 使能位和挂起位，否则 App 一开中断就进 BootLoader 遗留的 handler*/
  for (i = 0UL; i < 8UL; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  /*4) 清 PendSV/PendST 挂起，内核异常 ICPR 管不到*/
  SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

  /* 等上面的寄存器写生效再改 VTOR/MSP */
  __DSB();
  __ISB();

  SCB->VTOR = APP_BASE_ADDR;   /*5) App 的异常/中断才能找到自己的入口*/
  __DSB();
  __ISB();

  __set_MSP(app_msp);          /*6) 硬件复位时 MSP 取自向量表第 0 项，软件跳转必须手动赋值*/

  __enable_irq();              /*7) App 期望复位后总中断已打开，带着 PRIMASK=1 跳过去会卡死*/

  ((app_entry_t)app_reset)();  /*8) 向量表第 1 项就是可跳转的 Thumb 地址*/
}

/**
  * 功    能：初始化升级口 USART1（裸机寄存器版，不依赖 HAL）
  * 参    数：无
  * 返 回 值：无
  * 补    充：BootLoader 只用寄存器操作，这样它不占 HAL 的代码空间，也不会和 App 的 HAL 抢状态；
  *            BRR 按 HSI 16MHz 算，改主频必须同步改 UART_BRR，否则波特率偏差超出 3% 就全是乱码
  */
static void uart1_init(void)
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
static void uart1_send_str(const char *s)
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
static uint32_t crc32_update(uint32_t crc, uint8_t b)
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
static void frame_send(uint8_t cmd, const uint8_t *data, uint8_t len)
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
static void frame_reset(frame_parser_t *p)
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
static uint8_t frame_feed(frame_parser_t *p, uint8_t ch)
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
static void frame_reply(uint8_t cmd, uint8_t st)
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
static int uart1_wait_handshake(uint32_t ms)
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
static void uart1_send_dec(uint32_t v)
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
static void uart1_send_hex32(uint32_t v)
{
  uint8_t i;
  uint8_t d;

  for (i = 0U; i < 8U; i++)
  {
    d = (uint8_t)((v >> (28U - 4U * i)) & 0xFUL);
    uart1_send_byte((uint8_t)((d < 10U) ? ('0' + d) : ('A' + d - 10U)));
  }
}

/* ── 内部 Flash 操作：全部用寄存器，不经过 HAL ── */

static void flash_clear_errors(void)
{
  /*这些错误标志是"写 1 清零"，不清掉下一次操作会带着旧错误一起失败*/
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR |
              FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
}

static void flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0UL)   /*已经解锁还写一次 key 会报错，先看 LOCK 位*/
  {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  flash_clear_errors();
}

static void flash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;   /*用完立刻锁回去，防止跑飞误写*/
}

static int flash_busy_wait(void)
{
  uint32_t wait = 0UL;

  while ((FLASH->SR & FLASH_SR_BSY) != 0UL)   /*BSY 是硬件在忙，软件只能干等*/
  {
    wait++;
    if (wait > 20000000UL)   /*16MHz 下大约几秒，128KB 扇区最长 4 秒也够*/
    {
      return 0;
    }
  }
  return 1;
}

/**
  * 功    能：擦除一个扇区
  * 参    数：snb 扇区号 0~11
  * 返 回 值：1 成功，0 失败（超时或写了错误标志）
  * 补    充：必须先把 PSIZE 和 SNB 配好再置 SER，最后用 STRT 触发；
  *            擦完一定要清 SER，否则紧接着的编程操作会当成擦除命令
  */
static int flash_erase_sector(uint8_t snb)
{
  flash_clear_errors();

  FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB_Msk | FLASH_CR_PSIZE_Msk | FLASH_CR_PG))
            | ((uint32_t)snb << FLASH_CR_SNB_Pos);
  FLASH->CR |= FLASH_CR_SER;
  FLASH->CR |= FLASH_CR_STRT;

  if (!flash_busy_wait())
  {
    FLASH->CR &= ~FLASH_CR_SER;
    return 0;
  }
  FLASH->CR &= ~FLASH_CR_SER;

  if ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                    FLASH_SR_PGPERR | FLASH_SR_PGSERR)) != 0UL)
  {
    return 0;   /*写保护或对齐错误，这扇区没擦成*/
  }
  return 1;
}

/**
  * 功    能：往内部 Flash 写一个 32 位字
  * 参    数：addr 目标地址（必须 4 字节对齐）；word 要写的值
  * 返 回 值：1 成功，0 失败
  * 补    充：写前先读一遍，不是 0xFFFFFFFF 直接拒写 ——
  *            内部 Flash 只能把 1 变 0，在脏字上硬写会得到一个错值，而且事后看不出来
  */
static int flash_program_word(uint32_t addr, uint32_t word)
{
  flash_clear_errors();

  if (*(volatile uint32_t *)addr != 0xFFFFFFFFUL)
  {
    return 0;
  }

  /*PSIZE=10 是 x32 位并行，写一个字一次搞定，比 x8 快得多*/
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_PSIZE_Msk | FLASH_CR_SER | FLASH_CR_SNB_Msk))
            | FLASH_CR_PSIZE_1;
  FLASH->CR |= FLASH_CR_PG;

  *(volatile uint32_t *)addr = word;   /*写这个动作本身触发编程*/

  if (!flash_busy_wait())
  {
    FLASH->CR &= ~FLASH_CR_PG;
    return 0;
  }
  FLASH->CR &= ~FLASH_CR_PG;

  if ((FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                    FLASH_SR_PGPERR | FLASH_SR_PGSERR)) != 0UL)
  {
    return 0;
  }
  return 1;
}

static uint32_t flash_sector_start(uint8_t snb)
{
  if (snb <= 3U)   /*sector 0~3 每个 16KB*/
  {
    return 0x08000000UL + ((uint32_t)snb * 0x4000UL);
  }
  if (snb == 4U)   /*sector 4 是 64KB*/
  {
    return 0x08010000UL;
  }
  return 0x08020000UL + ((uint32_t)(snb - 5U) * 0x20000UL);   /*sector 5~11 每个 128KB*/
}

static uint32_t flash_sector_size(uint8_t snb)
{
  if (snb <= 3U)
  {
    return 0x4000UL;
  }
  if (snb == 4U)
  {
    return 0x10000UL;
  }
  return 0x20000UL;
}

/**
  * 功    能：查一个地址落在哪个扇区
  * 参    数：addr 内部 Flash 里的地址
  * 返 回 值：扇区号 0~11；地址不在内部 Flash 范围内返回 0xFF
  * 补    充：扇区大小不均匀，所以只能用分段判断，不能直接除
  */
static uint8_t flash_sector_of(uint32_t addr)
{
  if (addr < 0x08010000UL)   /*sector 0~3 每 16KB，直接用偏移右移 14 位*/
  {
    return (uint8_t)((addr - 0x08000000UL) >> 14);
  }
  if (addr < 0x08020000UL)   /*sector 4 独占 0x08010000~0x0801FFFF*/
  {
    return 4U;
  }
  if (addr < 0x08080000UL)   /*sector 5~11 每 128KB*/
  {
    return (uint8_t)(5U + ((addr - 0x08020000UL) >> 17));
  }
  return 0xFFU;
}

/**
  * 功    能：处理擦除命令：只动 App 区，而且必须按扇区边界对齐
  * 参    数：data 8 字节载荷（起始地址 4B + 结束地址 4B，都是小端、结束地址不含）
  * 返 回 值：状态字节 ST_OK / ST_RANGE / ST_FLASH
  * 补    充：为什么要求对齐：擦除最小单位是整个扇区，
  *            传个"半扇区"进来擦不干净，还会让后面写数据的地址判断变得含糊，
  *            干脆一开始就拒绝，让 PC 端正对自己要传的镜像大小算好边界
  */
static uint8_t upgrade_erase(const uint8_t *data, uint8_t len)
{
  uint32_t start;
  uint32_t end;
  uint8_t  s0;
  uint8_t  s1;
  uint8_t  s;

  if (len != 8U)
  {
    return ST_RANGE;
  }

  start = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
          ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
  end   = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) |
          ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);

  if ((start < APP_BASE_ADDR) || (end > APP_LIMIT_ADDR) || (start >= end))
  {
    return ST_RANGE;   /*越界一步都不擦*/
  }

  s0 = flash_sector_of(start);
  s1 = flash_sector_of(end - 1UL);
  if ((s0 > 7U) || (s1 > 7U) || (s0 > s1))
  {
    return ST_RANGE;   /*sector 0~3 里有 BootLoader 自己和元数据，一律不许碰*/
  }
  if ((start != flash_sector_start(s0)) ||
      (end != (flash_sector_start(s1) + flash_sector_size(s1))))
  {
    return ST_RANGE;   /*不是整扇区边界就不擦*/
  }

  flash_unlock();
  for (s = s0; s <= s1; s++)
  {
    if (!flash_erase_sector(s))
    {
      flash_lock();
      return ST_FLASH;
    }
  }
  flash_lock();

  /*擦完把写入游标归零：数据块永远从 App 基址开始长*/
  g_write_addr = APP_BASE_ADDR;
  g_word_n     = 0U;
  g_written    = 0UL;

  /*擦除成功 = 升级正式开始：标志先立起来，从这里开始掉电都能被认出来*/
  ota_mark(0UL);
  return ST_OK;
}

/**
  * 功    能：处理数据块命令：按收到的顺序把字节攒成字，逐个写进 Flash
  * 参    数：data 数据段；len 本帧字节数
  * 返 回 值：状态字节
  * 补    充：地址由游标自己维护，PC 端不带地址；
  *            剩余不足 4 字节的尾巴留到"结束"命令时补齐，所以这里只管整字
  */
static uint8_t upgrade_write(const uint8_t *data, uint8_t len)
{
  uint32_t word;
  uint8_t  i;

  if ((g_write_addr < APP_BASE_ADDR) ||
      ((g_write_addr + (uint32_t)len) > APP_LIMIT_ADDR))
  {
    return ST_RANGE;   /*越界先拦住，别把 sector 3 的元数据写了*/
  }

  flash_unlock();

  for (i = 0U; i < len; i++)
  {
    g_word_buf[g_word_n] = data[i];
    g_word_n++;
    g_written++;

    if (g_word_n >= 4U)
    {
      word = ((uint32_t)g_word_buf[0]) | ((uint32_t)g_word_buf[1] << 8U) |
             ((uint32_t)g_word_buf[2] << 16U) | ((uint32_t)g_word_buf[3] << 24U);

      if (!flash_program_word(g_write_addr, word))
      {
        flash_lock();
        return ST_FLASH;
      }
      g_write_addr += 4UL;
      g_word_n = 0U;
    }
  }

  flash_lock();

  /*进度落盘：先写 Flash 再更新 EEPROM，断电最坏是 EEPROM 比实际少记一帧，
    方向安全 —— 不会把没写进去的字节记成已写*/
  if (len > 0U)
  {
    ota_mark(g_written);
  }
  return ST_OK;
}

/**
  * 功    能：处理结束命令：把不足一字的尾巴补齐写完
  * 参    数：无
  * 返 回 值：状态字节
  * 补    充：尾巴补 0x00，只影响镜像的最后一个字；镜像大小不一定是 4 的倍数，
  *            但不补齐的话这份数据就不是"整字写进去"的状态，读回来会缺字节
  */
static uint8_t upgrade_finish(void)
{
  uint32_t word = 0UL;
  uint8_t  i;

  if (g_word_n == 0U)
  {
    return ST_OK;   /*正好整字收完，没什么可补的*/
  }

  while (g_word_n < 4U)
  {
    g_word_buf[g_word_n] = 0x00U;
    g_word_n++;
  }

  for (i = 0U; i < 4U; i++)
  {
    word |= ((uint32_t)g_word_buf[i]) << (8U * i);
  }

  flash_unlock();
  if (!flash_program_word(g_write_addr, word))
  {
    flash_lock();
    return ST_FLASH;
  }
  flash_lock();

  g_write_addr += 4UL;
  g_word_n = 0U;
  return ST_OK;
}

/**
  * 功    能：升级模式主循环：一直收命令，按命令号分发
  * 参    数：无
  * 返 回 值：无（结束命令校验通过就跳 App；没通过就继续留在这儿等重传）
  * 补    充：每条命令都回一帧应答，PC 端拿到状态才发下一帧（停等），
  *            这样丢帧、错帧都能在局部重发，不用整包重来
  */
static void upgrade_loop(void)
{
  frame_parser_t p;
  uint8_t ch;
  uint8_t r;
  uint8_t st;

  /*还没擦除就直接发数据的话，地址检查会立刻把这块拦住*/
  g_write_addr = APP_BASE_ADDR;
  g_word_n     = 0U;
  g_written    = 0UL;

  led_init();
  led_on();   /*绿灯常亮 = 停在 BootLoader，没跳 App*/

  frame_reset(&p);

  for (;;)
  {
    if ((USART1->SR & USART_SR_RXNE) == 0UL)   /*升级模式不设超时，一直等到有字节*/
    {
      continue;
    }

    ch = (uint8_t)USART1->DR;
    r  = frame_feed(&p, ch);

    if (r == 0U)
    {
      continue;
    }

    if (r == 2U)   /*CRC 不对：不回话，PC 超时后重发这一帧*/
    {
      uart1_send_str("[BOOT] bad frame\r\n");
      continue;
    }

    switch (p.cmd)
    {
    case CMD_HANDSHAKE:
      /*升级模式里再收到握手照样回 ACK：断电续升级时 PC 端还是从握手开始走流程*/
      frame_reply(CMD_HANDSHAKE, ST_OK);
      break;

    case CMD_DATA:
      st = upgrade_write(p.data, p.len);
      frame_reply(CMD_DATA, st);
      break;

    case CMD_ERASE:
      st = upgrade_erase(p.data, p.len);
      frame_reply(CMD_ERASE, st);
      break;

    case CMD_END:
      st = upgrade_finish();
      frame_reply(CMD_END, st);
      if (st == ST_OK)
      {
        ota_clear();   /*升级顺利走完才清标志；清了以后断电上电都走正常跳转*/
        uart1_send_str("[BOOT] written ");
        uart1_send_dec(g_written);
        uart1_send_str(" bytes, head=0x");
        uart1_send_hex32(*(volatile uint32_t *)APP_BASE_ADDR);
        uart1_send_str("\r\n");

        if (app_is_valid())
        {
          uart1_send_str("[BOOT] jump to app\r\n");
          jump_to_app();
        }
        uart1_send_str("[BOOT] app invalid, stay in upgrade mode\r\n");
      }
      break;

    default:
      frame_reply(p.cmd, ST_RANGE);   /*不认识的命令也回一句，PC 不会一直干等*/
      break;
    }
  }
}

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
static void ee_init(void)
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
static void ota_mark(uint32_t written)
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
static void ota_clear(void)
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
static int ota_is_marked(void)
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

/* SysTick 当毫秒计时器用：只计数不产生中断，省掉写中断服务函数和管 NVIC 的麻烦 */
static void systick_start_1ms(void)
{
  SysTick->LOAD = 16000UL - 1UL;   /*HCLK=16MHz，数 16000 拍正好 1ms*/
  SysTick->VAL  = 0UL;             /*写 VAL 清零，重装值才会立刻生效*/
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void systick_stop(void)
{
  SysTick->CTRL = 0UL;
}

/* LED 与延时 */
static void led_init(void)
{
  RCC->AHB1ENR |= LED_GPIO_CLK_EN;

  LED_PORT->MODER   = (LED_PORT->MODER & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->OTYPER &= ~LED_PIN_MASK;
  LED_PORT->OSPEEDR = (LED_PORT->OSPEEDR & ~(3UL << (LED_PIN * 2U))) | (3UL << (LED_PIN * 2U));
  LED_PORT->PUPDR   = (LED_PORT->PUPDR & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
  LED_PORT->BSRR     = LED_PIN_MASK;   /*输出高，灯灭*/
}

static void led_on(void)
{
  LED_PORT->BSRR = (LED_PIN_MASK << 16U);   /*BSRR 高 16 位复位，低电平点亮*/
}

static void led_off(void)
{
  LED_PORT->BSRR = LED_PIN_MASK;   /*输出高，灯灭*/
}

/* 跑在 HSI 16MHz 上，只要肉眼可见的闪烁；volatile 防被优化掉 */
static void delay_loop(uint32_t n)
{
  volatile uint32_t i;
  for (i = 0UL; i < n; i++)
  {
  }
}
