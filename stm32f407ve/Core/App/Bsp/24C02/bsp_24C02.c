/* bsp_24C02.c
 * 板载 24C02 EEPROM 驱动：初始化探测、任意地址读、页写(自动分页)、上电自检。
 * 硬件：与 BH1750 共用 I2C1(PB6/PB7)，7 位地址 0x50(A2/A1/A0 全接地)，256 字节。
 * 原理图：U13，SCL=PB6/SDA=PB7，R18/R32 10K 上拉，WP 脚低电平(写允许)。
 *
 * ══════ 这颗芯片是什么、和另外两种存储怎么比（项目里三种存储凑齐了）══════
 *
 * 24C02 = 2Kbit = 256 字节 EEPROM，分 32 页、每页 8 字节。
 * EEPROM 与 Flash 的本质区别，一张表说清：
 *
 *   | 能力          | EEPROM(24C02)      | NOR Flash(W25Q128)   | 内部 Flash(sector11) |
 *   |---------------|--------------------|----------------------|---------------------|
 *   | 0 能改回 1？  | 能，字节级直接写    | 不能，写前必擦        | 不能，写前必擦       |
 *   | 最小擦除单位  | 不需要擦除         | 4KB 扇区              | 16K/64K/128KB 扇区   |
 *   | 擦除代价      | 无(内部写周期~5ms) | 芯片自己忙，CPU 照跑  | CPU 停摆 1~2 秒      |
 *   | 怎么知道写完  | ACK 轮询           | 读状态寄存器 BUSY 位  | 硬件标志位           |
 *   | 寿命          | 100 万次/字节      | 10 万次/扇区          | 1 万次/扇区          |
 *
 * 「0 能改回 1」是关键：EEPROM 每个存储单元自带独立的电荷泵，写前不需要
 * 先整片恢复成全 1 —— 所以它能按【字节】改写，这是 Flash 永远做不到的。
 * 也因此它天生适合存"经常变化的小参数"(校准系数/计数器/标志位)。
 *
 * ── 器件地址是怎么来的 ──
 *   I2C 器件地址高位固定 1010，低 3 位接 A2/A1/A0 引脚。
 *   本板三个地址脚全接地 → 1010_000 = 0x50(7位) → 总线字节 0xA0(写)/0xA1(读)。
 *   同一条总线想挂 8 颗 24C02，就靠这三个脚区分(接出 000~111 八种组合)。
 *
 * ── 页写的"回绕"坑 ──
 *   一次连续写最多 8 字节(一页)，且【不能跨页】：地址在页内递增，
 *   到页尾会绕回本页页首继续写 —— 假设从 0x06 写 4 字节，实际写进
 *   0x06,0x07,0x00,0x01，把本页开头覆盖了！
 *   所以驱动必须自己按页边界切段，这正是 AT24C02_WriteBytes 干的事。
 *   (对比：W25Q128 是 256B 页，同样有这个坑，bsp_W25Q128 里也有同样处理。)
 *
 * ── ACK 轮询：怎么知道 EEPROM 写完了 ──
 *   STOP 之后芯片开始内部写周期(手册 max 5ms)。期间它【不应答任何地址】。
 *   所以：反复发"START + 0xA0"看有没有 ACK，收到 ACK = 写完了。
 *   与 W25Q128 读状态寄存器 BUSY 位是同一件事的两种实现 ——
 *   区别只在"器件忙时怎么表达"：EEPROM 用"不回话"表达忙，Flash 用状态位表达。
 *
 * ── 读为什么没有页的概念 ──
 *   读操作内部地址计数器在整个 256B 范围内自动 +1(到 0xFF 回绕到 0x00)，
 *   不受页边界约束。只有写才有 8 字节页的限制。
 *
 * ── WP 写保护脚 ──
 *   WP 拉高 = 整片只读。本板 WP 为低(写允许)。若真机出现"读得到、写完读回
 *   还是旧值"，第一件事查 WP 是不是被拉高了(飞线接地可解)。
 *
 * ── 存储区规划(256B，先定规矩防打架) ──
 *   0x00~0xEF  用户区：校准系数、运行参数等，按需再细分
 *   0xF8~0xFF  保留区：上电自检专用 + 将来 OTA 升级标志/计数的候选位置
 */

#include "bsp_24C02.h"
#include "bsp_I2CBus.h"
#include "usart.h"          /* UART_Printf：自检结果上电打一行 */
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* ====================== 器件参数 ====================== */
#define AT24C02_ADDR_7BIT       0x50U                       /* 1010 + A2A1A0(接地) */
#define AT24C02_ADDR_W          ((AT24C02_ADDR_7BIT) << 1)  /* 0xA0 总线字节(写方向) */
#define AT24C02_PAGE_SIZE       8U                          /* 32 页 × 8B = 256B */
#define AT24C02_MEM_SIZE        256U
#define AT24C02_TWR_TIMEOUT_MS  50U                         /* 写周期 max 5ms，留 10 倍余量 */

/* 上电自检开关：写保留区一页并读回验证。想省时间/省写次数可置 0 */
#ifndef AT24C02_SELFTEST
#define AT24C02_SELFTEST        1
#endif

/* 保留区(自检页)：只动这一页，不碰用户区 */
#define AT24C02_SELFTEST_ADDR   0xF8U


/**
  * 功    能：毫秒级延时——RTOS 已跑就 osDelay(1) 让出 CPU，boot 阶段退化为 HAL_Delay
  * 参    数：ms 延时毫秒数
  * 返 回 值：无
  * 补    充：osDelay 在任务里让出 CPU(别的任务能跑)；HAL_Delay 是忙等。
  *            两个都要能跑，是因为本驱动将来既会在任务里用(OTA 标志)，
  *            也可能在极早期初始化里用(那时内核还没起)。
  */
static void AT24C02_Delay1ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        (void)osDelay(ms);
    }
    else
    {
        HAL_Delay(ms);
    }
}


/**
  * 功    能：等 EEPROM 内部写周期结束——反复发器件地址直到它回 ACK
  * 参    数：timeoutMs 最长等多久(ms)
  * 返 回 值：0 = 写周期已结束(收到 ACK)；1 = 超时(器件不应答或没焊)
  * 补    充：这就是【ACK 轮询】。写周期内器件连自己的地址都不应答，
  *            I2CBus_Probe 会走 AF(无应答)分支返回 1 —— 这是"正常业务"
  *            不是总线故障，所以每轮失败只等 1ms 再试，不做总线恢复。
  */
static uint8_t AT24C02_WaitReady(uint32_t timeoutMs)
{
    do
    {
        if (I2CBus_Probe(AT24C02_ADDR_W) == 0U)
        {
            return 0U;                     /* 收到 ACK：写完了 */
        }
        AT24C02_Delay1ms(1U);
    } while (--timeoutMs > 0U);

    return 1U;                             /* 超时 */
}


uint8_t AT24C02_Init(void)
{
    (void)I2CBus_Init();                   /* 幂等：总线已在跑就只是重配一遍 */

    /* 探测：只发地址字节看有没有 ACK，不产生任何写入 */
    if (I2CBus_Probe(AT24C02_ADDR_W) != 0U)
    {
        return 1U;                         /* 不在位 */
    }

#if AT24C02_SELFTEST
    {
        /* 自检 = 写保留区一页 + 读回比对。
         * 【这里绝对不能再包一层 I2CBus_Lock()！】
         * WriteBytes/ReadBytes 内部各自拿锁；osMutexNew(NULL) 建的是【非递归锁】，
         * 任务对自己已持有的锁再次 acquire 会永远阻塞 —— 2026-11-02 真机上就是
         * 这么把 SensorTask 卡死在初始化里的(现象: 启动日志打了 BH1750 init ok 后
         * 再无 24C02/DHT11 输出, 任务列表里 SensorTask 运行计数冻结)。
         * 写与读回之间不需要原子性: 此时全系统只有 SensorTask 一个 I2C 用户。 */
        static const uint8_t pattern[AT24C02_PAGE_SIZE] =
        {
            0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0x77U, 0x88U
        };
        uint8_t rd[AT24C02_PAGE_SIZE];
        uint8_t i, fail = 0U;

        if (AT24C02_WriteBytes(AT24C02_SELFTEST_ADDR, pattern,
                               AT24C02_PAGE_SIZE) != 0U)
        {
            fail = 1U;
        }
        else if (AT24C02_ReadBytes(AT24C02_SELFTEST_ADDR, rd,
                                   AT24C02_PAGE_SIZE) != 0U)
        {
            fail = 1U;
        }
        else
        {
            for (i = 0U; i < AT24C02_PAGE_SIZE; i++)
            {
                if (rd[i] != pattern[i])
                {
                    fail = 1U;             /* 读回不一致：写没进去(查 WP 脚) */
                    break;
                }
            }
        }

        if (fail != 0U)
        {
            UART_Printf("[24C02] selftest FAILED (check WP pin / wiring)\r\n");
            return 1U;
        }
        UART_Printf("[24C02] selftest ok (256B, addr 0x50)\r\n");
    }
#endif

    return 0U;
}


uint8_t AT24C02_ReadBytes(uint8_t memAddr, uint8_t *buf, uint16_t num)
{
    uint8_t err;

    if ((buf == NULL) || (num == 0U) ||
        ((uint16_t)memAddr + num > AT24C02_MEM_SIZE))
    {
        return 1U;
    }

    /* 随机读的标准姿势：先"写"一个字地址(不 STOP) → RESTART 转读。
     * 之后地址计数器自动 +1，连续收 num 字节(顺序读)，末字节 NACK。 */
    I2CBus_Lock();
    err = I2CBus_WriteReadRaw(AT24C02_ADDR_W, &memAddr, 1U, buf, num);
    I2CBus_Unlock();

    return err;
}


uint8_t AT24C02_WriteBytes(uint8_t memAddr, const uint8_t *buf, uint16_t num)
{
    uint8_t  pkt[1U + AT24C02_PAGE_SIZE];  /* [字地址 + 一页数据] 的组包缓冲 */
    uint16_t addr   = memAddr;
    uint16_t remain = num;

    if ((buf == NULL) || (num == 0U) ||
        ((uint16_t)memAddr + num > AT24C02_MEM_SIZE))
    {
        return 1U;
    }

    I2CBus_Lock();

    while (remain > 0U)
    {
        /* ── 本页还能写多少：页大小 - 页内偏移。
         * 例：从 0x06 起 → 页内偏移 6，本页只剩 2 字节(0x06,0x07)，
         *     再多就掉进"回绕"坑(绕回 0x00 覆盖页首)。 */
        uint8_t pageOff = (uint8_t)(addr & (AT24C02_PAGE_SIZE - 1U));
        uint8_t chunk   = (uint8_t)(AT24C02_PAGE_SIZE - pageOff);
        if ((uint16_t)chunk > remain)
        {
            chunk = (uint8_t)remain;
        }

        /* 组包：[0]=字地址，[1..chunk]=数据。
         * 为什么必须一个事务发完：STOP 一发，芯片就认为"这次写结束"，
         * 立刻进写周期 —— 地址和数据被一个 START..STOP 完整包住是页写的硬性格式。 */
        pkt[0] = (uint8_t)addr;
        {
            uint8_t i;
            for (i = 0U; i < chunk; i++)
            {
                pkt[1U + i] = buf[(uint16_t)(num - remain) + i];
            }
        }

        if (I2CBus_WriteRaw(AT24C02_ADDR_W, pkt, (uint8_t)(1U + chunk)) != 0U)
        {
            I2CBus_Unlock();
            return 1U;                     /* 总线失败 */
        }

        /* 写周期：STOP 后芯片内部开始写(最长 5ms)，ACK 轮询等它写完 */
        if (AT24C02_WaitReady(AT24C02_TWR_TIMEOUT_MS) != 0U)
        {
            I2CBus_Unlock();
            return 1U;                     /* 一直不应答：器件没了或挂了 */
        }

        addr   += chunk;
        remain -= chunk;
    }

    I2CBus_Unlock();
    return 0U;
}
