/* bsp_WifiStore.c
 * WiFi 凭据存储：内部 Flash sector11，单条 WifiCred_t，magic + 32 位加和校验。
 * 硬件：0x080E0000 起 128KB，IROM 已裁到 448KB 避开程序区。
 * 注意：擦除 128KB 期间 CPU 停 1~2s，必须先把 IWDG 窗口放宽到 4s。
 *
 * ══════ 为什么存内部 Flash、为什么擦除会卡住 CPU ══════
 *
 * ── 为什么不存到外部 W25Q128 ──
 *   WiFi 凭据的用途是「开机自动连网」，而要访问外部 Flash 得先有 SPI 驱动、
 *   要读文件还得先挂载 LittleFS —— 这些在启动早期未必就绪。
 *   存内部 Flash 的好处是：上电后直接读地址就行，不依赖任何驱动和文件系统。
 *   所以这里把扇区 11 单独划出来当「参数区」，既不用跟程序抢地方，
 *   也不会被程序升级（App 区）碰到。
 *
 * ── 【关键】为什么擦 Flash 会把 CPU 卡住 1~2 秒 ──
 *   这是 STM32 内部 Flash 的物理限制：擦除/编程期间，Flash 存储体正在被高压改写，
 *   【不能同时被读取】。而我们的代码本身就存在同一块 Flash 里（XIP，就地执行），
 *   于是擦的时候指令取不出来 —— CPU 只能停下来等，这叫「总线阻塞」。
 *   所以：
 *     1) 擦 128KB 扇区一次要 1~2 秒，这段时间【整个系统都在停着】，
 *        不是「别的任务变慢」，是彻底停了，连中断都进不来。
 *     2) 必须先【把看门狗超时窗口放宽】（本工程从 1s 放到 4s），
 *        否则擦到一半狗就咬人，表现为「一存 WiFi 密码就复位」。
 *     3) 这也解释了为什么参数只用一个扇区、而且只存一条记录：
 *        扇区是 STM32F4 的最小擦除单位，配置区不可能更小，只能一次擦这么大。
 *   对比外部 W25Q128：它是独立芯片，擦它的时候 CPU 照常跑（只是等 SPI 通信），
 *   所以那边的 4KB 擦除只需要「轮询 BUSY」而不用停 CPU。
 *
 * ── magic + 校验和：怎么区分「没写过」和「写坏了」 ──
 *   刚擦完的 Flash 全是 0xFF，不是全 0。所以判断有无数据不能靠「是不是 0」：
 *     magic 不等于 'WIFI' → 从没写过（或已被擦掉） → 用默认值
 *     magic 对但 checksum 不对 → 【写坏了】（掉电写到一半）→ 也当无效处理
 *   两个一起用，才能区分「第一次上电」和「上次保存失败了」。
 *
 * ── 为什么还要一个 LittleFS 版本并做迁移 ──
 *   V1.2 之后引入了 LittleFS，凭据改成存进文件系统里（`/wifi.cfg`），
 *   好处是能和别的配置放一起管理，也不用再单独占一个物理扇区。
 *   但老设备里扇区中还留着旧格式的数据，所以 WifiStoreLfs_Migrate 做一次性搬运：
 *   发现文件不存在 + 扇区里有合法记录 → 搬过去，之后就以文件为准。
 */
#include "bsp_WifiStore.h"

#include "stm32f4xx_hal.h"
#include "bsp_IWDG.h"
#include <string.h>


/* ---------------------------------------------------------------------------
 * Flash 记录布局(整个扇区只放一条，类型见 WifiNet.h 的 WifiCred_t)：
 *   字段      宽度  偏移  含义
 *   magic      4B    0     固定 0x57494649('WIFI')，擦除态为 0xFFFFFFFF
 *   ssid      33B    4     WIFI_SSID_LEN+1，'\0' 结尾
 *   pwd       65B    37    WIFI_PWD_LEN+1，'\0' 结尾
 *   checksum   4B    102   前 102 字节的 32 位加和
 *   合计 108B(尾部含 2B 对齐填充)；写 WifiStore_Save / 读 WifiStore_Load
 * ------------------------------------------------------------------------- */
/**
  * 功    能：算凭据结构的校验和：前 102 字节按 32 位累加（最大约 26010）
  * 参    数：c  WifiCred_t 指针，不能为空
  * 返 回 值：前 102 字节的 32 位加和(最大约 26010)
  * 补    充：用来在读出时判断这份记录有没有被写坏
  */
static uint32_t Wifi_CalcChecksum(const WifiCred_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint32_t sum = 0U;
    uint32_t n;
    /* 只算 checksum 之前的字节，不能把自身算进去 */
    uint32_t bodyLen = (uint32_t)((const uint8_t *)&c->checksum - p);

    for (n = 0U; n < bodyLen; n++)
    {
        sum += (uint32_t)p[n];
    }
    return sum;
}

/* 从 Flash 扇区读凭据，校验 magic 和校验和。
 * 任一项不对就清零输出并返回 0；返回 1 表示拿到可用凭据。 */
/* 内部 Flash 内存映射，可直接把地址当结构体指针读 */
uint8_t WifiStore_Load(WifiCred_t *out)
{
    const WifiCred_t *stored = (const WifiCred_t *)WIFI_FLASH_BASE;   /* 0x080E0000: sector11 起始 */

    memset(out, 0, sizeof(*out));

    if (stored->magic != WIFI_FLASH_MAGIC)
    {
        return 0U;                       /* 擦除态(0xFF...)或没写过 */
    }
    memcpy(out, stored, sizeof(*out));
    if (out->checksum != Wifi_CalcChecksum(out))
    {
        memset(out, 0, sizeof(*out));
        return 0U;
    }
    return 1U;
}

/**
  * 功    能：擦除该扇区再写入凭据，最后回读比对。擦除/编程/回读任一失败都返回 0
  * 参    数：in  待保存凭据，不能为空，ssid/pwd 超长部分截断
  * 返 回 值：1=擦除+编程+回读全部成功，0=任一步失败
  * 补    充：ssid/pwd 超长的部分会被截断
  */
uint8_t WifiStore_Save(const WifiCred_t *in)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t               err = 0U;
    HAL_StatusTypeDef      halSt;
    WifiCred_t             buf;
    const uint32_t        *src;
    volatile uint32_t     *dst;
    uint32_t               words;
    uint32_t               i;

    /* 组帧到 4 字节对齐缓冲:Flash 按 32 位字编程,结构体长度向上取整 */
    memset(&buf, 0, sizeof(buf));
    buf.magic = WIFI_FLASH_MAGIC;
    strncpy(buf.ssid, in->ssid, WIFI_SSID_LEN);
    strncpy(buf.pwd,  in->pwd,  WIFI_PWD_LEN);
    buf.checksum = Wifi_CalcChecksum(&buf);
    words = (uint32_t)((sizeof(buf) + 3U) / 4U);   /* 108B -> 27 个 32 位字 */

    /* 擦除 128KB 期间 CPU 取指停顿 1~2s，全任务冻结，1s 窗口必然复位，先放宽到 4s */
    IWDG_SetReload(IWDG_RELOAD_FLASH_SAVE);
    HAL_FLASH_Unlock();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = FLASH_SECTOR_11;      /* 用最后一个扇区(128KB),远离程序区 */
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7~3.6V */
    halSt = HAL_FLASHEx_Erase(&erase, &err);

    if (halSt == HAL_OK)
    {
        src = (const uint32_t *)&buf;
        dst = (volatile uint32_t *)WIFI_FLASH_BASE;
        for (i = 0U; i < words; i++)
        {
            halSt = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,          /* 每次编程 1 个 32 位字 */
                                      (uint32_t)(dst + i), src[i]);
            if (halSt != HAL_OK)
            {
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    IWDG_SetReload(IWDG_RELOAD_NORMAL);     /* 恢复 1s 日常窗口 */

    if (halSt != HAL_OK)
    {
        return 0U;
    }

    /* 回读校验,确保真的写进去了 */
    if (memcmp((const void *)WIFI_FLASH_BASE, &buf, sizeof(buf)) != 0)
    {
        return 0U;
    }
    return 1U;
}
