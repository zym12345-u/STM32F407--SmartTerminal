/* bsp_bootflag.c
 * OTA-2 启动标志：24C02 页 0xE8，8 字节一页。
 * 常量必须与 BootLoader/port/bsp_ee.h 的 EE_OTA2_* 保持同值。
 *
 * ── 这一页在整条链路里的位置 ──
 *   ① App 下载完固件 → 写 state=APPLY（RequestApply）
 *   ② 用户复位 → BootLoader 看到 APPLY → 搬槽 B → 写 state=CONFIRM / count=0
 *   ③ 新固件启动、全部任务健康存活 3 秒 → MonitorTask 调 ConfirmBoot → 整页清 0
 *   ④ 若 ③ 连续 3 次都没发生（新固件跑不起来）→ 第 4 次上电回滚槽 A
 *
 * ── 为什么整页 8 字节一次写 ──
 *   24C02 的页写缓冲正好 8 字节，一次写只有一个内部编程周期（约 5ms）。
 *   分两次写（先 magic 再 state）不但慢一倍，还会出现"magic 已落盘、
 *   state 还是旧值"的中间态——上电时无法判断这份标志是否可信。
 *
 * ── 为什么页地址选 0xE8 ──
 *   24C02 共 256 字节 = 32 页。0xF0 给串口 IAP 的进度页用，0xF8~0xFF 是
 *   BootLoader 自检页，0xE8 是中间空出来的一页，谁也不碰谁。
 */

#include "bsp_bootflag.h"
#include "bsp_24C02.h"

/**
  * 功    能：把整页写成 magic + state + count
  * 参    数：state 状态；count 未确认次数
  * 返 回 值：0 = 成功；1 = EEPROM 写失败
  * 补    充：buf 用 {0} 初始化，所以第 6、7 字节（保留位）固定是 0；
  *            这与 BootLoader 侧 ota2_state_write 的写法必须一致，
  *            否则两边对"整页内容"的理解会分叉。
  */
static uint8_t BootFlag_Write(uint8_t state, uint8_t count)
{
  uint8_t buf[8] = {0U};

  /*magic 按小端铺 4 字节：EEPROM 是字节器件，写进去的字节序就是将来读出来的*/
  buf[0] = (uint8_t)(BOOTFLAG_MAGIC);
  buf[1] = (uint8_t)(BOOTFLAG_MAGIC >> 8U);
  buf[2] = (uint8_t)(BOOTFLAG_MAGIC >> 16U);
  buf[3] = (uint8_t)(BOOTFLAG_MAGIC >> 24U);
  buf[4] = state;
  buf[5] = count;

  /*AT24C02_WriteBytes 内部已经处理了 ACK 轮询与 5ms 写周期，这里只翻译返回值：
   *0 = 成功，非 0 = 失败（本函数统一成 0/1，方便调用方 if）*/
  return (AT24C02_WriteBytes(BOOTFLAG_MEM, buf, 8U) == 0U) ? 0U : 1U;
}

/**
  * 功    能：请求"下次上电把槽 B 搬进 App 区"
  * 参    数：无
  * 返 回 值：0 = 成功；1 = 写失败
  * 补    充：OTA 下载成功后由 bsp_OTA.c 调用；只置请求，不复位。
  *            count 固定写 0：搬运成功时 BootLoader 会重新从 0 开始计未确认次数。
  */
uint8_t BootFlag_RequestApply(void)
{
  return BootFlag_Write(BOOTFLAG_ST_APPLY, 0U);
}

/**
  * 功    能：App 启动后向 BootLoader 确认"这版固件能跑"
  * 参    数：无
  * 返 回 值：0 = EEPROM 读写失败；1 = 无需确认（普通启动）；2 = 确认成功
  * 补    充：三态返回的理由——
  *            0 要让调用方重试（MonitorTask 最多重试 3 次）；
  *            1 是绝大多数情况（没有待确认状态），此时【不写 EEPROM】，
  *              否则每次上电都写一页，24C02 的 100 万次寿命会被白白磨掉；
  *            2 才是真正"盖章"，调用方据此打一行日志。
  */
uint8_t BootFlag_ConfirmBoot(void)
{
  uint8_t buf[8];
  uint32_t magic;

  if (AT24C02_ReadBytes(BOOTFLAG_MEM, buf, 8U) != 0U)
  {
    return 0U;   /*读失败按"未确认"处理，让调用方重试；绝不能当成功*/
  }

  /*magic 按小端拼回 32 位：与写入顺序一一对应*/
  magic = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8U) |
          ((uint32_t)buf[2] << 16U) | ((uint32_t)buf[3] << 24U);
  if (magic != BOOTFLAG_MAGIC)
  {
    return 1U;   /*没有待确认状态：普通启动，不动 EEPROM*/
  }
  if (buf[4] != BOOTFLAG_ST_CONFIRM)
  {
    return 1U;   /*不是等待确认（例如 App 刚写了 APPLY 还没复位）*/
  }

  /*整页清 0：magic 一起清掉，下次上电读 magic 就失败，等价于"没有请求"。
   *比单独把 state 写 NONE 更彻底——少一种"magic 对但 state 非法"的组合*/
  return (BootFlag_Write(0U, 0U) == 0U) ? 2U : 0U;
}

/**
  * 功    能：清掉整页标志（放弃搬运请求）
  * 参    数：无
  * 返 回 值：无
  * 补    充：串口 IAP 之前调用——"串口升级"和"槽 B 搬运"是两个互斥的升级路径，
  *            同时挂着会变成"先搬 OTA 镜像、再被 PC 覆盖"，谁也不是用户想要的。
  *            写失败只影响"标志没清掉"，不影响本次串口升级本身。
  */
void BootFlag_Clear(void)
{
  uint8_t buf[8] = {0U};

  (void)AT24C02_WriteBytes(BOOTFLAG_MEM, buf, 8U);
}
