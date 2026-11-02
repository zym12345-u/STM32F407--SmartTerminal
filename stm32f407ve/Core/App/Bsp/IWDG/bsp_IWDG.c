/* bsp_IWDG.c
 * 独立看门狗驱动 + 任务心跳监控：直写 KR/PR/RLR/SR，不依赖 HAL_IWDG。
 * 硬件：LSI ≈32kHz，256 分频，RLR=125，窗口约 1s。
 * 注意：IWDG 启动后无法关闭；内部 Flash 擦除期间必须把窗口放宽到 4s。
 */
#include "bsp_IWDG.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#define IWDG_PRESCALER_256  6U          /* PR 寄存器值 6 对应 256 分频 */
#define IWDG_RELOAD         125U        /* T = 256 * 125 / 32000 ≈ 1.00s */
#define WDT_HEARTBEAT_TIMEOUT_MS   3000U   /* 任务心跳超时: 3s 未更新视为卡死 */
#define WDT_FIRSTBEAT_GRACE_MS   30000U   /* 首跳宽限: 上电 30s 内还没报过心跳不判死,
                                           * 超过则认为卡死在初始化里(见 CheckAllAlive) */

/* 各任务最后心跳时间戳(tick) */
static volatile uint32_t s_lastBeat[WDT_TASK_MAX] = {0U};

/* ---------------------------------------------------------------------------
 * IWDG 超时窗口(按 LSI 标称 32kHz 估算)：
 *   分频/RLR  窗口  用途
 *   256/125   1.0s  日常窗口 IWDG_RELOAD_NORMAL
 *   256/500   4.0s  内部 Flash 擦除期临时窗口 IWDG_RELOAD_FLASH_SAVE
 *   写入者    IWDG_Init 首次写 RLR，IWDG_SetReload 运行期改写
 *   喂狗者    仅 MonitorTask，且 Watchdog_CheckAllAlive() 返回 1
 * ------------------------------------------------------------------------- */

/**
  * 功    能：启动独立看门狗：LSI 约 32kHz，重装值 125 对应约 1s 超时
  * 参    数：无
  * 返 回 值：无
  * 补    充：启动后无法关闭，所以要等系统确认能正常喂狗之后再调
  */
void IWDG_Init(void)
{
    /*使能 LSI，等 LSIRDY 置位后再写 IWDG，否则写入无效*/
    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) { }

    /*启动 IWDG(0xCCCC)，启动后无法关闭*/
    IWDG->KR = 0xCCCCU;

    /*0x5555 解锁 PR/RLR*/
    IWDG->KR = 0x5555U;

    /*256 分频*/
    IWDG->PR = IWDG_PRESCALER_256;

    /*RLR=125，T≈1s*/
    IWDG->RLR = IWDG_RELOAD;

    /*等寄存器同步完成*/
    while (IWDG->SR != 0U) { }

    /*首次喂狗*/
    IWDG_Refresh();
}

/**
  * 功    能：喂一次狗：往键寄存器写 0xAAAA 重装计数。喂晚了直接复位，
  * 参    数：无
  * 返 回 值：无
  * 补    充：所以只允许 MonitorTask 在确认所有任务都活着之后调用
  */
void IWDG_Refresh(void)
{
    /*喂狗 0xAAAA；只有 MonitorTask 在全任务存活时调用*/
    IWDG->KR = 0xAAAAU;
}

/**
  * 功    能：改看门狗的超时窗口：T ≈ 256 × rlr / 32000 秒，rlr=125 约 1s、500 约 4s
  * 参    数：rlr  重装载值 1~4095；T ≈ 256*rlr/32000 s，125≈1s，500≈4s
  * 返 回 值：无
  * 补    充：需要给 Flash 擦除这类长操作留余量时用它，而不是靠中途偷偷喂狗
  */
void IWDG_SetReload(uint16_t rlr)
{
    IWDG->KR  = 0x5555U;            /* 0x5555 解锁 PR/RLR */
    IWDG->RLR = rlr;                /* 内部 Flash 擦除前用 500，擦完恢复 125 */
    while (IWDG->SR != 0U) { }      /* 写 RLR 后必须等 SR 清零，新窗口才生效 */
    IWDG->KR  = 0xAAAAU;            /* 立即喂一次, 新窗口起算 */
}

/**
  * 功    能：登记「我还活着」：把当前 tick 记进对应任务的槽位
  * 参    数：taskId  任务编号 0~WDT_TASK_MAX-1，越界忽略
  * 返 回 值：无
  * 补    充：每个任务在自己的主循环里调；taskId 越界直接忽略，不会写坏数组
  */
void Watchdog_Heartbeat(WDT_TaskId_t taskId)
{
    if (taskId < WDT_TASK_MAX)
    {
        s_lastBeat[taskId] = xTaskGetTickCount();        /* tick 计数, 1kHz 下单位 ms */
    }
}

/**
  * 功    能：逐个比对各任务最后上报的时间，超过 3000ms 就算卡死（MONITOR 自己不查）
  * 参    数：无
  * 返 回 值：1=全部存活，0=有任务超过 3000ms 未上报(调用方不得喂狗)
  */
uint8_t Watchdog_CheckAllAlive(void)
{
    uint32_t now = xTaskGetTickCount();
    uint8_t  i;

    /*检查除 MonitorTask 外的所有任务心跳*/
    for (i = 0U; i < WDT_TASK_MAX; i++)
    {
        if (i == WDT_TASK_MONITOR)
        {
            continue;
        }
        /* STORAGE 槽位 boot 后没有常驻任务上报，参与判死会在 3s 后必然误复位，必须跳过 */
        if (i == WDT_TASK_STORAGE)
        {
            continue;
        }
        /*心跳为 0：任务还没跑过主循环(初始化阶段)。分两种情况：
         *  a) 开机不到 30s(首跳宽限期) —— 真在慢慢初始化，不判死；
         *  b) 开机超过 30s 仍是 0 —— 它卡死/死在初始化里了，判死。
         * 旧逻辑对"从未上报"一律豁免，盲区在于：任务若挂在 init 里
         *  (如 2026-11-02 的 24C02 自死锁事故)，心跳恒为 0，看门狗
         *  永远看不见它 —— 挂了 60 多秒系统照样喂狗，毫无反应。
         *  加宽限期后：这种任务会在 30s 后触发 task stuck 打印 + 停止
         *  喂狗 → 1s 后复位，每次复位前都能打印出"哪个 task 恒为 0"。 */
        if (s_lastBeat[i] == 0U)
        {
            if (now > pdMS_TO_TICKS(WDT_FIRSTBEAT_GRACE_MS))
            {
                return 0U;                  /* 上电 30s 还没跳过一次：卡在初始化里 */
            }
        }
        else if ((now - s_lastBeat[i]) > pdMS_TO_TICKS(WDT_HEARTBEAT_TIMEOUT_MS))
        {
            return 0U;                      /* 任务 i 卡死 */
        }
    }
    return 1U;
}

/**
  * 功    能：取某个任务最后一次心跳的时刻（tick，1kHz 即 ms）
  * 参    数：taskId  任务编号 0~WDT_TASK_MAX-1
  * 返 回 值：tick 值(1kHz，单位 ms)；越界或从未上报返回 0
  */
uint32_t Watchdog_GetLastBeat(WDT_TaskId_t taskId)
{
    if (taskId < WDT_TASK_MAX)
    {
        return s_lastBeat[taskId];
    }
    return 0U;
}
