/* bsp_BH1750.c
 * BH1750 光照传感器驱动：读连续高分辨率模式，输出 float lux。
 * 硬件：SCL=PB6 / SDA=PB7，开漏上拉，100kHz，7 位地址 0x23。
 * 注意：上电后第一次转换要等 120ms；总线 BUSY 死锁恢复已收进 bsp_I2CBus 层。
 *
 * ══════ 这层和 bsp_I2CBus 怎么分工 ══════
 *   bsp_I2CBus（总线层）：管 PB6/PB7 引脚、I2C1 寄存器、START/STOP/ACK 时序、
 *                        总线互斥锁、死锁恢复 —— 它不认识任何具体器件。
 *   本文件（器件层）：只管 BH1750 的命令字和数据含义 ——
 *                        0x23 是谁、上电命令、模式选择、raw/1.2 换算。
 *   同一条总线上还挂着 24C02(0x50)，谁也不许绕过总线层直接摸 I2C1 寄存器，
 *   否则两边的状态机会互相踩（这正是抽这层的理由）。
 *
 *   I2C 的电气/协议基础(开漏上拉、起始停止、ACK、死锁成因)见 bsp_I2CBus.c 文件头。
 */
#include "bsp_BH1750.h"
#include "bsp_I2CBus.h"


/*全局标志*/
static uint8_t  s_bh1750_ready;   /* 初始化完成标志 */


/*BH1750 对外接口*/

/**
  * 功    能：初始化光照传感器：上电、设成连续高分辨率模式，失败会重试 3 次
  * 参    数：无
  * 返 回 值：BH1750_OK=成功，BH1750_ERROR=重试 3 次仍失败
  * 补    充：三条命令(上电→复位→设模式)在【一把锁内】发完——中间被别的
  *            任务(比如 24C02 的访问)插进来，BH1750 可能缺一条命令进不了
  *            目标模式。但 HAL_Delay(120) 在锁外等：等的是传感器自己的
  *            光电转换，跟总线没关系，持锁白占。
  */
BH1750_Status_t BH1750_Init(void)
{
    uint8_t cmd;
    uint8_t retry;
    uint8_t ok;

    s_bh1750_ready = 0;

    /*初始化 I2C1 总线(幂等：总线已在跑就只是重配一遍，锁也只建一次)*/
    (void)I2CBus_Init();

    /*整条序列最多重试 BH1750_RETRY_CNT 次*/
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        I2CBus_Lock();
        /*上电*/
        cmd = BH1750_CMD_POWER_ON;
        ok  = (I2CBus_WriteRaw(BH1750_I2C_ADDR, &cmd, 1U) == 0U);    /* 7 位 0x23 左移 1 位 = 0x46(W) */

        /*复位清数据寄存器，不做会读到上次的残留值*/
        if (ok)
        {
            cmd = BH1750_CMD_RESET;
            ok  = (I2CBus_WriteRaw(BH1750_I2C_ADDR, &cmd, 1U) == 0U);
        }

        /*连续高分辨率：1 lux，转换 ≤120ms*/
        if (ok)
        {
            cmd = BH1750_CMD_CONT_H_MODE;
            ok  = (I2CBus_WriteRaw(BH1750_I2C_ADDR, &cmd, 1U) == 0U);
        }
        I2CBus_Unlock();

        if (!ok)
            continue;

        /*等第一次转换(锁外：跟总线无关，持锁白占)*/
        HAL_Delay(120);

        s_bh1750_ready = 1;
        return BH1750_OK;
    }

    return BH1750_ERROR;
}


/* 读一次光照值，单位 lux（0~54612），读成功后把 is_valid 置 1。
 * 返回 BH1750_OK 成功；BH1750_ERROR 表示 data 为空、没初始化或重试 3 次仍失败。 */
/* ---------------------------------------------------------------------------
 * 连续高分辨率模式数据(只读)：
 *   名称   类型/位宽  含义               单位/范围     谁写/谁读
 *   DataH  uint8      高 8 位            0x00~0xFF     BH1750 写 / 主机读
 *   DataL  uint8      低 8 位            0x00~0xFF     BH1750 写 / 主机读
 *   raw    uint16     (DataH<<8)|DataL   0~65535       主机算
 *   lux    float      raw / 1.2          0~54612       主机算
 * ------------------------------------------------------------------------- */
BH1750_Status_t BH1750_Read(BH1750_Data_t *data)
{
    uint8_t  buf[2];
    uint16_t raw;
    uint8_t  retry;

    if (data == NULL)
        return BH1750_ERROR;

    data->is_valid = 0;

    if (!s_bh1750_ready)
        return BH1750_ERROR;

    /*重试 3 次，过滤偶发干扰。读 2 字节是一次总线事务，锁包住它*/
    for (retry = 0; retry < BH1750_RETRY_CNT; retry++)
    {
        /*连续模式下直接读 2 字节*/
        I2CBus_Lock();
        if (I2CBus_ReadRaw(BH1750_I2C_ADDR, buf, 2U) == 0U)          /* 0x47(R): 直接读最近一次转换结果 */
        {
            I2CBus_Unlock();
            raw  = ((uint16_t)buf[0] << 8) | buf[1];
            data->lux      = (float)raw / 1.2f;    /* 换算系数 1.2 出自 BH1750 数据手册 */
            data->is_valid = 1;
            return BH1750_OK;
        }
        I2CBus_Unlock();
    }

    /*读失败清标志，下次必须重新 Init*/
    s_bh1750_ready = 0;
    return BH1750_ERROR;
}
