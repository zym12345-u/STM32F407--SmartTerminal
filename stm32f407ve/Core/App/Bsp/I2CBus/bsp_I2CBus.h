/* bsp_I2CBus.h
 * I2C1 总线共享层：硬件初始化、原始读写、ACK 探测、总线恢复、跨任务互斥。
 * 硬件：I2C1 = PB6(SCL) / PB7(SDA)，开漏上拉，100kHz。
 * 挂在总线上的器件：BH1750(7位地址 0x23) / 24C02(7位地址 0x50)。
 * 注意：本层只管"总线"，不认识任何具体器件；器件语义(命令字/字地址/页)在各自驱动里。
 *       原始读写函数(WriteRaw/ReadRaw/...)【不带锁】，锁由器件驱动按"事务"粒度加。
 */
#ifndef __BSP_I2CBUS_H
#define __BSP_I2CBUS_H

#include "stm32f4xx_hal.h"


/**
  * 功    能：初始化 I2C1 的引脚(PB6/PB7 复用开漏)和寄存器(100kHz)，并创建总线互斥锁
  * 参    数：无
  * 返 回 值：0 = 成功；1 = 时钟/引脚配置失败(当前实现恒为 0)
  * 补    充：幂等——重复调用只是把寄存器重配一遍(先关 PE 再配)，锁只创建一次。
  *            由 BH1750_Init / AT24C02_Init 各自调用，谁先跑都一样。
  */
uint8_t I2CBus_Init(void);


/**
  * 功    能：总线互斥锁——拿不到就一直等(osWaitForever)
  * 参    数：无
  * 返 回 值：无
  * 补    充：内核未启动或锁未创建时直通(此场景只有一条执行流，无并发可竞争)。
  *            【为什么锁在这层而不是散在各处】：I2C1 的 CR1/DR/SR1/SR2 是一套
  *            有状态的外设状态机，两个任务并发操作它，A 刚发完 START、B 又发一个，
  *            状态机直接踩乱。锁保护的是"外设状态机"这个共享资源，与
  *            usart.c 的 uart1PrintfMutex 保护共享缓冲是同一类问题。
  *            【锁的粒度 = 一次完整事务】：器件驱动的"发命令→转读→收数据"这一串
  *            必须在锁内一次做完(Linux 的 i2c_transfer 同理)，中途松手别的器件
  *            就会插进来，RESTART 序列被拆断。
  */
void I2CBus_Lock(void);


/**
  * 功    能：放总线互斥锁
  * 参    数：无
  * 返 回 值：无
  * 补    充：与 I2CBus_Lock 严格配对；器件驱动保证所有出口都放锁(单一出口写法)。
  */
void I2CBus_Unlock(void);


/**
  * 功    能：主机模式发一串字节：START → 从机地址+写 → 数据 → STOP
  * 参    数：addr8  8 位从机地址(7位地址左移1位后放最低位方向,写方向由本函数置0)；
  *           pData  待发数据；len 字节数 1~65535
  * 返 回 值：0 = 成功；1 = 某一步超时(已自动尝试总线恢复)
  * 补    充：【不带锁】。任何超时都会调 I2CBus_Recovery 尝试把总线救回来。
  */
uint8_t I2CBus_WriteRaw(uint8_t addr8, const uint8_t *pData, uint16_t len);


/**
  * 功    能：主机模式读一串字节：START → 从机地址+读 → 收数据 → STOP
  * 参    数：addr8  8 位从机地址(读方向由本函数置1)；pData 接收缓冲；len 1~65535，传 0 直接失败
  * 返 回 值：0 = 成功；1 = 参数非法或超时(已自动尝试总线恢复)
  * 补    充：【不带锁】。单字节读的 NACK 时序(先清 ACK 再清 ADDR)在函数内处理。
  */
uint8_t I2CBus_ReadRaw(uint8_t addr8, uint8_t *pData, uint16_t len);


/**
  * 功    能：先写后读的【组合事务】：START → 地址+W → 写 wrLen 字节 → 重复起始
  *           RESTART → 地址+R → 收 rdLen 字节 → STOP
  * 参    数：addr8  8 位从机地址(方向位由本函数分别置 W/R)；
  *           pWr/wrLen 先发的内容(通常是器件的寄存器/存储地址)，wrLen 1~65535；
  *           pRd/rdLen 随后要收的内容，rdLen 1~65535
  * 返 回 值：0 = 成功；1 = 参数非法或某步超时(已自动尝试总线恢复)
  * 补    充：【不带锁】。这是读 EEPROM / 读寄存器型器件的标准姿势：
  *            「先写要读的地址，不 STOP、直接 RESTART 转读」。
  *            中间不能 STOP 再重新 START——那会让别的节点以为总线空了插进来
  *            (原子性)；对 EEPROM 还有个更实际的原因：STOP 会把"写字地址"当成
  *            一次完整写操作触发内部写周期，后面根本读不回来。
  */
uint8_t I2CBus_WriteReadRaw(uint8_t addr8, const uint8_t *pWr, uint16_t wrLen,
                            uint8_t *pRd, uint16_t rdLen);


/**
  * 功    能：探测器件在不在——只发"地址字节"看它回不回 ACK
  * 参    数：addr8  8 位从机地址(方向位强制为写)
  * 返 回 值：0 = 器件应答(在位)；1 = 无应答或总线超时
  * 补    充：【不带锁】。两个用途：
  *            ① 上电检测器件焊没焊/在不在(初始化自检)；
  *            ② EEPROM 的【ACK 轮询】：写入后内部写周期(最长 5ms)期间器件
  *               不应答任何地址，反复 Probe 直到收到 ACK 就知道写完了——
  *               这和 W25Q128 轮询状态寄存器 BUSY 位是同一件事的两种实现。
  *            无应答属"正常业务"(写周期中)，所以这里只发 STOP 释放总线，
  *            不触发 BusRecovery。
  */
uint8_t I2CBus_Probe(uint8_t addr8);


#endif /* __BSP_I2CBUS_H */
