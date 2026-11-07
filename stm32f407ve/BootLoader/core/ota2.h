/* BootLoader/core/ota2.h
 * OTA-2 启动处理入口：槽 B 搬运、失败回滚槽 A、启动确认计数回滚。
 * main.c 在串口升级窗口关闭后、boot_meta_report() 之前调用一次。
 */
#ifndef OTA2_H
#define OTA2_H

/**
  * 功    能：按 24C02 的 OTA-2 状态页执行搬运/确认/回滚
  * 参    数：无
  * 返 回 值：无
  * 补    充：内部完成 App 区擦写、槽记录校验、元数据写入和状态清理；
  *            任何失败都只打日志并让启动流程继续走到 app_is_valid() 判断
  */
void ota2_boot_handle(void);

#endif /* OTA2_H */
