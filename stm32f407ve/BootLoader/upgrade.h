/* BootLoader/upgrade.h
 * 升级模式主循环：收命令、分发、写 Flash、校验整包、跳 App。
 */
#ifndef UPGRADE_H
#define UPGRADE_H

void upgrade_loop(void);

#endif /* UPGRADE_H */
