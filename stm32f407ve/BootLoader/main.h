/* BootLoader/main.h
 * main.c 里的上电决策对外接口：App 向量表校验 + 跳转。
 * upgrade.c 在升级完成后要靠这两个函数决定"跳不跳、怎么跳"。
 */
#ifndef BOOT_MAIN_H
#define BOOT_MAIN_H

int  app_is_valid(void);
void jump_to_app(void);

#endif /* BOOT_MAIN_H */
