# V2.0 发布说明

- 日期：2026-11-08
- 分支：`main`
- 标签：`v2.0`
- 验证：BootLoader 与 App 均 0 Error / 0 Warning；串口 IAP、WiFi OTA、回滚、启动确认均已真机通过

## 主要变更

- **BootLoader**：内部 Flash 前 48KB，App 迁至 `0x08010000`；元数据 sector 3 改为追加记录（v3），掉电不留“已擦未写”裸窗
- **串口 IAP**：上电 3s 握手窗口 + 屏幕按钮进入 60s 升级窗口；帧协议、整包 CRC32、断点续传、受控擦写，PC 端工具 `Tools/iap_upgrade.py`
- **W25Q 双固件槽**：槽 A 存当前 App 整区备份，槽 B 存 OTA 镜像；32B 槽记录含 magic、格式、版本、大小、镜像 CRC 和记录 CRC
- **WiFi OTA**：ESP8266 HTTP 流式下载 `app.meta` / `app.bin` 到槽 B，`+IPD` 二进制分块落盘，下载后整包 CRC 校验
- **搬运 / 回滚 / 确认**：BootLoader 先校验槽 A 整区 CRC，再把槽 B 搬到内部 App 区并回读双 CRC；连续 3 次上电未确认则自动回滚槽 A
- **App 侧**：OTA 页显示 Current / New，下载成功自动置 24C02 `0xE8` 搬运请求；MonitorTask 全任务健康 3s 后确认启动

## 内部 Flash 分区

| 区域 | 地址范围 | 大小 | 说明 |
|---|---|---|---|
| BootLoader | `0x08000000 ~ 0x0800BFFF` | 48KB | sector 0~2 |
| 升级元数据 | `0x0800C000 ~ 0x0800FFFF` | 16KB | sector 3，只用前 4KB |
| App | `0x08010000 ~ 0x0807FFFF` | 448KB | sector 4~7 |

## W25Q128 双槽

| 槽 | 基址 | 槽大小 | 用途 |
|---|---|---|---|
| 槽 A | `0x00800000` | `0x00070000` | 当前 App 整区备份 |
| 槽 B | `0x00871000` | `0x00070000` | OTA 镜像 |

槽 B 必须从 `0x00871000` 开始：槽 A 的 448KB 备份写到 `0x00870FFF`，若槽 B 从 `0x00870000` 起，其 4KB 记录扇区会压掉槽 A 尾部，导致槽 A 整区 CRC 永久失效。

## 验证记录（2026-11-07 ~ 2026-11-08）

- 槽 A 备份：`size=458752`，`crc=0x2C6AE8BC`，PASS
- WiFi OTA 下载：`size=312368`，`crc=0x0D6EE0EE`，约 33.4s，PASS
- 搬运启动确认：`[OTA2] apply ok, waiting confirm` → `[BOOT] meta v3 fw=1.5` → `[MON] boot confirmed`
- 未确认回滚：`1/3` → `2/3` → `3/3` → `[OTA2] unconfirmed x3, rollback slot A` → `[OTA2] rollback ok`，元数据回 `fw=1.4`
- 串口 IAP：擦除 7312 ms；写入 312368 字节 / 2441 帧 / 47.2 s；`[BOOT] meta saved, fw v1.4` → `[BOOT] jump to app` → `DONE`

## 构建指标

- App：`Code=232532 (227.1KB) / RO=79624 (77.8KB) / RW=1156 (1.1KB) / ZI=113020 (110.4KB)`
- BootLoader：`Code=8140 (7.9KB) / RO=424 (0.4KB) / RW=20B / ZI=5476 (5.3KB)`

## 升级方式

- **首次从 V1.4 升级到 V2.0**：必须先用 SWD 烧录 BootLoader + App；V1.4 没有 BootLoader，无法远程安装 boot 本身
- **串口 IAP**：`python stm32f407ve/Tools/iap_upgrade.py <app.bin> -p COMx`
- **WiFi OTA**：`python stm32f407ve/Tools/ota_serve.py`，屏幕 OTA 页点 Download，成功后复位

## 已知限制

- 回滚后 `meta.size` 记录为槽 A 整区 448KB，不是实际 App 字节数；不影响启动，列入 V2.1 清理项
- OTA 服务器地址仍是编译期宏；需要手动点 Download 并手动复位，V2.1 做产品化
- 仅 HTTP，无 HTTPS/鉴权，当前按局域网使用设计
- MQTT 通道远程升级尚未实现，排在 V2.2

## 迁移注意

- 旧布局（槽 B = `0x00870000`）下的槽 A 备份和槽 B 镜像均已失效；升级到 V2.0 后必须重新备份槽 A、重新下载 OTA 镜像
