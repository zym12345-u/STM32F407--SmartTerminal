/* bsp_OTA.h
 * WiFi OTA（第一阶段：只下载到 W25Q 槽 B，不改启动路径）。
 * 服务端约定：PC 上跑 Tools/ota_serve.py，同目录提供 app.bin 与 app.meta。
 *
 * app.meta 是纯文本，三行键值，例如：
 *   version=1.5
 *   size=305856
 *   crc=3396383332
 * size 必须是 4 的整数倍（BootLoader 搬运时按字写内部 Flash）；
 * crc 用标准 CRC32（多项式 0xEDB88320，初值/终值 0xFFFFFFFF），与 IAP 工具一致。
 */
#ifndef __BSP_OTA_H
#define __BSP_OTA_H

#include <stdint.h>
#include "bsp_W25Q128.h"

/*══════ 部署参数：测试前把 IP 改成 PC 的局域网地址 ══════*/
#define OTA_SRV_HOST   "192.168.10.3"   /* PC 的 IPv4；ota_serve.py 启动时会打印 */
#define OTA_SRV_PORT   8080U             /* HTTP 端口，与 ota_serve.py --port 一致 */
#define OTA_META_PATH  "/app.meta"
#define OTA_BIN_PATH   "/app.bin"

/*槽 B 的记录格式：1=整区备份（槽 A），2=OTA 镜像（只有前 size 字节有效）*/
#define OTA_REC_FMT    2UL

/**
  * 功    能：把 app.bin 下载到 W25Q 槽 B，并回读校验、写槽记录
  * 参    数：无
  * 返 回 值：1 = 成功；0 = 失败（原因写 g_otaStatus.err）
  * 补    充：只在 WifiTask 里调用（ESP8266 单消费者 + W25Q 与 WifiStore 共用 SPI1）；
  *            阻塞数十秒，全程补报 WIFI 心跳；失败时槽 B 不写记录，BootLoader 不会认它
  */
uint8_t OTA_DownloadToSlotB(void);

/**
  * 功    能：把“没开始就失败”的原因写进共享状态（如 WiFi 未连接）
  * 参    数：err  OtaErr_t
  * 返 回 值：无
  * 补    充：WifiTask 用；GUI 轮询到 FAIL 后按 err 显示
  */
void OTA_ReportFail(uint8_t err);

#endif /* __BSP_OTA_H */
