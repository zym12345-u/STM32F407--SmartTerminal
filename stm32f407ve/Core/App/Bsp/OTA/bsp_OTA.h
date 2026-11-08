/* bsp_OTA.h
 * WiFi OTA：把固件下载进 W25Q 槽 B，并置"下次上电搬运"的请求。
 * 服务端约定：PC 上跑 Tools/ota_serve.py，同目录提供 app.bin 与 app.meta。
 *
 * ── 本模块的边界 ──
 *   不碰内部 Flash：不擦 App 区、不切换启动。
 *   下载成功且搬运标志写妥后，延时 3s 复位交给 BootLoader；
 *   "槽 B → App 区"的搬运与回滚仍由 BootLoader/core/ota2.c 在复位后完成。
 *
 * app.meta 是纯文本，三行键值，例如：
 *   version=1.5
 *   size=305856
 *   crc=3396383332
 * 三个坑：
 *   1) size 必须是 4 的整数倍（BootLoader 搬运时按 32 位字写内部 Flash），
 *      ota_serve.py 会自动补 0xFF 到 4 字节对齐
 *   2) crc 是【十进制】，不是十六进制；手改 meta 时最容易在这翻车，
 *      所以脚本生成后不要手工编辑（bin 一变 meta 必须重生成）
 *   3) crc 用标准 CRC32（多项式 0xEDB88320，初值/终值 0xFFFFFFFF），
 *      与 iap_upgrade.py / binascii.crc32 一致，不能用 MCU 硬件 CRC 算
 */
#ifndef __BSP_OTA_H
#define __BSP_OTA_H

#include <stdint.h>
#include "bsp_W25Q128.h"

/*══════ 部署参数：出厂默认值，运行时由 LittleFS /ota.conf 覆盖 ══════
 * 优先级：/ota.conf（串口 otacfg 命令或直接写文件）> 下面两个宏。
 * 文件不存在/损坏时自动回退到这里，所以不带配置也能跑。
 * ota_serve.py 启动时会打印本机 IP，可串口执行 otacfg <ip> <port> 改配置，无需重编。*/
#define OTA_SRV_HOST   "192.168.10.3"   /* PC 的 IPv4，作为编译期默认值 */
#define OTA_SRV_PORT   8080U             /* HTTP 端口，与 ota_serve.py --port 一致 */
#define OTA_META_PATH  "/app.meta"       /* 路径写死：服务端目录里只能有这两个文件角色 */
#define OTA_BIN_PATH   "/app.bin"

/*槽 B 的记录格式：1=整区备份（槽 A），2=OTA 镜像（只有前 size 字节有效）。
 *本模块只写格式 2；格式 1 由 GuiTask 的整区备份流程写，两边不能互换：
 *格式 1 的 size 恒为 448KB（整区），格式 2 的 size 是真实镜像长度。*/
#define OTA_REC_FMT    2UL

/**
  * 功    能：把 app.bin 下载到 W25Q 槽 B，并回读校验、写槽记录
  * 参    数：无
  * 返 回 值：1 = 成功；0 = 失败（原因写 g_otaStatus.err）
  * 补    充：只在 WifiTask 里调用（ESP8266 单消费者 + W25Q 与 WifiStore 共用 SPI1）；
  *            阻塞数十秒，全程补报 WIFI 心跳；失败时槽 B 不写记录，BootLoader 不会认它。
  *            成功且搬运标志写妥后置 g_otaStatus.rebootPending=1，由 WifiTask 3s 后复位
  */
uint8_t OTA_DownloadToSlotB(void);

/**
  * 功    能：把“没开始就失败”的原因写进共享状态（如 WiFi 未连接）
  * 参    数：err  OtaErr_t
  * 返 回 值：无
  * 补    充：WifiTask 用；GUI 轮询到 FAIL 后按 err 显示
  */
void OTA_ReportFail(uint8_t err);

/**
  * 功    能：取当前运行固件的版本号
  * 参    数：ver  输出 (主<<8|次)，不能为空
  * 返 回 值：1 = 读自 sector 3 元数据；0 = 元数据不可信，用 CanData.h 编译期常量
  * 补    充：判据与 BootLoader 的 meta_read 一致：magic/格式/大小/记录 CRC/'MEND' 标记；
  *            GUI 与检查更新共用这一份，避免两套判据漂移
  */
uint8_t OTA_GetCurrentVer(uint16_t *ver);

/**
  * 功    能：只 GET /app.meta 检查是否有更高版本，不擦槽、不下载、不写记录
  * 参    数：newVer  输出发现的更高版本；无更新或失败时为 0
  * 返 回 值：OTA_ERR_NONE = 检查完成；其它 = 失败原因
  * 补    充：仅 WifiTask 调用；meta 请求最多 3 次，退避 1s/3s
  */
uint8_t OTA_CheckUpdate(uint16_t *newVer);

/**
  * 功    能：下载成功并写妥搬运标志后，延时 3s 自动复位交给 BootLoader
  * 参    数：无
  * 返 回 值：无，函数内 NVIC 复位不返回
  * 补    充：仅 WifiTask 调用；3s 内每 500ms 补一次 WiFi 心跳，GUI 显示 Done - rebooting...
  */
void OTA_RebootToApply(void);

#endif /* __BSP_OTA_H */
