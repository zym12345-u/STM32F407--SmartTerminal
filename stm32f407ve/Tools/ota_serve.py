#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OTA 固件包生成 + 局域网 HTTP 服务（配合 bsp_OTA.c 使用）

做两件事：
  1) 从 Keil 的 .axf 生成 app.bin，算出 CRC32，写同目录的 app.meta
  2) 起一个 HTTP 服务，让开发板通过 ESP8266 下载 /app.meta 和 /app.bin

用法：
  python ota_serve.py                       # 默认取 MDK-ARM/stm32f407ve/stm32f407ve.axf
  python ota_serve.py --port 8080
  python ota_serve.py --bin app_image.bin   # 已经有 bin 时直接用，不跑 fromelf
  python ota_serve.py --version 1.5         # 不读 CanData.h，手工指定版本

注意：
  - 目标版本默认读 Core/App/Types/CanData.h 的 CAN_FW_VER_MAJOR/MINOR
  - 生成的 app.bin / app.meta 放在 axf（或 bin）所在目录
  - Windows 防火墙可能拦截 8080，首次运行请在弹出的对话框里允许 Python
"""

import argparse
import binascii
import os
import re
import shutil
import socket
import subprocess
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.dirname(SCRIPT_DIR)

DEFAULT_AXF = os.path.join(PROJ_DIR, "MDK-ARM", "stm32f407ve", "stm32f407ve.axf")
DEFAULT_FROMELF = r"D:\KeilMDK5\ARM\ARMCLANG\bin\fromelf.exe"
CANDATA_H = os.path.join(PROJ_DIR, "Core", "App", "Types", "CanData.h")

OTA_IMG_MAX = 444 * 1024       # 槽 B 镜像区上限（与 bsp_OTA.c 一致）


def find_fromelf(explicit):
    if explicit:
        return explicit
    if os.path.isfile(DEFAULT_FROMELF):
        return DEFAULT_FROMELF
    alt = os.path.join(os.path.dirname(DEFAULT_FROMELF), "ARMCC", "bin", "fromelf.exe")
    if os.path.isfile(alt):
        return alt
    raise SystemExit("[OTA] 找不到 fromelf.exe，请用 --fromelf 指定")


def read_version_from_header():
    """从 CanData.h 读主/次版本号；读不到就返回 None，由调用方要求 --version"""
    if not os.path.isfile(CANDATA_H):
        return None
    text = open(CANDATA_H, "r", encoding="utf-8", errors="ignore").read()
    mj = re.search(r"#define\s+CAN_FW_VER_MAJOR\s+0x([0-9A-Fa-f]+)U?", text)
    mn = re.search(r"#define\s+CAN_FW_VER_MINOR\s+0x([0-9A-Fa-f]+)U?", text)
    if not mj or not mn:
        return None
    return int(mj.group(1), 16), int(mn.group(1), 16)


def build_bin(axf, fromelf):
    out_dir = os.path.dirname(axf)
    out_bin = os.path.join(out_dir, "app.bin")
    print("[OTA] fromelf: %s" % fromelf)
    print("[OTA] axf    : %s" % axf)
    subprocess.run([fromelf, "--bin", "--output", out_bin, axf], check=True)
    return out_bin


def pad_and_crc(path):
    data = bytearray(open(path, "rb").read())
    if len(data) % 4:
        pad = 4 - (len(data) % 4)
        data += b"\xFF" * pad
        print("[OTA] 末尾补齐 %d 字节 0xFF（按字写 Flash 要求 4 字节对齐）" % pad)
        open(path, "wb").write(data)
    size = len(data)
    crc = binascii.crc32(bytes(data)) & 0xFFFFFFFF
    return size, crc


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(0.5)
        s.connect(("223.5.5.5", 80))      # 只查路由，不真正发包
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description="OTA 固件包生成 + HTTP 服务")
    ap.add_argument("--axf", default=DEFAULT_AXF, help="Keil 生成的 axf 路径")
    ap.add_argument("--bin", default=None, help="已有 bin；给了就不再跑 fromelf")
    ap.add_argument("--fromelf", default=None, help="fromelf.exe 路径")
    ap.add_argument("--version", default=None, help="目标版本，如 1.5；默认读 CanData.h")
    ap.add_argument("--port", type=int, default=8080, help="HTTP 端口，默认 8080")
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
    args = ap.parse_args()

    if args.bin:
        src_bin = os.path.abspath(args.bin)
        if not os.path.isfile(src_bin):
            raise SystemExit("[OTA] 找不到 bin: %s" % src_bin)
        serve_dir = os.path.dirname(src_bin)
        # 固件里的路径固定是 /app.bin，源文件叫别的名字就复制一份
        app_bin = os.path.join(serve_dir, "app.bin")
        if os.path.abspath(src_bin) != os.path.abspath(app_bin):
            shutil.copyfile(src_bin, app_bin)
            print("[OTA] 已复制为 app.bin")
    else:
        if not os.path.isfile(args.axf):
            raise SystemExit("[OTA] 找不到 axf: %s（先编译 App）" % args.axf)
        serve_dir = os.path.dirname(os.path.abspath(args.axf))
        app_bin = build_bin(os.path.abspath(args.axf), find_fromelf(args.fromelf))

    # 版本号
    if args.version:
        ver = args.version
    else:
        v = read_version_from_header()
        if not v:
            raise SystemExit("[OTA] 读不到 CanData.h 版本号，请用 --version 指定")
        ver = "%d.%d" % v

    size, crc = pad_and_crc(app_bin)
    if size > OTA_IMG_MAX:
        raise SystemExit("[OTA] 镜像 %d 字节，超过槽 B 上限 %d" % (size, OTA_IMG_MAX))

    meta_path = os.path.join(serve_dir, "app.meta")
    with open(meta_path, "w", encoding="ascii", newline="\n") as f:
        f.write("version=%s\n" % ver)
        f.write("size=%d\n" % size)
        f.write("crc=%d\n" % crc)

    ip = lan_ip() or "PC 的局域网 IP"
    print("[OTA] 版本    : %s" % ver)
    print("[OTA] 大小    : %d 字节" % size)
    print("[OTA] CRC32   : 0x%08X" % crc)
    print("[OTA] 目录    : %s" % serve_dir)
    print("[OTA] 地址    : http://%s:%d/app.bin" % (ip, args.port))
    print("")
    print("开发板服务器地址二选一（默认值见 bsp_OTA.h 的 OTA_SRV_HOST/PORT）：")
    print("  1) 串口发送: otacfg %s %d" % (ip, args.port))
    print("  2) 写 LittleFS /ota.conf: host=%s  port=%d 后复位" % (ip, args.port))
    print("然后在屏幕 WiFi OTA 页点 Check，再点 Download")
    print("按 Ctrl+C 停止服务")
    print("")

    handler = partial(SimpleHTTPRequestHandler, directory=serve_dir)
    httpd = ThreadingHTTPServer((args.bind, args.port), handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[OTA] 服务已停止")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    sys.exit(main())
