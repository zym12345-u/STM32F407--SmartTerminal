"""生成 OTA 坏包测试文件：只损坏测试副本，不修改正式固件和开发板。"""

import argparse
import binascii
import shutil
import tempfile
from pathlib import Path

from ota_serve import DEFAULT_AXF, build_bin, find_fromelf, pad_and_crc


def main():
    parser = argparse.ArgumentParser(description="准备 OTA CRC 坏包测试文件")
    parser.add_argument("--version", default="1.7", help="测试包版本，默认 1.7")
    args = parser.parse_args()

    # 复用正式工具，从当前 Keil AXF 重新生成对齐后的正常 app.bin。
    bin_path = Path(build_bin(DEFAULT_AXF, find_fromelf(None)))
    size, good_crc = pad_and_crc(str(bin_path))

    work_dir = Path(tempfile.gettempdir()) / "stm32_ota_bad_test"
    good_dir = work_dir / "good"
    bad_dir = work_dir / "bad"
    good_dir.mkdir(parents=True, exist_ok=True)
    bad_dir.mkdir(parents=True, exist_ok=True)

    # 正常包用于后续断网测试；坏包用于本次 CRC 测试。
    for directory in (good_dir, bad_dir):
        shutil.copy2(bin_path, directory / "app.bin")
        with (directory / "app.meta").open("w", encoding="ascii", newline="\n") as meta:
            meta.write(f"version={args.version}\nsize={size}\ncrc={good_crc}\n")

    # 仅翻转坏包中间的一个字节；app.meta 中仍保留正常包的 CRC。
    bad_path = bad_dir / "app.bin"
    bad_data = bytearray(bad_path.read_bytes())
    index = len(bad_data) // 2
    old_value = bad_data[index]
    bad_data[index] ^= 0x01
    bad_path.write_bytes(bad_data)
    bad_crc = binascii.crc32(bad_data) & 0xFFFFFFFF

    if bad_crc == good_crc:
        raise SystemExit("[失败] 坏包 CRC 未发生变化，请停止测试并检查脚本")

    print(f"[正常包] {good_dir}")
    print(f"[坏  包] {bad_dir}")
    print(f"[证  据] 第 {index} 字节: 0x{old_value:02X} -> 0x{bad_data[index]:02X}")
    print(f"[证  据] meta CRC={good_crc}，坏包实际 CRC={bad_crc}")
    print("[预  期] 设备下载完成后必须报 err=9，不能自动复位升级")
    print("[下一步] 启动坏包 HTTP 服务：")
    print(f'python -m http.server 8080 --bind 0.0.0.0 --directory "{bad_dir}"')


if __name__ == "__main__":
    main()
