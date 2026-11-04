# iap_upgrade.py
# STM32F407 串口 IAP 升级脚本：等 COM 出现 -> 连发握手 -> 擦除 -> 128B 分帧停等写入 -> 结束 -> 跳 App
#
# 依赖：pip install pyserial
#
# 用法：
#   python iap_upgrade.py app.bin                 # 默认 COM3，等端口出现 180 秒
#   python iap_upgrade.py app.bin -p COM5
#   python iap_upgrade.py app.bin --fw-ver 1.4    # 固件版本随结束帧下发，写入 sector 3 元数据
#   python iap_upgrade.py app.bin --wait-port 0   # 不等端口，直接开（报错即退出）
#
# ── 与 BootLoader 的协议对应关系（改协议两边必须一起改）──
#   帧格式：A5 5A | 命令 | 长度 | 数据(0~128B) | CRC32(4B 小端)
#   CRC32：多项式 0xEDB88320 逐位，初值/终值 0xFFFFFFFF，只覆盖【命令+长度+数据】
#   命令：01 握手 / 02 数据块 / 03 结束(6B：CRC32+版本主/次) / 04 擦除(8B 起止地址，结束地址不含)
#   应答：命令号|0x80，数据 1 字节状态：0 成功 / 1 越界 / 2 Flash 出错 / 3 整包 CRC 不符
#
# ── 为什么先等 COM 出现再打开 ──
#   断电续升级场景：板子重上电后 USB 转串口要重新枚举，设备管理器里 COM 口
#   是"晚几秒才出现"的。脚本轮询等它出现再打开，就不用人掐着时间按回车。
#
# ── 为什么握手要连发而不是等横幅 ──
#   正常路径：BootLoader 上电有 3 秒窗口，连发保证窗口内必中一帧；
#   断电恢复路径：24C02 里有升级标志，BootLoader 跳过窗口直接进升级模式，
#   升级模式里收到握手照样回 ACK（main.c 的 upgrade_loop 就是这么写的）。
#   两种路径用同一套"连发握手"起步，脚本不用分辨现在是哪种情况。
#
# ── 数据帧为什么"只等不重发" ──
#   设备是顺序写入的：同一块数据写两遍，后面全部错位，镜像就废了。
#   超时后只做两件事：再等一会（可能只是 ACK 丢在半路）、发握手做存活探测
#   （能回说明设备还活着，是丢帧；不回说明死机/复位），然后报错交给人判断。

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print('缺少 pyserial，请先执行: pip install pyserial')
    sys.exit(1)

# ── 协议常量：与 BootLoader/main.c 里的 #define 一一对应 ──
SYNC0, SYNC1 = 0xA5, 0x5A
CMD_HANDSHAKE, CMD_DATA, CMD_END, CMD_ERASE = 0x01, 0x02, 0x03, 0x04
CMD_PROGRESS = 0x05        # 查询已写字节数（断电续传）
ST_OK, ST_RANGE, ST_FLASH, ST_CRC = 0, 1, 2, 3

APP_BASE = 0x08010000      # App 区起点（sector 4）
APP_LIMIT = 0x08080000     # App 区上界（不含）
SRAM_BASE, SRAM_LIMIT = 0x20000000, 0x20020000


def crc32(data: bytes) -> int:
    """逐位 CRC32（多项式 0xEDB88320）。不建查表，和 BootLoader 侧算法逐位对应。"""
    c = 0xFFFFFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if (c & 1) else c >> 1
    return c ^ 0xFFFFFFFF


def make_frame(cmd: int, data: bytes = b'') -> bytes:
    """组一帧：A5 5A | cmd | len | data | CRC32(小端，只盖 cmd+len+data)"""
    body = bytes([cmd, len(data)]) + data
    c = crc32(body)
    return bytes([SYNC0, SYNC1]) + body + c.to_bytes(4, 'little')


def parse_fw_ver(text: str):
    """把 --fw-ver 解析成 (主, 次)，每段必须 0~255；结束帧长度只有 1 字节，写不下更大的数"""
    parts = text.split('.')
    if len(parts) != 2:
        raise argparse.ArgumentTypeError('版本号格式应为 主.次，如 1.4')
    try:
        major, minor = int(parts[0]), int(parts[1])
    except ValueError:
        raise argparse.ArgumentTypeError('版本号两段都必须是十进制数字')
    if not (0 <= major <= 255 and 0 <= minor <= 255):
        raise argparse.ArgumentTypeError('版本号每段范围 0~255')
    return major, minor


def self_test():
    """开机自检：CRC 用标准向量 '123456789' -> 0xCBF43926；
    握手帧必须等于真机验证过的 A5 5A 01 00 BE 23 C2 58。
    防的是"脚本被改坏了自己不知道，把锅甩给单片机"。"""
    assert crc32(b'123456789') == 0xCBF43926, 'CRC 自检失败'
    assert make_frame(CMD_HANDSHAKE) == bytes.fromhex('A55A0100BE23C258'), '握手帧自检失败'


class LinkLost(Exception):
    """串口句柄失效（USB 驱动瞬时错误或设备重新枚举）。
    擦除阶段可安全重连重发（擦除幂等）；数据阶段必须停下，重发会写重导致镜像错位。"""


class IapSerial:
    """串口 + 收包缓冲 + 文本透传。所有接收都先进 acc，帧和文本都从这里面捞。"""

    def __init__(self, port: str, baud: int):
        self.port_name = port
        self.baud = baud
        self.ser = None
        self.acc = bytearray()      # 收到的原始字节（帧从这里解析）
        self.text = ''              # acc 的可打印视图，用来透传 [BOOT] 日志
        self.printed = 0            # text 里已经按行打印到哪儿了
        self.open()

    def open(self):
        self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.acc.clear()
        self.text = ''
        self.printed = 0

    def reopen(self, wait_s: float) -> bool:
        """掉线后重连：先关旧句柄，等设备重新枚举出现，再打开"""
        try:
            self.ser.close()
        except Exception:
            pass
        if not wait_port_appear(self.port_name, wait_s):
            return False
        try:
            self.open()
            return True
        except (serial.SerialException, OSError):
            return False

    def pump(self):
        """把串口上已有的字节全收进 acc，并同步维护可打印文本视图"""
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            chunk = self.ser.read(n)
        except (serial.SerialException, OSError) as exc:
            raise LinkLost(str(exc))
        self.acc += chunk
        self.text += ''.join(chr(b) if 32 <= b < 127 or b in (10, 13) else '.' for b in chunk)
        # 只打印设备日志行（以 [BOOT] 开头或含 jump to app），ACK 帧产生的 '.' 行不打扰
        while True:
            nl = self.text.find('\n', self.printed)
            if nl < 0:
                break
            line = self.text[self.printed:nl].strip('\r. ').strip()
            self.printed = nl + 1
            if line.startswith('[BOOT]') or 'jump to app' in line:
                print(f'  设备: {line}')
        # 防膨胀：握手前的噪声攒太多就扔一半（保留尾部，别切到帧中间的风险可接受）
        if len(self.acc) > 4000:
            del self.acc[:-2000]

    def find_ack(self, cmd: int) -> int:
        """在 acc 里找 cmd 的合法应答帧（帧头+命令号|0x80+长度1+CRC 全对才算）"""
        want = cmd | 0x80
        n = len(self.acc)
        for i in range(0, n - 8):
            if self.acc[i] == SYNC0 and self.acc[i + 1] == SYNC1 \
                    and self.acc[i + 2] == want and self.acc[i + 3] == 1:
                body = bytes(self.acc[i + 2:i + 5])
                recv = int.from_bytes(self.acc[i + 5:i + 9], 'little')
                if crc32(body) == recv:
                    return i
        return -1

    def query_progress(self, timeout_ms: int = 2500, poll_ms: int = 5) -> int:
        """发进度查询，解析带 4 字节数据的应答帧；返回已写字节数，失败返回 -1。

        应答帧长度字段是 4（不是状态字节的 1），所以不能复用 find_ack。
        帧长 = 2(帧头) + 1(命令) + 1(长度) + 4(数据) + 4(CRC) = 12 字节。
        """
        want = CMD_PROGRESS | 0x80
        deadline = time.monotonic() + timeout_ms / 1000
        while time.monotonic() < deadline:
            try:
                self.pump()
            except LinkLost:
                return -1
            n = len(self.acc)
            for i in range(0, n - 11):
                if self.acc[i] == SYNC0 and self.acc[i + 1] == SYNC1 \
                        and self.acc[i + 2] == want and self.acc[i + 3] == 4:
                    body = bytes(self.acc[i + 2:i + 8])      # 命令+长度+4B 数据
                    recv = int.from_bytes(self.acc[i + 8:i + 12], 'little')
                    if crc32(body) == recv:
                        val = int.from_bytes(self.acc[i + 4:i + 8], 'little')
                        del self.acc[:i + 12]
                        return val
            time.sleep(poll_ms / 1000)
        return -1

    def wait_ack(self, cmd: int, timeout_ms: int, poll_ms: int = 5) -> int:
        """停等核心：拿到应答返回状态字节；超时返回 -1；串口掉线返回 -2。
        轮询间隔默认 5ms：Windows 下 in_waiting 每次都走 ClearCommError 系统调用，
        1ms 高频轮询会把廉价 USB 转串口驱动压出 PermissionError(13)。"""
        deadline = time.monotonic() + timeout_ms / 1000
        while time.monotonic() < deadline:
            try:
                self.pump()
            except LinkLost:
                return -2
            idx = self.find_ack(cmd)
            if idx >= 0:
                st = self.acc[idx + 4]
                del self.acc[:idx + 9]        # 消费掉这帧及帧前噪声
                return st
            time.sleep(poll_ms / 1000)
        return -1

    def wait_text(self, pattern: str, timeout_ms: int) -> bool:
        """等设备打印某段文字（比如结束后的 jump to app）"""
        deadline = time.monotonic() + timeout_ms / 1000
        while time.monotonic() < deadline:
            try:
                self.pump()
            except LinkLost:
                return False
            if pattern in self.text:
                return True
            time.sleep(0.005)
        return False

    def send(self, cmd: int, data: bytes = b''):
        try:
            self.ser.write(make_frame(cmd, data))
        except (serial.SerialException, OSError) as exc:
            raise LinkLost(str(exc))


def handshake(link: IapSerial, wait_s: float, quiet: bool = False) -> bool:
    """连发握手直到拿到状态 0。
    正常上电走 3 秒窗口、断电恢复走 upgrade_loop，两条路径收到握手都回 ACK，
    所以重连后不用分辨板子处于哪条路径，一律这么起步。"""
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        try:
            link.send(CMD_HANDSHAKE)
            st = link.wait_ack(CMD_HANDSHAKE, 60)
        except LinkLost:
            return False
        if st == ST_OK:
            return True
        if st == -2:
            return False
    return False


def wait_port_appear(port: str, timeout_s: float) -> bool:
    """轮询等 COM 口出现（USB 重新枚举要几秒）。timeout=0 表示只查一次。"""
    deadline = time.monotonic() + timeout_s
    while True:
        if any(p.device == port for p in list_ports.comports()):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description='STM32F407 串口 IAP 升级')
    ap.add_argument('bin', nargs='?', help='App 镜像 .bin 文件路径（--erase-only 时可省略）')
    ap.add_argument('-p', '--port', default='COM3', help='串口号（默认 COM3）')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--wait-port', type=float, default=180,
                    help='等 COM 口出现的秒数，0=不等（默认 180）')
    ap.add_argument('--wait-boot', type=float, default=150,
                    help='等板子复位/握手的秒数（默认 150）')
    ap.add_argument('--chunk', type=int, default=128, help='每帧数据字节数（默认 128）')
    ap.add_argument('--fw-ver', type=parse_fw_ver, default=(1, 4),
                    help='固件版本 主.次（默认 1.4）；随结束帧下发并写入 sector 3 元数据，'
                         'App 界面和上电日志都读它')
    ap.add_argument('--erase-only', action='store_true',
                    help='只擦除 App 区并清升级标志，不写数据（用于模拟出厂空片）')
    args = ap.parse_args()

    self_test()

    img = b''
    if args.bin:
        try:
            with open(args.bin, 'rb') as f:
                img = f.read()
        except OSError:
            print(f'FAIL: 打不开镜像文件 {args.bin}')
            return 1
        if len(img) < 8 or len(img) > (APP_LIMIT - APP_BASE):
            print(f'FAIL: 镜像大小 {len(img)} 字节不对（App 区最大 {APP_LIMIT - APP_BASE} 字节）')
            return 1
        msp = int.from_bytes(img[0:4], 'little')
        reset = int.from_bytes(img[4:8], 'little')
        if not (SRAM_BASE <= msp < SRAM_LIMIT) or (msp & 7) != 0:
            print(f'FAIL: 镜像向量表不像 App: MSP=0x{msp:08X}')
            return 1
        if not (APP_BASE <= reset < APP_LIMIT) or (reset & 1) != 1:
            print(f'FAIL: 镜像向量表不像 App: reset=0x{reset:08X}')
            return 1
        # 补齐到 4 字节：设备按字写 Flash，续传偏移 written 必须是 4 的倍数，
        # 否则断电时缓冲里那 1~3 个半字没法复原。CRC 也按补齐后的长度算。
        pad = (-len(img)) & 3
        if pad:
            img += b'\x00' * pad
        print(f'镜像 {len(img)} 字节' + (f'（含 {pad} 字节对齐填充）' if pad else '')
              + f', MSP=0x{msp:08X} reset=0x{reset:08X}')
    elif not args.erase_only:
        print('FAIL: 没给镜像文件（或者用 --erase-only 只擦除）')
        return 1
    else:
        print('擦除模式：不写数据，只清空 App 区')
    print(f'等待 {args.port} 出现（最长 {args.wait_port:.0f} 秒，可插拔 USB 或复位板子）...')
    if not wait_port_appear(args.port, args.wait_port):
        print(f'FAIL: {args.port} 一直没出现')
        return 1

    link = IapSerial(args.port, args.baud)
    print(f'{args.port} 已打开 {args.baud} 8N1，开始连发握手（每 60ms 一帧，'
          f'等 {args.wait_boot:.0f} 秒；没复位的板子现在按 RST）')

    # ── 握手：连发到拿到状态 0。正常窗口期与断电恢复(直接进升级模式)都吃这一套 ──
    if not handshake(link, args.wait_boot):
        print(f'FAIL: {args.wait_boot:.0f} 秒内握手一直没成功（板子没复位？串口不对？）')
        return 1
    print('握手成功，已进升级模式')

    # ── 先问设备写到哪了：断电续传靠这个决定要不要擦除 ──
    img_len = len(img) if args.bin else 0
    link.send(CMD_PROGRESS)
    resume = link.query_progress()
    if resume < 0:
        resume = 0
    if 0 < resume < img_len:
        print(f'检测到断点：设备已写 {resume}/{img_len} 字节，跳过擦除直接续传')
    else:
        resume = 0

        # ── 擦除 App 区（sector 4~7）。擦除最久要十几秒，给 45 秒 ──
        # 这一步可以放心重连重发：擦除是幂等的，多发一遍只是再擦一次，不会写坏任何东西
        erase = APP_BASE.to_bytes(4, 'little') + APP_LIMIT.to_bytes(4, 'little')
        st = None
        for attempt in range(1, 4):
            try:
                if st == -2:
                    print(f'串口掉线，第 {attempt} 次重连（等设备重新枚举，最长 30 秒）...')
                    if not link.reopen(30):
                        print('FAIL: 串口没能重新打开')
                        return 1
                    if not handshake(link, 15):
                        print('FAIL: 重连后握手不成功（板子可能已复位且不在升级模式）')
                        return 1
                    print('重连成功，重新发擦除')
                t0 = time.monotonic()
                link.send(CMD_ERASE, erase)
                st = link.wait_ack(CMD_ERASE, 45000, poll_ms=10)
            except LinkLost:
                st = -2
            if st != -2:
                break
        if st == -2:
            print('FAIL: 擦除阶段串口反复掉线，重连 3 次都失败')
            return 1
        if st == -1:
            print('FAIL: 擦除无应答（45 秒）')
            return 1
        if st == ST_RANGE:
            print('FAIL: 擦除被拒：地址越界或没按扇区对齐')
            return 1
        if st == ST_FLASH:
            print('FAIL: 擦除失败：Flash 报了错误标志')
            return 1
        print(f'擦除成功，耗时 {(time.monotonic() - t0) * 1000:.0f} ms '
              f'(0x{APP_BASE:08X}~0x{APP_LIMIT:08X})')

    # ── 只擦除模式：发结束命令清掉 24C02 的升级标志，设备自检 App 后会停在升级模式 ──
    # 不发结束命令的话标志还在，断电重上电会直接回升级模式，看不到闪灯
    if args.erase_only:
        link.send(CMD_END)
        st = link.wait_ack(CMD_END, 5000)
        print(f'结束命令状态={st}（同时清掉了升级标志）')
        if link.wait_text('app invalid', 3000):
            print('设备已确认 App 区无效，停在升级模式')
        print('下一步：给板子断电重上电，应看到 window 3s → window closed 后绿灯闪烁')
        return 0

    # ── 数据块：128B 一帧，停等。无应答只等不重发，再发握手做存活探测 ──
    off, frames = resume, 0
    t0 = time.monotonic()
    while off < len(img):
        part = img[off:off + args.chunk]
        try:
            link.send(CMD_DATA, part)
            st = link.wait_ack(CMD_DATA, 800)
            if st == -1:
                print(f'第 {frames} 帧(偏移 0x{off:X}) 800ms 无应答，再等 1200ms')
                st = link.wait_ack(CMD_DATA, 1200)
            if st == -1:
                link.send(CMD_HANDSHAKE)
                probe = link.wait_ack(CMD_HANDSHAKE, 500)
                print(f'FAIL: 第 {frames} 帧(偏移 0x{off:X}) 无应答，'
                      f'存活探测={probe}（-1 无响应 / 0 仍在升级模式，是丢帧）')
                return 1
        except LinkLost:
            print(f'FAIL: 第 {frames} 帧(偏移 0x{off:X}) 串口掉线。'
                  f'数据阶段不能自动重发（同一块写两遍会让后续全部错位）；'
                  f'给板子断电重上电，再重跑脚本即可（24C02 有标志会直接进升级模式）')
            return 1
        if st == -2:
            print(f'FAIL: 第 {frames} 帧(偏移 0x{off:X}) 串口掉线。'
                  f'数据阶段不能自动重发（同一块写两遍会让后续全部错位）；'
                  f'给板子断电重上电，再重跑脚本即可（24C02 有标志会直接进升级模式）')
            return 1
        if st == ST_RANGE:
            print(f'FAIL: 第 {frames} 帧(偏移 0x{off:X}) 被拒：越界')
            return 1
        if st == ST_FLASH:
            print(f'FAIL: 第 {frames} 帧(偏移 0x{off:X}) Flash 编程失败')
            return 1
        off += len(part)
        frames += 1
        if frames % 200 == 0:
            print(f'已写 {off}/{len(img)} 字节, {frames} 帧')
    spent = time.monotonic() - t0
    print(f'全部数据发完: {off} 字节 {frames} 帧, 耗时 {spent:.1f} s ({off / spent:.0f} 字节/秒)')

    # ── 结束：把整包 CRC 和固件版本一起送过去，设备自己算一遍比对 ──
    # zlib.crc32 与上面的 crc32() 是同一个算法（反射、初值 0xFFFFFFFF、末尾取反），
    # 但由 C 实现，300KB 算得飞快；逐位版留给几十字节的帧用。
    # 版本号由设备写进 sector 3 元数据：App 界面显示、上电日志打印都靠它。
    import zlib
    img_crc = zlib.crc32(img) & 0xFFFFFFFF
    ver_major, ver_minor = args.fw_ver
    print(f'整包 CRC32: 0x{img_crc:08X}, 固件版本: v{ver_major}.{ver_minor}')
    link.send(CMD_END, img_crc.to_bytes(4, 'little') + bytes([ver_major, ver_minor]))
    st = link.wait_ack(CMD_END, 6000)   # 设备要算 CRC + 擦写 sector3，留足时间
    if st != ST_OK:
        if st == ST_CRC:
            print('FAIL: 整包 CRC 不匹配——设备里写的内容和镜像不一致，请重跑升级')
        else:
            print(f'FAIL: 结束命令失败, 状态={st}')
        return 1
    print('结束命令已应答')

    if link.wait_text('jump to app', 5000):
        print('设备已跳转 App')
    else:
        print('（5 秒内没看到 jump to app，不影响结果，可看串口助手确认）')

    print('DONE')
    return 0


if __name__ == '__main__':
    sys.exit(main())
