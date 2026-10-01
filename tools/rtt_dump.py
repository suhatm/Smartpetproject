#!/usr/bin/env python3
"""SWD 直读 SEGGER RTT 日志（nRF54L / nRF52，走 nrfutil 通道）

为什么需要它
------------
本机 J-Link 是 OB 克隆探针，装了 Segger 软件后 JLink.exe / RTT Viewer /
GDB Server 一连接就强制升级固件并失败。而 nrfutil 通道不检查探针固件版本，
所以用 `nrfutil device read --direct` 直接读 RAM 里的 RTT 控制块和上行缓冲区，
把 printk 日志重建出来，是最可靠的看日志方式。

用法
----
    # 从 map 文件自动定位 _SEGGER_RTT
    python tools/rtt_dump.py --map build/Smartpetproject/zephyr/zephyr.map

    # 或手工给地址
    python tools/rtt_dump.py --cb 0x20001070

    # 只看最后 30 行
    python tools/rtt_dump.py --map ... --tail 30

烧录新固件后的必要步骤（否则日志会静默！）
------------------------------------------
    nrfutil device program --firmware <hex>
    nrfutil device write --address <CB地址> --value 0 --direct   # 清控制块
    nrfutil device reset
    python tools/rtt_dump.py --map ...

原因：RTT 控制块位于 NOLOAD 段，跨复位存活。SEGGER RTT 的 INIT() 是惰性的，
看到 acID[0] == 'S' 就认为已初始化而跳过；上一版固件遗留的 WrOff 卡在缓冲区
末端时，新固件所有 printk 会被 NO_BLOCK_SKIP 模式静默丢弃，极易误判"固件没跑"。
清掉控制块首字再复位即可。

控制块布局（实测）
------------------
    struct SEGGER_RTT_CB {
        char     acID[16];      // "SEGGER RTT\0\0\0\0\0\0"
        unsigned MaxNumUpBuffers;   // @+0x10
        unsigned MaxNumDownBuffers; // @+0x14
        struct SEGGER_RTT_BUFFER_UP aUp[N];  // @+0x18
    };
    aUp[0] = { const char *sName;   // @+0x00  -> flash 里的 "Terminal" 字符串
               unsigned    pBuffer; // @+0x04  -> 上行缓冲区首地址
               unsigned    SizeOfBuffer;
               unsigned    WrOff;
               unsigned    RdOff; }

注意：缓冲区所在 RAM 不会被编程擦除，WrOff 之后可能残留上一版固件的旧日志，
只信任 [0, WrOff) 区间。
"""

import argparse
import re
import struct
import subprocess
import sys

CHUNK = 96  # nrfutil 单次读 128B 有时返回不完整；96B 稳定且不慢


def raw_read(addr, n, family="nrf54l", tries=4):
    """读 n 字节；校验实际长度，不足则重试（nrfutil 偶发短读/超时）。"""
    best = b""
    for _ in range(tries):
        proc = subprocess.run(
            ["nrfutil", "device", "read", "--address", hex(addr),
             "--bytes", str(n), "--direct", "--family", family],
            capture_output=True, text=True, timeout=90)
        data = bytearray()
        for line in proc.stdout.splitlines():
            m = re.match(r"^(0x[0-9A-Fa-f]+):\s+((?:[0-9A-Fa-f]{8}\s*)+)", line)
            if not m:
                continue
            for word in re.findall(r"[0-9A-Fa-f]{8}", m.group(2)):
                data += struct.pack("<I", int(word, 16))
        if len(data) >= n:
            return bytes(data[:n])
        if len(data) > len(best):
            best = bytes(data)
    return best


def find_cb_from_map(map_path):
    """从 zephyr.map 里取 _SEGGER_RTT 符号地址。"""
    pat = re.compile(r"^\s*0x0*([0-9a-fA-F]{8})\s+_SEGGER_RTT\s*$")
    with open(map_path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = pat.match(line)
            if m:
                return int(m.group(1), 16)
    return None


def main():
    ap = argparse.ArgumentParser(description="SWD read SEGGER RTT log")
    ap.add_argument("--map", help="zephyr.map 路径，用于自动定位 _SEGGER_RTT")
    ap.add_argument("--cb", help="RTT 控制块地址（如 0x20001070），与 --map 二选一")
    ap.add_argument("--family", default="nrf54l", help="nrfutil device family，默认 nrf54l")
    ap.add_argument("--tail", type=int, default=0, help="只输出最后 N 行")
    ap.add_argument("--raw", action="store_true", help="去掉控制字符后原样输出")
    args = ap.parse_args()

    if args.cb:
        cb_addr = int(args.cb, 16)
    elif args.map:
        cb_addr = find_cb_from_map(args.map)
        if cb_addr is None:
            sys.exit(f"在 {args.map} 中找不到 _SEGGER_RTT 符号")
    else:
        sys.exit("需要 --map 或 --cb 指定 RTT 控制块地址")

    cb = raw_read(cb_addr, 0x2C, args.family)
    if cb[:10] != b"SEGGER RTT":
        sys.exit(f"0x{cb_addr:08X} 处不是 RTT 控制块（读到 {cb[:16]!r}）："
                 "固件没跑 / 地址不对 / 控制块被复位")

    sname, pbuf, bsize, wroff, rdoff = struct.unpack_from("<IIIII", cb, 0x18)
    if wroff == 0 or wroff > bsize:
        sys.exit(f"WrOff 异常（{wroff}/{bsize}）：缓冲区无有效日志"
                 "（烧录后忘了清控制块？）")
    print(f"# CB=0x{cb_addr:08X} pBuffer=0x{pbuf:08X} size={bsize} "
          f"WrOff={wroff} RdOff={rdoff} sName=0x{sname:08X}", file=sys.stderr)

    log = b""
    addr = pbuf
    while len(log) < wroff:
        want = min(CHUNK, wroff - len(log))
        # nrfutil 对非 4 对齐的 --bytes 有时直接返回空，向上取整到 4 再截断
        want = (want + 3) & ~3
        chunk = raw_read(addr, want, args.family, tries=4)
        if not chunk:
            print(f"# 读 0x{addr:08X} 失败，日志可能不完整", file=sys.stderr)
            break
        log += chunk
        addr += len(chunk)
    log = log[:wroff]

    if args.raw:
        sys.stdout.write(log.decode("utf-8", errors="replace"))
        return

    text = "".join(chr(b) if 32 <= b < 127 or b in (10, 13, 9) else "."
                   for b in log)
    lines = [ln for ln in text.splitlines() if ln.strip(".").strip()]
    if args.tail:
        lines = lines[-args.tail:]
    print("\n".join(lines))


if __name__ == "__main__":
    main()
