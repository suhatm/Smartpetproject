#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SWD 直读 RTT 上行缓冲区并推进 RdOff，模拟 RTT host 持续取日志。
用法: python tools/rtt_tail.py [秒数] [CB地址(hex)] [BUF地址(hex)]
依赖: nrfutil 在 PATH 中。nRF54L 项目默认 CB aUp[0].WrOff @ CB+0x24。
"""
import re
import subprocess
import sys
import time

CB_WR_RD = 0x20001094   # aUp[0].WrOff(+0) / RdOff(+4)
BUF = 0x20000070
SIZE = 0x1000
DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 60
if len(sys.argv) > 2:
    CB_WR_RD = int(sys.argv[2], 16)
if len(sys.argv) > 3:
    BUF = int(sys.argv[3], 16)


def nrf(*args):
    r = subprocess.run(["nrfutil", "device", *args],
                       capture_output=True, text=True, timeout=30)
    return r.stdout


def read_words(addr, nbytes):
    out = nrf("read", "--address", hex(addr), "--bytes", str(nbytes),
              "--width", "8", "--direct")
    data = bytearray()
    for line in out.splitlines():
        m = re.match(r"^0x[0-9A-Fa-f]+:\s+(.+?)\s{2,}\|", line)
        if not m:
            continue
        data += bytes(int(x, 16) for x in m.group(1).split())
    return bytes(data)


def main():
    last = -1
    t0 = time.time()
    while time.time() - t0 < DUR:
        try:
            w = int.from_bytes(read_words(CB_WR_RD, 4), "little")
        except Exception:
            time.sleep(1)
            continue
        if last >= 0 and w != last:
            if w > last:
                chunk = read_words(BUF + last, w - last)
            else:
                chunk = read_words(BUF + last, SIZE - last) + read_words(BUF, w)
            sys.stdout.write(chunk.decode("ascii", "replace"))
            sys.stdout.flush()
            nrf("write", "--address", hex(CB_WR_RD + 4), "--value", hex(w),
                "--direct")
        last = w
        time.sleep(1)


if __name__ == "__main__":
    main()
