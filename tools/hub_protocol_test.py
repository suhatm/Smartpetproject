#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环 Sensor Hub 协议上板测试（task-V1.09，协议 V0.5a）

覆盖：连接/MTU/信息读取、帧解析+CRC8 校验、全指令 ACK（含 REPROBE
带参/无参语义）、传感器帧流统计、录音->列表->下载(CRC32 校验)->删除、
SD_TEST、低电/事件帧观察。bleak + Windows 蓝牙。

用法：
    python tools/hub_protocol_test.py [--quick] [--no-rec]

依赖：托管 venv（C:\\Users\\pc\\.workbuddy\\binaries\\python\\envs\\default）
"""
import argparse
import asyncio
import struct
import sys
import time
import zlib
from collections import Counter

from bleak import BleakClient, BleakScanner

UUID_DATA = "e5a00021-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_CMD = "e5a00022-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_ACK = "e5a00023-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_INFO = "e5a00024-1e5c-4b8f-9a2d-6c0f7e8d9a0b"

TYPE_NAMES = {
    0x01: "IMU_U4", 0x02: "BODY_IMU_U1", 0x03: "QVAR", 0x04: "PVDF",
    0x05: "TEMP", 0x06: "MIC", 0x07: "AUDIO_FILE", 0x08: "BATTERY",
    0x10: "MODULE_STATUS", 0x20: "CMD_ACK", 0x21: "EVENT",
}
EVENT_NAMES = {1: "MOD_STATE", 2: "WDT_RESET", 3: "LOW_BATT",
               4: "QVAR_THR", 5: "REC_STATE", 6: "REC_FILE_DONE",
               7: "SD_TEST_DONE"}

results = []  # (name, ok, detail)


def report(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameParser:
    """按 SYNC 扫描对齐解析串帧流，CRC8 校验。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(b"\x55\xaa")
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            ln = self.buf[4]   # 布局：SYNC(2) TYPE SEQ LEN PAYLOAD CRC
            total = 6 + ln
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + ln])
            crc = self.buf[5 + ln]
            if crc8(body) == crc:
                frames.append((body[0], body[1], bytes(self.buf[5:5 + ln])))
            else:
                self.bad_crc += 1
            del self.buf[:total]
        return frames


class HubTest:
    def __init__(self):
        self.parser = FrameParser()
        self.ack_parser = FrameParser()
        self.data_counts = Counter()
        self.acks = []
        self.events = []
        self.file_chunks = []
        self.seq_gaps = Counter()

    def on_data(self, _, data: bytearray):
        last_seq = {}
        for ftype, seq, payload in self.parser.feed(bytes(data)):
            self.data_counts[ftype] += 1
            if ftype == 0x07:
                fid = struct.unpack("<H", payload[:2])[0]
                off = struct.unpack("<I", payload[2:6])[0]
                self.file_chunks.append((fid, off, payload[6:]))

    def on_ack(self, _, data: bytearray):
        for ftype, seq, payload in self.ack_parser.feed(bytes(data)):
            if ftype == 0x20:
                self.acks.append(payload)
            elif ftype == 0x21:
                self.events.append(payload)

    async def wait_ack(self, cmd, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for i, a in enumerate(self.acks):
                if a[0] == cmd:
                    return self.acks.pop(i)
            await asyncio.sleep(0.05)
        return None

    async def wait_event(self, eid, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for i, e in enumerate(self.events):
                if e[0] == eid:
                    return self.events.pop(i)
            await asyncio.sleep(0.1)
        return None

    async def run(self, quick=False, do_rec=True):
        print("== 扫描 SmartPet ...")
        # 名字在扫描响应里，Windows 被动扫描常拿不到；同时按服务 UUID 匹配
        dev = await BleakScanner.find_device_by_name("SmartPet", timeout=10)
        if not dev:
            dev = await BleakScanner.find_device_by_filter(
                lambda d, adv: "e5a00020-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
                in [u.lower() for u in (adv.service_uuids or [])],
                timeout=10)
        if not dev:
            report("扫描发现设备", False, "未找到 SmartPet")
            return
        report("扫描发现设备", True, dev.address)

        # Windows 会缓存旧固件的 GATT 表，强制走 uncached 服务发现
        async with BleakClient(dev, timeout=15,
                               winrt={"use_cached_services": False}) as cli:
            report("BLE 连接", cli.is_connected, f"MTU={cli.mtu_size}")

            info = await cli.read_gatt_char(UUID_INFO)
            info_s = info.decode(errors="replace")
            report("读固件信息 0x24", "v1.11" in info_s, info_s)

            await cli.start_notify(UUID_DATA, self.on_data)
            await cli.start_notify(UUID_ACK, self.on_ack)
            report("订阅 0x21/0x23", True)

            async def cmd(hexs):
                await cli.write_gatt_char(UUID_CMD, bytes.fromhex(hexs),
                                          response=True)

            # ---- 基础指令 ----
            await cmd("7F DE AD BE EF")
            a = await self.wait_ack(0x7F)
            report("FACTORY_PING", a is not None and a[2] == 0
                   and a[3:7] == b"\xde\xad\xbe\xef",
                   f"result={a[2] if a else '无应答'}")

            await cmd("11")
            a = await self.wait_ack(0x11)
            report("GET_VERSION", a is not None and a[2] == 0 and b"v1.11" in a[3:],
                   bytes(a[3:]).decode(errors="replace") if a else "无应答")

            await cmd("10")
            a = await self.wait_ack(0x10)
            report("GET_STATUS", a is not None and a[2] == 0)

            await cmd("0C")
            a = await self.wait_ack(0x0C)
            report("GET_BATTERY", a is not None and a[2] == 0)

            await cmd("01 01")
            a = await self.wait_ack(0x01)
            ok1 = a is not None and a[2] == 0
            await asyncio.sleep(0.3)
            await cmd("01 00")
            a = await self.wait_ack(0x01)
            report("LED_SET 开/关", ok1 and a is not None and a[2] == 0)

            await cmd("7E")  # 未知指令
            a = await self.wait_ack(0x7E)
            report("未知指令应答-12", a is not None and a[2] == (256 - 12),
                   f"result={a[2] - 256 if a and a[2] > 127 else (a[2] if a else '?')}")

            await cmd("02 02 00")  # PWR_SET ANALOG off
            a = await self.wait_ack(0x02)
            okp = a is not None and a[2] == 0
            await cmd("02 02 01")  # 恢复
            a = await self.wait_ack(0x02)
            report("PWR_SET ANALOG 关/开", okp and a is not None and a[2] == 0)

            await cmd("08 00 F4 01 10")  # QVAR_THR_SET ch=0 thr=500 hyst=16
            a = await self.wait_ack(0x08)
            report("QVAR_THR_SET", a is not None and a[2] == 0)

            # ---- REPROBE（task-V1.09：协议 §7.2 要求 sensor_id(1B)） ----
            await cmd("06")  # 无参：固件应回 result=-1 拒执行
            a = await self.wait_ack(0x06)
            report("REPROBE 无参应答-1", a is not None and a[2] == 255,
                   f"result={a[2] - 256 if a and a[2] > 127 else (a[2] if a else '?')}")
            await cmd("06 01")  # sensor_id=1 IMU_U1（柔性板六轴）
            a = await self.wait_ack(0x06)
            report("REPROBE(IMU_U1)", a is not None and a[2] in (0, 1),
                   f"result={a[2]}（0=在位 1=仍不在位）")
            await cmd("06 02")  # sensor_id=2 QVAR
            a = await self.wait_ack(0x06)
            report("REPROBE(QVAR)", a is not None and a[2] in (0, 1),
                   f"result={a[2]}")
            await cmd("06 04")  # sensor_id=4 TEMP
            a = await self.wait_ack(0x06)
            report("REPROBE(TEMP)", a is not None and a[2] in (0, 1),
                   f"result={a[2]}")

            # ---- 帧流统计 ----
            wait_s = 4 if quick else 8
            print(f"== 采集数据帧 {wait_s}s ...")
            await asyncio.sleep(wait_s)
            cnt = self.data_counts
            print("   帧计数:", {TYPE_NAMES.get(k, hex(k)): v for k, v in cnt.items()})
            report("IMU_U4 帧流(0x01)", cnt[0x01] > wait_s * 10,
                   f"{cnt[0x01]} 帧")
            report("MODULE_STATUS 帧(0x10)", cnt[0x10] >= max(1, wait_s // 2),
                   f"{cnt[0x10]} 帧")
            report("BATTERY 帧(0x08)", cnt[0x08] >= 1, f"{cnt[0x08]} 帧")
            report("帧 CRC8 校验", self.parser.bad_crc == 0,
                   f"bad={self.parser.bad_crc}")
            print(f"   其他帧: QVAR={cnt[0x03]} PVDF={cnt[0x04]} "
                  f"TEMP={cnt[0x05]} MIC={cnt[0x06]} U1={cnt[0x02]}")

            # ---- 录音链路 ----
            if do_rec:
                print("== 录音测试（3s 立体声）...")
                self.data_counts[0x06] = 0
                await cmd("07 01 03 03 00")  # REC_CTRL start ch=3 dur=3s
                a = await self.wait_ack(0x07)
                report("REC_CTRL 开始", a is not None and a[2] == 0,
                       f"result={a[2] if a else '无应答'}")
                ev = await self.wait_event(0x05, timeout=5)
                report("EVENT REC_STATE=1", ev is not None and ev[1] == 1)
                ev = await self.wait_event(0x05, timeout=8)
                fid = struct.unpack("<H", ev[2:4])[0] if ev else 0
                report("EVENT REC_STATE=2(完成)", ev is not None and ev[1] == 2,
                       f"file_id={fid}")
                report("录音期间 MIC RMS 帧", self.data_counts[0x06] >= 2,
                       f"{self.data_counts[0x06]} 帧")

                await cmd("09")
                a = await self.wait_ack(0x09)
                n_entries = (len(a) - 3) // 6 if a and a[2] == 0 else 0
                report("REC_LIST", a is not None and a[2] == 0 and n_entries >= 1,
                       f"{n_entries} 个文件")

                if fid:
                    print(f"== 下载 file_id={fid} ...")
                    self.file_chunks.clear()
                    await cmd(f"0A {fid & 0xFF:02X} {fid >> 8:02X}")
                    a = await self.wait_ack(0x0A)
                    report("REC_READ ACK", a is not None and a[2] == 0)
                    ev = await self.wait_event(0x06, timeout=60)
                    if ev:
                        total = struct.unpack("<I", ev[3:7])[0]
                        crc = struct.unpack("<I", ev[7:11])[0]
                        blob = b"".join(d for _, _, d in sorted(
                            self.file_chunks, key=lambda x: x[1]))
                        got = len(blob)
                        ok_len = got == total
                        ok_crc = (zlib.crc32(blob) & 0xFFFFFFFF) == crc
                        report("REC_READ 长度", ok_len, f"{got}/{total}B")
                        report("REC_READ CRC32", ok_crc,
                               f"calc={zlib.crc32(blob) & 0xFFFFFFFF:08X} "
                               f"exp={crc:08X}")
                        if blob[:4] == b"RIFF":
                            report("WAV 头", True, blob[:12])
                        else:
                            report("WAV 头", False, blob[:12].hex())
                        # 断点续传：offset=总长的 1/2 重下
                        off = total // 2
                        self.file_chunks.clear()
                        await cmd(f"0A {fid & 0xFF:02X} {fid >> 8:02X} "
                                  f"{off & 0xFF:02X} {(off >> 8) & 0xFF:02X} "
                                  f"{(off >> 16) & 0xFF:02X} {(off >> 24) & 0xFF:02X}")
                        a = await self.wait_ack(0x0A)
                        ev2 = await self.wait_event(0x06, timeout=60)
                        if ev2:
                            blob2 = b"".join(d for _, _, d in sorted(
                                self.file_chunks, key=lambda x: x[1]))
                            report("断点续传(offset)", len(blob2) == total - off
                                   and blob2 == blob[off:],
                                   f"{len(blob2)}/{total - off}B")
                        else:
                            report("断点续传(offset)", False, "无 REC_FILE_DONE")
                    else:
                        report("REC_READ 完成事件", False, "60s 超时")

                    await cmd(f"0B {fid & 0xFF:02X} {fid >> 8:02X}")
                    a = await self.wait_ack(0x0B)
                    report("REC_DELETE", a is not None and a[2] == 0)

                # ---- SD_TEST ----
                print("== SD_TEST（1MB 写读校验）...")
                await cmd("0D 01 01")
                a = await self.wait_ack(0x0D)
                report("SD_TEST ACK", a is not None and a[2] == 0)
                ev = await self.wait_event(0x07, timeout=60)
                if ev:
                    res = ev[1] - 256 if ev[1] > 127 else ev[1]
                    w = struct.unpack("<H", ev[2:4])[0]
                    r = struct.unpack("<H", ev[4:6])[0]
                    bad = struct.unpack("<H", ev[6:8])[0]
                    report("SD_TEST 结果", res == 0 and bad == 0,
                           f"result={res} w={w}KB/s r={r}KB/s bad={bad}")
                else:
                    report("SD_TEST 完成事件", False, "60s 超时")

        print("\n==== 汇总 ====")
        fails = [r for r in results if not r[1]]
        for name, ok, detail in results:
            print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")
        print(f"== {len(results) - len(fails)}/{len(results)} 通过"
              + (f"，失败: {[f[0] for f in fails]}" if fails else "，全部通过"))
        return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="帧流统计 4s")
    ap.add_argument("--no-rec", action="store_true", help="跳过录音/SD 测试")
    args = ap.parse_args()
    t = HubTest()
    try:
        rc = asyncio.run(t.run(quick=args.quick, do_rec=not args.no_rec))
    except KeyboardInterrupt:
        rc = 2
    sys.exit(rc or 0)


if __name__ == "__main__":
    main()
