#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_ble_monitor.py —— 电脑端 BLE 六轴数据监视器（替代手机 App 看十六进制）

直接连 SmartPet，订阅 e5a00011 通知、向 e5a00012 写 01 启动采集，
把 [seq][cnt][N x 12B] 帧流解码成真实物理量打印/存 CSV。

帧格式（每帧 12 字节，小端 6 x int16）：
    ax, ay, az : 加速度，单位 mg   (1000 mg = 1 g)
    gx, gy, gz : 角速度，单位 dps*10

用法：
    python imu_ble_monitor.py --scan              # 先扫一下，确认能看见设备
    python imu_ble_monitor.py                     # 连接 -> 订阅 -> 启动 -> 实时显示
    python imu_ble_monitor.py --csv out.csv       # 同时存 CSV
    python imu_ble_monitor.py --frames            # 每帧一行（默认是一行原地刷新）
    python imu_ble_monitor.py --read-poll         # 不订阅通知，改为轮询 Read 特征值
    python imu_ble_monitor.py --mac AA:BB:CC:DD:EE:FF

依赖：pip install bleak   （Windows 需蓝牙适配器 + 蓝牙服务已启动）
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import struct
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("缺少依赖：pip install bleak")

DEVICE_NAME = "SmartPet"
# 广播里带的是 LED 服务的 128-bit UUID；Windows 拿不到 scan response 里的
# 设备名（name 常为空），所以扫描时【按 UUID 匹配】最可靠。
UUID_ADV = "e5a00001-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_DATA = "e5a00011-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_CTRL = "e5a00012-1e5c-4b8f-9a2d-6c0f7e8d9a0b"

FRAME = struct.Struct("<6h")  # ax ay az gx gy gz
FRAME_BYTES = FRAME.size


def adv_name(dev, adv) -> str:
    return adv.local_name or dev.name or ""


def is_target(dev, adv) -> bool:
    if adv_name(dev, adv) == DEVICE_NAME:
        return True
    return UUID_ADV in [str(u).lower() for u in (adv.service_uuids or [])]


def decode_packet(pkt: bytes | bytearray):
    """[seq][cnt][frame x N] -> 逐帧 yield (seq, idx, total, (ax,ay,az,gx,gy,gz))"""
    if len(pkt) < 2:
        return
    seq, cnt = pkt[0], pkt[1]
    body = bytes(pkt[2:])
    usable = min(cnt, len(body) // FRAME_BYTES)
    for i in range(usable):
        yield seq, i + 1, usable, FRAME.unpack_from(body, i * FRAME_BYTES)


class Monitor:
    def __init__(self, args):
        self.args = args
        self.frames = 0
        self.packets = 0
        self.lost_seq = 0
        self.last_seq = None
        self.t0 = time.time()
        self.csv_w = None
        self.csv_f = None
        self.latest = None
        self.last_draw = 0.0
        self.read_handle = None
        if args.csv:
            self.csv_f = open(args.csv, "w", newline="", encoding="utf-8")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_writer_header()

    def csv_writer_header(self):
        self.csv_w.writerow(["t_s", "seq", "idx", "n", "ax_mg", "ay_mg",
                             "az_mg", "gx_dps10", "gy_dps10", "gz_dps10"])

    # ---------- 输出 ----------

    def emit(self, seq, idx, n, v):
        ax, ay, az, gx, gy, gz = v
        self.frames += 1
        self.latest = (seq, idx, n, v)
        if self.csv_w:
            self.csv_w.writerow([f"{time.time() - self.t0:.3f}", seq, idx, n,
                                 ax, ay, az, gx, gy, gz])

        if self.args.frames:
            print(f"seq={seq:3d} {idx:2d}/{n:<2d} | "
                  f"a=({ax:6d},{ay:6d},{az:6d}) mg | "
                  f"g=({gx:6d},{gy:6d},{gz:6d}) dps*10", flush=True)
            return

        now = time.time()
        interval = 1.0 if self.args.quiet else 0.1   # quiet：1s 一行，适合写日志
        if now - self.last_draw < interval:
            return
        self.last_draw = now
        rate = self.frames / max(now - self.t0, 1e-6)
        line = (f"seq={seq:3d} {idx:2d}/{n:<2d} | "
                f"a=({ax:6d},{ay:6d},{az:6d}) mg | "
                f"g=({gx:6d},{gy:6d},{gz:6d}) dps*10 | "
                f"{self.frames} 帧 {rate:4.1f}/s 丢包={self.lost_seq}")
        if self.args.quiet:
            print(line, flush=True)
        else:
            sys.stdout.write("\r" + line.ljust(118))
            sys.stdout.flush()

    def on_notify(self, _handle, data):
        self.packets += 1
        seq = data[0] if data else None
        if seq is not None:
            if self.last_seq is not None:
                gap = (seq - self.last_seq) & 0xFF
                if gap > 1:
                    self.lost_seq += gap - 1
            self.last_seq = seq
        if self.args.raw:
            print(f"\n[{time.strftime('%H:%M:%S')}] raw({len(data)}B) "
                  f"{bytes(data).hex(' ')}", flush=True)
        for item in decode_packet(data):
            self.emit(*item)

    # ---------- 主流程 ----------

    async def find(self):
        """找到 SmartPet 就返回；--wait > 0 时反复扫描直到出现或超时"""
        if self.args.mac:
            return self.args.mac
        deadline = time.time() + max(self.args.wait, self.args.scan_timeout)
        first = True
        while True:
            print(f"扫描 BLE 设备（找 \"{DEVICE_NAME}\"）...", flush=True)
            found = await BleakScanner.discover(timeout=self.args.scan_timeout,
                                                return_adv=True)
            for dev, adv in sorted(found.values(), key=lambda x: -x[1].rssi):
                name = adv_name(dev, adv)
                hit = is_target(dev, adv)
                if first:
                    mark = "  <== 目标（按服务 UUID 命中）" if hit else ""
                    print(f"  {dev.address}  RSSI={adv.rssi:4d}  "
                          f"name='{name}'{mark}")
                if hit:
                    print(f"找到目标：{dev.address}  RSSI={adv.rssi}  "
                          f"name='{name}'", flush=True)
                    return dev
            first = False
            if time.time() >= deadline:
                return None
            print("  未发现目标，3s 后重试（手机若还连着，请先断开或关掉手机蓝牙）",
                  flush=True)
            await asyncio.sleep(3)

    async def run(self):
        target = await self.find()
        if target is None:
            print(f"\n没找到 \"{DEVICE_NAME}\"。可能原因：")
            print("  1) 手机 nRF Connect 还连着（广播已停）—— 先在手机上 Disconnect，"
                  "或关掉手机蓝牙")
            print("  2) 板子没在广播 —— 复位一下板子")
            return 1

        addr = target if isinstance(target, str) else target.address
        attempts = max(1, self.args.retry)
        for i in range(1, attempts + 1):
            print(f"\n连接 {addr} ...（第 {i}/{attempts} 次）")
            try:
                await self.session(addr)
                return 0
            except Exception as exc:                       # noqa: BLE001
                print(f"!! 会话中断：{exc!r}", flush=True)
                if i < attempts:
                    print("   2s 后重连（RSSI 偏低时链路会被监督超时断开，"
                          "属无线环境问题，非固件问题）", flush=True)
                    await asyncio.sleep(2)
        print("\n多次重试仍失败。可尝试：把板子挪近电脑 / 远离 USB3 接口，"
              "或直接用手机 App。")
        return 1

    async def session(self, addr):
        async with BleakClient(addr, timeout=20.0) as client:
            print(f"已连接。MTU={client.mtu_size}")
            svcs = client.services
            if not any(s.uuid.lower() == UUID_DATA for s in svcs):
                print("!! 没发现 e5a00011 服务，GATT 数据库可能被系统缓存了")
                print("   可尝试：Windows 设置里删除该设备后重试")

            if self.args.no_ctrl:
                print("跳过写控制点（验证固件 APP_IMU_AUTOSTART 自动开流）")
            else:
                try:
                    await client.write_gatt_char(UUID_CTRL, b"\x01",
                                                 response=True)
                    print("已向 e5a00012 写 01（启动采集）")
                except Exception as exc:                   # noqa: BLE001
                    print(f"!! 写控制点失败：{exc!r}")

            if self.args.read_poll:
                await self.poll_loop(client)
            else:
                await client.start_notify(UUID_DATA, self.on_notify)
                print("已订阅 e5a00011 通知。Ctrl+C 停止。\n")
                deadline = (time.time() + self.args.duration
                            if self.args.duration > 0 else None)
                try:
                    while client.is_connected:
                        if deadline is not None and time.time() >= deadline:
                            print(f"\n达到 --duration {self.args.duration}s，停止采集")
                            break
                        await asyncio.sleep(0.2)
                except asyncio.CancelledError:
                    pass
                finally:
                    try:
                        await client.write_gatt_char(UUID_CTRL, b"\x00",
                                                     response=True)
                        print("\n已向 e5a00012 写 00（停止采集）")
                    except Exception:                      # noqa: BLE001
                        pass

    async def poll_loop(self, client):
        """兜底：不订阅通知，直接周期读数据特征值（需要固件 read 返回帧）"""
        print("轮询模式：周期读 e5a00011。Ctrl+C 停止。\n")
        try:
            while client.is_connected:
                data = await client.read_gatt_char(UUID_DATA)
                if len(data) == FRAME_BYTES:
                    self.emit(0, 1, 1, FRAME.unpack(data))
                else:
                    if self.args.raw:
                        print(f"\nread({len(data)}B) {bytes(data).hex(' ')}")
                await asyncio.sleep(1.0 / self.args.poll_hz)
        except asyncio.CancelledError:
            pass

    def close(self):
        if self.args.frames or self.args.raw:
            print()
        print(f"\n统计：{self.packets} 包 / {self.frames} 帧，"
              f"耗时 {time.time() - self.t0:.1f}s，序号丢包 {self.lost_seq}")
        if self.args.csv:
            self.csv_f.close()
            print(f"CSV 已写入 {self.args.csv}")


async def scan_only(timeout: float):
    print(f"扫描 {timeout}s ...")
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    if not found:
        print("  没发现任何 BLE 设备——检查适配器/蓝牙服务")
        return 1
    for dev, adv in sorted(found.values(), key=lambda x: -x[1].rssi):
        name = adv_name(dev, adv)
        mark = "  <== SmartPet" if is_target(dev, adv) else ""
        print(f"  {dev.address}  RSSI={adv.rssi:4d}  name='{name}'{mark}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="SmartPet 六轴数据 BLE 监视器",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true", help="只扫描设备后退出")
    ap.add_argument("--mac", help="直接连指定地址，跳过扫描")
    ap.add_argument("--csv", help="把每帧写入 CSV")
    ap.add_argument("--frames", action="store_true", help="每帧一行（默认原地刷新）")
    ap.add_argument("--raw", action="store_true", help="额外打印原始十六进制包")
    ap.add_argument("--read-poll", action="store_true",
                    help="不订阅通知，轮询读数据特征值")
    ap.add_argument("--poll-hz", type=float, default=5.0, help="轮询频率")
    ap.add_argument("--scan-timeout", type=float, default=6.0)
    ap.add_argument("--wait", type=float, default=0.0,
                    help="设备没出现时反复扫描的总时长（秒），0=只扫一次")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="采集指定秒数后自动停止（0=一直跑到 Ctrl+C）")
    ap.add_argument("--quiet", action="store_true",
                    help="每秒一行（适合重定向到日志文件）")
    ap.add_argument("--no-ctrl", action="store_true",
                    help="不写控制点（验证固件 APP_IMU_AUTOSTART 自动开流）")
    ap.add_argument("--retry", type=int, default=3,
                    help="会话中途掉线的重连次数（默认 3）")
    args = ap.parse_args()

    if args.scan:
        return asyncio.run(scan_only(args.scan_timeout))

    mon = Monitor(args)
    try:
        return asyncio.run(mon.run())
    except KeyboardInterrupt:
        return 0
    finally:
        mon.close()


if __name__ == "__main__":
    sys.exit(main())
