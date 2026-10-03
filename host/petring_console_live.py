#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环传感器控制  v1.08 正式版（真实 BLE 连接，tkinter）

版本历史：
  v1.06  首版正式版（真实 BLE，协议 V0.5）
  v1.07  高帧率数据流卡死修复：UI 刷新节流（帧监视批量刷、波形标脏重绘、
         状态灯/下载进度节流）+ 波形画布增量重绘（静态框架一次绘制，
         动态曲线 tag 局部刷新）
  v1.08  配合固件 task-V1.07：意外断线自动重连（10 次 × 3s，覆盖固件
         WDT 复位窗口）；录音启动 6s 无 ACK 超时兜底（固件失败静默
         曾致按钮永久禁用）；录音中显示双麦克风实时电平；断线时复位
         SD 测试/下载挂起状态

与 host/petring_console_demo.py（模拟数据 DEMO）共用同一套界面：
继承 demo 的 PetRingConsole，仅把 SimLink 换成真实 BleWorker
（bleak + asyncio 后台线程），所有控件行为改为真实协议交互
（协议 V0.5：Sensor Hub e5a00020，帧 55 AA + TYPE + SEQ + LEN +
PAYLOAD + CRC8(0x07)）。

运行依赖：系统 Python 3.13（tkinter）+ bleak（pip install --user bleak）
启动：host/run_live.bat
"""
import asyncio
import math
import os
import queue
import struct
import threading
import time
import tkinter as tk
import zlib
from tkinter import ttk

from bleak import BleakClient, BleakScanner

from petring_console_demo import PetRingConsole, SimLink  # 复用界面与控件

# ---------------------------------------------------------------- 版本信息
APP_VERSION = "v1.08"           # 上位机版本
APP_BUILD = "2026-10-03"        # 构建日期
APP_PROTOCOL = "V0.5a"          # 适配的通信协议版本

RECONNECT_MAX = 10              # 意外断线自动重连次数上限
RECONNECT_DELAY_MS = 3000       # 每次重连间隔（覆盖固件 WDT 复位窗口）
REC_ACK_TIMEOUT = 6.0           # 录音启动 ACK 超时（s）

# ---------------------------------------------------------------- 协议常量
# （与 tools/hub_protocol_test.py、固件 ble_sensor_hub.h 保持一致）

UUID_DATA = "e5a00021-1e5c-4b8f-9a2d-6c0f7e8d9a0b"   # 数据流 Notify
UUID_CMD = "e5a00022-1e5c-4b8f-9a2d-6c0f7e8d9a0b"    # 指令 Write
UUID_ACK = "e5a00023-1e5c-4b8f-9a2d-6c0f7e8d9a0b"    # 应答/事件 Notify
UUID_INFO = "e5a00024-1e5c-4b8f-9a2d-6c0f7e8d9a0b"   # 信息 Read

TYPE_NAMES = {
    0x01: "IMU_U4", 0x02: "BODY_IMU_U1", 0x03: "QVAR", 0x04: "PVDF",
    0x05: "TEMP", 0x06: "MIC", 0x07: "AUDIO_FILE", 0x08: "BATTERY",
    0x10: "MODULE_STATUS", 0x20: "CMD_ACK", 0x21: "EVENT",
}
EVENT_NAMES = {1: "MOD_STATE", 2: "WDT_RESET", 3: "LOW_BATT",
               4: "QVAR_THR", 5: "REC_STATE", 6: "REC_FILE_DONE",
               7: "SD_TEST_DONE"}
CMD_NAMES = {
    0x01: "LED_SET", 0x02: "PWR_SET", 0x03: "SENSOR_EN", 0x04: "RATE_SET",
    0x05: "QVAR_CFG", 0x06: "REPROBE", 0x07: "REC_CTRL",
    0x08: "QVAR_THR_SET", 0x09: "REC_LIST", 0x0A: "REC_READ",
    0x0B: "REC_DELETE", 0x0C: "GET_BATTERY", 0x0D: "SD_TEST",
    0x10: "GET_STATUS", 0x11: "GET_VERSION", 0x7F: "FACTORY_PING",
}
MOD_KEYS = ("IMU_U4", "IMU_U1", "QVAR_A", "QVAR_B",
            "PVDF", "TEMP", "MIC", "SD")
SD_TEST_ERR = {-1: "无卡/初始化失败", -2: "挂载失败", -3: "写失败",
               -4: "读失败", -5: "校验不一致"}

# 原始码换算（与固件 overlay 一致：accel FS_8G、gyro FS_2000DPS）
ACC_MG_PER_LSB = 8000.0 / 32768.0          # ±8g -> mg
GYRO_DDPS_PER_LSB = 20000.0 / 32768.0      # ±2000dps -> 0.1dps（demo 显示单位）

DOWNLOAD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "downloads")


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 \
                else (crc << 1) & 0xFF
    return crc


class FrameParser:
    """SYNC 扫描对齐 + CRC8 校验的串帧解析器。"""

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


# ---------------------------------------------------------------- BLE 链路

class LiveLink(SimLink):
    """与 SimLink 属性兼容，但数据全部由真实 BLE 帧驱动（不做任何模拟）。"""

    def __init__(self):
        super().__init__()
        # 模块状态未知，等待 0x10 MODULE_STATUS / EVENT 0x01 更新
        self.mod_state = {k: 0 for k in MOD_KEYS}
        self.pwr = {"SENS": False, "STORE": False, "ANALOG": False}
        self.has_batt = False
        self.last_batt = (0, 0, False)      # (mv, pct, charging)
        self.last_mic = (0, 0)              # (rms_l, rms_r) 0x06 帧

    def battery_sample(self, dt_s):          # noqa: ARG002 - 兼容旧调用
        return self.last_batt


class BleWorker(threading.Thread):
    """独立线程跑 asyncio/bleak；GUI 经队列下发任务，回调经 gui.after 回主线程。"""

    def __init__(self, gui):
        super().__init__(daemon=True, name="BleWorker")
        self.gui = gui
        self.jobs = queue.Queue()
        self.client = None
        self.parser_data = FrameParser()
        self.parser_ack = FrameParser()
        self._stop = False

    # ---- GUI 侧调用（线程安全）----
    def submit(self, coro_name, *args):
        self.jobs.put((coro_name, args))

    def stop(self):
        self.jobs.put(("disconnect", ()))
        self._stop = True

    def _cb(self, fn, *args):
        self.gui.after(0, lambda: fn(*args))

    # ---- 线程主循环 ----
    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._main())
        loop.close()

    async def _main(self):
        while True:
            try:
                name, args = self.jobs.get_nowait()
            except queue.Empty:
                if self._stop and self.client is None:
                    return
                await asyncio.sleep(0.02)
                continue
            try:
                await getattr(self, "_do_" + name)(*args)
            except Exception as e:  # noqa: BLE001
                self._cb(self.gui._on_worker_error, name, str(e))
            if self._stop and self.client is None:
                return

    # ---- 任务 ----
    async def _do_scan(self):
        try:
            devs = await BleakScanner.discover(timeout=5.0,
                                               return_adv=True)
        except Exception as e:  # noqa: BLE001
            self._cb(self.gui._on_scan_done, [], f"扫描失败：{e}")
            return
        found = []
        for addr, (dev, adv) in devs.items():
            name = dev.name or ""
            if "SmartPet" in name or "e5a00020" in "".join(
                    str(u) for u in (adv.service_uuids or [])):
                found.append((name or "SmartPet", addr, adv.rssi))
        self._cb(self.gui._on_scan_done, found, "")

    async def _do_connect(self, addr):
        self._cb(self.gui._on_conn_state, "connecting", 0, b"")
        client = BleakClient(
            addr, disconnected_callback=lambda c: self._cb(
                self.gui._on_conn_state, "dropped", 0, b""),
            timeout=20.0,
            # Windows 会缓存旧固件的 GATT 表，强制重新做服务发现
            winrt={"use_cached_services": False})
        try:
            await client.connect()
            self.client = client
            await client.start_notify(UUID_DATA, self._notify_data)
            await client.start_notify(UUID_ACK, self._notify_ack)
            try:
                info = bytes(await client.read_gatt_char(UUID_INFO))
            except Exception:  # noqa: BLE001
                info = b""
            self._cb(self.gui._on_conn_state, "connected",
                     client.mtu_size, info)
        except Exception as e:  # noqa: BLE001
            try:
                await client.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self.client = None
            self._cb(self.gui._on_conn_state, "failed", 0,
                     str(e).encode("utf-8", "replace"))

    async def _do_disconnect(self):
        if self.client is not None:
            try:
                await self.client.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self.client = None
        self._cb(self.gui._on_conn_state, "disconnected", 0, b"")

    async def _do_write(self, data: bytes):
        if self.client is None or not self.client.is_connected:
            self._cb(self.gui._on_worker_error, "write", "未连接")
            return
        await self.client.write_gatt_char(UUID_CMD, data, response=True)

    # ---- Notify 回调（worker 线程上下文，仅解析后抛回 GUI）----
    def _notify_data(self, _char, data: bytearray):
        frames = self.parser_data.feed(bytes(data))
        self._cb(self.gui._rx_stream, frames)

    def _notify_ack(self, _char, data: bytearray):
        frames = self.parser_ack.feed(bytes(data))
        self._cb(self.gui._rx_eventch, frames)


# ---------------------------------------------------------------- 正式版主窗口

class PetRingLive(PetRingConsole):
    """继承 demo 全部界面，替换数据链路为真实 BLE。"""

    def __init__(self):
        # 注意：demo 基类 __init__ 末尾会直接调 _tick() -> _flush_ui()，
        # 这些缓冲必须在 super().__init__() 之前就存在，否则 tkinter
        # 的属性回退（Widget.__getattr__ -> self.tk）会抛 AttributeError
        self._rx_buf = []                   # 帧监视行缓存（_tick 批量刷）
        self._rd_marks = []                 # 待重绘波形（_tick 批量刷）
        self._rd_marked = set()
        super().__init__()
        self.title(f"宠物环传感器控制  {APP_VERSION} 正式版"
                   f"（真实 BLE · 协议 {APP_PROTOCOL}）")
        self.link = LiveLink()
        self.devices = {}                   # 显示名 -> (addr, rssi)
        self.xfer = {}                      # file_id -> {"buf":bytearray}
        self.xfer_target = None             # 正在下载的 file_id
        self.sd_testing = False
        self._rec_pending = False
        self.worker = BleWorker(self)
        self.worker.start()
        self._rewire_demo_only()
        self._resync_static()
        # ---- 自动重连 / 挂起状态 ----
        self._reconnect_addr = None     # 意外断线后重连目标
        self._reconnect_attempts = 0
        self._user_disconnect = False   # 手动断开则不自动重连
        self._closing = False
        self._rec_pending_since = 0.0
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        os.makedirs(DOWNLOAD_DIR, exist_ok=True)
        self.log(f"正式版 {APP_VERSION}（构建 {APP_BUILD}，协议 {APP_PROTOCOL}）："
                 f" bleak 真实连接。先“扫描”，选中设备后“连接”。")

    def _resync_static(self):
        """用 LiveLink 初始状态重刷一遍静态控件（构造期画的是 SimLink 默认值）。"""
        self._refresh_mod_lamps()
        for dom, on in self.link.pwr.items():
            self.pwr_lamps[dom].configure(
                fg="#2ecc71" if on else "#3a4148")
        for lamp, _color in self.led_lamps:
            lamp.configure(fg="#3a4148")
        for ch in ("A", "B"):
            self.qvar_vals[ch].configure(text="—")
        for key in ("U4", "U1"):
            for lbl in self.imu_vals[key]:
                lbl.configure(text="—")
        for nm in ("heart_mv", "raw_mv", "ref_mv"):
            self.pvdf_vals[nm].configure(text="—")
        self.sd_lamp.configure(fg="#5d6d7e")
        self.lbl_batt.configure(text="—", foreground=self.FG)

    # ---------------- demo 专用控件改接线 ----------------
    def _find_btn(self, text):
        for w in self._walk(self):
            if isinstance(w, (ttk.Button, tk.Button)) and \
                    w.cget("text") == text:
                return w
        return None

    def _walk(self, root):
        yield root
        for c in root.winfo_children():
            yield from self._walk(c)

    def _rewire_demo_only(self):
        # “模拟插/拔充电器”：真实版无此概念，隐藏
        self.btn_chg.pack_forget()
        # “模拟插/拔柔性板 FPC”：改为 REPROBE 指令
        self.btn_fpc.configure(text="重新探测外设（CMD 0x06）",
                               command=self.on_reprobe)
        # “模拟插/拔 SD 卡”：改为刷新状态
        b = self._find_btn("模拟插/拔 SD 卡")
        if b is not None:
            b.configure(text="刷新 SD 状态", command=self.on_sd_refresh)
        # “注入校验错误（演示）”：隐藏
        for w in self._walk(self):
            if isinstance(w, ttk.Checkbutton) and \
                    "注入校验错误" in str(w.cget("text")):
                w.pack_forget()
        # SD 测试框标题：V0.5 已转正
        for w in self._walk(self):
            if isinstance(w, ttk.LabelFrame) and \
                    "待协议评审" in str(w.cget("text")):
                w.configure(text=" 读写测试（CMD 0x0D SD_TEST，协议 V0.5） ")

    # ---------------- 连接管理 ----------------
    def on_scan(self):
        # 手动扫描 = 接管连接管理，取消进行中的自动重连
        self._user_disconnect = True
        self._reconnect_addr = None
        self._reconnect_attempts = 0
        self.btn_scan.configure(state="disabled")
        self.log("扫描中（5s）……")
        self.worker.submit("scan")

    def _on_scan_done(self, found, err):
        self.btn_scan.configure(state="normal")
        if err:
            self.log(err)
            return
        self.devices.clear()
        values = []
        for name, addr, rssi in found:
            label = f"{name} ({addr[-5:]})"
            self.devices[label] = (addr, rssi)
            values.append(label)
        combo = self.dev_var
        # 更新下拉列表
        for w in self._walk(self):
            if isinstance(w, ttk.Combobox):
                w.configure(values=values, state="readonly")
        if values:
            combo.set(values[0])
            self.log(f"扫描到 {len(values)} 台设备：" +
                     "；".join(f"{n} RSSI={r}dBm"
                               for n, (_, r) in
                               ((v, self.devices[v]) for v in values)))
        else:
            self.log("未发现 SmartPet 设备（确认固件已烧录且未被他机连接）。")

    def on_connect(self):
        if self.link.connected:
            self._user_disconnect = True   # 手动断开，不触发自动重连
            self._reconnect_addr = None
            self.btn_conn.configure(state="disabled")
            self.worker.submit("disconnect")
            return
        label = self.dev_var.get()
        if label not in self.devices:
            self.log("请先“扫描”选择设备。")
            return
        self._user_disconnect = False
        self._reconnect_addr = self.devices[label][0]
        self._reconnect_attempts = 0
        self.btn_conn.configure(state="disabled")
        self.worker.submit("connect", self._reconnect_addr)

    def _schedule_reconnect(self):
        """意外断线后的自动重连：固定间隔重试，覆盖固件 WDT 复位窗口。"""
        if self._closing or self._user_disconnect:
            return
        if self._reconnect_addr is None:
            return
        if self._reconnect_attempts >= RECONNECT_MAX:
            self.log("自动重连失败：请确认设备在范围内后重新扫描连接。")
            self.lbl_conn.configure(text="● 未连接", fg="#e74c3c")
            return
        self._reconnect_attempts += 1
        self.lbl_conn.configure(
            text=f"● 重连中（{self._reconnect_attempts}/{RECONNECT_MAX}）…",
            fg="#f1c40f")
        self.after(RECONNECT_DELAY_MS, lambda: self.worker.submit(
            "connect", self._reconnect_addr))

    def _on_conn_state(self, state, mtu, info):
        self.btn_conn.configure(state="normal")
        if state == "connecting":
            if self._reconnect_attempts == 0:
                self.lbl_conn.configure(text="● 连接中…", fg="#f1c40f")
            return
        if state == "connected":
            was_reconnect = self._reconnect_attempts > 0
            self.link.connected = True
            self._reconnect_attempts = 0
            self._user_disconnect = False
            self.btn_conn.configure(text="断开")
            self.lbl_mtu.configure(text=f"MTU: {mtu}")
            label = self.dev_var.get()
            rssi = self.devices.get(label, ("", 0))[1]
            self.lbl_rssi.configure(text=f"RSSI: {rssi} dBm")
            self.lbl_conn.configure(
                text="● 已连接（自动重连成功）" if was_reconnect
                     else "● 已连接", fg="#2ecc71")
            ver = info.decode("utf-8", "replace") if info else ""
            self.log(f"已连接，MTU={mtu}，已订阅 0x21/0x23。"
                     + (f"固件信息：{ver}" if ver else ""))
            # 连接即取版本/状态/电量
            self._send_cmd(0x11)            # GET_VERSION
            self._send_cmd(0x10)            # GET_STATUS -> 0x10 帧
            self._send_cmd(0x0C)            # GET_BATTERY -> 0x08 帧
            return
        # failed / dropped / disconnected
        was = self.link.connected
        self.link.connected = False
        self.btn_conn.configure(text="连接")
        self.lbl_mtu.configure(text="MTU: —")
        self.lbl_rssi.configure(text="RSSI: —")
        self.link.recording = False
        self._rec_pending = False
        self.xfer_target = None
        # 挂起状态复位（断线时 SD 测试/下载不再等待）
        if self.sd_testing:
            self.sd_testing = False
            self.sd_prog.stop()
            self.sd_prog.configure(mode="determinate", value=0)
            self.btn_sd_test.configure(state="normal")
            self.lbl_sd_status.configure(text="连接断开，测试中止")
        self.btn_rec_start.configure(state="normal")
        self.btn_rec_stop.configure(state="disabled")
        if state == "dropped":
            self.log("连接意外断开，尝试自动重连……")
            self.term_print("<< LINK DOWN")
            self.lbl_conn.configure(
                text=f"● 重连中（0/{RECONNECT_MAX}）…", fg="#f1c40f")
            self._schedule_reconnect()
            return
        if state == "failed":
            if self._reconnect_attempts > 0:
                # 自动重连途中的一次失败：继续下一轮
                self._schedule_reconnect()
                return
            self.lbl_conn.configure(text="● 未连接", fg="#e74c3c")
            self.log("连接失败：" + info.decode("utf-8", "replace"))
            return
        # 主动断开（disconnected）
        self.lbl_conn.configure(text="● 未连接", fg="#e74c3c")
        if was:
            self.log("已断开。LED override 归还电源 UI 状态机（协议 §7.4）。")


    def _on_worker_error(self, where, msg):
        self.log(f"BLE 错误（{where}）：{msg}")

    def _on_close(self):
        self._closing = True
        self.worker.stop()
        self.after(300, self.destroy)

    # ---------------- 指令发送 ----------------
    def _send_cmd(self, cmd, params=b""):
        if not self.link.connected:
            self.term_print("!! 未连接，指令未发送")
            return False
        data = bytes([cmd]) + params
        hexs = " ".join(f"{b:02X}" for b in data)
        self.term_print(f">> WRITE 0x22: {hexs}"
                        f"  （{CMD_NAMES.get(cmd, '?')}）")
        self.worker.submit("write", data)
        return True

    def on_led(self, idx, on):
        self.link.led[idx] = on
        lamp, color = self.led_lamps[idx]
        lamp.configure(fg=color if on else "#3a4148")
        mask = (1 if self.link.led[0] else 0) | (2 if self.link.led[1] else 0)
        self._send_cmd(0x01, bytes([mask]))

    def on_pwr(self, dom, on):
        self.link.pwr[dom] = on
        self.pwr_lamps[dom].configure(fg="#2ecc71" if on else "#3a4148")
        d = {"SENS": 0, "STORE": 1, "ANALOG": 2}[dom]
        self._send_cmd(0x02, bytes([d, 1 if on else 0]))
        if dom == "ANALOG" and on:
            self.log("ANALOG 上电：首次约 1200ms VBIAS 稳定等待（协议 §10.3）。")

    def on_pwr_all(self, on):
        for dom in ("SENS", "STORE", "ANALOG"):
            self.on_pwr(dom, on)

    def on_thr_set(self, ch, spin):
        v = int(spin.get())
        self.link.qvar_thr[ch] = v
        self.qvar_waves[ch].set_thresholds((v, -v))
        self._send_cmd(0x08, struct.pack("<BHB", 0 if ch == "A" else 1,
                                         v, 50))
        self.log(f"QVAR-{ch} 阈值下发 ±{v} LSB（滞回 50）。")

    def on_reprobe(self):
        if self._send_cmd(0x06):
            self.log("REPROBE 已下发：重新探测全部外设，"
                     "状态变化经 EVENT 0x01 上报。")

    def on_sd_refresh(self):
        self._send_cmd(0x10)
        self._send_cmd(0x09)

    # ---------------- 录音 ----------------
    def on_rec_start(self):
        lk = self.link
        lk.rec_channels = (self.ch_l.get(), self.ch_r.get())
        if not any(lk.rec_channels):
            self.log("请至少选择一个声道。")
            return
        lk.rec_duration = max(1, int(self.rec_dur.get()))
        mask = (1 if lk.rec_channels[0] else 0) | \
               (2 if lk.rec_channels[1] else 0)
        if not self._send_cmd(0x07, struct.pack("<BBH", 1, mask,
                                                lk.rec_duration)):
            return
        self._rec_pending = True
        self._rec_pending_since = time.monotonic()
        self.btn_rec_start.configure(state="disabled")

    def on_rec_stop(self, auto=False):  # noqa: ARG002
        if not self.link.recording and not self._rec_pending:
            return
        self._send_cmd(0x07, struct.pack("<BBH", 0, 0, 0))

    def _rec_ui_started(self):
        lk = self.link
        lk.recording = True
        lk.rec_elapsed = 0.0
        self._rec_pending = False
        self.btn_rec_start.configure(state="disabled")
        self.btn_rec_stop.configure(state="normal")
        self.btn_rec_play.configure(state="disabled")
        self.log(f"录音开始：时长 {lk.rec_duration}s，"
                 f"设备端录至 SD 卡（PCM 16kHz/16bit）。")

    def _rec_ui_stopped(self, file_id, done):
        lk = self.link
        lk.recording = False
        self._rec_pending = False
        self.btn_rec_start.configure(state="normal")
        self.btn_rec_stop.configure(state="disabled")
        self.rec_prog["value"] = 100 if done else 0
        if done and file_id:
            self.lbl_rec.configure(text=f"录音完成 file_id={file_id}，下载中…")
            self.log(f"录音完成（file_id={file_id}），自动下载到本地……")
            self._start_xfer(file_id)
            self._send_cmd(0x09)            # 顺手刷新文件列表
        else:
            self.lbl_rec.configure(text="已停止")
            self.log("录音已停止。")

    # ---------------- SD 卡 ----------------
    def on_sd_toggle(self):  # 已被 _rewire 换成 on_sd_refresh，保留兜底
        self.on_sd_refresh()

    def on_sd_test_start(self):
        if self.sd_testing:
            return
        size_mb = max(1, min(64, int(self.sd_size.get())))
        verify = 1 if self.sd_verify.get() else 0
        if not self._send_cmd(0x0D, bytes([size_mb, verify])):
            return
        self.sd_testing = True
        self.btn_sd_test.configure(state="disabled")
        self.sd_prog.configure(mode="indeterminate")
        self.sd_prog.start(12)
        self.lbl_sd_status.configure(
            text=f"设备端测试中：写 {size_mb}MB → "
                 + ("读回校验…" if verify else "跳过校验…"))
        self.sd_res_w.configure(text="…")
        self.sd_res_r.configure(text="…")
        self.sd_res_v.configure(text="…", foreground=self.FG)

    def _sd_test_done(self, result, w_kbps, r_kbps, bad):
        self.sd_testing = False
        self.sd_prog.stop()
        self.sd_prog.configure(mode="determinate", value=100)
        self.btn_sd_test.configure(state="normal")
        verify_on = self.sd_verify.get()
        self.sd_res_w.configure(text=f"{w_kbps / 1024:.2f} MB/s")
        self.sd_res_r.configure(
            text=f"{r_kbps / 1024:.2f} MB/s" if verify_on else "—")
        if result == 0:
            self.sd_res_v.configure(text="通过 ✔", foreground="#2ecc71")
            self.lbl_sd_status.configure(text="就绪")
            self.log(f"SD 测试通过：写 {w_kbps / 1024:.2f} MB/s，"
                     f"读 {r_kbps / 1024:.2f} MB/s。")
        else:
            err = SD_TEST_ERR.get(result, f"错误 {result}")
            self.sd_res_v.configure(
                text=f"失败 ✘（{err}" + (f"，{bad} 块" if bad else "") + "）",
                foreground="#e74c3c")
            self.lbl_sd_status.configure(text="就绪")
            self.log(f"SD 测试失败：{err}。")

    def _sd_refresh_files_log(self):
        self._send_cmd(0x09)

    def on_sd_download(self):
        fid = self._sd_selected()
        if fid is None:
            self.log("请先在文件列表中选择要下载的文件。")
            return
        self._start_xfer(fid)

    def _start_xfer(self, fid):
        buf = self.xfer.setdefault(fid, {"buf": bytearray()})["buf"]
        offset = len(buf)
        self.xfer_target = fid
        if self._send_cmd(0x0A, struct.pack("<HI", fid, offset)):
            self.lbl_sd_status.configure(
                text=f"下载 file_id={fid} 中"
                     + (f"（断点续传 @{offset}B）" if offset else "…"))

    def on_sd_delete(self):
        fid = self._sd_selected()
        if fid is None:
            self.log("请先在文件列表中选择要删除的文件。")
            return
        self._send_cmd(0x0B, struct.pack("<H", fid))

    # ---------------- 指令终端 ----------------
    def on_cmd_send(self):
        cmd = self.cmd_entry.get().strip().upper() or "00"
        params = self.param_entry.get().strip().upper()
        try:
            cmd_b = bytes([int(cmd, 16)])
            par_b = bytes(int(p, 16) for p in params.split()) if params else b""
        except ValueError:
            self.term_print("!! 指令/参数需为十六进制（如 01 / DE AD BE EF）")
            return
        self._send_cmd(cmd_b[0], par_b)

    # ---------------- 帧接收（GUI 线程） ----------------
    def _rx_stream(self, frames):
        for ftype, seq, payload in frames:
            self._emit_rx(0x21, ftype, seq, payload)
            h = self._STREAM_H.get(ftype)
            if h is not None:
                try:
                    h(self, payload)
                except Exception as e:  # noqa: BLE001
                    self.log(f"帧解析错误 TYPE=0x{ftype:02X}：{e}")

    def _rx_eventch(self, frames):
        for ftype, seq, payload in frames:
            self._emit_rx(0x23, ftype, seq, payload)
            if ftype == 0x20 and len(payload) >= 3:
                self._h_ack(payload)
            elif ftype == 0x21 and payload:
                self._h_event(payload)

    # ---- 数据帧处理 ----
    def _h_imu(self, key, payload):
        if len(payload) < 1:
            return
        st = payload[0] & 0x0F
        mk = f"IMU_{key}"
        if self.link.mod_state[mk] != st:
            self.link.mod_state[mk] = st
            self._refresh_mod_lamps()
        if st != 1:
            for lbl in self.imu_vals[key]:
                lbl.configure(text="—")
            self.imu_stats[key].configure(
                text="RMS |a|   — mg\nRMS |g|   — dps")
            self._imu_hist[key].clear()
            return
        n = (len(payload) - 1) // 12
        last = None
        for i in range(n):
            raw = struct.unpack_from("<6h", payload, 1 + i * 12)
            f = tuple(int(round(raw[k] * ACC_MG_PER_LSB)) for k in range(3)) \
                + tuple(int(round(raw[3 + k] * GYRO_DDPS_PER_LSB))
                        for k in range(3))
            last = f
            hist = self._imu_hist[key]
            hist.append(f)
            if len(hist) > 50:
                hist.pop(0)
            ui = self.imu_ui[key]
            if ui["mode"].get() == "overview":
                ui["wa"].push(f[0:3])
                ui["wg"].push(f[3:6])
            else:
                idx = ("ax", "ay", "az", "gx", "gy", "gz").index(
                    ui["comp"].get())
                ui["ws"].push(f[idx])
        if last is None:
            return
        for lbl, val in zip(self.imu_vals[key], last):
            lbl.configure(text=str(val))
        hist = self._imu_hist[key]
        nh = len(hist)
        rms_a = math.sqrt(sum(s[0] ** 2 + s[1] ** 2 + s[2] ** 2
                              for s in hist) / nh)
        rms_g = math.sqrt(sum(s[3] ** 2 + s[4] ** 2 + s[5] ** 2
                              for s in hist) / nh) / 10
        self.imu_stats[key].configure(
            text=f"RMS |a| {rms_a:5.0f} mg\nRMS |g| {rms_g:5.1f} dps")
        ui = self.imu_ui[key]
        if ui["mode"].get() == "overview":
            self._rd(ui["wa"])
            self._rd(ui["wg"])
        else:
            self._rd(ui["ws"])

    def _h_imu_u4(self, payload):
        self._h_imu("U4", payload)

    def _h_imu_u1(self, payload):
        self._h_imu("U1", payload)

    def _h_qvar(self, payload):
        if len(payload) < 6:
            return
        st, a, b, valid = struct.unpack("<BhhB", payload[:6])
        changed = False
        for ch, v, bit in (("A", a, 1), ("B", b, 2)):
            mk = f"QVAR_{ch}"
            nst = 1 if (valid & bit) else (3 if st == 3 else 2)
            if self.link.mod_state[mk] != nst:
                self.link.mod_state[mk] = nst
                changed = True
            if valid & bit:
                self.qvar_vals[ch].configure(text=str(v))
                self.qvar_waves[ch].push(v)
                self._rd(self.qvar_waves[ch])
                alarm = abs(v) > self.link.qvar_thr[ch]
                self.qvar_lamps[ch].configure(
                    fg="#e74c3c" if alarm else "#3a4148")
            else:
                self.qvar_vals[ch].configure(text="—")
                self.qvar_lamps[ch].configure(fg="#3a4148")
        if changed:
            self._refresh_mod_lamps()

    def _h_pvdf(self, payload):
        if len(payload) < 13:
            return
        st, heart, raw, ref = struct.unpack("<Biii", payload[:13])
        if self.link.mod_state["PVDF"] != st:
            self.link.mod_state["PVDF"] = st
            self._refresh_mod_lamps()
        if st != 1:
            for nm in ("heart_mv", "raw_mv", "ref_mv"):
                self.pvdf_vals[nm].configure(text="—")
            return
        self.pvdf_vals["heart_mv"].configure(text=str(heart))
        self.pvdf_vals["raw_mv"].configure(text=str(raw))
        self.pvdf_vals["ref_mv"].configure(text=str(ref))
        self.pvdf_wave.push(heart - ref)
        self._rd(self.pvdf_wave)
        self.pvdf_wave2.push(raw - ref)
        self._rd(self.pvdf_wave2)

    def _h_temp(self, payload):
        if len(payload) < 3:
            return
        st = payload[0]
        if self.link.mod_state["TEMP"] != st:
            self.link.mod_state["TEMP"] = st
            self._refresh_mod_lamps()

    def _h_mic(self, payload):  # RMS 帧：状态(1)+左RMS(2)+右RMS(2)
        if len(payload) >= 5:
            _st, l, r = struct.unpack("<BHH", payload[:5])
            self.link.last_mic = (l, r)
        if self.link.mod_state["MIC"] != 1:
            self.link.mod_state["MIC"] = 1
            self._refresh_mod_lamps()

    def _h_battery(self, payload):
        if len(payload) < 5:
            return
        _, mv, pct, flags = struct.unpack("<BHBB", payload[:5])
        chg = bool(flags & 0x01)
        self.link.last_batt = (mv, pct, chg)
        self.link.has_batt = True

    def _h_modstatus(self, payload):
        if len(payload) < 5:
            return
        st0, st1, pwr, led, _pct = payload[:5]
        m = self.link.mod_state
        vals = (st0 & 3, (st0 >> 2) & 3, (st0 >> 4) & 3, (st0 >> 6) & 3,
                st1 & 3, (st1 >> 2) & 3, (st1 >> 4) & 3, (st1 >> 6) & 3)
        for k, v in zip(MOD_KEYS, vals):
            m[k] = v
        self._refresh_mod_lamps()
        self.link.pwr["SENS"] = bool(pwr & 1)
        self.link.pwr["STORE"] = bool(pwr & 2)
        self.link.pwr["ANALOG"] = bool(pwr & 4)
        for dom, on in self.link.pwr.items():
            self.pwr_lamps[dom].configure(
                fg="#2ecc71" if on else "#3a4148")
        self.link.led[0] = bool(led & 1)
        self.link.led[1] = bool(led & 2)
        for i, (lamp, color) in enumerate(self.led_lamps):
            lamp.configure(fg=color if self.link.led[i] else "#3a4148")

    def _h_audio(self, payload):
        if len(payload) < 6:
            return
        fid, offset = struct.unpack_from("<HI", payload, 0)
        data = payload[6:]
        ent = self.xfer.setdefault(fid, {"buf": bytearray()})
        buf = ent["buf"]
        if offset == len(buf):
            buf += data
        elif offset > len(buf):
            # 中间丢块：放弃本次，等待下轮或重传
            self.log(f"下载 file_id={fid} 丢块（期望 @{len(buf)} "
                     f"实到 @{offset}），将自动断点续传。")
        # 进度（有列表估算大小时；节流，每 ~8 块刷一次 UI）
        kb = next((f[2] for f in self.link.sd_files if f[0] == fid), 0)
        if kb:
            ent["ui_n"] = ent.get("ui_n", 0) + 1
            if ent["ui_n"] >= 8:
                ent["ui_n"] = 0
                pct = min(100, len(buf) / (kb * 1024) * 100)
                self.sd_prog.configure(mode="determinate", value=pct)
                self.lbl_sd_status.configure(
                    text=f"下载 file_id={fid}：{len(buf) // 1024} / ~{kb} KB")

    _STREAM_H = {0x01: _h_imu_u4, 0x02: _h_imu_u1, 0x03: _h_qvar,
                 0x04: _h_pvdf, 0x05: _h_temp, 0x06: _h_mic,
                 0x07: _h_audio, 0x08: _h_battery, 0x10: _h_modstatus}

    # ---- 应答/事件处理 ----
    def _h_ack(self, payload):
        cmd, _req_seq, result = payload[0], payload[1], \
            struct.unpack("<b", payload[2:3])[0]
        data = payload[3:]
        name = CMD_NAMES.get(cmd, "?")
        extra = ""
        if cmd == 0x11 and data:            # GET_VERSION
            extra = " data='" + data.decode("utf-8", "replace") + "'"
        elif cmd == 0x7F and data:          # PING 回显
            extra = " data=" + " ".join(f"{b:02X}" for b in data)
        self.term_print(f"<< CMD_ACK cmd=0x{cmd:02X}({name}) "
                        f"result={result}{extra}")
        if cmd == 0x09:                     # REC_LIST
            if result == 0:
                files = []
                for i in range(0, len(data) - 5, 6):
                    fid, dur, kb = struct.unpack_from("<HHH", data, i)
                    files.append((fid, dur, kb, "—"))
                self.link.sd_files = files
                self._sd_refresh_files()
                self.log(f"SD 文件列表刷新：{len(files)} 个文件。")
            else:
                self.log(f"REC_LIST 失败 result={result}"
                         "（SD 不在位或未挂载）。")
        elif cmd == 0x0B and result == 0:   # REC_DELETE
            self._send_cmd(0x09)
        elif cmd == 0x07:                   # REC_CTRL
            if result == 0 and self._rec_pending:
                self._rec_ui_started()
            elif result != 0 and self._rec_pending:
                self._rec_pending = False
                self.btn_rec_start.configure(state="normal")
                self.log(f"录音启动失败 result={result}"
                         "（SD 不在位或正忙）。")
        elif cmd == 0x0A and result != 0:   # REC_READ
            self.log(f"下载启动失败 result={result}（文件不存在或正忙）。")
            self.xfer_target = None
            self.lbl_sd_status.configure(text="就绪")
        elif cmd == 0x0D and result != 0:   # SD_TEST 立即失败
            self._sd_test_done(result, 0, 0, 0)

    def _h_event(self, payload):
        eid = payload[0]
        if eid == 0x01 and len(payload) >= 3:       # MOD_STATE
            mod, nst = payload[1], payload[2]
            if mod < len(MOD_KEYS):
                self.link.mod_state[MOD_KEYS[mod]] = nst
                self._refresh_mod_lamps()
                self.term_print(f"<< EVENT 0x21 id=0x01 "
                                f"module={MOD_KEYS[mod]} new_state={nst}")
        elif eid == 0x02:                            # WDT_RESET
            self.log("⚠ 设备发生过看门狗复位（EVENT WDT_RESET）。")
            self.term_print("<< EVENT 0x21 id=0x02 WDT_RESET")
        elif eid == 0x03 and len(payload) >= 4:      # LOW_BATT
            mv, pct = struct.unpack_from("<HB", payload, 1)
            self.log(f"⚠ 低电告警：{pct}%（{mv}mV）。")
            self.term_print(f"<< EVENT 0x21 id=0x03 LOW_BATTERY pct={pct}")
        elif eid == 0x04 and len(payload) >= 5:      # QVAR_THR
            ch, crossed, raw = struct.unpack_from("<BBh", payload, 1)
            c = "A" if ch == 0 else "B"
            self.qvar_lamps[c].configure(
                fg="#e74c3c" if crossed else "#3a4148")
            self.term_print(f"<< EVENT 0x21 id=0x04 QVAR_THR_CROSSED "
                            f"ch={c} crossed={crossed} raw={raw}")
        elif eid == 0x05 and len(payload) >= 5:      # REC_STATE
            state, fid, _el = struct.unpack_from("<BHB", payload, 1)
            self.term_print(f"<< EVENT 0x21 id=0x05 REC_STATE "
                            f"state={state} file_id={fid}")
            if state == 1:
                if not self.link.recording:
                    self._rec_ui_started()
            elif state == 2:
                self._rec_ui_stopped(fid, done=True)
            elif state == 0:
                self._rec_ui_stopped(fid, done=False)
        elif eid == 0x06 and len(payload) >= 11:     # REC_FILE_DONE
            fid, total, crc = struct.unpack_from("<HII", payload, 1)
            self._xfer_done(fid, total, crc)
        elif eid == 0x07 and len(payload) >= 8:      # SD_TEST_DONE
            result = struct.unpack("<b", payload[1:2])[0]
            w_kbps, r_kbps, bad = struct.unpack_from("<HHH", payload, 2)
            self.term_print(f"<< EVENT 0x21 id=0x07 SD_TEST_DONE "
                            f"result={result} w={w_kbps}KB/s "
                            f"r={r_kbps}KB/s bad={bad}")
            self._sd_test_done(result, w_kbps, r_kbps, bad)

    def _xfer_done(self, fid, total, crc):
        buf = bytes(self.xfer.pop(fid, {"buf": bytearray()})["buf"])
        if len(buf) < total:
            # 少块：自动断点续传一次
            self.xfer[fid] = {"buf": bytearray(buf)}
            self.log(f"下载 file_id={fid} 不完整"
                     f"（{len(buf)}/{total}B），断点续传…")
            self._send_cmd(0x0A, struct.pack("<HI", fid, len(buf)))
            return
        buf = buf[:total]
        ok = (zlib.crc32(buf) & 0xFFFFFFFF) == crc
        path = os.path.join(DOWNLOAD_DIR, f"REC{fid:04d}.wav")
        with open(path, "wb") as fp:
            fp.write(buf)
        self.sd_prog.configure(mode="determinate", value=100)
        self.lbl_sd_status.configure(
            text=f"file_id={fid} 下载完成（{total // 1024}KB，"
                 f"CRC32 {'✔' if ok else '✘ 不一致'}）")
        self.log(f"文件 file_id={fid} 已保存 {path}"
                 f"（{total}B，CRC32 {'通过' if ok else '失败'}）。")
        self.term_print(f"<< XFER DONE id={fid} bytes={total} "
                        f"crc32={'OK' if ok else 'BAD'}")
        if ok:
            self.link.rec_file = path
            self.btn_rec_play.configure(state="normal")
            self.lbl_rec.configure(text=f"已下载 file_id={fid}，可播放")
        self.xfer_target = None
        self._send_cmd(0x09)                # 刷新列表

    # ---------------- 帧监视 ----------------
    def _emit_rx(self, ch, ftype, seq, payload):
        if self.frame_pause.get():
            return
        body = bytes([ftype, seq, len(payload)]) + payload
        frame = bytes([0x55, 0xAA]) + body + bytes([crc8(body)])
        hexs = " ".join(f"{b:02X}" for b in frame)
        tag = "0x21" if ch == 0x21 else "0x23"
        # 只入缓存，_tick 每 100ms 批量刷一次（逐帧 insert+see 在
        # ~200 帧/s 数据流下会吃满主线程导致界面卡死）
        self._rx_buf.append(
            f"[{tag} 0x{ftype:02X} {TYPE_NAMES.get(ftype, '?'):13s}"
            f" #{seq:3d}] {hexs}\n")
        if len(self._rx_buf) > 300:
            del self._rx_buf[:100]          # 洪峰丢弃最旧，防积压

    def _rd(self, w):
        """波形重绘节流：只标脏，_tick 里统一重绘（每画布每 100ms 至多一次）"""
        if id(w) not in self._rd_marked:
            self._rd_marked.add(id(w))
            self._rd_marks.append(w)

    def _flush_ui(self):
        if getattr(self, "frame_txt", None) is None:
            return                          # 基类 __init__ 尚未建好控件
        buf = self._rx_buf
        if buf:
            self._rx_buf = []
            self.frame_txt.insert("end", "".join(buf))
            lines = int(self.frame_txt.index("end-1c").split(".")[0])
            if lines > 500:
                self.frame_txt.delete("1.0", "250.0")
            self.frame_txt.see("end")
        if self._rd_marks:
            waves = self._rd_marks
            self._rd_marks = []
            self._rd_marked = set()
            for w in waves:
                try:
                    w.redraw()
                except Exception:
                    pass

    def _emit_frame(self, ftype, payload):  # demo 残留接口，正式版不用
        pass

    # ---------------- 周期刷新 ----------------
    def _tick(self):
        self._flush_ui()
        lk = self.link
        # 电池：仅显示真实帧数据（未连接/未收到时显示 —）
        # 注：super().__init__ 期间 link 仍是 SimLink，用 getattr 兼容
        if getattr(lk, "has_batt", False):
            mv, pct, chg = lk.last_batt
            self._draw_battery(mv, pct, chg)
            self.batt_big["soc"].configure(text=f"{pct} %")
            self.batt_big["vbat"].configure(text=f"{mv} mV")
            self.batt_big["state"].configure(
                text="⚡ 充电中" if chg else "电池供电")
            low = pct <= lk.BATT_LOW_PCT
            self.batt_low_lamp.configure(
                fg="#e74c3c" if low else "#3a4148")
            self._batt_chart_cnt = getattr(self, "_batt_chart_cnt", 0) + 1
            if self._batt_chart_cnt >= 5:
                self._batt_chart_cnt = 0
                self.batt_chart.push(time.time(), mv, pct)
                self.batt_chart.redraw()
        # 录音进度（客户端按时间估计；完成以 REC_STATE 事件为准）
        if lk.recording:
            lk.rec_elapsed += 0.1
            pct = min(100, lk.rec_elapsed / lk.rec_duration * 100)
            self.rec_prog["value"] = pct
            ml, mr = getattr(lk, "last_mic", (0, 0))
            self.lbl_rec.configure(
                text=f"录音中 {lk.rec_elapsed:.1f}s / {lk.rec_duration}s"
                     f"（录至 SD）  L:{ml:5d}  R:{mr:5d}")
        # 录音启动 ACK 超时兜底（旧固件失败静默时会卡“等待启动”）
        # 注：super().__init__ 期间就会跑 _tick，属性用 getattr 兜底
        if getattr(self, "_rec_pending", False) and \
                (time.monotonic() -
                 getattr(self, "_rec_pending_since", 0.0)) > REC_ACK_TIMEOUT:
            self._rec_pending = False
            self.btn_rec_start.configure(state="normal")
            self.log("录音启动超时：未收到 ACK（确认连接与 SD 状态后重试）。")
        self.after(100, self._tick)


if __name__ == "__main__":
    app = PetRingLive()
    app.mainloop()
