# -*- coding: utf-8 -*-
"""BLE 后端：独立 QThread 内跑 asyncio + bleak，经 Qt 信号与 GUI 解耦。

信号：
    scanned(list[dict])            扫描结果 [{name, address, rssi}]
    conn_state(bool, str)          连接状态变化（msg 含 MTU/地址/断开原因）
    info_text(str)                 0x24 固件信息
    data_frame(int, int, bytes)    0x21 数据帧 (type, seq, payload)
    ack_frame(bytes)               0x20 指令应答 payload
    event_frame(bytes)             0x21 EVENT 帧 payload
    log(str)                       后端日志
    bad_crc(int)                   CRC 错误计数变化
"""
import asyncio
import threading

from PySide6.QtCore import QThread, Signal

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from . import protocol as P


class BleBackend(QThread):
    scanned = Signal(list)
    conn_state = Signal(bool, str)
    info_text = Signal(str)
    data_frame = Signal(int, int, bytes)
    ack_frame = Signal(bytes)
    event_frame = Signal(bytes)
    log = Signal(str)
    bad_crc = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._loop = None
        self._client = None
        self._parser = P.FrameParser()
        self._ack_parser = P.FrameParser()
        self._scan_future = None
        self._stop = threading.Event()

    # ---------- 线程主循环 ----------
    def run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()
        self._loop.close()

    def stop(self):
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)

    def _submit(self, coro):
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(coro, self._loop)

    # ---------- 对外 API（GUI 线程调用） ----------
    def scan(self, timeout=5.0):
        self._submit(self._scan_coro(timeout))

    def connect_device(self, address: str):
        self._submit(self._connect_coro(address))

    def disconnect_device(self):
        self._submit(self._disconnect_coro())

    def send_cmd(self, data: bytes):
        self._submit(self._write_coro(data))

    # ---------- 协程 ----------
    async def _scan_coro(self, timeout):
        self.log.emit(f"扫描 {timeout:.0f}s ...")
        try:
            devs = await BleakScanner.discover(timeout=timeout,
                                               return_adv=False)
        except BleakError as e:
            self.log.emit(f"扫描失败: {e}")
            self.scanned.emit([])
            return
        out = [{"name": d.name or "(未命名)", "address": d.address,
                "rssi": d.rssi} for d in devs]
        out.sort(key=lambda x: x["rssi"], reverse=True)
        self.scanned.emit(out)
        self.log.emit(f"发现 {len(out)} 台设备")

    async def _connect_coro(self, address):
        await self._disconnect_coro()
        self._parser = P.FrameParser()
        self._ack_parser = P.FrameParser()
        try:
            self.log.emit(f"连接 {address} ...")
            cli = BleakClient(address, timeout=15.0,
                              disconnected_callback=self._on_disconnect)
            await cli.connect()
            self._client = cli
            await cli.start_notify(P.UUID_DATA, self._on_data)
            await cli.start_notify(P.UUID_ACK, self._on_ack)
            info = await cli.read_gatt_char(P.UUID_INFO)
            self.info_text.emit(info.decode(errors="replace"))
            self.conn_state.emit(True,
                                   f"已连接 {address}  MTU={cli.mtu_size}")
            self.log.emit(f"连接成功，MTU={cli.mtu_size}")
        except (BleakError, asyncio.TimeoutError, OSError) as e:
            self._client = None
            self.conn_state.emit(False, f"连接失败: {e}")
            self.log.emit(f"连接失败: {e}")

    async def _disconnect_coro(self):
        if self._client:
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            except BleakError:
                pass
            self._client = None

    async def _write_coro(self, data: bytes):
        if not (self._client and self._client.is_connected):
            self.log.emit("未连接，指令丢弃: " + data.hex(" ").upper())
            return
        try:
            await self._client.write_gatt_char(P.UUID_CMD, data,
                                               response=True)
            self.log.emit("TX " + data.hex(" ").upper())
        except BleakError as e:
            self.log.emit(f"写入失败: {e}")

    # ---------- notify 回调（bleak 线程上下文，直接发信号即可） ----------
    def _on_data(self, _, data: bytearray):
        for ftype, seq, payload in self._parser.feed(bytes(data)):
            self.data_frame.emit(ftype, seq, payload)
        self.bad_crc.emit(self._parser.bad_crc + self._ack_parser.bad_crc)

    def _on_ack(self, _, data: bytearray):
        for ftype, _seq, payload in self._ack_parser.feed(bytes(data)):
            if ftype == P.T_CMD_ACK:
                self.ack_frame.emit(payload)
            elif ftype == P.T_EVENT:
                self.event_frame.emit(payload)
        self.bad_crc.emit(self._parser.bad_crc + self._ack_parser.bad_crc)

    def _on_disconnect(self, _client):
        self._client = None
        self.conn_state.emit(False, "连接已断开")
        self.log.emit("设备断开连接")
