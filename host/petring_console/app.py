# -*- coding: utf-8 -*-
"""宠物环传感器控制台（正式版）——task-V1.06，协议 V0.5。

bleak 真实 BLE 连接 + PySide6 GUI。与固件 Sensor Hub 服务
（e5a00020）通信，覆盖：连接/信息、LED、三电源、六轴×2、QVAR+阈值、
PVDF、温度、麦克风 RMS、录音控制/文件管理（断点续传+CRC32）、
SD 卡测试、电池 SOC、帧监视、指令终端。
"""
import os
import struct
import time
import wave
import zlib
from collections import Counter

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QListWidget,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QSpinBox,
    QSplitter, QTabWidget, QTableWidget, QTableWidgetItem, QDoubleSpinBox,
    QTextEdit, QVBoxLayout, QWidget,
)

from . import protocol as P
from .backend import BleBackend
from .widgets import BatteryPanel, Lamp, LiveChart

# int16 原始码换算（与固件 overlay 对齐：±8g / ±2000dps）
ACC_LSB_PER_MG = 4096 / 1000.0        # ±8g: 0.244 mg/LSB -> mg = raw*0.244
GYRO_LSB_PER_MDPS = 70000 / 1000.0    # ±2000dps: 70 mdps/LSB
ACC_SCALE_MG = 0.244
GYRO_SCALE_MDPS = 70.0

AXIS_COLORS = ["#e74c3c", "#27ae60", "#3498db"]
AXIS_NAMES = ["X", "Y", "Z"]


def i16(b, o):
    return struct.unpack_from("<h", b, o)[0]


def i32(b, o):
    return struct.unpack_from("<i", b, o)[0]


def u16(b, o):
    return struct.unpack_from("<H", b, o)[0]


def u32(b, o):
    return struct.unpack_from("<I", b, o)[0]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("宠物环传感器控制台 v1.06（正式版）")
        self.resize(1360, 880)

        self.backend = BleBackend(self)
        self.backend.start()

        self.frame_counts = Counter()
        self.last_seq = {}
        self.seq_gaps = 0
        self.rec_state = 0            # 0 idle 1 recording 2 done
        self.rec_file_id = 0
        self.xfer = None              # 文件下载状态 dict
        self.sd_busy = False
        self.t_start = time.monotonic()

        self._build_ui()
        self._wire_backend()

        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self._refresh_stats)
        self.stat_timer.start(500)

    # ================= UI =================
    def _build_ui(self):
        root = QSplitter(Qt.Horizontal)
        self.setCentralWidget(root)

        # ---- 左栏：连接 + 状态 + 电池 ----
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(6, 6, 6, 6)

        gb = QGroupBox("BLE 连接")
        g = QGridLayout(gb)
        self.dev_list = QListWidget()
        self.dev_list.setMaximumHeight(120)
        g.addWidget(self.dev_list, 0, 0, 1, 3)
        self.btn_scan = QPushButton("扫描")
        self.btn_scan.clicked.connect(self.on_scan)
        self.btn_conn = QPushButton("连接")
        self.btn_conn.clicked.connect(self.on_connect)
        self.btn_disconn = QPushButton("断开")
        self.btn_disconn.clicked.connect(self.backend.disconnect_device)
        g.addWidget(self.btn_scan, 1, 0)
        g.addWidget(self.btn_conn, 1, 1)
        g.addWidget(self.btn_disconn, 1, 2)
        self.conn_lamp = Lamp("gray")
        self.conn_label = QLabel("未连接")
        hl = QHBoxLayout()
        hl.addWidget(self.conn_lamp)
        hl.addWidget(self.conn_label, 1)
        g.addLayout(hl, 2, 0, 1, 3)
        self.info_label = QLabel("固件信息：—")
        self.info_label.setWordWrap(True)
        g.addWidget(self.info_label, 3, 0, 1, 3)
        lv.addWidget(gb)

        gb = QGroupBox("模块状态（0x10）")
        g = QGridLayout(gb)
        self.mod_lamps = {}
        self.mod_labels = {}
        for i, name in enumerate(["U4六轴", "U1六轴", "QVAR", "SD卡",
                                  "PVDF", "麦克风"]):
            lamp = Lamp("gray")
            lab = QLabel("—")
            g.addWidget(QLabel(name), i, 0)
            g.addWidget(lamp, i, 1)
            g.addWidget(lab, i, 2)
            self.mod_lamps[name] = lamp
            self.mod_labels[name] = lab
        lv.addWidget(gb)

        gb = QGroupBox("电池（0x08）")
        bv = QVBoxLayout(gb)
        self.batt = BatteryPanel()
        bv.addWidget(self.batt)
        btn = QPushButton("即时查询 GET_BATTERY")
        btn.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_GET_BATTERY])))
        bv.addWidget(btn)
        lv.addWidget(gb)
        lv.addStretch(1)
        left.setMaximumWidth(340)
        root.addWidget(left)

        # ---- 右侧页签 ----
        self.tabs = QTabWidget()
        root.addWidget(self.tabs)
        root.setStretchFactor(1, 1)

        self._tab_overview()
        self._tab_imu()
        self._tab_qvar()
        self._tab_pvdf()
        self._tab_mic()
        self._tab_sd()
        self._tab_monitor()
        self._tab_terminal()

    def _tab_overview(self):
        w = QWidget()
        v = QVBoxLayout(w)

        gb = QGroupBox("LED 控制（LED_SET 0x01）")
        h = QHBoxLayout(gb)
        self.led_btns = []
        for i, nm in enumerate(("LED0(红)", "LED1(蓝)")):
            b_on = QPushButton(f"{nm} 开")
            b_off = QPushButton(f"{nm} 关")
            b_on.clicked.connect(
                lambda _=0, i=i: self.send_cmd(bytes([P.CMD_LED_SET, i, 1])))
            b_off.clicked.connect(
                lambda _=0, i=i: self.send_cmd(bytes([P.CMD_LED_SET, i, 0])))
            h.addWidget(b_on)
            h.addWidget(b_off)
        b = QPushButton("全部关")
        b.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_LED_SET, 0, 0]))
                          or self.send_cmd(bytes([P.CMD_LED_SET, 1, 0])))
        h.addWidget(b)
        h.addWidget(QLabel("注：LED1 蓝灯极性为已知硬件待整改项，现板不亮属正常"))
        h.addStretch(1)
        v.addWidget(gb)

        gb = QGroupBox("三电源域（PWR_SET 0x02）")
        h = QHBoxLayout(gb)
        for did, nm in ((1, "SENS"), (2, "ANALOG"), (3, "STORE")):
            b_on = QPushButton(f"{nm} 开")
            b_off = QPushButton(f"{nm} 关")
            b_on.clicked.connect(
                lambda _=0, d=did: self.send_cmd(bytes([P.CMD_PWR_SET, d, 1])))
            b_off.clicked.connect(
                lambda _=0, d=did: self.send_cmd(bytes([P.CMD_PWR_SET, d, 0])))
            h.addWidget(b_on)
            h.addWidget(b_off)
        h.addStretch(1)
        v.addWidget(gb)

        gb = QGroupBox("传感器使能（SENSOR_EN 0x03）/ 速率（RATE_SET 0x04）")
        g = QGridLayout(gb)
        self.sen_checks = {}
        for i, (sid, nm) in enumerate(
                ((1, "IMU_U4"), (2, "BODY_IMU_U1"), (3, "QVAR"),
                 (4, "PVDF"), (5, "MIC_RMS"))):
            ck = QCheckBox(nm)
            ck.setChecked(sid in (1, 3, 4))
            g.addWidget(ck, 0, i)
            self.sen_checks[sid] = ck
        b = QPushButton("应用使能")
        b.clicked.connect(self.on_sensor_en)
        g.addWidget(b, 0, 5)
        g.addWidget(QLabel("目标"), 1, 0)
        self.rate_target = QComboBox()
        self.rate_target.addItems(["IMU_U4", "BODY_IMU_U1", "QVAR", "PVDF"])
        g.addWidget(self.rate_target, 1, 1)
        g.addWidget(QLabel("ODR(Hz)"), 1, 2)
        self.rate_hz = QSpinBox()
        self.rate_hz.setRange(1, 416)
        self.rate_hz.setValue(30)
        g.addWidget(self.rate_hz, 1, 3)
        b = QPushButton("设定速率")
        b.clicked.connect(self.on_rate_set)
        g.addWidget(b, 1, 4)
        b = QPushButton("重新探测 REPROBE")
        b.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_REPROBE])))
        g.addWidget(b, 1, 5)
        v.addWidget(gb)

        gb = QGroupBox("设备信息 / 帧统计")
        g = QGridLayout(gb)
        b = QPushButton("GET_VERSION")
        b.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_GET_VERSION])))
        g.addWidget(b, 0, 0)
        b = QPushButton("GET_STATUS")
        b.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_GET_STATUS])))
        g.addWidget(b, 0, 1)
        b = QPushButton("PING")
        b.clicked.connect(lambda: self.send_cmd(
            bytes([P.CMD_FACTORY_PING, 0xDE, 0xAD, 0xBE, 0xEF])))
        g.addWidget(b, 0, 2)
        self.stat_label = QLabel("—")
        self.stat_label.setWordWrap(True)
        g.addWidget(self.stat_label, 1, 0, 1, 3)
        v.addWidget(gb)
        v.addStretch(1)
        self.tabs.addTab(w, "总览/控制")

    def _tab_imu(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.imu_charts = {}
        for key, nm in (("U4", "U4 主板六轴（30Hz，±8g/±2000dps）"),
                        ("U1", "U1 柔性板六轴")):
            gb = QGroupBox(nm)
            cv = QVBoxLayout(gb)
            acc = LiveChart("加速度 (mg)", window_s=10)
            acc.set_series([(f"a{a}", AXIS_COLORS[i])
                            for i, a in enumerate(AXIS_NAMES)])
            gyr = LiveChart("角速度 (dps)", window_s=10)
            gyr.set_series([(f"g{a}", AXIS_COLORS[i])
                            for i, a in enumerate(AXIS_NAMES)])
            cv.addWidget(acc, 1)
            cv.addWidget(gyr, 1)
            self.imu_charts[key] = (acc, gyr)
            v.addWidget(gb)
        self.tabs.addTab(w, "六轴")

    def _tab_qvar(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.qvar_chart = LiveChart("QVAR raw (LSB)", window_s=15)
        self.qvar_chart.set_series([("QVAR-A", "#e74c3c"),
                                    ("QVAR-B", "#3498db")])
        v.addWidget(self.qvar_chart, 1)

        gb = QGroupBox("阈值设定（QVAR_THR_SET 0x08）")
        g = QGridLayout(gb)
        g.addWidget(QLabel("通道"), 0, 0)
        self.qvar_ch = QComboBox()
        self.qvar_ch.addItems(["A", "B"])
        g.addWidget(self.qvar_ch, 0, 1)
        g.addWidget(QLabel("阈值 |Δraw|"), 0, 2)
        self.qvar_thr = QSpinBox()
        self.qvar_thr.setRange(1, 32767)
        self.qvar_thr.setValue(500)
        g.addWidget(self.qvar_thr, 0, 3)
        g.addWidget(QLabel("滞回"), 0, 4)
        self.qvar_hyst = QSpinBox()
        self.qvar_hyst.setRange(0, 255)
        self.qvar_hyst.setValue(16)
        g.addWidget(self.qvar_hyst, 0, 5)
        b = QPushButton("设定")
        b.clicked.connect(self.on_qvar_thr)
        g.addWidget(b, 0, 6)
        self.qvar_trig = QLabel("触发：—")
        g.addWidget(self.qvar_trig, 1, 0, 1, 7)
        v.addWidget(gb)
        self.tabs.addTab(w, "QVAR 静电")

    def _tab_pvdf(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.pvdf_chart = LiveChart("PVDF (原始码)", window_s=10)
        self.pvdf_chart.set_series([("HEART", "#e74c3c"), ("RAW", "#27ae60"),
                                    ("REF", "#7f8c8d")])
        v.addWidget(self.pvdf_chart, 1)
        self.pvdf_label = QLabel("heart=— raw=— ref=—")
        v.addWidget(self.pvdf_label)
        self.tabs.addTab(w, "PVDF 压电")

    def _tab_mic(self):
        w = QWidget()
        v = QVBoxLayout(w)

        gb = QGroupBox("录音控制（REC_CTRL 0x07）")
        g = QGridLayout(gb)
        g.addWidget(QLabel("声道"), 0, 0)
        self.rec_ch = QComboBox()
        self.rec_ch.addItems(["立体声(L+R)", "仅左", "仅右"])
        g.addWidget(self.rec_ch, 0, 1)
        g.addWidget(QLabel("时长(s)"), 0, 2)
        self.rec_dur = QSpinBox()
        self.rec_dur.setRange(0, 600)
        self.rec_dur.setValue(10)
        self.rec_dur.setSpecialValueText("手动停")
        g.addWidget(self.rec_dur, 0, 3)
        self.btn_rec_start = QPushButton("开始录音")
        self.btn_rec_start.clicked.connect(self.on_rec_start)
        self.btn_rec_stop = QPushButton("停止")
        self.btn_rec_stop.clicked.connect(
            lambda: self.send_cmd(bytes([P.CMD_REC_CTRL, 0x00])))
        g.addWidget(self.btn_rec_start, 0, 4)
        g.addWidget(self.btn_rec_stop, 0, 5)
        self.rec_label = QLabel("空闲")
        g.addWidget(self.rec_label, 1, 0, 1, 6)
        v.addWidget(gb)

        gb = QGroupBox("录音文件（LIST 0x09 / READ 0x0A / DELETE 0x0B）")
        g = QGridLayout(gb)
        self.file_list = QListWidget()
        self.file_list.setMaximumHeight(140)
        g.addWidget(self.file_list, 0, 0, 1, 4)
        b = QPushButton("刷新列表")
        b.clicked.connect(lambda: self.send_cmd(bytes([P.CMD_REC_LIST])))
        g.addWidget(b, 1, 0)
        b = QPushButton("下载选中")
        b.clicked.connect(self.on_rec_download)
        g.addWidget(b, 1, 1)
        b = QPushButton("删除选中")
        b.clicked.connect(self.on_rec_delete)
        g.addWidget(b, 1, 2)
        b = QPushButton("播放已下载")
        b.clicked.connect(self.on_play_wav)
        g.addWidget(b, 1, 3)
        self.xfer_bar = QProgressBar()
        self.xfer_bar.setRange(0, 100)
        g.addWidget(self.xfer_bar, 2, 0, 1, 4)
        self.xfer_label = QLabel("—")
        g.addWidget(self.xfer_label, 3, 0, 1, 4)
        v.addWidget(gb)

        self.mic_chart = LiveChart("麦克风 RMS", window_s=15)
        self.mic_chart.set_series([("L", "#e74c3c"), ("R", "#3498db")])
        v.addWidget(self.mic_chart, 1)
        self.tabs.addTab(w, "麦克风/录音")

    def _tab_sd(self):
        w = QWidget()
        v = QVBoxLayout(w)
        gb = QGroupBox("SD 卡信息（CSNP1GCR01-BOW SD NAND）")
        h = QHBoxLayout(gb)
        self.sd_lamp = Lamp("gray")
        h.addWidget(self.sd_lamp)
        self.sd_info = QLabel("状态未知 — 128MB / SPI 4线 / FAT32 / 内置ECC")
        h.addWidget(self.sd_info, 1)
        v.addWidget(gb)

        gb = QGroupBox("SD 自测（SD_TEST 0x0D：写图样→读回→逐块校验+测速）")
        g = QGridLayout(gb)
        g.addWidget(QLabel("大小(MB)"), 0, 0)
        self.sd_size = QSpinBox()
        self.sd_size.setRange(1, 16)
        self.sd_size.setValue(1)
        g.addWidget(self.sd_size, 0, 1)
        self.sd_verify = QCheckBox("读回校验")
        self.sd_verify.setChecked(True)
        g.addWidget(self.sd_verify, 0, 2)
        b = QPushButton("开始测试")
        b.clicked.connect(self.on_sd_test)
        g.addWidget(b, 0, 3)
        self.sd_result = QLabel("—")
        self.sd_result.setWordWrap(True)
        g.addWidget(self.sd_result, 1, 0, 1, 4)
        v.addWidget(gb)
        v.addStretch(1)
        self.tabs.addTab(w, "SD 卡")

    def _tab_monitor(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.mon_table = QTableWidget(0, 4)
        self.mon_table.setHorizontalHeaderLabels(
            ["时间", "方向", "类型", "载荷(hex)"])
        self.mon_table.horizontalHeader().setSectionResizeMode(
            3, QHeaderView.Stretch)
        v.addWidget(self.mon_table, 1)
        h = QHBoxLayout()
        self.mon_pause = QCheckBox("暂停")
        h.addWidget(self.mon_pause)
        b = QPushButton("清空")
        b.clicked.connect(lambda: self.mon_table.setRowCount(0))
        h.addWidget(b)
        self.mon_stat = QLabel("—")
        h.addWidget(self.mon_stat, 1)
        v.addLayout(h)
        self.tabs.addTab(w, "帧监视")

    def _tab_terminal(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.term_log = QTextEdit()
        self.term_log.setReadOnly(True)
        self.term_log.setStyleSheet("font-family:Consolas; font-size:12px;")
        v.addWidget(self.term_log, 1)
        h = QHBoxLayout()
        self.term_input = QLineEdit()
        self.term_input.setPlaceholderText(
            "hex 指令，如: 01 01（LED0 开）/ 7F DE AD BE EF（PING）")
        self.term_input.returnPressed.connect(self.on_term_send)
        h.addWidget(self.term_input, 1)
        b = QPushButton("发送")
        b.clicked.connect(self.on_term_send)
        h.addWidget(b)
        v.addLayout(h)
        quick = QHBoxLayout()
        for label, hexs in (("LED0开", "01 01"), ("LED全关", "01 00 01 00"),
                            ("GET_STATUS", "10"), ("GET_VERSION", "11"),
                            ("GET_BATTERY", "0C"), ("REC_LIST", "09"),
                            ("SD_TEST 1MB", "0D 01 01"),
                            ("PING", "7F DE AD BE EF")):
            b = QPushButton(label)
            b.clicked.connect(
                lambda _=0, s=hexs: self.send_cmd(bytes.fromhex(s)))
            quick.addWidget(b)
        quick.addStretch(1)
        v.addLayout(quick)
        self.tabs.addTab(w, "指令终端")

    # ================= 后端信号 =================
    def _wire_backend(self):
        b = self.backend
        b.scanned.connect(self.on_scanned)
        b.conn_state.connect(self.on_conn_state)
        b.info_text.connect(
            lambda s: self.info_label.setText("固件信息：" + s))
        b.data_frame.connect(self.on_frame)
        b.ack_frame.connect(self.on_ack)
        b.event_frame.connect(self.on_event)
        b.log.connect(self.term_log.append)
        b.bad_crc.connect(
            lambda n: self.mon_stat.setText(
                f"CRC错误 {n}  seq跳变 {self.seq_gaps}"))

    # ================= 指令 =================
    def send_cmd(self, data: bytes):
        self.backend.send_cmd(data)

    def on_scan(self):
        self.dev_list.clear()
        self.backend.scan()

    def on_scanned(self, devs):
        self.dev_list.clear()
        for d in devs:
            self.dev_list.addItem(
                f"{d['name']}  {d['address']}  {d['rssi']}dBm")
        for i, d in enumerate(devs):
            if d["name"] == P.DEVICE_NAME:
                self.dev_list.setCurrentRow(i)
                break

    def on_connect(self):
        it = self.dev_list.currentItem()
        if not it:
            QMessageBox.warning(self, "未选择", "请先扫描并选择设备")
            return
        addr = it.text().split()[1]
        self.backend.connect_device(addr)

    def on_conn_state(self, ok, msg):
        self.conn_lamp.set_color("green" if ok else "gray")
        self.conn_label.setText(msg)
        if not ok:
            self.rec_state = 0
            self.rec_label.setText("空闲")

    def on_sensor_en(self):
        mask = 0
        for sid, ck in self.sen_checks.items():
            if ck.isChecked():
                mask |= 1 << (sid - 1)
        self.send_cmd(bytes([P.CMD_SENSOR_EN, mask]))

    def on_rate_set(self):
        sid = self.rate_target.currentIndex() + 1
        hz = self.rate_hz.value()
        self.send_cmd(bytes([P.CMD_RATE_SET, sid,
                             hz & 0xFF, (hz >> 8) & 0xFF]))

    def on_qvar_thr(self):
        ch = self.qvar_ch.currentIndex()
        thr = self.qvar_thr.value()
        hyst = self.qvar_hyst.value()
        self.send_cmd(bytes([P.CMD_QVAR_THR_SET, ch,
                             thr & 0xFF, (thr >> 8) & 0xFF, hyst]))

    def on_rec_start(self):
        ch = [0x03, 0x01, 0x02][self.rec_ch.currentIndex()]
        dur = self.rec_dur.value()
        self.send_cmd(bytes([P.CMD_REC_CTRL, 0x01, ch,
                             dur & 0xFF, (dur >> 8) & 0xFF]))

    def on_rec_download(self):
        it = self.file_list.currentItem()
        if not it:
            return
        fid = int(it.text().split()[0])
        self.xfer = {"fid": fid, "chunks": [], "total": 0, "done": False}
        self.xfer_bar.setValue(0)
        self.xfer_label.setText(f"下载 REC{fid:04d}.wav ...")
        self.send_cmd(bytes([P.CMD_REC_READ, fid & 0xFF, (fid >> 8) & 0xFF]))

    def on_rec_delete(self):
        it = self.file_list.currentItem()
        if not it:
            return
        fid = int(it.text().split()[0])
        self.send_cmd(bytes([P.CMD_REC_DELETE, fid & 0xFF,
                             (fid >> 8) & 0xFF]))

    def on_play_wav(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "选择已下载 WAV", self._dl_dir(), "WAV (*.wav)")
        if not path:
            return
        try:
            from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
            from PySide6.QtCore import QUrl
            if not hasattr(self, "_player"):
                self._player = QMediaPlayer(self)
                self._audio = QAudioOutput(self)
                self._player.setAudioOutput(self._audio)
            self._player.setSource(QUrl.fromLocalFile(path))
            self._player.play()
        except ImportError:
            os.startfile(path)  # noqa: S606  回退系统播放器

    def on_sd_test(self):
        self.sd_result.setText("测试进行中（设备侧执行，完成后 EVENT 上报）...")
        self.send_cmd(bytes([P.CMD_SD_TEST, self.sd_size.value(),
                             1 if self.sd_verify.isChecked() else 0]))

    def on_term_send(self):
        s = self.term_input.text().strip()
        if not s:
            return
        try:
            self.send_cmd(bytes.fromhex(s))
        except ValueError:
            self.term_log.append("!! hex 解析失败: " + s)
        self.term_input.clear()

    # ================= 帧处理 =================
    def on_frame(self, ftype, seq, payload: bytes):
        now = time.monotonic() - self.t_start
        if ftype in self.last_seq and seq != (self.last_seq[ftype] + 1) % 256:
            self.seq_gaps += 1
        self.last_seq[ftype] = seq
        self.frame_counts[ftype] += 1

        if not self.mon_pause.isChecked() and ftype != P.T_AUDIO_FILE:
            self._mon_add(now, "RX", P.TYPE_NAMES.get(ftype, f"0x{ftype:02X}"),
                          P.fmt_hex(payload))

        if ftype in (P.T_IMU_U4, P.T_BODY_IMU_U1):
            self._handle_imu(ftype, payload)
        elif ftype == P.T_QVAR:
            self._handle_qvar(payload)
        elif ftype == P.T_PVDF:
            self._handle_pvdf(payload)
        elif ftype == P.T_TEMP:
            pass  # 温度经模块状态/统计展示，帧监视可见
        elif ftype == P.T_MIC:
            if len(payload) >= 5:
                self.mic_chart.add_point([u16(payload, 1), u16(payload, 3)])
        elif ftype == P.T_BATTERY:
            if len(payload) >= 5:
                self.batt.update_batt(u16(payload, 1), payload[3], payload[4])
        elif ftype == P.T_MODULE_STATUS:
            self._handle_mod_status(payload)
        elif ftype == P.T_AUDIO_FILE:
            self._handle_file_chunk(payload)

    def _handle_imu(self, ftype, payload):
        key = "U4" if ftype == P.T_IMU_U4 else "U1"
        acc_c, gyr_c = self.imu_charts[key]
        if (payload[0] & 0x0F) != 1:   # 非 PRESENT
            return
        body = payload[1:]
        n = len(body) // 12
        for s in range(n):
            o = s * 12
            acc = [i16(body, o + k * 2) * ACC_SCALE_MG for k in range(3)]
            gyr = [i16(body, o + 6 + k * 2) / 1000.0 * GYRO_SCALE_MDPS
                   for k in range(3)]
            acc_c.add_point(acc)
            gyr_c.add_point(gyr)

    def _handle_qvar(self, payload):
        if len(payload) >= 6:
            valid = payload[5]
            a = i16(payload, 1) if valid & 0x01 else None
            b = i16(payload, 3) if valid & 0x02 else None
            self.qvar_chart.add_point([a, b])
        elif len(payload) >= 5:
            self.qvar_chart.add_point([i16(payload, 1), i16(payload, 3)])

    def _handle_pvdf(self, payload):
        if len(payload) < 13:
            return
        h, r, ref = i32(payload, 1), i32(payload, 5), i32(payload, 9)
        self.pvdf_chart.add_point([h, r, ref])
        self.pvdf_label.setText(f"heart={h}  raw={r}  ref={ref}")

    def _handle_mod_status(self, st):
        names = ["U4六轴", "U1六轴", "QVAR", "SD卡", "PVDF", "麦克风"]
        idxs = [0, 1, 2, 5, 6, 7]
        color = {0: "gray", 1: "green", 2: "orange", 3: "gray"}
        for nm, ix in zip(names, idxs):
            if ix < len(st):
                v = st[ix]
                self.mod_lamps[nm].set_color(color.get(v, "gray"))
                self.mod_labels[nm].setText(P.MOD_STATES.get(v, "?"))
        if len(st) > 5:
            sd = st[5]
            self.sd_lamp.set_color(color.get(sd, "gray"))
            self.sd_info.setText(
                f"{P.MOD_STATES.get(sd, '?')} — 128MB / SPI 4线 / FAT32 / 内置ECC")

    def _handle_file_chunk(self, payload):
        if not self.xfer or len(payload) < 6:
            return
        fid = u16(payload, 0)
        off = u32(payload, 2)
        if fid == self.xfer["fid"]:
            self.xfer["chunks"].append((off, payload[6:]))
            got = sum(len(d) for _, d in self.xfer["chunks"])
            if self.xfer["total"]:
                self.xfer_bar.setValue(min(100, got * 100 // self.xfer["total"]))
            self.xfer_label.setText(f"下载中 REC{fid:04d}.wav  {got} B")

    # ================= ACK / EVENT =================
    def on_ack(self, p: bytes):
        cmd = p[0]
        result = p[2] - 256 if p[2] > 127 else p[2]
        name = P.CMD_NAMES.get(cmd, f"0x{cmd:02X}")
        self._mon_add(time.monotonic() - self.t_start, "ACK",
                      name, P.fmt_hex(p))
        self.term_log.append(f"ACK {name} result={result} "
                             + (P.fmt_hex(p[3:]) if len(p) > 3 else ""))
        if cmd == P.CMD_REC_LIST and result == 0:
            self.file_list.clear()
            for o in range(3, len(p) - 5, 6):
                fid = u16(p, o)
                size = u32(p, o + 2)
                self.file_list.addItem(f"{fid}  REC{fid:04d}.wav  {size} B")
        elif cmd == P.CMD_SD_TEST and result != 0:
            self.sd_result.setText(
                f"下发失败 result={result}（{P.SD_TEST_ERRORS.get(result, '?')}）")

    def on_event(self, p: bytes):
        eid = p[0]
        name = P.EVENT_NAMES.get(eid, f"0x{eid:02X}")
        self._mon_add(time.monotonic() - self.t_start, "EVT",
                      name, P.fmt_hex(p))
        self.term_log.append(f"EVENT {name} {P.fmt_hex(p[1:])}")
        if eid == P.EV_REC_STATE and len(p) >= 5:
            state, fid, el = p[1], u16(p, 2), p[4]
            self.rec_state = state
            self.rec_file_id = fid
            txt = {0: "空闲", 1: f"录音中 REC{fid:04d} {el}s",
                   2: f"录音完成 REC{fid:04d}.wav"}.get(state, str(state))
            self.rec_label.setText(txt)
            if state == 2:
                self.send_cmd(bytes([P.CMD_REC_LIST]))  # 自动刷新列表
        elif eid == P.EV_REC_FILE_DONE and len(p) >= 11:
            fid, total, crc = u16(p, 1), u32(p, 3), u32(p, 7)
            self._finish_xfer(fid, total, crc)
        elif eid == P.EV_SD_TEST_DONE and len(p) >= 8:
            res = p[1] - 256 if p[1] > 127 else p[1]
            w, r, bad = u16(p, 2), u16(p, 4), u16(p, 6)
            txt = (f"{P.SD_TEST_ERRORS.get(res, res)}  "
                   f"写 {w} KB/s  读 {r} KB/s  错误块 {bad}")
            self.sd_result.setText(txt)
        elif eid == P.EV_QVAR_THR and len(p) >= 4:
            ch = "A" if p[1] == 0 else "B"
            self.qvar_trig.setText(
                f"触发：QVAR-{ch} raw={i16(p, 2)} @ "
                + time.strftime("%H:%M:%S"))
        elif eid == P.EV_LOW_BATT:
            self.term_log.append("!! 低电告警 EVENT")

    def _finish_xfer(self, fid, total, crc):
        if not self.xfer or self.xfer["fid"] != fid:
            return
        blob = b"".join(d for _, d in sorted(self.xfer["chunks"],
                                             key=lambda x: x[0]))
        ok_len = len(blob) == total
        ok_crc = (zlib.crc32(blob) & 0xFFFFFFFF) == crc
        if ok_len and ok_crc:
            path = os.path.join(self._dl_dir(), f"REC{fid:04d}.wav")
            with open(path, "wb") as f:
                f.write(blob)
            self.xfer_label.setText(
                f"下载完成 {total} B，CRC32 校验通过 → {path}")
            self.term_log.append(f"REC_READ 完成: {path}")
        else:
            self.xfer_label.setText(
                f"校验失败 len={len(blob)}/{total} crc_ok={ok_crc}，"
                f"可重新下载（断点续传由设备 offset 参数支持）")
        self.xfer_bar.setValue(100 if ok_len and ok_crc else 0)
        self.xfer = None

    # ================= 杂项 =================
    def _mon_add(self, t, direction, typ, payload_hex):
        row = self.mon_table.rowCount()
        self.mon_table.insertRow(row)
        self.mon_table.setItem(row, 0, QTableWidgetItem(f"{t:8.2f}"))
        self.mon_table.setItem(row, 1, QTableWidgetItem(direction))
        self.mon_table.setItem(row, 2, QTableWidgetItem(typ))
        self.mon_table.setItem(row, 3, QTableWidgetItem(payload_hex))
        if self.mon_table.rowCount() > 800:
            self.mon_table.removeRow(0)
        self.mon_table.scrollToBottom()

    def _refresh_stats(self):
        c = self.frame_counts
        parts = [f"{P.TYPE_NAMES.get(k, hex(k))}={v}"
                 for k, v in sorted(c.items())]
        self.stat_label.setText(
            "帧计数: " + ("  ".join(parts) if parts else "暂无数据")
            + f"   |   seq 跳变 {self.seq_gaps}")

    def _dl_dir(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "downloads")
        os.makedirs(d, exist_ok=True)
        return d

    def closeEvent(self, e):
        self.backend.disconnect_device()
        self.backend.stop()
        self.backend.wait(2000)
        e.accept()


def main():
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
