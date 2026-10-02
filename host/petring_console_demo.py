#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环传感器控制 —— 上位机 DEMO（模拟数据版，无真实 BLE）

用途：评审 GUI 布局与功能集。所有数据由内置模拟器产生，
点击"连接"后开始出数；界面元素与《宠物环传感器控制_通信协议》V0.3 一一对应。

运行：py host/petring_console_demo.py   或  host\\run_demo.bat
（依赖标准库 tkinter：python.org 安装的系统 Python 自带；
 WorkBuddy 托管 Python 无 tkinter，请勿用其运行）

功能映射：
  连接面板      ↔ 扫描/连接/MTU/RSSI
  电池电量      ↔ TYPE 0x08 BATTERY + CMD 0x0C GET_BATTERY + EVENT 0x02 低电
                （nPM1300 库仑计 SOC：SOC 积分为主、电压经 OCV 曲线反查）
  模块状态灯    ↔ MODULE_STATUS 帧 0x10（UNKNOWN/PRESENT/ABSENT/DEGRADED）
  LED 控制      ↔ CMD 0x01 LED_SET
  电源域控制    ↔ CMD 0x02 PWR_SET（单开/单关/全开/全关测试矩阵）
  六轴×2        ↔ TYPE 0x01/0x02（U1 柔性板可演示 ABSENT 灰显）
  QVAR×2        ↔ TYPE 0x03 + CMD 0x08 QVAR_THR_SET + EVENT 0x04 阈值触发
  PVDF          ↔ TYPE 0x04 heart/raw/ref mV + 波形
  麦克风录音    ↔ CMD 0x07 REC_CTRL（声道/时长/启停）+ 0x0A REC_READ 下载后本地播放
  帧监视        ↔ e5a00021 Notify 原始帧（SYNC/TYPE/SEQ/LEN/PAYLOAD/CRC8）
  指令终端      ↔ e5a00022 Write + e5a00023 CMD_ACK
"""

import math
import os
import random
import struct
import tempfile
import time
import tkinter as tk
import wave
from tkinter import ttk

# ---------------------------------------------------------------- 模拟链路

class SimLink:
    """模拟设备端：产生各传感器数据、维护电源/LED/录音状态。"""

    def __init__(self):
        self.connected = False
        self.t0 = time.time()
        self.led = [False, False]                    # LED0 红, LED1 蓝
        self.pwr = {"SENS": True, "STORE": True, "ANALOG": False}
        # 模块连接状态：0=UNKNOWN 1=PRESENT 2=ABSENT 3=DEGRADED
        self.mod_state = {
            "IMU_U4": 1, "IMU_U1": 2, "QVAR_A": 2, "QVAR_B": 1,
            "PVDF": 1, "TEMP": 2, "MIC": 1, "SD": 1,
        }
        self.qvar_thr = {"A": 3000, "B": 3000}       # 阈值 |Δraw|
        self.qvar_alarm = {"A": False, "B": False}
        self.recording = False
        self.rec_elapsed = 0.0
        self.rec_duration = 10
        self.rec_channels = (True, True)             # L, R
        self.rec_file = None                          # 录完生成的 wav 路径
        self.seq = {}                                 # 每 TYPE 独立序号
        # ---- 电池（模拟 nPM1300 库仑计 SOC，协议 V0.3 TYPE 0x08 BATTERY）----
        # 算法语义对齐固件 nrf_fuel_gauge：SOC 为状态量（电流积分），
        # 电压由 SOC 经锂电开路电压曲线反查（非线性平台期），不再用电压线性映射。
        self.batt_soc = 83.0                          # SOC %（0~100 浮点）
        self.batt_charging = False                    # 充电中标志
        self.batt_low_warned = False                  # 低电 EVENT 去抖

    BATT_LOW_PCT = 15                                 # 低电告警门限 %
    # 锂电典型开路电压曲线（SOC% → mV），分段线性插值
    _OCV_TABLE = ((0, 3300), (5, 3450), (10, 3550), (20, 3650),
                  (50, 3780), (80, 3980), (95, 4120), (100, 4200))

    def batt_percent(self):
        return max(0, min(100, int(round(self.batt_soc))))

    def batt_voltage_mv(self):
        """由 SOC 反查开路电压（模拟 nrf_fuel_gauge 的 vbat 输出）。"""
        s = max(0.0, min(100.0, self.batt_soc))
        tab = self._OCV_TABLE
        for i in range(1, len(tab)):
            if s <= tab[i][0]:
                s0, v0 = tab[i - 1]
                s1, v1 = tab[i]
                return int(v0 + (v1 - v0) * (s - s0) / (s1 - s0))
        return tab[-1][1]

    def battery_sample(self, dt_s):
        """SOC 电流积分模拟：充电 +0.14%/s（约 12 分钟+10%），
        放电 -0.03%/s 基础 + 负载加权（电源域/LED 越多掉电越快）。
        返回 (vbat_mv, percent, charging)。"""
        if self.batt_charging:
            self.batt_soc = min(100.0, self.batt_soc + 0.14 * dt_s)
        else:
            load = sum(self.pwr.values()) * 0.012 + sum(self.led) * 0.005
            self.batt_soc = max(0.0, self.batt_soc - (0.03 + load) * dt_s)
        return self.batt_voltage_mv(), self.batt_percent(), self.batt_charging

    # ---- 传感器模拟 ----
    def imu_frame(self, base_pitch):
        t = time.time() - self.t0
        ax = int(800 * math.sin(2 * math.pi * 0.5 * t + base_pitch) + random.gauss(0, 20))
        ay = int(600 * math.sin(2 * math.pi * 0.35 * t) + random.gauss(0, 20))
        az = int(1000 + random.gauss(0, 15))
        gx = int(250 * math.sin(2 * math.pi * 0.5 * t) + random.gauss(0, 8))
        gy = int(200 * math.cos(2 * math.pi * 0.4 * t) + random.gauss(0, 8))
        gz = int(random.gauss(0, 10))
        return (ax, ay, az, gx, gy, gz)

    def qvar_sample(self):
        t = time.time() - self.t0
        a = None
        if self.mod_state["QVAR_A"] == 1:
            a = int(4000 * math.sin(2 * math.pi * 0.2 * t) + random.gauss(0, 150))
        b = int(-13000 + 2500 * math.sin(2 * math.pi * 0.13 * t) + random.gauss(0, 120))
        for ch, v in (("A", a), ("B", b)):
            if v is not None:
                self.qvar_alarm[ch] = abs(v) > self.qvar_thr[ch]
        return a, b

    def pvdf_sample(self):
        t = time.time() - self.t0
        if not self.pwr["ANALOG"]:
            return None
        beat = 180 * math.exp(-((t % 1.0) ** 2) / 0.002)      # 模拟心拍尖峰 1Hz
        ref = 1488 + random.gauss(0, 1)
        heart = ref + beat + 30 * math.sin(2 * math.pi * 5 * t) + random.gauss(0, 3)
        raw = ref + beat * 1.4 + random.gauss(0, 5)
        return (int(heart), int(raw), int(ref))

    def next_seq(self, ftype):
        self.seq[ftype] = (self.seq.get(ftype, -1) + 1) & 0xFF
        return self.seq[ftype]

# ---------------------------------------------------------------- 波形控件

class WaveCanvas(tk.Canvas):
    """单通道波形画布，环形缓冲，支持可选阈值线。"""

    def __init__(self, master, title="", unit="", ymin=-100, ymax=100,
                 width=340, height=110, color="#1a76d2", **kw):
        super().__init__(master, width=width, height=height,
                         bg="#101418", highlightthickness=1,
                         highlightbackground="#3a4148", **kw)
        self.unit = unit
        self.ymin, self.ymax = ymin, ymax
        self.color = color
        self.w, self.h = width, height
        self.data = []
        self.thr_lines = []          # 阈值线 y 值列表
        self.title = title
        self._draw_frame()

    def set_range(self, ymin, ymax):
        self.ymin, self.ymax = ymin, ymax

    def set_thresholds(self, values):
        self.thr_lines = list(values)

    def push(self, v):
        self.data.append(v)
        if len(self.data) > self.w:
            self.data.pop(0)

    def clear(self):
        self.data.clear()
        self._draw_frame()

    def _draw_frame(self):
        self.delete("all")
        self.create_text(6, 4, anchor="nw", fill="#9aa4ad",
                         font=("微软雅黑", 8), text=self.title)
        if self.unit:
            self.create_text(self.w - 6, 4, anchor="ne", fill="#9aa4ad",
                             font=("微软雅黑", 8), text=self.unit)

    def redraw(self):
        self._draw_frame()
        span = (self.ymax - self.ymin) or 1

        def y_of(v):
            v = max(self.ymin, min(self.ymax, v))
            return self.h - 8 - (v - self.ymin) / span * (self.h - 24)

        # 零线
        if self.ymin < 0 < self.ymax:
            y0 = y_of(0)
            self.create_line(0, y0, self.w, y0, fill="#2c343c")
        # 阈值线
        for tv in self.thr_lines:
            y = y_of(tv)
            self.create_line(0, y, self.w, y, fill="#c0392b", dash=(4, 3))
        # 波形
        n = len(self.data)
        if n >= 2:
            pts = []
            x0 = self.w - n
            for i, v in enumerate(self.data):
                pts += [x0 + i, y_of(v)]
            self.create_line(*pts, fill=self.color, width=1.4)

# ---------------------------------------------------------------- 主窗口

class PetRingConsole(tk.Tk):
    BG = "#1b1f24"
    FG = "#d7dde3"
    ACCENT = "#1a76d2"

    def __init__(self):
        super().__init__()
        self.title("宠物环传感器控制  v0.1 DEMO（模拟数据）")
        self.configure(bg=self.BG)
        self.geometry("1180x760")
        self.link = SimLink()
        self._style()
        self._build_top()
        self._build_left_status()
        self._build_control_area()
        self._build_tabs()
        self._build_log()
        self._tick()

    # ---------------- 样式 ----------------
    def _style(self):
        st = ttk.Style(self)
        st.theme_use("clam")
        st.configure("TNotebook", background=self.BG, borderwidth=0)
        st.configure("TNotebook.Tab", padding=(14, 6), font=("微软雅黑", 10))
        st.configure("TFrame", background=self.BG)
        st.configure("TLabel", background=self.BG, foreground=self.FG,
                     font=("微软雅黑", 10))
        st.configure("Header.TLabel", font=("微软雅黑", 11, "bold"),
                     foreground="#ffffff")
        st.configure("TButton", font=("微软雅黑", 9), padding=(10, 4))
        st.configure("TCheckbutton", background=self.BG, foreground=self.FG)

    # ---------------- 顶部连接栏 ----------------
    def _build_top(self):
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=10, pady=(8, 4))
        ttk.Label(bar, text="设备:", style="Header.TLabel").pack(side="left")
        self.dev_var = tk.StringVar(value="SmartPet")
        ttk.Combobox(bar, textvariable=self.dev_var, width=14, state="readonly",
                     values=["SmartPet"]).pack(side="left", padx=(4, 12))
        self.btn_scan = ttk.Button(bar, text="扫描", command=self.on_scan)
        self.btn_scan.pack(side="left")
        self.btn_conn = ttk.Button(bar, text="连接", command=self.on_connect)
        self.btn_conn.pack(side="left", padx=6)
        self.lbl_mtu = ttk.Label(bar, text="MTU: —")
        self.lbl_mtu.pack(side="left", padx=(18, 6))
        self.lbl_rssi = ttk.Label(bar, text="RSSI: —")
        self.lbl_rssi.pack(side="left", padx=6)
        # --- 电池电量（协议 V0.3 TYPE 0x08 BATTERY，源：nPM1300 电量计）---
        batt_box = ttk.Frame(bar)
        batt_box.pack(side="left", padx=(18, 4))
        self.batt_canvas = tk.Canvas(batt_box, width=46, height=18,
                                     bg=self.BG, highlightthickness=0)
        self.batt_canvas.pack(side="left")
        self.lbl_batt = ttk.Label(batt_box, text="—")
        self.lbl_batt.pack(side="left", padx=(4, 2))
        self.btn_chg = ttk.Button(batt_box, text="模拟插/拔充电器",
                                  command=self.on_toggle_charge)
        self.btn_chg.pack(side="left", padx=(8, 0))
        self.lbl_conn = tk.Label(bar, text="● 未连接", fg="#e74c3c",
                                 bg=self.BG, font=("微软雅黑", 11, "bold"))
        self.lbl_conn.pack(side="right")

    def on_toggle_charge(self):
        self.link.batt_charging = not self.link.batt_charging
        self._log("INFO", "充电器 " + ("已插入，开始充电" if self.link.batt_charging
                                       else "已拔除，电池供电"))

    def _draw_battery(self, mv, pct, charging):
        c = self.batt_canvas
        c.delete("all")
        # 电池外壳 + 正极头
        c.create_rectangle(1, 3, 40, 15, outline="#9aa4ad", width=1)
        c.create_rectangle(41, 7, 45, 11, fill="#9aa4ad")
        # 电量填充：>50 绿 / 20~50 黄 / <20 红
        color = "#2ecc71" if pct > 50 else ("#f1c40f" if pct > 20 else "#e74c3c")
        fillw = int(37 * pct / 100)
        if fillw > 0:
            c.create_rectangle(3, 5, 3 + fillw, 13, fill=color, width=0)
        if charging:
            c.create_text(20, 9, text="⚡", fill="#ffffff",
                          font=("微软雅黑", 8, "bold"))
        state = "充电中" if charging else "放电"
        self.lbl_batt.configure(
            text=f"{pct}%  {mv}mV  {state}",
            foreground=color)

    # ---------------- 左侧模块状态 ----------------
    def _build_left_status(self):
        self.left = ttk.Frame(self)
        self.left.pack(side="left", fill="y", padx=(10, 4), pady=4)
        ttk.Label(self.left, text="模块状态", style="Header.TLabel").pack(anchor="w")
        self.mod_labels = {}
        names = [("IMU_U4", "主板六轴 U4"), ("IMU_U1", "柔性板六轴 U1"),
                 ("QVAR_A", "静电 A（U1）"), ("QVAR_B", "静电 B（U4）"),
                 ("PVDF", "压电膜链路"), ("TEMP", "温度 U2"),
                 ("MIC", "双麦克风"), ("SD", "SD 卡")]
        box = ttk.Frame(self.left)
        box.pack(fill="x", pady=4)
        for key, cn in names:
            row = ttk.Frame(box)
            row.pack(fill="x", pady=1)
            lamp = tk.Label(row, text="●", fg="#7f8c8d", bg=self.BG,
                            font=("微软雅黑", 12))
            lamp.pack(side="left")
            ttk.Label(row, text=f" {cn}").pack(side="left")
            self.mod_labels[key] = lamp
        ttk.Label(self.left, text="图例：绿=在位 灰=不在位 橙=降级",
                  font=("微软雅黑", 8)).pack(anchor="w", pady=(2, 8))
        self.btn_fpc = ttk.Button(self.left, text="模拟插/拔柔性板 FPC",
                                  command=self.on_toggle_fpc)
        self.btn_fpc.pack(fill="x", pady=4)
        self._refresh_mod_lamps()

    def _refresh_mod_lamps(self):
        colors = {0: "#7f8c8d", 1: "#2ecc71", 2: "#5d6d7e", 3: "#e67e22"}
        for k, lamp in self.mod_labels.items():
            lamp.configure(fg=colors[self.link.mod_state[k]])

    # ---------------- 右侧控制区（LED/电源/录音） ----------------
    def _build_control_area(self):
        self.ctrl = ttk.Frame(self)
        self.ctrl.pack(side="top", fill="x", padx=6, pady=4)

        # --- LED ---
        led_f = ttk.LabelFrame(self.ctrl, text=" LED 控制 ", padding=6)
        led_f.pack(side="left", fill="y", padx=4)
        self.led_lamps = []
        for i, (name, color) in enumerate((("LED0（红）", "#e74c3c"),
                                           ("LED1（蓝）", "#3498db"))):
            row = ttk.Frame(led_f)
            row.pack(fill="x", pady=2)
            lamp = tk.Label(row, text="●", fg="#3a4148", bg=self.BG,
                            font=("微软雅黑", 13))
            lamp.pack(side="left")
            self.led_lamps.append((lamp, color))
            ttk.Label(row, text=f" {name}").pack(side="left", padx=(0, 8))
            ttk.Button(row, text="开", width=4,
                       command=lambda i=i: self.on_led(i, True)).pack(side="left")
            ttk.Button(row, text="关", width=4,
                       command=lambda i=i: self.on_led(i, False)).pack(side="left",
                                                                       padx=(3, 0))

        # --- 电源域 ---
        pwr_f = ttk.LabelFrame(self.ctrl, text=" 电源域控制（SENS/STORE/ANALOG） ",
                               padding=6)
        pwr_f.pack(side="left", fill="y", padx=8)
        self.pwr_lamps = {}
        for dom in ("SENS", "STORE", "ANALOG"):
            row = ttk.Frame(pwr_f)
            row.pack(fill="x", pady=2)
            lamp = tk.Label(row, text="●", fg="#2ecc71", bg=self.BG,
                            font=("微软雅黑", 13))
            lamp.pack(side="left")
            self.pwr_lamps[dom] = lamp
            ttk.Label(row, text=f" {dom:6s}").pack(side="left", padx=(0, 8))
            ttk.Button(row, text="开", width=4,
                       command=lambda d=dom: self.on_pwr(d, True)).pack(side="left")
            ttk.Button(row, text="关", width=4,
                       command=lambda d=dom: self.on_pwr(d, False)).pack(side="left",
                                                                         padx=(3, 0))
        row = ttk.Frame(pwr_f)
        row.pack(fill="x", pady=(6, 0))
        ttk.Button(row, text="全开", command=lambda: self.on_pwr_all(True)).pack(
            side="left", expand=True, fill="x")
        ttk.Button(row, text="全关", command=lambda: self.on_pwr_all(False)).pack(
            side="left", expand=True, fill="x", padx=(4, 0))

        # --- 录音 ---
        rec_f = ttk.LabelFrame(self.ctrl, text=" 麦克风录音（录至 SD，下载后本地播放） ",
                               padding=6)
        rec_f.pack(side="left", fill="y", padx=4)
        row1 = ttk.Frame(rec_f)
        row1.pack(fill="x", pady=2)
        self.ch_l = tk.BooleanVar(value=True)
        self.ch_r = tk.BooleanVar(value=True)
        ttk.Label(row1, text="声道:").pack(side="left")
        ttk.Checkbutton(row1, text="左", variable=self.ch_l).pack(side="left")
        ttk.Checkbutton(row1, text="右", variable=self.ch_r).pack(side="left")
        ttk.Label(row1, text="  时长(s):").pack(side="left")
        self.rec_dur = tk.Spinbox(row1, from_=1, to=60, width=4)
        self.rec_dur.delete(0, "end")
        self.rec_dur.insert(0, "10")
        self.rec_dur.pack(side="left")
        row2 = ttk.Frame(rec_f)
        row2.pack(fill="x", pady=4)
        self.btn_rec_start = ttk.Button(row2, text="● 开始录音",
                                        command=self.on_rec_start)
        self.btn_rec_start.pack(side="left")
        self.btn_rec_stop = ttk.Button(row2, text="■ 停止", state="disabled",
                                       command=self.on_rec_stop)
        self.btn_rec_stop.pack(side="left", padx=4)
        self.btn_rec_play = ttk.Button(row2, text="▶ 播放", state="disabled",
                                       command=self.on_rec_play)
        self.btn_rec_play.pack(side="left")
        self.rec_prog = ttk.Progressbar(rec_f, length=220, mode="determinate")
        self.rec_prog.pack(fill="x", pady=2)
        self.lbl_rec = ttk.Label(rec_f, text="空闲")
        self.lbl_rec.pack(anchor="w")

    # ---------------- 中部页签 ----------------
    def _build_tabs(self):
        self.nb = ttk.Notebook(self)
        self.nb.pack(side="top", fill="both", expand=True, padx=10, pady=4)
        self._tab_imu()
        self._tab_qvar()
        self._tab_pvdf()
        self._tab_frames()
        self._tab_terminal()

    def _tab_imu(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 六轴 IMU ")
        self.imu_vals = {}
        self.imu_waves = {}
        for idx, (key, title) in enumerate((("U4", "主板 U4"), ("U1", "柔性板 U1"))):
            lf = ttk.LabelFrame(tab, text=f" {title} LSM6DSV16X ", padding=6)
            lf.grid(row=0, column=idx, padx=8, pady=6, sticky="n")
            grid = ttk.Frame(lf)
            grid.pack()
            self.imu_vals[key] = []
            axes = [("ax", "mg"), ("ay", "mg"), ("az", "mg"),
                    ("gx", "dps×10"), ("gy", "dps×10"), ("gz", "dps×10")]
            for r, (nm, unit) in enumerate(axes):
                ttk.Label(grid, text=nm).grid(row=r, column=0, sticky="e")
                v = ttk.Label(grid, text="—", width=10, font=("Consolas", 10))
                v.grid(row=r, column=1, sticky="w")
                ttk.Label(grid, text=unit, font=("微软雅黑", 8)).grid(
                    row=r, column=2, sticky="w")
                self.imu_vals[key].append(v)
            wa = WaveCanvas(lf, title=f"{title} 加速度 ax/ay/az", unit="mg",
                            ymin=-2200, ymax=2200, color="#2ecc71")
            wa.pack(pady=2)
            wg = WaveCanvas(lf, title=f"{title} 陀螺仪 gx", unit="dps×10",
                            ymin=-600, ymax=600, color="#e67e22")
            wg.pack(pady=2)
            self.imu_waves[key] = (wa, wg)

    def _tab_qvar(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 静电感应 QVAR ")
        self.qvar_vals = {}
        self.qvar_waves = {}
        self.qvar_lamps = {}
        for idx, ch in enumerate(("A", "B")):
            src = "U1/AQVAR1·2" if ch == "A" else "U4/BQVAR1·2"
            lf = ttk.LabelFrame(tab, text=f" QVAR-{ch}（{src}） ", padding=6)
            lf.grid(row=0, column=idx, padx=8, pady=6, sticky="n")
            row = ttk.Frame(lf)
            row.pack(fill="x")
            ttk.Label(row, text="原始值:").pack(side="left")
            v = ttk.Label(row, text="—", width=10, font=("Consolas", 11, "bold"))
            v.pack(side="left")
            self.qvar_vals[ch] = v
            ttk.Label(row, text="  触发:").pack(side="left")
            lamp = tk.Label(row, text="●", fg="#3a4148", bg=self.BG,
                            font=("微软雅黑", 13))
            lamp.pack(side="left")
            self.qvar_lamps[ch] = lamp
            wc = WaveCanvas(lf, title=f"QVAR-{ch} raw", unit="LSB",
                            ymin=-16000, ymax=16000,
                            color="#9b59b6" if ch == "A" else "#1abc9c")
            wc.pack(pady=4)
            self.qvar_waves[ch] = wc
            row2 = ttk.Frame(lf)
            row2.pack(fill="x", pady=2)
            ttk.Label(row2, text="阈值 |Δraw|:").pack(side="left")
            sp = tk.Spinbox(row2, from_=100, to=30000, increment=100, width=7)
            sp.delete(0, "end")
            sp.insert(0, "3000")
            sp.pack(side="left", padx=4)
            ttk.Button(row2, text="应用（QVAR_THR_SET）",
                       command=lambda c=ch, s=sp: self.on_thr_set(c, s)).pack(
                side="left", padx=4)
            setattr(self, f"thr_spin_{ch}", sp)

    def _tab_pvdf(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 压电膜 PVDF ")
        top = ttk.Frame(tab)
        top.pack(fill="x", pady=4)
        self.pvdf_vals = {}
        for name in ("heart_mv", "raw_mv", "ref_mv"):
            ttk.Label(top, text=f"{name}:").pack(side="left", padx=(10, 2))
            v = ttk.Label(top, text="—", width=8, font=("Consolas", 11, "bold"))
            v.pack(side="left")
            ttk.Label(top, text="mV").pack(side="left")
            self.pvdf_vals[name] = v
        self.pvdf_wave = WaveCanvas(tab, title="heart - ref（去基线差分）",
                                    unit="mV", ymin=-400, ymax=400,
                                    width=760, height=180, color="#e74c3c")
        self.pvdf_wave.pack(pady=4)
        self.pvdf_wave2 = WaveCanvas(tab, title="raw - ref", unit="mV",
                                    ymin=-400, ymax=400,
                                    width=760, height=180, color="#f39c12")
        self.pvdf_wave2.pack(pady=4)
        ttk.Label(tab, text="提示：ANALOG 域断电时链路无输出（波形归零、状态转灰）",
                  font=("微软雅黑", 8)).pack(anchor="w", padx=8)

    def _tab_frames(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 数据帧监视 ")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=2)
        ttk.Button(bar, text="清空", command=lambda: self.frame_txt.delete(
            "1.0", "end")).pack(side="left")
        self.frame_pause = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="暂停滚动",
                        variable=self.frame_pause).pack(side="left", padx=8)
        self.frame_txt = tk.Text(tab, height=18, bg="#101418", fg="#9fd3a8",
                                 font=("Consolas", 9), state="normal")
        self.frame_txt.pack(fill="both", expand=True)

    def _tab_terminal(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 指令终端 ")
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=6)
        ttk.Label(row, text="CMD(hex):").pack(side="left")
        self.cmd_entry = ttk.Entry(row, width=8, font=("Consolas", 10))
        self.cmd_entry.insert(0, "01")
        self.cmd_entry.pack(side="left", padx=4)
        ttk.Label(row, text="PARAMS(hex, 空格分隔):").pack(side="left")
        self.param_entry = ttk.Entry(row, width=30, font=("Consolas", 10))
        self.param_entry.insert(0, "03")
        self.param_entry.pack(side="left", padx=4)
        ttk.Button(row, text="发送", command=self.on_cmd_send).pack(side="left",
                                                                    padx=6)
        ttk.Label(row, text="  快捷:").pack(side="left")
        for label, cmd, params in (("LED0开", "01", "01"), ("LED全关", "01", "00"),
                                   ("GET_STATUS", "10", ""), ("GET_VERSION", "11", ""),
                                   ("PING", "7F", "DE AD BE EF")):
            ttk.Button(row, text=label,
                       command=lambda c=cmd, p=params: self.on_cmd_quick(c, p)
                       ).pack(side="left", padx=2)
        self.term_txt = tk.Text(tab, height=14, bg="#101418", fg="#8ecbff",
                                font=("Consolas", 9))
        self.term_txt.pack(fill="both", expand=True, pady=4)

    # ---------------- 底部日志 ----------------
    def _build_log(self):
        self.log_txt = tk.Text(self, height=4, bg="#14181d", fg="#7fb3d5",
                               font=("Consolas", 9))
        self.log_txt.pack(side="bottom", fill="x", padx=10, pady=(0, 8))
        self.log("DEMO 启动：数据为模拟生成，界面元素对应通信协议 V0.2。")

    def log(self, msg):
        self.log_txt.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_txt.see("end")

    # ---------------- 事件处理 ----------------
    def on_scan(self):
        self.log("扫描到 1 台设备：SmartPet  RSSI=-52dBm  （含 Sensor Hub e5a00020）")

    def on_connect(self):
        lk = self.link
        lk.connected = not lk.connected
        if lk.connected:
            self.btn_conn.configure(text="断开")
            self.lbl_mtu.configure(text="MTU: 247")
            self.lbl_rssi.configure(text="RSSI: -52 dBm")
            self.lbl_conn.configure(text="● 已连接", fg="#2ecc71")
            self.log("已连接 SmartPet，MTU 协商 247，已订阅 0x21/0x23。")
            self.term_print("<< CMD_ACK cmd=0x11 result=0 data='v1.06;task-V1.06;DEMO'")
        else:
            self.btn_conn.configure(text="连接")
            self.lbl_mtu.configure(text="MTU: —")
            self.lbl_rssi.configure(text="RSSI: —")
            self.lbl_conn.configure(text="● 未连接", fg="#e74c3c")
            self.log("已断开。LED override 归还电源 UI 状态机（协议 §7.4）。")

    def on_toggle_fpc(self):
        lk = self.link
        new = 2 if lk.mod_state["IMU_U1"] == 1 else 1
        for k in ("IMU_U1", "QVAR_A", "TEMP"):
            lk.mod_state[k] = new
        self._refresh_mod_lamps()
        state = "PRESENT" if new == 1 else "ABSENT"
        self.log(f"柔性板 FPC {'插入' if new == 1 else '拔出'} → "
                 f"EVENT MODULE_STATE_CHANGED U1→{state}（REPROBE 后自动恢复数据帧）")
        self.term_print(f"<< EVENT 0x21 id=0x01 module=1 new_state={new}")

    def on_led(self, idx, on):
        self.link.led[idx] = on
        lamp, color = self.led_lamps[idx]
        lamp.configure(fg=color if on else "#3a4148")
        mask = (1 if self.link.led[0] else 0) | (2 if self.link.led[1] else 0)
        self.term_print(f">> CMD 0x01 LED_SET mask=0x{mask:02X}")
        self.term_print("<< CMD_ACK cmd=0x01 result=0")

    def on_pwr(self, dom, on):
        self.link.pwr[dom] = on
        self.pwr_lamps[dom].configure(fg="#2ecc71" if on else "#3a4148")
        d = {"SENS": 0, "STORE": 1, "ANALOG": 2}[dom]
        self.term_print(f">> CMD 0x02 PWR_SET domain={d}({dom}) onoff={1 if on else 0}")
        self.term_print("<< CMD_ACK cmd=0x02 result=0")
        if dom == "ANALOG" and on:
            self.log("ANALOG 上电：首次约 1200ms VBIAS 稳定等待（协议 §10.3）。")
        if dom == "ANALOG" and not on:
            self.log("ANALOG 断电：PVDF 采集自动暂停，波形归零。")

    def on_pwr_all(self, on):
        for dom in ("SENS", "STORE", "ANALOG"):
            self.link.pwr[dom] = on
            self.pwr_lamps[dom].configure(fg="#2ecc71" if on else "#3a4148")
        self.term_print(f">> CMD 0x02 PWR_SET ×3 onoff={1 if on else 0}（全开/全关）")
        self.term_print("<< CMD_ACK cmd=0x02 result=0 ×3")

    def on_thr_set(self, ch, spin):
        v = int(spin.get())
        self.link.qvar_thr[ch] = v
        self.qvar_waves[ch].set_thresholds((v, -v))
        self.term_print(f">> CMD 0x08 QVAR_THR_SET ch={ch} thr={v} hyst=50")
        self.term_print("<< CMD_ACK cmd=0x08 result=0")
        self.log(f"QVAR-{ch} 阈值更新为 ±{v} LSB，越限将上报 EVENT 0x04。")

    # ---- 录音 ----
    def on_rec_start(self):
        lk = self.link
        lk.rec_channels = (self.ch_l.get(), self.ch_r.get())
        if not any(lk.rec_channels):
            self.log("请至少选择一个声道。")
            return
        lk.rec_duration = max(1, int(self.rec_dur.get()))
        lk.recording = True
        lk.rec_elapsed = 0.0
        self.btn_rec_start.configure(state="disabled")
        self.btn_rec_stop.configure(state="normal")
        self.btn_rec_play.configure(state="disabled")
        mask = (1 if lk.rec_channels[0] else 0) | (2 if lk.rec_channels[1] else 0)
        self.term_print(f">> CMD 0x07 REC_CTRL action=1 ch_mask={mask} "
                        f"duration={lk.rec_duration}s")
        self.term_print("<< CMD_ACK cmd=0x07 result=0")
        self.log(f"录音开始：声道 mask={mask:02b}，时长 {lk.rec_duration}s，"
                 f"设备端录至 SD 卡（PCM 16kHz/16bit）。")

    def on_rec_stop(self, auto=False):
        lk = self.link
        if not lk.recording:
            return
        lk.recording = False
        self.btn_rec_start.configure(state="normal")
        self.btn_rec_stop.configure(state="disabled")
        dur = lk.rec_elapsed if not auto else lk.rec_duration
        lk.rec_file = self._synthesize_wav(dur, lk.rec_channels)
        self.term_print("<< EVENT 0x21 id=0x05 REC_STATE state=2 file_id=1")
        self.log(f"录音完成（{dur:.1f}s）→ 已下载文件 file_id=1 到本地，可播放。"
                 f"（真实链路：CMD 0x0A REC_READ → 0x07 AUDIO_FILE 分块上传）")
        self.btn_rec_play.configure(state="normal")
        self.lbl_rec.configure(text=f"已保存 file_id=1（{dur:.1f}s）")

    def on_rec_play(self):
        if not self.link.rec_file:
            return
        try:
            import winsound
            winsound.PlaySound(self.link.rec_file,
                               winsound.SND_FILENAME | winsound.SND_ASYNC)
            self.log("播放中……（本地声卡）")
        except Exception as e:  # noqa: BLE001
            self.log(f"播放失败：{e}")

    def _synthesize_wav(self, dur, channels):
        """DEMO 用：合成提示音 wav（左 440Hz / 右 660Hz），模拟下载回的录音文件。"""
        dur = min(max(dur, 0.5), 30)
        rate = 16000
        n = int(rate * dur)
        nch = 2 if all(channels) else 1
        path = os.path.join(tempfile.gettempdir(), "petring_rec_demo.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(nch)
            w.setsampwidth(2)
            w.setframerate(rate)
            frames = bytearray()
            for i in range(n):
                t = i / rate
                env = min(1.0, t * 4) * min(1.0, (dur - t) * 4)
                l = int(8000 * env * math.sin(2 * math.pi * 440 * t))
                r = int(8000 * env * math.sin(2 * math.pi * 660 * t))
                if nch == 2:
                    frames += struct.pack("<hh", l, r)
                else:
                    frames += struct.pack("<h", l if channels[0] else r)
            w.writeframes(bytes(frames))
        return path

    # ---- 指令终端 ----
    def term_print(self, s):
        self.term_txt.insert("end", s + "\n")
        self.term_txt.see("end")

    def on_cmd_quick(self, cmd, params):
        self.cmd_entry.delete(0, "end")
        self.cmd_entry.insert(0, cmd)
        self.param_entry.delete(0, "end")
        self.param_entry.insert(0, params)
        self.on_cmd_send()

    def on_cmd_send(self):
        cmd = self.cmd_entry.get().strip().upper() or "00"
        params = self.param_entry.get().strip().upper()
        self.term_print(f">> WRITE 0x22: {cmd} {params}")
        table = {
            "01": "<< CMD_ACK cmd=0x01 result=0（LED_SET）",
            "02": "<< CMD_ACK cmd=0x02 result=0（PWR_SET）",
            "03": "<< CMD_ACK cmd=0x03 result=0（SENSOR_EN）",
            "04": "<< CMD_ACK cmd=0x04 result=0（RATE_SET）",
            "05": "<< CMD_ACK cmd=0x05 result=0（QVAR_CFG）",
            "06": "<< CMD_ACK cmd=0x06 result=0（REPROBE）",
            "07": "<< CMD_ACK cmd=0x07 result=0（REC_CTRL）",
            "08": "<< CMD_ACK cmd=0x08 result=0（QVAR_THR_SET）",
            "09": "<< CMD_ACK cmd=0x09 result=0 data=[file_id=1, 10s, 32KB]",
            "0A": "<< CMD_ACK cmd=0x0A result=0（REC_READ 开始上传 → 0x07 帧流）",
            "0B": "<< CMD_ACK cmd=0x0B result=0（REC_DELETE）",
            "0C": "<< CMD_ACK cmd=0x0C result=0 → BATTERY 帧随后到达（GET_BATTERY）",
            "10": "<< CMD_ACK cmd=0x10 result=0 → MODULE_STATUS 帧随后到达",
            "11": "<< CMD_ACK cmd=0x11 result=0 data='v1.06;task-V1.06;DEMO'",
            "7F": "<< CMD_ACK cmd=0x7F result=0 data=DE AD BE EF（回显）",
        }
        self.term_print(table.get(cmd, f"<< CMD_ACK cmd=0x{cmd} result=-12（暂不支持）"))

    # ---------------- 周期刷新 ----------------
    def _tick(self):
        lk = self.link
        # 电池电量：未连接也持续模拟充放电；连接后 2s 一帧 BATTERY 上报
        mv, pct, chg = lk.battery_sample(0.1)
        self._draw_battery(mv, pct, chg)
        if lk.connected:
            self._batt_cnt = getattr(self, "_batt_cnt", 0) + 1
            if self._batt_cnt >= 20:
                self._batt_cnt = 0
                flags = (1 if chg else 0) | (4 if pct <= lk.BATT_LOW_PCT else 0)
                self._emit_frame(0x08, struct.pack("<BHBB", 1, mv, pct, flags))
            if pct <= lk.BATT_LOW_PCT and not lk.batt_low_warned:
                lk.batt_low_warned = True
                self.term_print(
                    f"<< EVENT 0x21 id=0x02 LOW_BATTERY pct={pct}")
            elif pct > lk.BATT_LOW_PCT + 5:
                lk.batt_low_warned = False
        if lk.connected:
            # 六轴
            for key, pitch in (("U4", 0.0), ("U1", 1.3)):
                if lk.mod_state[f"IMU_{key}"] == 1:
                    f = lk.imu_frame(pitch)
                    for lbl, val in zip(self.imu_vals[key], f):
                        lbl.configure(text=str(val))
                    wa, wg = self.imu_waves[key]
                    for w, v in zip((wa, wg), (f[0], f[3])):
                        w.push(v)
                        w.redraw()
                    self._emit_frame(0x01 if key == "U4" else 0x02,
                                     struct.pack("<B6h", 1, *f))
            # QVAR
            a, b = lk.qvar_sample()
            for ch, v in (("A", a), ("B", b)):
                if v is not None:
                    self.qvar_vals[ch].configure(text=str(v))
                    self.qvar_waves[ch].push(v)
                    self.qvar_waves[ch].redraw()
                    alarm = lk.qvar_alarm[ch]
                    self.qvar_lamps[ch].configure(
                        fg="#e74c3c" if alarm else "#3a4148")
                    if alarm and random.random() < 0.1:
                        self.term_print(
                            f"<< EVENT 0x21 id=0x04 QVAR_THR_CROSSED ch={ch} raw={v}")
            valid = (1 if a is not None else 0) | (2 if b is not None else 0)
            payload = struct.pack("<BhhB", 1, a or 0, b or 0, valid)
            self._emit_frame(0x03, payload)
            # PVDF
            p = lk.pvdf_sample()
            if p:
                heart, raw, ref = p
                self.pvdf_vals["heart_mv"].configure(text=str(heart))
                self.pvdf_vals["raw_mv"].configure(text=str(raw))
                self.pvdf_vals["ref_mv"].configure(text=str(ref))
                self.pvdf_wave.push(heart - ref)
                self.pvdf_wave.redraw()
                self.pvdf_wave2.push(raw - ref)
                self.pvdf_wave2.redraw()
                self._emit_frame(0x04, struct.pack("<B3i", 1, heart, raw, ref))
            else:
                for nm in ("heart_mv", "raw_mv", "ref_mv"):
                    self.pvdf_vals[nm].configure(text="—")
            # 录音进度
            if lk.recording:
                lk.rec_elapsed += 0.1
                pct = min(100, lk.rec_elapsed / lk.rec_duration * 100)
                self.rec_prog["value"] = pct
                self.lbl_rec.configure(
                    text=f"录音中 {lk.rec_elapsed:.1f}s / {lk.rec_duration}s")
                if lk.rec_elapsed >= lk.rec_duration:
                    self.on_rec_stop(auto=True)
        self.after(100, self._tick)      # 10Hz GUI 刷新（demo 足够）

    def _emit_frame(self, ftype, payload):
        """按协议帧格式打印到帧监视器。"""
        if self.frame_pause.get():
            return
        seq = self.link.next_seq(ftype)
        crc = 0
        body = bytes([ftype, seq, len(payload)]) + payload
        for byte in body:
            crc ^= byte                       # demo 简化：用 XOR 占位 CRC8
        frame = bytes([0x55, 0xAA]) + body + bytes([crc])
        names = {0x01: "IMU_U4", 0x02: "BODY_IMU_U1", 0x03: "QVAR", 0x04: "PVDF"}
        hexs = " ".join(f"{b:02X}" for b in frame)
        self.frame_txt.insert(
            "end", f"[0x{ftype:02X} {names.get(ftype, '?'):11s}] {hexs}\n")
        lines = int(self.frame_txt.index("end-1c").split(".")[0])
        if lines > 400:
            self.frame_txt.delete("1.0", "200.0")
        self.frame_txt.see("end")


if __name__ == "__main__":
    app = PetRingConsole()
    app.mainloop()
