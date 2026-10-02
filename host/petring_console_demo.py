#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环传感器控制 —— 上位机 DEMO（模拟数据版，无真实 BLE）

用途：评审 GUI 布局与功能集。所有数据由内置模拟器产生，
点击"连接"后开始出数；界面元素与《宠物环传感器控制_通信协议》V0.4 一一对应。

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
  六轴×2        ↔ TYPE 0x01/0x02；总览=六分量多色合成图，分量=下拉选单轴大图
                （轴名分色、数值大号加粗、RMS |a|/|g| 统计；U1 可演示 ABSENT 灰显）
  QVAR×2        ↔ TYPE 0x03 + CMD 0x08 QVAR_THR_SET + EVENT 0x04 阈值触发
  PVDF          ↔ TYPE 0x04 heart/raw/ref mV + 波形
  SD 卡测试     ↔ CSNP1GCR01-BOW SD NAND（128MB/SPI 4线/FAT32）：
                拟新增 CMD 0x0D SD_TEST（写伪随机图样→读回逐块校验）
                 + 0x09 REC_LIST / 0x0A 下载 / 0x0B 删除 文件管理
  电池页签      ↔ 大字 SOC/VBAT/充放态 + 充放电历史曲线（X=时间，双 Y 轴分色）
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
        self.sd_files = []                            # SD 录音文件 (id, 时长s, 大小KB, 时间)
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
    """单通道波形画布，环形缓冲，支持可选阈值线。带 Y 轴刻度与网格。"""

    ML, MR, MT, MB = 46, 8, 22, 6     # 左留白放刻度，上留白放标题

    def __init__(self, master, title="", unit="", ymin=-100, ymax=100,
                 width=380, height=120, color="#1a76d2", **kw):
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

    def _pw(self):
        return self.w - self.ML - self.MR

    def push(self, v):
        self.data.append(v)
        if len(self.data) > self._pw():
            self.data.pop(0)

    def clear(self):
        self.data.clear()
        self._draw_frame()

    def _ticks(self):
        vals = [self.ymin, self.ymax]
        if self.ymin < 0 < self.ymax:
            vals.insert(1, 0)
        return vals

    def _y_of(self, v):
        span = (self.ymax - self.ymin) or 1
        v = max(self.ymin, min(self.ymax, v))
        ph = self.h - self.MT - self.MB
        return self.MT + ph - (v - self.ymin) / span * ph

    def _draw_frame(self):
        self.delete("all")
        pw = self._pw()
        self.create_text(self.ML, 4, anchor="nw", fill="#c8d2da",
                         font=("微软雅黑", 9, "bold"), text=self.title)
        if self.unit:
            self.create_text(self.w - 6, 4, anchor="ne", fill="#9aa4ad",
                             font=("微软雅黑", 9), text=self.unit)
        # 横向网格 + Y 轴刻度（min/0/max）
        for v in self._ticks():
            y = self._y_of(v)
            self.create_line(self.ML, y, self.ML + pw, y, fill="#232a31")
            self.create_text(self.ML - 4, y, anchor="e", fill="#7f8c8d",
                             font=("Consolas", 8), text=f"{v:g}")

    def redraw(self):
        self._draw_frame()
        pw = self._pw()
        # 零线（比网格略亮）
        if self.ymin < 0 < self.ymax:
            y0 = self._y_of(0)
            self.create_line(self.ML, y0, self.ML + pw, y0, fill="#39434c")
        # 阈值线
        for tv in self.thr_lines:
            y = self._y_of(tv)
            self.create_line(self.ML, y, self.ML + pw, y,
                             fill="#c0392b", dash=(4, 3))
        # 波形
        n = len(self.data)
        if n >= 2:
            pts = []
            x0 = self.ML + pw - n
            for i, v in enumerate(self.data):
                pts += [x0 + i, self._y_of(v)]
            self.create_line(*pts, fill=self.color, width=1.4)

# ---------------------------------------------------------------- 多序列波形

class MultiWaveCanvas(tk.Canvas):
    """多序列波形画布：同一量纲的多条曲线（如 ax/ay/az）分色显示，
    带图例、Y 轴刻度与网格。"""

    ML, MR, MT, MB = 46, 8, 24, 6

    def __init__(self, master, title="", unit="", ymin=-100, ymax=100,
                 series=(("s1", "#2ecc71"),), width=430, height=136, **kw):
        super().__init__(master, width=width, height=height,
                         bg="#101418", highlightthickness=1,
                         highlightbackground="#3a4148", **kw)
        self.unit = unit
        self.ymin, self.ymax = ymin, ymax
        self.w, self.h = width, height
        self.title = title
        self.series = [(nm, c, []) for nm, c in series]
        self._draw_frame()

    def set_range(self, ymin, ymax):
        self.ymin, self.ymax = ymin, ymax

    def _pw(self):
        return self.w - self.ML - self.MR

    def push(self, values):
        pw = self._pw()
        for (_, _, buf), v in zip(self.series, values):
            buf.append(v)
            if len(buf) > pw:
                buf.pop(0)

    def clear(self):
        for _, _, buf in self.series:
            buf.clear()
        self._draw_frame()

    def _y_of(self, v):
        span = (self.ymax - self.ymin) or 1
        v = max(self.ymin, min(self.ymax, v))
        ph = self.h - self.MT - self.MB
        return self.MT + ph - (v - self.ymin) / span * ph

    def _ticks(self):
        vals = [self.ymin, self.ymax]
        if self.ymin < 0 < self.ymax:
            vals.insert(1, 0)
        return vals

    def _draw_frame(self):
        self.delete("all")
        pw = self._pw()
        self.create_text(self.ML, 4, anchor="nw", fill="#c8d2da",
                         font=("微软雅黑", 9, "bold"), text=self.title)
        if self.unit:
            self.create_text(self.w - 6, 4, anchor="ne", fill="#9aa4ad",
                             font=("微软雅黑", 9), text=self.unit)
        # 图例（标题右侧）
        lx = self.ML + len(self.title) * 12 + 12
        for nm, color, _ in self.series:
            self.create_line(lx, 11, lx + 14, 11, fill=color, width=2)
            self.create_text(lx + 17, 11, anchor="w", fill=color,
                             font=("Consolas", 9, "bold"), text=nm)
            lx += 17 + len(nm) * 9 + 16
        # 横向网格 + Y 轴刻度（min/0/max）
        for v in self._ticks():
            y = self._y_of(v)
            self.create_line(self.ML, y, self.ML + pw, y, fill="#232a31")
            self.create_text(self.ML - 4, y, anchor="e", fill="#7f8c8d",
                             font=("Consolas", 8), text=f"{v:g}")

    def redraw(self):
        self._draw_frame()
        pw = self._pw()
        # 零线（比网格略亮）
        if self.ymin < 0 < self.ymax:
            y0 = self._y_of(0)
            self.create_line(self.ML, y0, self.ML + pw, y0, fill="#39434c")
        for _, color, buf in self.series:
            n = len(buf)
            if n >= 2:
                pts = []
                x0 = self.ML + pw - n
                for i, v in enumerate(buf):
                    pts += [x0 + i, self._y_of(v)]
                self.create_line(*pts, fill=color, width=1.3)

# ---------------------------------------------------------------- 双 Y 轴时间图表

class DualAxisChart(tk.Canvas):
    """充放电历史图：X 轴时间，左 Y 轴序列 1（电压）、右 Y 轴序列 2（电量），
    两序列不同颜色，带网格与图例。"""

    ML, MR, MT, MB = 52, 52, 26, 22      # 四边留白（左右各放一根 Y 轴）

    def __init__(self, master, title="",
                 y1=("", -1, 1, "#f1c40f"), y2=("", 0, 100, "#2ecc71"),
                 width=900, height=300, **kw):
        super().__init__(master, width=width, height=height,
                         bg="#101418", highlightthickness=1,
                         highlightbackground="#3a4148", **kw)
        self.title = title
        self.y1_name, self.y1_min, self.y1_max, self.c1 = y1
        self.y2_name, self.y2_min, self.y2_max, self.c2 = y2
        self.w, self.h = width, height
        self.maxlen = width - self.ML - self.MR
        self.ts, self.v1, self.v2 = [], [], []
        self._draw_frame()

    def push(self, t, val1, val2):
        self.ts.append(t)
        self.v1.append(val1)
        self.v2.append(val2)
        if len(self.ts) > self.maxlen:
            self.ts.pop(0); self.v1.pop(0); self.v2.pop(0)

    def clear(self):
        self.ts.clear(); self.v1.clear(); self.v2.clear()
        self._draw_frame()

    def _pw(self):
        return self.w - self.ML - self.MR

    def _ph(self):
        return self.h - self.MT - self.MB

    def _y_of(self, v, lo, hi):
        v = max(lo, min(hi, v))
        return self.MT + self._ph() - (v - lo) / ((hi - lo) or 1) * self._ph()

    def _draw_frame(self):
        self.delete("all")
        pw, ph = self._pw(), self._ph()
        self.create_text(self.ML, 6, anchor="nw", fill="#9aa4ad",
                         font=("微软雅黑", 8), text=self.title)
        # 图例（右上）
        lx = self.w - self.MR - 150
        for nm, c in ((self.y1_name, self.c1), (self.y2_name, self.c2)):
            self.create_line(lx, 10, lx + 14, 10, fill=c, width=2)
            self.create_text(lx + 17, 10, anchor="w", fill=c,
                             font=("微软雅黑", 8), text=nm)
            lx += 17 + len(nm) * 9 + 22
        # 横向网格 + 双 Y 轴刻度（各 5 档）
        for i in range(5):
            frac = i / 4
            y = self.MT + ph - frac * ph
            self.create_line(self.ML, y, self.ML + pw, y, fill="#232a31")
            v1 = self.y1_min + frac * (self.y1_max - self.y1_min)
            v2 = self.y2_min + frac * (self.y2_max - self.y2_min)
            self.create_text(self.ML - 6, y, anchor="e", fill=self.c1,
                             font=("Consolas", 7), text=f"{v1:.0f}")
            self.create_text(self.ML + pw + 6, y, anchor="w", fill=self.c2,
                             font=("Consolas", 7), text=f"{v2:.0f}")
        # Y 轴名
        self.create_text(self.ML - 6, self.MT - 8, anchor="e", fill=self.c1,
                         font=("微软雅黑", 7), text="mV")
        self.create_text(self.ML + pw + 6, self.MT - 8, anchor="w",
                         fill=self.c2, font=("微软雅黑", 7), text="%")

    def redraw(self):
        self._draw_frame()
        n = len(self.ts)
        if n < 2:
            return
        pw = self._pw()
        x0 = self.ML + pw - n
        # X 轴时间刻度（5 档，HH:MM:SS）
        for i in range(5):
            idx = int(i / 4 * (n - 1))
            x = x0 + idx
            self.create_line(x, self.MT, x, self.MT + self._ph(),
                             fill="#1d2329")
            self.create_text(x, self.h - self.MB + 4, anchor="n",
                             fill="#7f8c8d", font=("Consolas", 7),
                             text=time.strftime("%H:%M:%S",
                                                time.localtime(self.ts[idx])))
        # 两条曲线
        for vals, lo, hi, c in ((self.v1, self.y1_min, self.y1_max, self.c1),
                                (self.v2, self.y2_min, self.y2_max, self.c2)):
            pts = []
            for i, v in enumerate(vals):
                pts += [x0 + i, self._y_of(v, lo, hi)]
            self.create_line(*pts, fill=c, width=1.6)

# ---------------------------------------------------------------- 主窗口

class PetRingConsole(tk.Tk):
    BG = "#1b1f24"
    FG = "#d7dde3"
    ACCENT = "#1a76d2"

    def __init__(self):
        super().__init__()
        self.title("宠物环传感器控制  v0.3 DEMO（模拟数据）")
        self.configure(bg=self.BG)
        self.geometry("1320x840")
        self.minsize(1180, 760)
        self.link = SimLink()
        self._style()
        self._build_top()
        self._build_body()
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
        st.configure("TRadiobutton", background=self.BG, foreground=self.FG)
        st.configure("TLabelframe", background=self.BG, foreground="#9fd3a8",
                     bordercolor="#3a4148")
        st.configure("TLabelframe.Label", background=self.BG,
                     foreground="#8ab4f8", font=("微软雅黑", 9, "bold"))
        st.configure("TSeparator", background="#3a4148")
        st.configure("Treeview", background="#101418",
                     fieldbackground="#101418", foreground="#d7dde3",
                     font=("Consolas", 9))
        st.configure("Treeview.Heading", font=("微软雅黑", 9))

    # ---------------- 顶部连接栏 ----------------
    def _build_top(self):
        outer = ttk.Frame(self)
        outer.pack(fill="x", padx=10, pady=(8, 2))
        bar = ttk.Frame(outer)
        bar.pack(fill="x")
        ttk.Label(bar, text="设备:", style="Header.TLabel").pack(side="left")
        self.dev_var = tk.StringVar(value="SmartPet")
        ttk.Combobox(bar, textvariable=self.dev_var, width=14, state="readonly",
                     values=["SmartPet"]).pack(side="left", padx=(4, 8))
        self.btn_scan = ttk.Button(bar, text="扫描", command=self.on_scan)
        self.btn_scan.pack(side="left")
        self.btn_conn = ttk.Button(bar, text="连接", command=self.on_connect)
        self.btn_conn.pack(side="left", padx=6)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y",
                                                   padx=12, pady=2)
        self.lbl_mtu = ttk.Label(bar, text="MTU: —")
        self.lbl_mtu.pack(side="left", padx=(0, 6))
        self.lbl_rssi = ttk.Label(bar, text="RSSI: —")
        self.lbl_rssi.pack(side="left", padx=6)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y",
                                                   padx=12, pady=2)
        # --- 电池电量（协议 V0.4 TYPE 0x08 BATTERY，nPM1300 库仑计 SOC）---
        batt_box = ttk.Frame(bar)
        batt_box.pack(side="left")
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
        ttk.Separator(outer, orient="horizontal").pack(fill="x", pady=(6, 0))

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

    # ---------------- 主体：左栏状态 + 右列（控制区/页签） ----------------
    def _build_body(self):
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=10, pady=6)
        self._build_left_status(body)
        ttk.Separator(body, orient="vertical").pack(side="left", fill="y",
                                                    padx=(2, 8))
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)
        self._build_control_area(right)
        self._build_tabs(right)

    # ---------------- 左侧模块状态 ----------------
    def _build_left_status(self, parent):
        self.left = ttk.LabelFrame(parent, text=" 模块状态 ", padding=8)
        self.left.pack(side="left", fill="y", padx=(0, 2))
        self.mod_labels = {}
        names = [("IMU_U4", "主板六轴 U4"), ("IMU_U1", "柔性板六轴 U1"),
                 ("QVAR_A", "静电 A（U1）"), ("QVAR_B", "静电 B（U4）"),
                 ("PVDF", "压电膜链路"), ("TEMP", "温度 U2"),
                 ("MIC", "双麦克风"), ("SD", "SD 卡")]
        for key, cn in names:
            row = ttk.Frame(self.left)
            row.pack(fill="x", pady=2)
            lamp = tk.Label(row, text="●", fg="#7f8c8d", bg=self.BG,
                            font=("微软雅黑", 12))
            lamp.pack(side="left")
            ttk.Label(row, text=f" {cn}").pack(side="left")
            self.mod_labels[key] = lamp
        ttk.Separator(self.left, orient="horizontal").pack(fill="x", pady=8)
        ttk.Label(self.left, text="图例：绿=在位 灰=不在位 橙=降级",
                  font=("微软雅黑", 8)).pack(anchor="w", pady=(0, 6))
        self.btn_fpc = ttk.Button(self.left, text="模拟插/拔柔性板 FPC",
                                  command=self.on_toggle_fpc)
        self.btn_fpc.pack(fill="x", pady=2)
        self._refresh_mod_lamps()

    def _refresh_mod_lamps(self):
        colors = {0: "#7f8c8d", 1: "#2ecc71", 2: "#5d6d7e", 3: "#e67e22"}
        for k, lamp in self.mod_labels.items():
            lamp.configure(fg=colors[self.link.mod_state[k]])

    # ---------------- 右侧控制区（LED/电源/录音） ----------------
    def _build_control_area(self, parent):
        self.ctrl = ttk.Frame(parent)
        self.ctrl.pack(side="top", fill="x", pady=(0, 4))

        # --- LED ---
        led_f = ttk.LabelFrame(self.ctrl, text=" LED 控制 ", padding=8)
        led_f.pack(side="left", fill="both", padx=(0, 6))
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
        ttk.Label(led_f, text="⚠ LED1(蓝) 现板极性接反，实测不亮\n"
                              "  （硬件待整改，非固件问题）",
                  font=("微软雅黑", 8), foreground="#e67e22",
                  justify="left").pack(anchor="w", pady=(4, 0))

        # --- 电源域 ---
        pwr_f = ttk.LabelFrame(self.ctrl, text=" 电源域控制（SENS/STORE/ANALOG） ",
                               padding=8)
        pwr_f.pack(side="left", fill="both", padx=6)
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
                               padding=8)
        rec_f.pack(side="left", fill="both", expand=True, padx=(6, 0))
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
    def _build_tabs(self, parent):
        self.nb = ttk.Notebook(parent)
        self.nb.pack(side="top", fill="both", expand=True, pady=(2, 0))
        self._tab_imu()
        self._tab_qvar()
        self._tab_pvdf()
        self._tab_sd()
        self._tab_battery()
        self._tab_frames()
        self._tab_terminal()

    def _tab_imu(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 六轴 IMU ")
        self.imu_vals = {}
        self.imu_ui = {}
        self.imu_stats = {}
        self._imu_hist = {"U4": [], "U1": []}
        axes = [("ax", "mg", "#2ecc71"), ("ay", "mg", "#3498db"),
                ("az", "mg", "#e74c3c"), ("gx", "dps×10", "#e67e22"),
                ("gy", "dps×10", "#f1c40f"), ("gz", "dps×10", "#9b59b6")]
        for idx, (key, title) in enumerate((("U4", "主板 U4"), ("U1", "柔性板 U1"))):
            lf = ttk.LabelFrame(tab, text=f" {title} LSM6DSV16X（ODR 30 Hz） ",
                                padding=6)
            lf.grid(row=0, column=idx, padx=8, pady=6, sticky="n")
            # 左列：六分量数值（轴名分色、数值大号加粗）+ RMS 统计
            grid = ttk.Frame(lf)
            grid.pack(side="left", fill="y", padx=(0, 8))
            self.imu_vals[key] = []
            for r, (nm, unit, color) in enumerate(axes):
                tk.Label(grid, text=nm, fg=color, bg=self.BG,
                         font=("Consolas", 11, "bold")).grid(
                    row=r, column=0, sticky="e")
                v = ttk.Label(grid, text="—", width=9,
                              font=("Consolas", 12, "bold"))
                v.grid(row=r, column=1, sticky="w", padx=(4, 0))
                ttk.Label(grid, text=unit, font=("微软雅黑", 9)).grid(
                    row=r, column=2, sticky="w", padx=(4, 0))
                self.imu_vals[key].append(v)
            ttk.Separator(grid, orient="horizontal").grid(
                row=6, column=0, columnspan=3, sticky="ew", pady=6)
            st_lbl = ttk.Label(grid, text="RMS |a|   — mg\nRMS |g|   — dps",
                               font=("Consolas", 9), justify="left")
            st_lbl.grid(row=7, column=0, columnspan=3, sticky="w")
            self.imu_stats[key] = st_lbl
            ttk.Label(grid, text="量程 ±8g / ±2000dps\nint16 原始码直传",
                      font=("微软雅黑", 8), foreground="#7f8c8d",
                      justify="left").grid(row=8, column=0, columnspan=3,
                                           sticky="w", pady=(4, 0))
            # 右列：模式选择 + 画布区
            right = ttk.Frame(lf)
            right.pack(side="left", fill="both", expand=True)
            ctl = ttk.Frame(right)
            ctl.pack(fill="x", pady=(0, 3))
            mode_var = tk.StringVar(value="overview")
            comp_var = tk.StringVar(value="ax")
            ttk.Radiobutton(ctl, text="总览（六分量合成）", value="overview",
                            variable=mode_var,
                            command=lambda k=key: self.on_imu_mode(k)).pack(
                side="left")
            ttk.Radiobutton(ctl, text="分量", value="single",
                            variable=mode_var,
                            command=lambda k=key: self.on_imu_mode(k)).pack(
                side="left", padx=(10, 2))
            comp_cb = ttk.Combobox(
                ctl, textvariable=comp_var, width=5, state="readonly",
                values=[nm for nm, _, _ in axes])
            comp_cb.pack(side="left")
            comp_cb.bind("<<ComboboxSelected>>",
                         lambda e, k=key: self.on_imu_comp(k))
            comp_cb.configure(state="disabled")
            # 总览：加速度三分量 + 陀螺仪三分量两张多序列图
            ov = ttk.Frame(right)
            wa = MultiWaveCanvas(
                ov, title=f"{title} 加速度", unit="mg", ymin=-2200, ymax=2200,
                series=(("ax", "#2ecc71"), ("ay", "#3498db"),
                        ("az", "#e74c3c")))
            wa.pack(pady=2)
            wg = MultiWaveCanvas(
                ov, title=f"{title} 陀螺仪", unit="dps×10", ymin=-600,
                ymax=600,
                series=(("gx", "#e67e22"), ("gy", "#f1c40f"),
                        ("gz", "#9b59b6")))
            wg.pack(pady=2)
            # 分量：单轴大图
            sg = ttk.Frame(right)
            ws = WaveCanvas(sg, title=f"{title} ax", unit="mg",
                            ymin=-2200, ymax=2200, width=430, height=292,
                            color="#2ecc71")
            ws.pack(pady=2)
            self.imu_ui[key] = {"mode": mode_var, "comp": comp_var,
                                "comp_cb": comp_cb, "ov_frame": ov,
                                "sg_frame": sg, "wa": wa, "wg": wg,
                                "ws": ws}
            ov.pack()          # 默认总览

    def on_imu_mode(self, key):
        ui = self.imu_ui[key]
        if ui["mode"].get() == "overview":
            ui["sg_frame"].pack_forget()
            ui["ov_frame"].pack()
            ui["comp_cb"].configure(state="disabled")
        else:
            ui["ov_frame"].pack_forget()
            ui["sg_frame"].pack()
            ui["comp_cb"].configure(state="readonly")
            self.on_imu_comp(key)

    def on_imu_comp(self, key):
        ui = self.imu_ui[key]
        comp = ui["comp"].get()
        title = "主板 U4" if key == "U4" else "柔性板 U1"
        is_gyro = comp.startswith("g")
        ui["ws"].set_range(-600, 600) if is_gyro else ui["ws"].set_range(
            -2200, 2200)
        ui["ws"].title = f"{title} {comp}"
        ui["ws"].unit = "dps×10" if is_gyro else "mg"
        ui["ws"].color = {"ax": "#2ecc71", "ay": "#3498db", "az": "#e74c3c",
                          "gx": "#e67e22", "gy": "#f1c40f",
                          "gz": "#9b59b6"}[comp]
        ui["ws"].clear()

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
                                    width=1020, height=190, color="#e74c3c")
        self.pvdf_wave.pack(pady=4)
        self.pvdf_wave2 = WaveCanvas(tab, title="raw - ref", unit="mV",
                                    ymin=-400, ymax=400,
                                    width=1020, height=190, color="#f39c12")
        self.pvdf_wave2.pack(pady=4)
        ttk.Label(tab, text="提示：ANALOG 域断电时链路无输出（波形归零、状态转灰）",
                  font=("微软雅黑", 8)).pack(anchor="w", padx=8)

    def _tab_sd(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" SD 卡测试 ")
        self.sd_test = None                 # 进行中的测试状态 dict
        # ---- 卡片信息 ----
        info = ttk.LabelFrame(
            tab, text=" 存储芯片：CSNP1GCR01-BOW（SD NAND · 贴片 TF 卡） ",
            padding=8)
        info.pack(fill="x", padx=8, pady=6)
        self.sd_lamp = tk.Label(info, text="●", fg="#2ecc71", bg=self.BG,
                                font=("微软雅黑", 14))
        self.sd_lamp.pack(side="left")
        ttk.Label(info, text=" 在位 · 容量 128 MB（1Gbit SLC）· SPI 4 线 25 MHz · "
                             "FAT32 · 内置 ECC/坏块管理 · 可用约 115 MB",
                  font=("微软雅黑", 10)).pack(side="left", padx=(4, 20))
        ttk.Button(info, text="模拟插/拔 SD 卡",
                   command=self.on_sd_toggle).pack(side="left")
        ttk.Label(info, text="测试原理：写伪随机图样 → 读回逐块比对（CRC 校验）",
                  font=("微软雅黑", 8)).pack(side="right")
        # ---- 读写测试 ----
        rw = ttk.LabelFrame(
            tab, text=" 读写测试（拟新增 CMD 0x0D SD_TEST，待协议评审） ",
            padding=8)
        rw.pack(fill="x", padx=8, pady=4)
        row = ttk.Frame(rw)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="测试大小:").pack(side="left")
        self.sd_size = tk.Spinbox(row, from_=1, to=64, width=4)
        self.sd_size.delete(0, "end")
        self.sd_size.insert(0, "4")
        self.sd_size.pack(side="left")
        ttk.Label(row, text="MB").pack(side="left", padx=(2, 12))
        self.sd_verify = tk.BooleanVar(value=True)
        ttk.Checkbutton(row, text="写后读回校验",
                        variable=self.sd_verify).pack(side="left")
        self.sd_inject_err = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="注入校验错误（演示）",
                        variable=self.sd_inject_err).pack(side="left", padx=8)
        self.btn_sd_test = ttk.Button(row, text="开始测试",
                                      command=self.on_sd_test_start)
        self.btn_sd_test.pack(side="right")
        self.sd_prog = ttk.Progressbar(rw, length=400, mode="determinate")
        self.sd_prog.pack(fill="x", pady=4)
        self.lbl_sd_status = ttk.Label(rw, text="就绪")
        self.lbl_sd_status.pack(anchor="w")
        res = ttk.Frame(rw)
        res.pack(fill="x", pady=(4, 0))
        for label, attr in (("写入速度:", "sd_res_w"), ("读取速度:", "sd_res_r"),
                            ("校验结果:", "sd_res_v")):
            ttk.Label(res, text=label).pack(side="left", padx=(0, 2))
            v = ttk.Label(res, text="—", width=22,
                          font=("Consolas", 11, "bold"))
            v.pack(side="left", padx=(0, 16))
            setattr(self, attr, v)
        # ---- 录音文件列表 ----
        files = ttk.LabelFrame(
            tab, text=" SD 录音文件（CMD 0x09 REC_LIST / 0x0A 下载 / 0x0B 删除） ",
            padding=8)
        files.pack(fill="both", expand=True, padx=8, pady=4)
        cols = ("id", "dur", "size", "time")
        self.sd_tree = ttk.Treeview(files, columns=cols, show="headings",
                                    height=5)
        for c, t, w in (("id", "文件 ID", 70), ("dur", "时长 (s)", 90),
                        ("size", "大小 (KB)", 90), ("time", "录制时间", 110)):
            self.sd_tree.heading(c, text=t)
            self.sd_tree.column(c, width=w, anchor="center")
        self.sd_tree.pack(fill="both", expand=True, pady=2)
        brow = ttk.Frame(files)
        brow.pack(fill="x", pady=2)
        ttk.Button(brow, text="刷新列表（0x09）",
                   command=self._sd_refresh_files_log).pack(side="left")
        ttk.Button(brow, text="下载所选（0x0A）",
                   command=self.on_sd_download).pack(side="left", padx=4)
        ttk.Button(brow, text="删除所选（0x0B）",
                   command=self.on_sd_delete).pack(side="left", padx=4)
        ttk.Label(tab, text="提示：STORE 域控制 SD 卡电源；SD 不在位时测试与录音均不可用。",
                  font=("微软雅黑", 8)).pack(anchor="w", padx=10, pady=2)

    # ---- SD 卡事件 ----
    def on_sd_toggle(self):
        lk = self.link
        lk.mod_state["SD"] = 2 if lk.mod_state["SD"] == 1 else 1
        self._refresh_mod_lamps()
        on = lk.mod_state["SD"] == 1
        self.sd_lamp.configure(fg="#2ecc71" if on else "#5d6d7e")
        self.log("SD 卡 " + ("插入，FAT32 挂载成功（/SD:）。"
                            if on else "拔出，已卸载。"))
        self.term_print(f"<< EVENT 0x21 id=0x01 module=7 "
                        f"new_state={lk.mod_state['SD']}")

    def on_sd_test_start(self):
        if self.link.mod_state["SD"] != 1:
            self.log("SD 卡不在位，无法测试。")
            return
        if self.sd_test:
            return
        size_mb = max(1, int(self.sd_size.get()))
        self.sd_test = {"total": size_mb * 1024, "done": 0, "phase": "write",
                        "w_speed": 0.0, "r_speed": 0.0, "bad": 0}
        self.btn_sd_test.configure(state="disabled")
        self.sd_prog["value"] = 0
        for attr in ("sd_res_w", "sd_res_r", "sd_res_v"):
            getattr(self, attr).configure(text="—", foreground=self.FG)
        self.term_print(
            f">> CMD 0x0D SD_TEST size={size_mb}MB "
            f"verify={1 if self.sd_verify.get() else 0}（拟新增，待评审）")
        self.log(f"SD 测试开始：写入 {size_mb}MB 伪随机图样（0xAA55 种子递增）……")

    def _sd_refresh_files(self):
        self.sd_tree.delete(*self.sd_tree.get_children())
        for fid, dur, kb, ts in self.link.sd_files:
            self.sd_tree.insert("", "end", iid=str(fid),
                                values=(fid, f"{dur:.1f}", kb, ts))

    def _sd_refresh_files_log(self):
        self._sd_refresh_files()
        self.term_print(">> CMD 0x09 REC_LIST")
        self.term_print(f"<< CMD_ACK cmd=0x09 result=0 "
                        f"count={len(self.link.sd_files)}")

    def _sd_selected(self):
        sel = self.sd_tree.selection()
        return int(sel[0]) if sel else None

    def on_sd_download(self):
        fid = self._sd_selected()
        if fid is None:
            self.log("请先在文件列表中选择一个录音文件。")
            return
        self.term_print(f">> CMD 0x0A REC_READ file_id={fid}")
        self.term_print("<< CMD_ACK cmd=0x0A result=0"
                        "（→ 0x07 AUDIO_FILE 分块上传，模拟完成）")
        self.log(f"录音文件 file_id={fid} 下载完成（模拟）。")

    def on_sd_delete(self):
        fid = self._sd_selected()
        if fid is None:
            self.log("请先在文件列表中选择一个录音文件。")
            return
        self.link.sd_files = [f for f in self.link.sd_files if f[0] != fid]
        self._sd_refresh_files()
        self.term_print(f">> CMD 0x0B REC_DELETE file_id={fid}")
        self.term_print("<< CMD_ACK cmd=0x0B result=0")
        self.log(f"录音文件 file_id={fid} 已从 SD 删除。")

    def _tab_battery(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text=" 电池电量 ")
        # 顶部大字状态区
        top = ttk.LabelFrame(tab, text=" 实时状态（BATTERY 帧 0x08） ",
                             padding=8)
        top.pack(fill="x", padx=8, pady=6)
        self.batt_big = {}
        for key, label, color in (
                ("soc", "电量 SOC", "#2ecc71"), ("vbat", "电压 VBAT", "#f1c40f"),
                ("state", "充放状态", "#3498db")):
            cell = ttk.Frame(top)
            cell.pack(side="left", padx=(6, 30))
            ttk.Label(cell, text=label, font=("微软雅黑", 9)).pack(anchor="w")
            v = ttk.Label(cell, text="—", font=("Consolas", 18, "bold"),
                          foreground=color)
            v.pack(anchor="w")
            self.batt_big[key] = v
        ttk.Label(top, text="  低电:").pack(side="left")
        self.batt_low_lamp = tk.Label(top, text="●", fg="#3a4148", bg=self.BG,
                                      font=("微软雅黑", 16))
        self.batt_low_lamp.pack(side="left")
        ttk.Button(top, text="模拟插/拔充电器",
                   command=self.on_toggle_charge).pack(side="right", padx=6)
        # 充放电历史曲线（X=时间，左 Y=电压 mV 黄色，右 Y=电量 % 绿色）
        chart_f = ttk.LabelFrame(tab, text=" 充放电曲线（X 轴时间） ",
                                 padding=6)
        chart_f.pack(fill="both", expand=True, padx=8, pady=4)
        self.batt_chart = DualAxisChart(
            chart_f, title="VBAT 与 SOC 历史",
            y1=("VBAT(mV)", 3300, 4200, "#f1c40f"),
            y2=("SOC(%)", 0, 100, "#2ecc71"), width=1060, height=300)
        self.batt_chart.pack(padx=4, pady=4)
        ttk.Label(tab,
                  text="算法：nPM1300 库仑计 SOC（nrf_fuel_gauge）——SOC 电流积分为主，"
                       "电压经 OCV 曲线交叉校验；低电门限 15%（5% 滞回）",
                  font=("微软雅黑", 8)).pack(anchor="w", padx=10, pady=2)

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
                                   ("SD_TEST", "0D", "04 01"),
                                   ("PING", "7F", "DE AD BE EF")):
            ttk.Button(row, text=label,
                       command=lambda c=cmd, p=params: self.on_cmd_quick(c, p)
                       ).pack(side="left", padx=2)
        self.term_txt = tk.Text(tab, height=14, bg="#101418", fg="#8ecbff",
                                font=("Consolas", 9))
        self.term_txt.pack(fill="both", expand=True, pady=4)

    # ---------------- 底部日志 ----------------
    def _build_log(self):
        sep = ttk.Frame(self)
        sep.pack(side="bottom", fill="x", padx=10)
        ttk.Separator(sep, orient="horizontal").pack(fill="x", pady=2)
        self.log_txt = tk.Text(self, height=4, bg="#14181d", fg="#7fb3d5",
                               font=("Consolas", 9))
        self.log_txt.pack(side="bottom", fill="x", padx=10, pady=(0, 8))
        self.log("DEMO 启动：数据为模拟生成，界面元素对应通信协议 V0.4。")

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
        # 录音文件同步进 SD 文件列表（PCM 16kHz/16bit）
        fid = max([f[0] for f in lk.sd_files], default=0) + 1
        nch = 2 if all(lk.rec_channels) else 1
        kb = int(dur * 16000 * 2 * nch / 1024)
        lk.sd_files.append((fid, dur, kb, time.strftime("%H:%M:%S")))
        self._sd_refresh_files()
        self.term_print(f"<< EVENT 0x21 id=0x05 REC_STATE state=2 file_id={fid}")
        self.log(f"录音完成（{dur:.1f}s，{kb}KB）→ 已下载文件 file_id={fid} "
                 f"到本地，可播放。（真实链路：CMD 0x0A REC_READ → "
                 f"0x07 AUDIO_FILE 分块上传）")
        self.btn_rec_play.configure(state="normal")
        self.lbl_rec.configure(text=f"已保存 file_id={fid}（{dur:.1f}s）")

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
            "0D": "<< CMD_ACK cmd=0x0D result=0 w=1.35MB/s r=2.86MB/s（SD_TEST）",
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
        # 电池页签：大字状态 + 历史曲线（0.5s 一点）
        self.batt_big["soc"].configure(text=f"{pct} %")
        self.batt_big["vbat"].configure(text=f"{mv} mV")
        self.batt_big["state"].configure(
            text="⚡ 充电中" if chg else "电池供电")
        low = pct <= lk.BATT_LOW_PCT
        self.batt_low_lamp.configure(fg="#e74c3c" if low else "#3a4148")
        self._batt_chart_cnt = getattr(self, "_batt_chart_cnt", 0) + 1
        if self._batt_chart_cnt >= 5:
            self._batt_chart_cnt = 0
            self.batt_chart.push(time.time(), mv, pct)
            self.batt_chart.redraw()
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
                    # RMS 统计（最近 50 帧，约 5 秒窗）
                    hist = self._imu_hist[key]
                    hist.append(f)
                    if len(hist) > 50:
                        hist.pop(0)
                    n = len(hist)
                    rms_a = math.sqrt(
                        sum(s[0] ** 2 + s[1] ** 2 + s[2] ** 2
                            for s in hist) / n)
                    rms_g = math.sqrt(
                        sum(s[3] ** 2 + s[4] ** 2 + s[5] ** 2
                            for s in hist) / n) / 10
                    self.imu_stats[key].configure(
                        text=f"RMS |a| {rms_a:5.0f} mg\n"
                             f"RMS |g| {rms_g:5.1f} dps")
                    ui = self.imu_ui[key]
                    if ui["mode"].get() == "overview":
                        ui["wa"].push(f[0:3])
                        ui["wa"].redraw()
                        ui["wg"].push(f[3:6])
                        ui["wg"].redraw()
                    else:
                        idx = ("ax", "ay", "az", "gx", "gy", "gz").index(
                            ui["comp"].get())
                        ui["ws"].push(f[idx])
                        ui["ws"].redraw()
                    self._emit_frame(0x01 if key == "U4" else 0x02,
                                     struct.pack("<B6h", 1, *f))
                else:
                    # 不在位：数值灰显为 —，统计复位，波形保持最后一帧
                    for lbl in self.imu_vals[key]:
                        lbl.configure(text="—")
                    self.imu_stats[key].configure(
                        text="RMS |a|   — mg\nRMS |g|   — dps")
                    self._imu_hist[key].clear()
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
        # ---- SD 读写测试推进（不依赖连接状态，仅依赖 SD 在位）----
        st = self.sd_test
        if st is not None:
            if st["phase"] == "write":
                spd = random.uniform(1.1, 1.6)          # MB/s，模拟 SPI 写速
                st["w_speed"] = spd
                st["done"] += spd * 102.4               # 0.1s 增量（KB）
                self.sd_prog["value"] = min(50, st["done"] / st["total"] * 50)
                self.lbl_sd_status.configure(
                    text=f"写入中 {min(st['done'], st['total']) / 1024:.1f}"
                         f" / {st['total'] // 1024} MB · {spd:.2f} MB/s")
                if st["done"] >= st["total"]:
                    st["done"] = 0
                    st["phase"] = "read" if self.sd_verify.get() else "done"
            elif st["phase"] == "read":
                spd = random.uniform(2.2, 3.4)          # MB/s，模拟读速
                st["r_speed"] = spd
                st["done"] += spd * 102.4
                if self.sd_inject_err.get() and random.random() < 0.04:
                    st["bad"] += 1
                self.sd_prog["value"] = 50 + min(
                    50, st["done"] / st["total"] * 50)
                self.lbl_sd_status.configure(
                    text=f"读回校验中 {min(st['done'], st['total']) / 1024:.1f}"
                         f" / {st['total'] // 1024} MB · {spd:.2f} MB/s")
                if st["done"] >= st["total"]:
                    st["phase"] = "done"
            if st is not None and st["phase"] == "done":
                verify_on = self.sd_verify.get()
                ok = st["bad"] == 0
                self.sd_res_w.configure(text=f"{st['w_speed']:.2f} MB/s")
                self.sd_res_r.configure(
                    text=f"{st['r_speed']:.2f} MB/s" if verify_on else "—")
                if verify_on:
                    self.sd_res_v.configure(
                        text=("通过 ✔" if ok
                              else f"失败 ✘（{st['bad']} 块不一致）"),
                        foreground="#2ecc71" if ok else "#e74c3c")
                else:
                    self.sd_res_v.configure(text="未启用",
                                            foreground="#9aa4ad")
                self.term_print(
                    f"<< CMD_ACK cmd=0x0D result={0 if ok else -5} "
                    f"w={st['w_speed']:.2f}MB/s r={st['r_speed']:.2f}MB/s")
                self.log(f"SD 测试完成：写 {st['w_speed']:.2f} MB/s，"
                         f"读 {st['r_speed']:.2f} MB/s，"
                         f"校验{'通过' if ok else '失败'}。")
                self.sd_test = None
                self.btn_sd_test.configure(state="normal")
                self.lbl_sd_status.configure(text="就绪")
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
