# -*- coding: utf-8 -*-
"""自定义控件：状态灯、多序列实时曲线（pyqtgraph）、电池大字体面板。"""
from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (QGridLayout, QLabel, QVBoxLayout, QWidget)

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")
pg.setConfigOptions(antialias=True)


class Lamp(QWidget):
    """圆形状态灯。"""

    COLORS = {"green": "#2ecc71", "red": "#e74c3c", "gray": "#95a5a6",
              "orange": "#f39c12", "blue": "#3498db"}

    def __init__(self, color="gray", size=14, parent=None):
        super().__init__(parent)
        self._color = self.COLORS[color]
        self.setFixedSize(size, size)

    def set_color(self, color: str):
        self._color = self.COLORS.get(color, color)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor(self._color))
        p.setPen(Qt.NoPen)
        p.drawEllipse(1, 1, self.width() - 2, self.height() - 2)
        p.end()


class LiveChart(QWidget):
    """多序列滚动曲线。series: [(name, color_hex), ...]，x 轴为秒（相对）。"""

    def __init__(self, title="", ymin=None, ymax=None, window_s=10.0,
                 max_points=2000, parent=None):
        super().__init__(parent)
        self.window_s = window_s
        self.max_points = max_points
        self.t0 = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.plot = pg.PlotWidget(title=title)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.legend = self.plot.addLegend(offset=(4, 4))
        if ymin is not None and ymax is not None:
            self.plot.setYRange(ymin, ymax)
        lay.addWidget(self.plot)
        self.series = []       # (curve, deque_t, deque_v)
        self._colors = []

    def set_series(self, series):
        """series: [(name, color_hex), ...]，重建曲线。"""
        self.plot.clear()
        self.series = []
        self._colors = [c for _, c in series]
        for name, color in series:
            curve = self.plot.plot(pen=pg.mkPen(color, width=2), name=name)
            self.series.append((curve, deque(maxlen=self.max_points),
                                deque(maxlen=self.max_points)))

    def add_point(self, values, t=None):
        """values: 与 set_series 等长的数值列表（None 跳过该序列）。"""
        if t is None:
            import time
            t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        x = t - self.t0
        for i, (curve, dt, dv) in enumerate(self.series):
            if i >= len(values) or values[i] is None:
                continue
            dt.append(x)
            dv.append(values[i])
            curve.setData(list(dt), list(dv))
        if x > self.window_s:
            self.plot.setXRange(x - self.window_s, x)

    def clear(self):
        self.t0 = None
        for curve, dt, dv in self.series:
            dt.clear()
            dv.clear()
            curve.setData([], [])


class BatteryPanel(QWidget):
    """大字 SOC / VBAT / 充放态显示。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        g = QGridLayout(self)
        self.soc = QLabel("--%")
        self.soc.setStyleSheet("font-size:42px; font-weight:bold; color:#27ae60;")
        self.vbat = QLabel("-- mV")
        self.vbat.setStyleSheet("font-size:20px; color:#2c3e50;")
        self.state = QLabel("未知")
        self.state.setStyleSheet("font-size:16px; color:#7f8c8d;")
        g.addWidget(QLabel("SOC"), 0, 0)
        g.addWidget(self.soc, 1, 0)
        g.addWidget(QLabel("电压"), 0, 1)
        g.addWidget(self.vbat, 1, 1)
        g.addWidget(QLabel("状态"), 0, 2)
        g.addWidget(self.state, 1, 2)

    def update_batt(self, vbat_mv: int, percent: int, flags: int):
        self.soc.setText(f"{percent}%")
        color = "#27ae60" if percent > 30 else ("#f39c12" if percent > 15
                                                 else "#e74c3c")
        self.soc.setStyleSheet(
            f"font-size:42px; font-weight:bold; color:{color};")
        self.vbat.setText(f"{vbat_mv} mV")
        parts = []
        if flags & 0x01:
            parts.append("充电中")
        if flags & 0x02:
            parts.append("已充满")
        if flags & 0x04:
            parts.append("低电!")
        self.state.setText(" / ".join(parts) if parts else "放电")
        self.state.setStyleSheet(
            "font-size:16px; color:#e74c3c; font-weight:bold;"
            if flags & 0x04 else "font-size:16px; color:#7f8c8d;")
