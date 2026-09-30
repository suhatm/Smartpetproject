#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环电源域测试上位机 GUI（Smartpetproject / task-V1.01）

图形界面版：三电源域（SENS/STORE/ANA）独立开关按钮、全开/全关、
实时状态轮询、自动化独立性测试、操作日志。

基于 tools/power_domain_test.py 的 SWD 邮箱协议实现（复用其底层类），
运行前需：固件已烧录（含 APP_POWER_DOMAIN_TEST_MB）、J-Link 连接、
build/zephyr/zephyr.map 存在（自动解析邮箱地址）。

启动：python tools/power_domain_gui.py
"""

import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

# 复用命令行版的协议实现（同目录导入）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from power_domain_test import (  # noqa: E402
    CMD, Mailbox, RESULT_SENS, RESULT_STORE, RESULT_ANA,
)

# ---------------------------------------------------------------- 常量

DOMAINS = [
    ("sens",  "SENS",  RESULT_SENS,  "VDD_SENS_3V0\n(IMU/麦克风/柔性板)"),
    ("store", "STORE", RESULT_STORE, "VDD_STORE_3V0_SW\n(SD NAND)"),
    ("ana",   "ANA",   RESULT_ANA,   "VDD_ANA_3V0\n(PVDF 运放, ANA_EN)"),
]

POLL_PERIOD_S = 2.0        # 状态自动轮询周期
COLOR_ON = "#e53935"       # 域开启指示（红=通电，符合硬件直觉）
COLOR_OFF = "#9e9e9e"      # 域关闭指示（灰=断电）


class PowerGui:
    """主应用类：界面 + 后台 SWD 操作线程"""

    def __init__(self, root):
        self.root = root
        self.mb = None
        self.poll_on = True
        self.cmd_lock = threading.Lock()   # 串行化所有 SWD 访问
        self.log_queue = []
        self.state_labels = {}
        self.state_dots = {}
        self._build_ui()
        self.root.after(100, self._connect)

    # ------------------------------------------------------------ 界面

    def _build_ui(self):
        self.root.title("宠物环电源域测试上位机 — Smartpetproject task-V1.01")
        self.root.geometry("760x560")
        self.root.minsize(700, 500)

        # 顶部：连接状态栏
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        self.conn_var = tk.StringVar(value="⏳ 正在连接（解析邮箱地址、校验魔数）…")
        ttk.Label(top, textvariable=self.conn_var, font=("Microsoft YaHei", 10, "bold")
                  ).pack(side=tk.LEFT)
        self.poll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="自动轮询状态", variable=self.poll_var,
                        command=self._toggle_poll).pack(side=tk.RIGHT)

        # 中部：三域卡片
        cards = ttk.Frame(self.root, padding=(8, 0))
        cards.pack(fill=tk.X)
        for i, (key, name, bit, desc) in enumerate(DOMAINS):
            card = ttk.LabelFrame(cards, text=f" {name} ", padding=10)
            card.grid(row=0, column=i, sticky="nsew", padx=6)
            cards.columnconfigure(i, weight=1)

            ttk.Label(card, text=desc, justify=tk.CENTER,
                      font=("Microsoft YaHei", 9)).pack(pady=(0, 6))

            dot = tk.Label(card, text="○ off", fg=COLOR_OFF,
                           font=("Microsoft YaHei", 12, "bold"))
            dot.pack(pady=4)
            self.state_dots[key] = dot

            btns = ttk.Frame(card)
            btns.pack(pady=4)
            ttk.Button(btns, text="开 ON", width=8,
                       command=lambda k=key: self._send_domain(k, "on")
                       ).pack(side=tk.LEFT, padx=4)
            ttk.Button(btns, text="关 OFF", width=8,
                       command=lambda k=key: self._send_domain(k, "off")
                       ).pack(side=tk.LEFT, padx=4)

        # 全局操作栏
        ops = ttk.Frame(self.root, padding=8)
        ops.pack(fill=tk.X)
        ttk.Button(ops, text="全开 ALL ON", command=lambda: self._send_simple(("on", "all"))
                   ).pack(side=tk.LEFT, padx=4)
        ttk.Button(ops, text="全关 ALL OFF", command=lambda: self._send_simple(("off", "all"))
                   ).pack(side=tk.LEFT, padx=4)
        ttk.Button(ops, text="立即刷新状态", command=self._refresh_now
                   ).pack(side=tk.LEFT, padx=4)
        ttk.Button(ops, text="自动化独立性测试", command=self._run_cycle
                   ).pack(side=tk.LEFT, padx=4)

        # 底部：日志窗口
        logf = ttk.LabelFrame(self.root, text=" 操作日志 ", padding=4)
        logf.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.log = scrolledtext.ScrolledText(logf, height=12, state=tk.DISABLED,
                                              font=("Consolas", 9))
        self.log.pack(fill=tk.BOTH, expand=True)

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_queue.append(f"[{ts}] {msg}")
        self.root.after(50, self._flush_log)

    def _flush_log(self):
        self.log.config(state=tk.NORMAL)
        while self.log_queue:
            self.log.insert(tk.END, self.log_queue.pop(0) + "\n")
            self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    # ------------------------------------------------------------ 连接

    def _connect(self):
        def work():
            try:
                mb = Mailbox()
                addr_s = f"0x{mb.addr:08X}"
                mb.check()
                self.mb = mb
                self._log(f"连接成功：邮箱地址 {addr_s}（zephyr.map 解析），魔数校验通过")
                self.root.after(0, lambda: self.conn_var.set(
                    f"✅ 已连接  邮箱 {addr_s}  （{POLL_PERIOD_S:.0f}s 自动轮询）"))
                self._poll_state()
            except Exception as e:
                self._log(f"连接失败：{e}")
                self.root.after(0, lambda: self.conn_var.set("❌ 连接失败（见日志）"))
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------ 状态轮询

    def _toggle_poll(self):
        self.poll_on = self.poll_var.get()
        self._log(f"自动轮询：{'开' if self.poll_on else '关'}")

    def _refresh_now(self):
        self._poll_state()

    def _poll_state(self):
        """周期性读状态（后台线程），刷新三域指示灯。"""
        if self.mb is None:
            return
        with self.cmd_lock:
            try:
                mb = self.mb.read()
            except Exception as e:
                self._log(f"状态读取失败：{e}")
                mb = None
        if mb is not None:
            self._update_dots(mb["result"], mb["rc"])
        if self.poll_on:
            self.root.after(int(POLL_PERIOD_S * 1000), self._poll_state)

    def _update_dots(self, result, rc):
        for key, name, bit, _ in DOMAINS:
            on = (result & bit) != 0
            dot = self.state_dots[key]
            def upd(d=dot, o=on):
                d.config(text="● ON " if o else "○ off",
                         fg=COLOR_ON if o else COLOR_OFF)
            self.root.after(0, upd)
        if rc != 0:
            self._log(f"警告：rc={rc}")

    # ------------------------------------------------------------ 命令下发

    def _send_domain(self, key, action):
        for k, name, _, _ in DOMAINS:
            if k == key:
                self._send_simple((action, k), f"{action.upper()} {name}")
                return

    def _send_simple(self, cmd_key, label=None):
        if label is None:
            label = f"{cmd_key[0].upper()} {cmd_key[1].upper()}"
        if self.mb is None:
            self._log("未连接，命令忽略")
            return

        def work():
            with self.cmd_lock:
                try:
                    mb = self.mb.send(CMD[cmd_key])
                except Exception as e:
                    self._log(f"{label} 失败：{e}")
                    return
            self._update_dots(mb["result"], mb["rc"])
            state = " ".join(
                f"{n}={'ON' if mb['result'] & b else 'off'}"
                for _, n, b, _ in DOMAINS)
            ok = "OK" if mb["rc"] == 0 else f"ERR({mb['rc']})"
            self._log(f"{label} → {state}  rc={ok}")
        threading.Thread(target=work, daemon=True).start()

    def _run_cycle(self):
        """自动化独立性测试（复用命令行版流程，逐项打日志）。"""
        if self.mb is None:
            self._log("未连接，命令忽略")
            return

        def work():
            self._log("== 自动化独立性测试开始 ==")
            failures = 0

            def expect(tag, mask_want, mb_ret):
                nonlocal failures
                ok = (mb_ret["result"] == mask_want) and (mb_ret["rc"] == 0)
                if not ok:
                    failures += 1
                self._update_dots(mb_ret["result"], mb_ret["rc"])
                self._log(f"[{'PASS' if ok else 'FAIL'}] {tag}  "
                          f"(rc={'OK' if mb_ret['rc'] == 0 else mb_ret['rc']})")

            with self.cmd_lock:
                ALL = RESULT_SENS | RESULT_STORE | RESULT_ANA
                try:
                    r = self.mb.send(CMD[("off", "all")]); expect("全关基准", 0, r)
                    for key, name, bit, _ in DOMAINS:
                        r = self.mb.send(CMD[("on", key)]); expect(f"单独开{name}", bit, r)
                        r = self.mb.send(CMD[("off", key)]); expect(f"单独关{name}", 0, r)
                    r = self.mb.send(CMD[("on", "all")]); expect("全开", ALL, r)
                    for key, name, bit, _ in DOMAINS:
                        r = self.mb.send(CMD[("off", key)])
                        expect(f"全开中单独关{name}", ALL & ~bit, r)
                        r = self.mb.send(CMD[("on", key)]); expect(f"恢复{name}", ALL, r)
                    r = self.mb.send(CMD[("off", "all")]); expect("全关收尾", 0, r)
                except Exception as e:
                    self._log(f"测试中断：{e}")
                    return
            self._log(f"== 测试结束：{failures} 项失败 "
                      f"{'（全部通过 ✓）' if failures == 0 else ''} ==")
        threading.Thread(target=work, daemon=True).start()


def main():
    root = tk.Tk()
    try:  # Windows 上让缩放更清晰（可选，失败忽略）
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    PowerGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
