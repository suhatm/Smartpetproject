#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宠物环电源域测试上位机（Smartpetproject / task-V1.01）

通过 SWD 内存邮箱（nrfutil device read/write）向固件下发三电源域
独立开关命令，用于测试：
    VDD_SENS_3V0（SENS）  —— nPM1300 LDSW1
    VDD_STORE_3V0_SW（STORE）—— nPM1300 LDSW2
    VDD_ANA_3V0（ANA）    —— SoC P1.06 ANA_EN（TPS7A2030）

用法：
    python tools/power_domain_test.py status          # 查询三域状态
    python tools/power_domain_test.py on  sens|store|ana|all
    python tools/power_domain_test.py off sens|store|ana|all
    python tools/power_domain_test.py cycle           # 自动化独立性测试

前提：
    1. J-Link 已接好（USB 到电脑 + 4 线到板子），板子已上电运行固件；
    2. build/zephyr/zephyr.map 存在（编译产物，用于解析邮箱地址）。

协议见 src/app/test_mailbox.h：上位机写 cmd -> seq -> status=1（最后写，
作为触发），固件执行后回填 result/rc 并写 status=2。
"""

import argparse
import os
import re
import subprocess
import sys
import time

# ---------------------------------------------------------------- 常量配置

# nrfutil 路径：默认 winget 安装位置，可用环境变量 NRFUTIL 覆盖
NRFUTIL_DEFAULT = (r"C:\Users\pc\AppData\Local\Microsoft\WinGet\Packages"
                   r"\NordicSemiconductor.nrfutil_Microsoft.Winget.Source_"
                   r"8wekyb3d8bbwe\nrfutil.exe")

# 工程根目录（本脚本位于 tools/ 下）
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_FILE = os.path.join(PROJECT_ROOT, "build", "zephyr", "zephyr.map")

MB_MAGIC = 0x50525433          # "PRT3"
MB_WORDS = 6                   # magic/cmd/seq/status/result/rc
MB_SIZE = MB_WORDS * 4

STATUS_IDLE, STATUS_PENDING, STATUS_DONE = 0, 1, 2

RESULT_SENS, RESULT_STORE, RESULT_ANA = 0x01, 0x02, 0x04

CMD = {
    ("on", "sens"): 1, ("off", "sens"): 2,
    ("on", "store"): 3, ("off", "store"): 4,
    ("on", "ana"): 5, ("off", "ana"): 6,
    ("on", "all"): 7, ("off", "all"): 8,
    ("status", None): 9,
}

DOMAIN_NAMES = {RESULT_SENS: "SENS ", RESULT_STORE: "STORE", RESULT_ANA: "ANA  "}

POLL_INTERVAL_S = 0.15         # 上位机轮询间隔
CMD_TIMEOUT_S = 5.0            # 单命令超时（固件主循环 20ms 周期，余量充足）


# ---------------------------------------------------------------- 底层访问

def run_nrfutil(args, retries=3):
    """执行 nrfutil 命令，返回 stdout；失败自动重试；最终失败抛 RuntimeError。"""
    nrfutil = os.environ.get("NRFUTIL", NRFUTIL_DEFAULT)
    cmd = [nrfutil] + args
    last_err = None
    for attempt in range(1, retries + 1):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=30, encoding="utf-8", errors="replace")
        except FileNotFoundError:
            raise RuntimeError(f"找不到 nrfutil：{nrfutil}\n"
                               f"请用环境变量 NRFUTIL 指定实际路径")
        except subprocess.TimeoutExpired:
            last_err = RuntimeError(f"nrfutil 命令超时（30s，第 {attempt} 次）")
            continue
        if r.returncode == 0:
            return r.stdout
        last_err = RuntimeError(f"nrfutil 失败（第 {attempt} 次）："
                                f"{' '.join(args)}\n{r.stdout}{r.stderr}")
        time.sleep(0.3)
    raise last_err


def read_words(addr, n):
    """读 n 个 32 位字，返回整数列表。

    nrfutil 输出格式（每行 4 字）：
        0x20000070: 47474553 52205245 00005454 00000000   |SEGGER RTT......|
    """
    out = run_nrfutil(["device", "read", "--address", hex(addr),
                       "--bytes", str(n * 4), "--width", "32",
                       "--direct", "--family", "nrf54l"])
    words = []
    for line in out.splitlines():
        m = re.match(r"\s*0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)", line)
        if m:
            for tok in m.group(1).split():
                words.append(int(tok, 16))
    if len(words) < n:
        raise RuntimeError(f"内存读取解析失败（期望 {n} 字，得到 {len(words)}）：\n{out}")
    return words[:n]


def write_word(addr, value):
    """写单个 32 位字（十进制传值，nrfutil 兼容性最好）。

    注意必须带 --family：否则 J-Link DLL 会弹设备选择对话框
    （GUIServer 模态框，曾默认指向损坏的 TLE9863QXW20 条目）。
    """
    run_nrfutil(["device", "write", "--address", hex(addr),
                 "--value", str(value & 0xFFFFFFFF),
                 "--direct", "--family", "nrf54l"])


def find_mailbox_addr():
    """从 zephyr.map 解析 power_test_mb 符号地址。"""
    if not os.path.isfile(MAP_FILE):
        raise RuntimeError(f"找不到 map 文件：{MAP_FILE}\n请先编译固件")
    with open(MAP_FILE, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if "power_test_mb" in line:
            # 格式1：地址与符号同行 " 0xADDR 0xSIZE power_test_mb"
            m = re.search(r"0x([0-9a-fA-F]{8,16})\s+0x[0-9a-fA-F]+\s+power_test_mb",
                          line)
            if m:
                return int(m.group(1), 16)
            # 格式2：符号独占一行，地址在上一行（.noinit 段条目）
            for j in (i, i - 1):
                m = re.search(r"0x([0-9a-fA-F]{8,16})", lines[j])
                if m:
                    return int(m.group(1), 16)
    raise RuntimeError("map 文件中未找到 power_test_mb 符号")


# ---------------------------------------------------------------- 协议封装

class Mailbox:
    def __init__(self):
        self.addr = find_mailbox_addr()
        self.seq = None

    def read(self):
        w = read_words(self.addr, MB_WORDS)
        return {"magic": w[0], "cmd": w[1], "seq": w[2],
                "status": w[3], "result": w[4], "rc": w[5]}

    def check(self):
        mb = self.read()
        if mb["magic"] != MB_MAGIC:
            raise RuntimeError(f"邮箱魔数不符：0x{mb['magic']:08X}"
                               f"（期望 0x{MB_MAGIC:08X}）\n"
                               f"可能固件未运行或版本不含测试邮箱")
        if self.seq is None:
            self.seq = mb["seq"]
        return mb

    def send(self, cmd):
        """下发命令并等待固件执行完成，返回最终邮箱内容。"""
        self.check()
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        write_word(self.addr + 0x04, cmd)      # cmd
        write_word(self.addr + 0x08, self.seq) # seq
        write_word(self.addr + 0x0C, STATUS_PENDING)  # status=1（触发）

        deadline = time.time() + CMD_TIMEOUT_S
        while time.time() < deadline:
            mb = self.read()
            if mb["status"] == STATUS_DONE and mb["seq"] == self.seq:
                return mb
            time.sleep(POLL_INTERVAL_S)
        raise RuntimeError(f"命令 {cmd} 超时（固件 {CMD_TIMEOUT_S}s 未应答）。"
                           f"请确认板子在运行、J-Link 连接正常")


# ---------------------------------------------------------------- 结果展示

def fmt_result(mask):
    parts = []
    for bit, name in DOMAIN_NAMES.items():
        parts.append(f"{name}={'ON ' if mask & bit else 'off'}")
    return "  ".join(parts)


def print_mb(tag, mb):
    rc = mb["rc"]
    rc_s = "OK" if rc == 0 else f"ERR({rc})"
    print(f"[{tag}] {fmt_result(mb['result'])}   rc={rc_s}")


# ---------------------------------------------------------------- 测试流程

def do_simple(mb, action, domain):
    cmd = CMD[(action, domain)]
    print(f"==> {action.upper()} {domain.upper()}")
    result = mb.send(cmd)
    print_mb("结果", result)
    return result


def do_cycle(mb):
    """自动化独立性测试：验证三域互不影响。

    序列：全关基准 -> 逐域单独开（验证其他两域不受影响）->
    全开 -> 逐域单独关 -> 全关收尾。
    """
    failures = []

    def expect(tag, mask_want, mb_ret):
        ok = (mb_ret["result"] == mask_want) and (mb_ret["rc"] == 0)
        print_mb(("PASS" if ok else "FAIL") + " " + tag, mb_ret)
        print(f"      期望 {fmt_result(mask_want)}")
        if not ok:
            failures.append(tag)

    ALL_OFF = 0
    print("== 自动化独立性测试开始 ==")

    r = mb.send(CMD[("off", "all")]); expect("全关基准", ALL_OFF, r)

    for bit, name in DOMAIN_NAMES.items():
        r = mb.send(CMD[("on", {"SENS ": "sens", "STORE": "store",
                                "ANA  ": "ana"}[name])])
        expect(f"单独开{name.strip()}", bit, r)
        r = mb.send(CMD[("off", {"SENS ": "sens", "STORE": "store",
                                 "ANA  ": "ana"}[name])])
        expect(f"单独关{name.strip()}", ALL_OFF, r)

    r = mb.send(CMD[("on", "all")])
    expect("全开", RESULT_SENS | RESULT_STORE | RESULT_ANA, r)

    for bit, name in DOMAIN_NAMES.items():
        d = {"SENS ": "sens", "STORE": "store", "ANA  ": "ana"}[name]
        r = mb.send(CMD[("off", d)])
        expect(f"全开中单独关{name.strip()}",
               (RESULT_SENS | RESULT_STORE | RESULT_ANA) & ~bit, r)
        r = mb.send(CMD[("on", d)])
        expect(f"恢复{name.strip()}",
               RESULT_SENS | RESULT_STORE | RESULT_ANA, r)

    r = mb.send(CMD[("off", "all")]); expect("全关收尾", ALL_OFF, r)

    print(f"\n== 自动化测试结束：{len(failures)} 项失败 ==")
    if failures:
        for f in failures:
            print(f"  ✗ {f}")
        sys.exit(1)
    print("  全部通过 ✓")


def main():
    p = argparse.ArgumentParser(description="宠物环电源域 SWD 测试上位机")
    p.add_argument("action", choices=["on", "off", "status", "cycle"],
                   help="on/off 需指定域")
    p.add_argument("domain", nargs="?", choices=["sens", "store", "ana", "all"],
                   help="sens=VDD_SENS_3V0, store=VDD_STORE_3V0_SW, "
                        "ana=VDD_ANA_3V0(ANA_EN), all=三域")
    args = p.parse_args()

    if args.action in ("on", "off") and not args.domain:
        p.error(f"{args.action} 需要指定域：sens|store|ana|all")

    mb = Mailbox()
    print(f"邮箱地址 0x{mb.addr:08X}（来自 zephyr.map）")
    cur = mb.check()
    print_mb("当前", cur)

    if args.action == "status":
        return
    if args.action == "cycle":
        do_cycle(mb)
        return
    do_simple(mb, args.action, args.domain)


if __name__ == "__main__":
    main()
