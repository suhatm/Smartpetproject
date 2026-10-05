# -*- coding: utf-8 -*-
"""Sensor Hub 协议 V0.5 编解码（宠物环 task-V1.06）。

帧格式：55 AA | TYPE | SEQ | LEN | PAYLOAD | CRC8(poly=0x07, init=0)
CRC8 覆盖 TYPE..PAYLOAD。
"""

# ---- GATT UUID ----
UUID_SERVICE = "e5a00020-1e5c-4b8f-9a2d-6c0f7e8d9a0b"
UUID_DATA = "e5a00021-1e5c-4b8f-9a2d-6c0f7e8d9a0b"   # Notify 数据流
UUID_CMD = "e5a00022-1e5c-4b8f-9a2d-6c0f7e8d9a0b"    # Write 指令
UUID_ACK = "e5a00023-1e5c-4b8f-9a2d-6c0f7e8d9a0b"    # Notify 应答/事件
UUID_INFO = "e5a00024-1e5c-4b8f-9a2d-6c0f7e8d9a0b"   # Read 信息

DEVICE_NAME = "SmartPet"

# ---- TYPE ----
T_IMU_U4 = 0x01
T_BODY_IMU_U1 = 0x02
T_QVAR = 0x03
T_PVDF = 0x04
T_TEMP = 0x05
T_MIC = 0x06
T_AUDIO_FILE = 0x07
T_BATTERY = 0x08
T_MODULE_STATUS = 0x10
T_CMD_ACK = 0x20
T_EVENT = 0x21

TYPE_NAMES = {
    T_IMU_U4: "IMU_U4", T_BODY_IMU_U1: "BODY_IMU_U1", T_QVAR: "QVAR",
    T_PVDF: "PVDF", T_TEMP: "TEMP", T_MIC: "MIC",
    T_AUDIO_FILE: "AUDIO_FILE", T_BATTERY: "BATTERY",
    T_MODULE_STATUS: "MODULE_STATUS", T_CMD_ACK: "CMD_ACK", T_EVENT: "EVENT",
}

# ---- CMD ----
CMD_LED_SET = 0x01
CMD_PWR_SET = 0x02
CMD_SENSOR_EN = 0x03
CMD_RATE_SET = 0x04
CMD_QVAR_CFG = 0x05
CMD_REPROBE = 0x06
CMD_REC_CTRL = 0x07
CMD_QVAR_THR_SET = 0x08
CMD_REC_LIST = 0x09
CMD_REC_READ = 0x0A
CMD_REC_DELETE = 0x0B
CMD_GET_BATTERY = 0x0C
CMD_SD_TEST = 0x0D
CMD_GET_STATUS = 0x10
CMD_GET_VERSION = 0x11
CMD_FACTORY_PING = 0x7F

CMD_NAMES = {
    0x01: "LED_SET", 0x02: "PWR_SET", 0x03: "SENSOR_EN", 0x04: "RATE_SET",
    0x05: "QVAR_CFG", 0x06: "REPROBE", 0x07: "REC_CTRL", 0x08: "QVAR_THR_SET",
    0x09: "REC_LIST", 0x0A: "REC_READ", 0x0B: "REC_DELETE",
    0x0C: "GET_BATTERY", 0x0D: "SD_TEST", 0x10: "GET_STATUS",
    0x11: "GET_VERSION", 0x7F: "FACTORY_PING",
}

# ---- EVENT ----
EV_MOD_STATE = 0x01
EV_WDT_RESET = 0x02
EV_LOW_BATT = 0x03
EV_QVAR_THR = 0x04
EV_REC_STATE = 0x05
EV_REC_FILE_DONE = 0x06
EV_SD_TEST_DONE = 0x07

EVENT_NAMES = {
    EV_MOD_STATE: "MOD_STATE", EV_WDT_RESET: "WDT_RESET",
    EV_LOW_BATT: "LOW_BATT", EV_QVAR_THR: "QVAR_THR",
    EV_REC_STATE: "REC_STATE", EV_REC_FILE_DONE: "REC_FILE_DONE",
    EV_SD_TEST_DONE: "SD_TEST_DONE",
}

SD_TEST_ERRORS = {
    0: "PASS", -1: "无卡/初始化失败", -2: "挂载失败",
    -3: "写失败", -4: "读失败", -5: "校验不一致",
}

MOD_STATES = {0: "ABSENT", 1: "PRESENT", 2: "DEGRADED", 3: "UNKNOWN"}

# 模块状态帧字节序（bridge module_status_tick 的 st[] 布局）
MOD_STATUS_FIELDS = ["U4_IMU", "U1_IMU", "QVAR", "LED", "SOC%",
                     "SD", "PVDF", "MIC"]


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameParser:
    """SYNC 扫描对齐 + CRC8 校验的流式解析器。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data: bytes):
        """返回 [(type, seq, payload), ...]"""
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
            ln = self.buf[3]
            total = 5 + ln + 1
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + ln])
            crc = self.buf[5 + ln]
            if crc8(body) == crc:
                frames.append((body[0], body[1], body[3:3 + ln]))
            else:
                self.bad_crc += 1
            del self.buf[:total]
        return frames


def fmt_hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)
