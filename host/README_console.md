# 宠物环传感器控制台（正式版上位机）

task-V1.06 正式上位机，替代模拟数据 demo（`petring_console_demo.py`）。
经 BLE 真实连接宠物环 Sensor Hub 服务（协议 V0.5，`e5a00020`）。

## 运行

```bat
host\run_console.bat
```

或手动：

```bat
C:\Users\pc\.workbuddy\binaries\python\envs\default\Scripts\python.exe -m petring_console
```

（工作目录为 `host/`；依赖 bleak / PySide6 / pyqtgraph，已装于托管 venv）

## 功能页签

| 页签 | 协议映射 |
|---|---|
| 总览/控制 | 扫描/连接/MTU/固件信息(0x24)；LED_SET 0x01；PWR_SET 0x02；SENSOR_EN 0x03；RATE_SET 0x04；REPROBE 0x06；GET_VERSION 0x11 / GET_STATUS 0x10 / PING 0x7F；帧统计 |
| 六轴 | 0x01 IMU_U4（多采样包，±8g/±2000dps 换算 mg/dps 六分量曲线）、0x02 BODY_IMU_U1 |
| QVAR 静电 | 0x03 A/B 双通道曲线（valid 位图无效通道断线）；QVAR_THR_SET 0x08；EVENT 0x04 触发显示 |
| PVDF 压电 | 0x04 heart/raw/ref 三通道曲线+数值 |
| 麦克风/录音 | REC_CTRL 0x07（声道/时长/启停）；0x06 RMS 曲线；REC_LIST 0x09 / REC_READ 0x0A（进度条+CRC32 校验+自动存 WAV 到 `host/petring_console/downloads/`）/ REC_DELETE 0x0B；EVENT 0x05 录音状态联动自动刷新列表；本地播放 |
| SD 卡 | SD_TEST 0x0D（1~16MB，写图样→读回→校验+测速），EVENT 0x07 结果（错误码 -1~-5 中文释义）；模块状态灯 |
| 帧监视 | 全量 RX/ACK/EVT 帧表（暂停/清空、CRC 错误与 seq 跳变计数） |
| 指令终端 | 任意 hex 指令下发 + 快捷按钮 + TX/RX 日志 |

左侧栏：连接状态、模块状态灯（0x10 帧驱动）、电池大字面板（0x08 帧：SOC/VBAT/充放态/低电红色告警）+ GET_BATTERY 0x0C。

## 换算系数

- 加速度：`raw * 0.244 mg`（±8g，与固件 overlay FS_8G 对齐）
- 角速度：`raw * 70 mdps`（±2000dps）
- QVAR/PVDF：原始码显示（nV/mV 换算待标定，协议 V0.5 决策）

## 文件结构

```
host/petring_console/
├── protocol.py   协议 V0.5 编解码（帧/CRC8/UUID/指令/事件常量）
├── backend.py    QThread + asyncio + bleak 后端（信号解耦）
├── widgets.py    Lamp / LiveChart(pyqtgraph) / BatteryPanel
├── app.py        主窗口与全部页签
└── downloads/    下载的录音 WAV（自动创建）
```
