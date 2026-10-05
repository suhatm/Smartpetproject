# 宠物环传感器控制 — 正式版上位机（tkinter 真实 BLE 版）

`petring_console_live.py`：与 DEMO（`petring_console_demo.py`，模拟数据）**同一套界面**，
数据链路替换为真实 BLE（bleak）。协议 V0.5a（Sensor Hub e5a00020）。

| 上位机版本 | 日期 | 变更 |
|---|---|---|
| v1.06 | 2026-10-02 | 首版正式版（真实 BLE，协议 V0.5） |
| v1.07 | 2026-10-03 | UI 刷新节流（帧监视批量刷 100ms、波形标脏统一重绘、状态灯与下载进度节流）+ 波形画布增量重绘（静态框架一次绘制、动态曲线 tag 局部刷新）——修复高帧率（~200 帧/s）数据流下界面卡死 |
| **v1.08** | 2026-10-03 | 配合固件 task-V1.07：意外断线**自动重连**（10 次 × 3s，覆盖固件看门狗复位窗口，状态栏显示重连进度）；录音启动 6s 无 ACK 超时兜底；录音中实时显示双麦克风 RMS 电平；断线时自动复位 SD 测试/下载/录音挂起状态 |
| **v1.09** | 2026-10-05 | 按协议 V0.5a 核对报告修复两处 bug：① **IMU 数值二次换算**（协议 §6.2 帧内 int16 已是 mg/dps×10 物理值、固件直传，原误乘量程系数致加速度显示缩小 4.096 倍、陀螺 1.638 倍）；② **REPROBE 无参下发**（协议 §7.2 要求 sensor_id(1B)，无参固件回 result=-1 拒执行），改为携带柔性板外设 IMU_U1/QVAR/TEMP（sensor_id=1/2/4）逐个下发。配合固件 task-V1.09 QVAR 帧级 DEGRADED 状态上报 |

## 运行

双击 `run_live.bat`，或：

```bat
cd host
C:\Users\pc\AppData\Local\Programs\Python\Python313\python.exe petring_console_live.py
```

依赖：系统 Python 3.13（自带 tkinter）+ 用户级 bleak
（已装：`pip install --user bleak`；重装用清华镜像
`-i https://pypi.tuna.tsinghua.edu.cn/simple`）。

## 使用流程

1. 设备上电（红灯开机闪），点 **扫描**（5s，过滤 SmartPet / e5a00020 广播）；
2. 下拉选中设备，点 **连接** —— 自动协商 MTU、订阅 0x21/0x23、读 0x24 信息，
   并自动下发 GET_VERSION / GET_STATUS / GET_BATTERY；
3. 各页签即实时数据：
   - **六轴 IMU**：U4/U1 原始码已换算 mg / dps（±8g / ±2000dps），不在位灰显"—"；
   - **QVAR**：双通道 raw + 阈值下发（CMD 0x08，滞回 50），触发 EVENT 0x04 亮红灯；
   - **PVDF**：heart/raw/ref mV（ANALOG 域开电后才有数）；
   - **SD 卡测试**：CMD 0x0D SD_TEST（进度条动画，结果走 EVENT 0x07，
     错误码 -1 无卡 / -2 挂载失败 / -3 写失败 / -4 读失败 / -5 校验不一致）；
     录音文件 REC_LIST/下载/删除，下载自动 CRC32 校验、断点续传，
     存至 `host/downloads/RECxxxx.wav`，可本地播放；
   - **电池**：0x08 帧驱动（nPM1300 库仑计 SOC），双 Y 轴历史曲线；
   - **帧监视**：0x21/0x23 双通道真实帧（含 SEQ 与 CRC8），可暂停；
   - **指令终端**：十六进制 CMD + PARAMS 直发 0x22，ACK 实时回显。
4. 左侧"重新探测外设（CMD 0x06）"= REPROBE（替代 demo 的 FPC 模拟按钮）。

## 与 demo 的差异

| demo（模拟） | 正式版 |
|---|---|
| SimLink 生成数据 | bleak 真实帧驱动 |
| 模拟插/拔充电器 | 已隐藏（真实充电由 0x08 帧 flags 反映） |
| 模拟插/拔 FPC / SD 卡 | 改为 REPROBE / 刷新状态（GET_STATUS+REC_LIST） |
| 注入校验错误 | 已隐藏（真实校验由设备完成） |
| 假 ACK 表 | 真实 ACK/EVENT（0x23 通道） |

## 测试

- 帧编解码：CRC8、坏帧丢弃、跨通知分片重组、垃圾字节对齐（已验证）；
- 集成：注入真实格式帧 → 全部控件联动断言通过；
- 上板联调：烧录 task-V1.06 固件后按"使用流程"验证。
