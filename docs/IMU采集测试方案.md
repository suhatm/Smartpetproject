# IMU 采集测试方案（task-V1.03）

> 目标：采集主板 U4（LSM6DSV16XTR）六轴信号，经 SENS_I2C 读取，
> 通过蓝牙 GATT 传输到手机 nRF Connect for Mobile。
> 本文档覆盖第一步（连通性自检），第二步（GATT 数据流）另行补充。

## 1. 硬件连接（对齐《元器件引脚及功能模块对应表》）

| 项目 | 网络名 | 说明 |
|------|--------|------|
| I2C 总线 | SENS_I2C_SDA=P1.02 / SCL=P1.03 | i2c21（TWIM），4.7kΩ 上拉 R2/R4，400kHz |
| 主板 IMU | U4 LSM6DSV16XTR | 地址由 R6 0Ω 设 SA0：SA0=0→0x6A，SA0=1→0x6B |
| 柔性板 | FPC1 | 第二颗 LSM6DSV16X（0Ω 设地址）+ TMP112，同总线 |
| 中断 | IMU_REF_INT1=P0.00 / INT2=P1.04 | 第二步 FIFO watermark / 事件中断用 |
| 供电 | VDD_SENS_3V0 | nPM1300 LDSW1（power_domain_set(POWER_DOMAIN_SENS)）|

LSM6DSV16X 关键参数：WHO_AM_I 寄存器 0x0F，期望值 **0x70**（NCS v3.4.0
lsm6dsv16x_reg.h：LSM6DSV16X_ID=0x70）；3KB FIFO，支持 FSM/MLC 端侧活动识别。

## 2. NFC 引脚归还（关键坑，2026-10-01 查证）

P1.02/P1.03 复位后默认为 **NFC 天线 pad**（NFCT.PADCONFIG 复位值 0x1=Enabled，
每次复位重新加载）。nRF54L 的归还机制与 nRF52 **不同**：

- nRF52：UICR.NFCPINS（nRF54L15 的 UICR **没有**此寄存器，overlay 里
  `&uicr { nfct-pins-as-gpios; }` 对 nRF54L 无效，已改为 `&nfct` 节点标注）
- nRF54L：**运行时直接写 NFCT.PADCONFIG=0**，立即归还为 GPIO，无需重启

归还动作在 `src/hardware/imu_sensor.c` 的 `imu_nfct_pads_reclaim()`
（SYS_INIT PRE_KERNEL_1，优先级 30）中完成，必须早于 i2c21 TWIM pinctrl 应用。

## 3. 第一步：连通性自检（本次实现）

流程（`imu_bringup_test()`，main.c 启动时执行，Kconfig `APP_IMU_TEST=y`）：

1. NFC pad 归还状态复核（PRE_KERNEL 已归还）
2. VDD_SENS_3V0 上电（LDSW1），等待 25ms（LDO 稳定 + IMU boot）
3. i2c21 总线扫描 0x08~0x77（WHO_AM_I 寄存器读做探测事务）
4. U4 WHO_AM_I 校验：0x6A/0x6B 两个候选地址，期望 0x70
5. SENS 电源域下电（保持原有电源行为），打印 summary

RTT 输出格式（key=value 风格，与现有日志一致）：

```
IMU_TEST,begin,bus=i2c21,sda=P1.02,scl=P1.03,rate=400kHz
IMU_TEST,nfc_pads=reclaimed(P1.02/P1.03->GPIO)
IMU_TEST,scan,addr=0x6B,ACK
IMU_TEST,u4,PASS,addr=0x6B,whoami=0x70
IMU_TEST,summary,PASS,devices=2,failures=0
```

判定标准：
- `summary,PASS`：链路通（NFC 归还 + 电源 + 总线 + WHO_AM_I 全过）
- `scan,no_device=FAIL`：先查 LDSW1 是否打开（power domain selftest 输出）、
  再查 R6 焊接、FPC 是否插好
- `whoami=FAIL` 但 scan 有 ACK：地址冲突或挂的不是 LSM6DSV16X，读 MISMATCH 行的 id 值定位

## 4. 第二步：GATT 数据流（规划，待第一步上板验证后实施）

- 设备树：`st,lsm6dsv16x` 节点挂 i2c21（accel ±8g / 30Hz，gyro 2000dps / 30Hz）
- 数据通路：INT1（P0.00）FIFO watermark 中断 → TWIM 批量读 → 环形缓冲 → Notification
- GATT 服务：`e5a00010`（Notify 数据 `e5a00011`，Write 控制点 `e5a00012`），
  帧格式 `[seq:1B][cnt:1B][6×int16 ×N]`，MTU 247
- 功耗：日常 gyro 关（accel 低功耗 30Hz 约 12µA），连接间隔 45~60ms

## 5. 变更清单（task-V1.03 第一步）

- `app.overlay`：+i2c21（400kHz，pinctrl P1.02/P1.03），&uicr 改 &nfct 标注
- `src/hardware/imu_sensor.c/h`：NFC pad 归还 + 连通性自检
- `src/app/main.c`：启动序列插入 4.5 步自检（失败不阻断启动）
- `Kconfig`：+APP_IMU_TEST
- `CMakeLists.txt`：+imu_sensor.c
