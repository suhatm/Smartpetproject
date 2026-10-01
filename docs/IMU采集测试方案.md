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

## 4. 第二步：GATT 数据流（已实现，2026-10-01）

### 4.1 实现（与最初规划的差异说明）

- 设备树：`st,lsm6dsv16x@6b` 挂 i2c21（accel ±8g / 30Hz，gyro 2000dps / 30Hz），
  标记 `zephyr,deferred-init`——VDD_SENS_3V0 复位后默认关闭，驱动若在
  POST_KERNEL 自动初始化会在无电状态下 whoami 失败；改为上电后由
  `imu_stream_start()` 手动 `device_init()`
- 数据通路（MVP 简化）：INT1（P0.00）drdy 触发 → `sensor_sample_fetch`
  单帧读 → 64 帧 `k_msgq` 环形缓冲 → 50ms 打包 notify。
  未用 FIFO watermark 批量读（那需要 SENSOR_ASYNC_API/RTIO 流式，栈开销大，
  30Hz 单帧读完全无压力）；高 ODR 场景再升级
- 触发失败自动降级 33ms 定时轮询（自制板 INT 走线兜底），RTT 日志可辨
- GATT：Service `e5a00010`；数据流 `e5a00011`（Read+Notify）；
  控制点 `e5a00012`（Read+Write，0x01 启动 / 0x00 停止）
- 帧格式（小端）：`[seq:1B][cnt:1B][cnt×12B]`，帧 = ax,ay,az（mg int16）
  + gx,gy,gz（dps×10 int16）；MTU 247 时 20 帧/包，未请求大 MTU 自动缩包
- 断开连接自动停止采集（省电 + 防孤儿流）

### 4.2 App 操作步骤（nRF Connect for Mobile）

1. 连接广播名 **SmartPet**
2. （建议）右上角 Connect 设置里勾选 Request MTU 247 或在连接后手动
   Request MTU（新版 App 默认自动请求大 MTU）
3. 展开未知服务 `e5a00010-...`，对 `e5a00011` 点击 **Enable Notify**
4. 对 `e5a00012` 写入 `01`（Write → 单字节）
5. `e5a00011` 应开始持续收到数据包；读 `e5a00011` 可查 3 字节状态
   `[active][src:1=trigger 2=polling][seq]`
6. 停止：写 `00` 到 `e5a00012`，或直接断开连接

### 4.3 RTT 日志关键字

- `IMU_BLE,ccc,value=0x0001,notify_on`：App 订阅成功
- `IMU_BLE,ctrl_cmd=1` / `IMU_BLE,start_rc=0`：启动命令与结果
- `IMU_STREAM,started,src=trigger,odr=30Hz,fs=8g|2000dps`：触发模式启动
- `IMU_STREAM,started,src=polling,period_ms=33`：降级轮询（INT 有问题）
- `IMU_STREAM,stopped,frames=N,dropped=M,errors=K`：停止统计
- `IMU_BLE,notify_rc=-12`：ACL TX 缓冲耗尽丢包（偶发正常，持续出现调大
  `CONFIG_BT_CONN_TX_MAX` 或降低打包频率）

## 5. 变更清单（task-V1.03）

### 第一步（commit 8183554）

- `app.overlay`：+i2c21（400kHz，pinctrl P1.02/P1.03），&uicr 改 &nfct 标注
- `src/hardware/imu_sensor.c/h`：NFC pad 归还 + 连通性自检
- `src/app/main.c`：启动序列插入 4.5 步自检（失败不阻断启动）
- `Kconfig`：+APP_IMU_TEST
- `CMakeLists.txt`：+imu_sensor.c

### 第二步（本次）

- `app.overlay`：+imu_u4 节点（st,lsm6dsv16x@6b，deferred-init，INT1→P0.00）
- `src/hardware/imu_stream.c/h`（新）：上电→device_init→drdy 触发→
  k_msgq 缓冲，轮询兜底
- `src/bluetooth/ble_imu_service.c/h`（新）：e5a00010 服务 + 50ms 打包
  notify（MTU 自适应）+ 断开自动停
- `prj.conf`：+SENSOR/LSM6DSV16X 驱动、trigger global-thread、BT MTU 247
- `Kconfig`：+APP_BLE_IMU_STREAM；`CMakeLists.txt`：+两个新源文件
- 板测第一步已 PASS（U4@0x6B，WHO_AM_I=0x70）
