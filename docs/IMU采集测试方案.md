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

固件默认 `APP_IMU_AUTOSTART=y`：**连接即自动开始采集**，App 侧只剩订阅一步。

1. 连接广播名 **SmartPet**
2. （建议）连接后 Request MTU 247（新版 App 默认自动请求）
3. 展开未知服务 `e5a00010-...`，对 `e5a00011` 点 **↓ 订阅**（Enable Notify），
   进入通知页，数据应立刻开始滚动
4. **订阅按钮只点一次**——再点一次就是取消订阅（CCC 0x0001 → 0x0000），
   采集照跑但数据全被丢，表现为"手机端一片空白"（见 §6.5）
5. 停止：写 `00` 到 `e5a00012`，或直接断开连接（断开自动停）

手动启停（`APP_IMU_AUTOSTART=n` 时）：写 `01` 到 `e5a00012` 启动。读
`e5a00011` 返回 3 字节状态 `[active][src:1=trigger 2=polling][seq]`。

### 4.2.1 电脑端操作（推荐，绕开手机 App 的一切不确定性）

```bash
pip install bleak
python tools/imu_ble_monitor.py --scan                 # 确认能扫到
python tools/imu_ble_monitor.py --duration 20 --quiet  # 连/订阅/启动/解码
python tools/imu_ble_monitor.py --csv data.csv         # 存 CSV 便于分析
python tools/imu_ble_monitor.py --raw                  # 附带原始十六进制包
```

注意两点（都实测踩过）：

- **扫描按服务 UUID 匹配**（`e5a00001-...`），不要按名字——Windows 的 BLE
  扫描拿不到 scan response，设备名常为空字符串，按名字找会一直"找不到"。
- 手机若还连着板子，板子已停止广播，电脑扫不到；先断开手机或关掉手机蓝牙。
  板子在断开后会自动重新广播（`BLE,adv_started`）。

### 4.3 RTT 日志关键字

- `IMU_BLE,autostart`：连接即自动开流（APP_IMU_AUTOSTART）
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

## 6. 上板踩坑记录（2026-10-01 实测，两个都是"静默失败"）

这两个坑的共同特征：**函数全部返回 0，日志看上去一切正常，就是没数据**。

### 6.1 drdy 触发注册成功但零中断（`frames=0`）

现象：日志有 `IMU_STREAM,started,src=trigger`，`start_rc=0`，但停止时
`IMU_STREAM,stopped,frames=0`，手机上收不到任何包。

原因：`sensor_trigger_set()` 的 `trig->chan` 传了 `SENSOR_CHAN_ALL`。
驱动 `lsm6dsv16x_trigger_set()` 内部是：

```c
case SENSOR_TRIG_DATA_READY:
    if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) { handler_drdy_acc = handler; enable_xl_int(); }
    else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) { handler_drdy_gyr = handler; enable_g_int(); }
    break;
```

`SENSOR_CHAN_ALL` 两个分支都不命中 → INT1 路由没开、handler 没挂，
但函数照样 `return 0`。

修法：`trig->chan = SENSOR_CHAN_ACCEL_XYZ`（accel/gyro 同 ODR，
挂 accel 一个即可，`sensor_sample_fetch()` 一次读六轴）。
另加 600ms 看门狗：触发模式启动后若一帧未入队，自动降级为 33ms 轮询
（`IMU_STREAM,fallback,polling,reason=no_drdy`），防止走线问题导致静默无数据。

### 6.2 CCC 描述符权限写错 → 手机订阅被拒绝（无 `IMU_BLE,ccc` 日志）

现象：手机 nRF Connect 上 Notify 看起来"已使能"，但设备端日志里
完全没有 `IMU_BLE,ccc`，Notify 一包不发。

原因：`BT_GATT_CCC(_changed, _perm)` 的**第二个参数是权限位，不是 CCC 值**。
写成 `BT_GATT_CCC(ccc_cfg_changed, BT_GATT_CCC_NOTIFY)` 时，
`BT_GATT_CCC_NOTIFY`(0x0001) 恰好等于 `BT_GATT_PERM_READ`(0x01)，
CCC 描述符变成**只读** → 手机写 CCC 被协议栈以
`Write Not Permitted` 拒绝 → `ccc_cfg_changed` 永不回调。

修法：`BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)`
（对齐官方样例 `nrf/samples/bluetooth/peripheral_power_profiling`）。

### 6.3 排查手法备忘：SWD 直读 RTT

本机 J-Link 是克隆探针，Segger 工具链不可用（见用户级记忆），
读日志走 nrfutil 直读 RAM：

1. `.map` 里取 `_SEGGER_RTT` 地址（本次 `0x20001070`）
2. 烧录后先 `nrfutil device write --address 0x20001070 --value 0 --direct`
   再复位——清掉上一版固件遗留的 RTT 控制块，否则新固件日志静默
3. 控制块布局：`acID[16] + MaxUp[4] + MaxDown[4] + aUp[0]`，
   `aUp[0] = {sName, pBuffer, SizeOfBuffer, WrOff, RdOff}`（各 4B）
4. 按 96B 分块 `nrfutil device read --bytes N --direct --family nrf54l`
   读 pBuffer 区（一次读 128B 有时会截断，脚本要校验长度并重试）
   项目内脚本：`tools/rtt_dump.py --map <zephyr.map> --tail N`

### 6.4 定标系数错 1000 倍 → int16 溢出，数据"看着像数据"

现象：通道、通知、帧率全部正常，但物理量离谱——`a=(2196,17568,19312) mg`
（远超 ±8g 量程），`gx` 恒为 -4899。**最容易误判成"传感器坏了/接线不对"。**

原因：`imu_stream.c` 的单位换算分母少写三个 0。

```c
/* 错：1 g 算出 1_000_000 mg，超 int16（32767）后回绕 */
mg = micro * 101971621 / 1e9;
/* 对：8 g 量程对应 8000 mg，永不溢出 */
mg = micro * 101971621 / 1e12;
```

回绕值可反推：`1_000_000 mod 65536 = 16960`，与实测 16836/17324 吻合，
这就是"数据是活的但全是垃圾"的指纹。gyro 同理（`/1e6` 应为 `/1e10`）。

修法要点：
- accel：`mg  = micro_ms2  × 101971621 / 1e12`（1 m/s² = 101.971621 mg）
- gyro ：`dps10 = micro_rads × 5729578 / 1e10`（1 rad/s = 572.957795 dps×10）
- 校验不变式：**静止时 |a| 恒等于 1000 mg**（与摆放姿态无关），
  gyro 各轴应接近 0（只剩噪声）。这条不变式一次就能验出定标错。

### 6.5 手机端"订阅后被立即取消"（采了 11000 帧一包没发）

现象：RTT 日志里每轮连接都是

```
BLE,connected
IMU_BLE,ccc,value=0x0001,notify_on     ← 订阅成功
IMU_BLE,ccc,value=0x0000,notify_off    ← 紧接着又被取消
IMU_BLE,ctrl_cmd=1                     ← 然后才写启动命令
IMU_STREAM,stopped,frames=64,dropped=11122
```

`dropped=11122` 是关键指纹：采集侧 30Hz 跑了约 6 分钟一切正常，但因为
**没有订阅**，`notify_work` 每拍都 `goto resched`，队列 64 帧塞满后全部丢弃，
手机自然永远空屏。

原因不在固件（已用电脑端工具 `tools/imu_ble_monitor.py` 独立验证：
订阅后 800 包 / 1182 帧 / 40s、`丢包=0`，通路完全正常），而是 App 操作顺序：
nRF Connect for Mobile 上订阅（↓）与取消订阅是**同一个按钮**，用户在
"点了订阅没看到数据"（因为当时还没写 `01`，采集没开）时又点了一次，
于是把订阅取消了。

修法（两侧一起做）：

- 固件侧：新增 `APP_IMU_AUTOSTART`（默认 y）——**连接即自动开始采集**，
  App 只需订阅一步，彻底消除"先订阅还是先写控制点"的顺序依赖。
- 使用侧：订阅按钮**只点一次**，之后不要再碰；数据在订阅瞬间就会出现。

### 6.6 附：本机 Windows BLE 扫描的两个坑

- **别按设备名找**：Windows 拿不到 scan response，`adv.local_name` 常为空
  （实测 SmartPet 显示为 `name=''`）。按**广播里的 128-bit 服务 UUID**
  （`e5a00001-...`）匹配才可靠。
- 手机连着时板子已停广播，电脑扫不到；断开后固件会自动重启广播
  （`BLE,adv_started`），无需复位板子。

## 7. 验收基线（2026-10-01 电脑端实测）

| 指标 | 实测值 | 判定 |
|------|--------|------|
| 采样率 | 443 帧 / 15s = 29.5 帧/s | ✅ 等于 30Hz ODR |
| 通知丢包 | 序号丢包 0（800 包/40s 亦为 0） | ✅ |
| 静态 accel | `(3, 19, 1000) mg` → \|a\|=1000 mg = 1g | ✅ 定标正确 |
| 静态 gyro | `(-4, 1, 1) dps×10` ≈ 0 | ✅ |
| 采集模式 | `IMU_STREAM,started,src=trigger` | ✅ INT1 走线正常，未降级轮询 |

## 8. 变更清单（task-V1.03，第三步：定标与自动开流）

- `src/hardware/imu_stream.c`：修正 accel/gyro 定标分母（1e9→1e12、1e6→1e10），
  消除 int16 溢出；补换算推导注释与"静止 \|a\|=1000 mg"不变式说明
- `src/bluetooth/ble_imu_service.c`：连接即自动开流（`IMU_BLE,autostart`）
- `Kconfig`：+`APP_IMU_AUTOSTART`（默认 y，量产可按功耗置 n）
- `tools/imu_ble_monitor.py`（新）：电脑端 BLE 六轴监视器（bleak），
  按服务 UUID 匹配、自动等待设备、解码 mg/dps×10、CSV 落盘
- `tools/rtt_dump.py`（新，前一步）：SWD 直读 RTT 日志
