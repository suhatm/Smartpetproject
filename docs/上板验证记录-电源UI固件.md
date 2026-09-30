# 电源 UI 固件 · 上板烧录验证记录

- 日期：2026-09-30
- 固件：commit `f39699f`（BOOT,SMARTPET_POWER_UI,v1.0）
- 板卡：LIHAObohuan 宠物环（nRF54L15 + nPM1300），探针 J-Link OB 克隆（SN 4294967295）
- 烧录：`west flash`（nrfutil runner，program + verify + reset 均通过）
- 日志读取：nrfutil device read 直读 RAM 中 RTT 缓冲区（本机 nrfutil 8.2.1 无 rtt 子命令，绕过 Segger 栈）

## 1. 实测 RTT 日志（SWD 直读 0x20000070，512 字节全量重建）

```text
*** Booting nRF Connect SDK v3.4.0-99553055607b ***
*** Using Zephyr OS v4.4.0-b0f801e4e3d19 ***
BOOT,SMARTPET_POWER_UI,v1.0
POWER_POLICY,wake_hold_ms=608,shutdown_hold_ms=3000,emergency_reset_s=10
HW_NOTE,led2_polarity_reversed_expect_no_blue
SELFTEST,domains=PASS,failures=0
SYSTEM_STATE,mode=active,vbus=0,rails=off
LED_MAP,red=nPM1300_LED0,blue=nPM1300_LED1
LED_PATTERN,boot=red_blink_3s,ready=both_blink_3s,shutdown=blue_blink_3s
BUTTON_UI,short=status_ack,hold_1s=arming_red,hold_3s=release_to_ship_red_blue
```

主循环运行 1 分钟以上 WrOff 稳定（无 FAULT、无异常事件、LED 寄存器写持续成功）。

## 2. 通过项

| 验证项 | 结果 | 依据 |
|---|---|---|
| 烧录 + 校验 + 复位 | ✅ | nrfutil runner 报 flashed successfully |
| 固件启动、无早期崩溃 | ✅ | BOOT 行打印，日志推进到主循环 |
| PMIC（nPM1300）I2C 通信 | ✅ | SELFTEST 无 rc 错误 |
| 三电源域独立控制（SENS/STORE/ANALOG 开→回读→关→回读） | ✅ | `SELFTEST,domains=PASS,failures=0` |
| LDSW 主机模式 / ANA_EN(P1.06) GPIO | ✅ | 同上（回读校验含 LDSW 状态位与 GPIO 电平） |
| LEDDRV 主机模式寄存器写 | ✅ | 任一失败会打印 FAULT，全程无 |
| VBUS 初始状态读取 | ✅ | `mode=active,vbus=0`（电池供电运行，未接 USB） |

## 3. 踩坑记录：RTT 缓冲区跨固件残留导致日志静默 ⚠️

**现象**：烧录复位后 RTT 日志一直是旧固件（FSM 演示程序）的内容，且 WrOff 卡在 0x3FF 不动，一度误判为新固件未运行。

**根因**：
1. `.rtt_buff_data`（含 `_SEGGER_RTT` 控制块）是 **NOLOAD 段**（map 文件无 load address），复位不清零；
2. SEGGER RTT 的 `INIT()` 惰性初始化仅在 `acID[0] != 'S'` 时执行——旧固件留下的合法控制块使新固件**跳过初始化**；
3. 旧缓冲区已满（WrOff=1023），默认 NO_BLOCK_SKIP 模式丢弃一切新写入 → 新固件日志全部静默。

**解决办法（SWD 一条命令）**：

```bash
nrfutil device write --address <RTT控制块地址> --value 0 --direct
nrfutil device reset
```

控制块地址取自 `build/zephyr/zephyr.map` 的 `_SEGGER_RTT` 符号（本工程为 0x20000470）。清零 acID 首字后复位，固件会重新 `_DoInit()`，日志恢复正常。

**以后每次烧录新固件若发现 RTT 无输出，先清这个字再复位。**

## 4. 人工验证结果（2026-09-30 电池供电实测，全部通过 ✅）

- [x] 开机 LED0（红）闪 3s / 就绪双灯同闪 3s——目视确认（含 Ship 唤醒后的完整序列）；
- [x] **LED1（蓝）实板可正常点亮**——关机蓝闪 3s 用户目视确认。原"极性接反"判断在实物上不成立
  （网表/符号库分析与实物引脚存在差异，原理图层面建议仍保持整改提示，但实物无需返修）；
- [x] SW1 短按（<1s）：蓝灯双闪状态确认；
- [x] SW1 按住 1~3s：红灯常亮（武装提示）；
- [x] SW1 按住 ≥3s 松开：关域 → 蓝闪 3s → 进 Ship 整机断电。客观证据：断电后 SWD 无法连接
  目标（"Unable to recognize the device: Unknown device family"，探针在位但目标无电），
  电池供电下 VBUS=0 无关机挂起路径，直接进 Ship；
- [x] Ship 态长按唤醒开机（PMIC 硬件 ship-to-active）——唤醒成功，RTT 记录到完整 Boot 日志；
- [x] （附带验证）PMIC 10s 应急 power-cycle：唤醒操作总时长超 10s 触发，RTT 记录到两次连续 Boot。

### 4.1 本轮补充证据（RTT 日志，断电前读出）

```text
[Boot #1] *** Booting nRF Connect SDK v3.4.0 *** → BOOT,SMARTPET_POWER_UI,v1.0
          → SELFTEST,domains=PASS → SYSTEM_STATE,mode=active,vbus=0
[Boot #2] *** Booting nRF Connect SDK v3.4.0 *** → BOOT,SMARTPET_POWER_UI,v1.0
          →（缓冲区满截断；两次 Boot 由超 10s 应急 power-cycle 间隔）
```

按键事件日志（press/release）因进 Ship 断电后 RAM 掉电无法回读，以用户目视确认为准。

## 5. 剩余待验证项（需 USB 线）

- [ ] 充电中（VBUS 在位）长按关机：应挂起关机（`ship_deferred=1`），拔线后真正进 Ship；
- [ ] 充电监护模式 LED 指示（红 1s 周期闪）。

## 6. 心跳指示固件验证（2026-09-30，commit 4395986）

- 新增 `CONFIG_APP_HEARTBEAT_LED`（默认开）：ACTIVE 期间每 5s 双灯同亮 100ms；
  配套 `可配置项.txt` 汇总全部 7 个可配置项。
- 烧录 + RTT 验证通过：`LED_PATTERN,heartbeat=both_flash_100ms_every_5000ms`、
  SELFTEST PASS、无 FAULT、主循环静默运行。
- 构建注意：本 SDK 复用 build 目录增量编译会触发 nrf_security 模块 Kconfig
  malformed 报错（CONFIG_MBEDTLS_CONFIG_FILE 空 cache 变量被回喂 Kconfig），
  需 `west build --pristine` 全量重建。

## 7. 纯电池独立运行排查（2026-09-30 下午，结案：板子正常）

用户报告"拔掉 J-Link 板子就死、无心跳"，经系统性排除法诊断：

| 实验 | 结果 | 排除的假设 |
|---|---|---|
| 拔电池（J-Link 在） | SWD 立即失联 | VTREF 倒灌供电（SoC 电确实来自电池） |
| 查网表 + Nordic 官方资料 | NRESET 内置 13kΩ 永久上拉 | 复位脚悬空导致假死 |
| 电池重插（不按键） | SWD 立即可连 + 完整 boot 日志 | **冷启动默认 Ship 假设——实为自动开机（Active）** |
| 逐根拔 SWD 线（单变量） | 每根拔掉都不死 | 探针驱动 nRESET/供电假设 |
| 4 根全拔盯 20s | **心跳持续在闪** | 全部剩余假设——板子纯电池独立运行正常 |

**结案结论**：板子纯电池独立运行完全正常。此前"无反应"为误判，原因组合：
1. 板子处于 Ship 态时短按无反应（设计行为），需长按 ≥0.6s（OTP 默认更长，建议 2~3s）唤醒；
2. 心跳灯效 100ms/5s 较短促，肉眼易漏看；
3. 插拔电池后 PMIC 冷启动会**自动开机**，无需按键。

**产品注意（待决策）**：nPM1300 OTP 冷启动默认 Active——意味着产品出厂/运输前必须
手动进 Ship（或改 nPM PowerUP OTP 配置默认 Ship），否则插上电池即开机耗电。

已知硬件事实补充：LED2（蓝）实板验证可正常点亮（关机蓝闪、短按确认均可见），
原理图网表层面的"极性接反"判断对实物不成立（符号库 pin 定义差异所致）。

## 8. 三电源域 SWD 邮箱测试（2026-09-30，task-V1.01，commit 14d5d8e）

通信方案：uart20 禁用（引脚冲突）、无 USB CDC → 采用 **SWD 共享内存邮箱**：
固件轮询 RAM 固定地址的 `power_test_mb` 结构（.noinit 段，map 符号解析地址），
Python 上位机 `tools/power_domain_test.py` 经 nrfutil device read/write 读写邮箱。

命令集：`on/off sens|store|ana|all`（单域独立/全开/全关）、`status`（查询）、
`cycle`（自动化独立性测试）。Kconfig：`APP_POWER_DOMAIN_TEST_MB`（默认开，
量产须关）。

自动化测试结果（16/16 PASS）：
- 全关基准 → 逐域单独开/关（SENS/STORE/ANALOG 互不影响）
- 全开 → 全开中逐域单独关 → 恢复 → 全关收尾
- 回读校验：LDSW1/LDSW2 状态位 + ANA_EN GPIO 电平，rc 全 OK

调试中修复：邮箱命令执行后需 20ms settle 再回读 LDSW 状态位
（软启动时延，首轮测试 SENS/STORE 显示 off 的根因）；
上位机 nrfutil 读写增加 3 次自动重试（偶发 worker 超时）。

资源占用：FLASH 2.52% / RAM 4.80%（含测试邮箱）。

