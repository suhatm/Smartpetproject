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

## 4. 待人工验证项（无法远程完成）

- [ ] 开机 LED0（红）闪 3s / 就绪双灯同闪 3s 的**目视**确认；
- [ ] LED1（蓝）：已知极性接反（方案 §8.1），当前预期不亮，整改后复测；
- [ ] SW1 短按（<1s）：蓝灯双闪状态确认；
- [ ] SW1 按住 1~3s：红灯常亮（武装提示）；
- [ ] SW1 按住 ≥3s 松开：关域 → 蓝闪 3s → 进 Ship（整机掉电）；
- [ ] 充电中（VBUS 在位）长按关机：应挂起关机（`ship_deferred=1`），拔线后真正进 Ship；
- [ ] Ship 态长按 0.6s 唤醒开机（PMIC 硬件行为）。
