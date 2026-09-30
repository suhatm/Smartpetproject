# 宠物环（LIHAObohuan）电源管理 · 开关机 · LED 指示 产品方案

> 版本：V1.0（2026-09-30）
> 硬件基线：LIHAObohuan 主板 fubenxiangmu（nRF54L15-QFAA + nPM1300-QEAA-R7），网表来源《元器件引脚及功能模块对应表》
> 固件基线：nRF Connect SDK v3.4.0，目标板 `nrf54l15dk/nrf54l15/cpuapp`，控制台 SEGGER RTT
> 参考工程：`C:\Users\pc\Desktop\tttt\petChoker`（LIHAObohuan EVT0 电源/按键/LED 已验证实现，本方案在其基础上按新板引脚与需求修订）

---

## 1. 需求定义

| 编号 | 需求 | 说明 |
|---|---|---|
| P1 | SW1 作为唯一开关机按键 | SW1 接 nPM1300 SHPHLD 脚（非 SoC GPIO），按键事件经 PMIC 中断上报主控 |
| P2 | 开机指示 | 开机过程 **LED0（红）闪烁 3 秒** |
| P3 | 关机指示 | 关机过程 **LED1（蓝）闪烁 3 秒**（与开机使用不同指示灯） |
| P4 | 正常工作指示 | 系统就绪/工作正常时 **LED0+LED1 同时闪烁 3 秒** |
| P5 | 三路电源域独立控制 | VDD_SENS_3V0、VDD_STORE_3V0_SW、VDD_ANA_3V0 互不依赖、可单独开关 |
| P6 | nPM1300 模式管理 | Ship / Hibernate / Active 之间按状态机切换 |

LED 与电源轨的硬件对应（来自网表）：

| 信号 | 硬件连接 | 驱动方式 |
|---|---|---|
| LED0（红，LED1/KT-0603R） | VSYS → LED → nPM1300.LED0 脚 | nPM1300 LEDDRV 低边灌电流 5 mA |
| LED1（蓝，LED2/MHT192DBCT） | nPM1300.LED1 脚 → LED → VSYS（**极性存疑，见 §8.1**） | 同上 |
| VDD_SENS_3V0 | nPM1300 LDSW1/LDO1（LSOUT1） | PMIC I2C（BASE 0x08 LDSW 模块） |
| VDD_STORE_3V0_SW | nPM1300 LDSW2/LDO2（LSOUT2） | PMIC I2C（BASE 0x08 LDSW 模块） |
| VDD_ANA_3V0 | VSYS → TPS7A2030（EN=ANA_EN） | SoC GPIO P1.06（**注意与 petChoker EVT0 的 P1.05 不同**） |

---

## 2. 引脚资源与 uart20 冲突审查（结论：必须禁用 uart20）

### 2.1 冲突事实

nRF54L15DK 板级 dts 为 `uart20` 预定义的 pinctrl（`nrf54l15dk_nrf54l_05_10_15-pinctrl.dtsi`）：

| uart20 信号 | DK 默认引脚 | 本板实际用途 | 冲突后果 |
|---|---|---|---|
| UART_TX | **P1.04** | IMU_REF_INT2（主板 IMU 中断输入） | TX 常高驱动 IMU 中断脚 |
| UART_RX | **P1.05** | ADC_HEART（AIN1，PVDF 心率 ADC） | RX 上拉污染模拟输入 |
| UART_RTS | **P1.06** | **ANA_EN**（模拟 LDO 使能输出） | **RTS 常高 → TPS7A2030 始终导通，模拟域无法关断** |
| UART_CTS | **P1.07** | **PMIC_TWI_SDA**（PMIC I2C 数据） | **CTS 上拉与 I2C 总线竞争，PMIC 通信失败** |

**四脚全冲突。** 其中对电源功能直接致命的是 RTS=P1.06（ANA_EN 被拉高，P5 独立控制失效）与 CTS=P1.07（PMIC I2C 不可用，LDO1/LDO2/Led/Ship 全部失效）。

VDD_SENS_3V0 与 VDD_STORE_3V0_SW 本身是 PMIC 的 LDO 输出、不占用 SoC GPIO，但它们的控制通道是 PMIC_TWI（P1.07/P1.08）——P1.07 正是 uart20 CTS，冲突同源。

### 2.2 处置方案（强制）

Zephyr 中 `CONFIG_SHELL=y` / `CONFIG_CONSOLE=y` 会经 `SHELL_BACKEND_SERIAL` **select** `CONFIG_SERIAL`，并按 DK pinctrl 初始化 uart20，覆盖 prj.conf 中的 `=n`（本机 SmartPetRing 项目已踩过此坑）。必须三管齐下：

```ini
# prj.conf
CONFIG_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_USE_SEGGER_RTT=y          # 控制台走 RTT（与 AGENTS.md 一致）
CONFIG_RTT_CONSOLE=y
CONFIG_UART_CONSOLE=n
CONFIG_SERIAL=n
CONFIG_SHELL_BACKEND_SERIAL=n    # 关键：防止 select 链反向打开 SERIAL
```

```dts
/* app.overlay */
&uart20 {
	status = "disabled";       /* 关键：屏蔽 DK 板级 pinctrl 对 P1.04~P1.07 的占用 */
};
```

禁用后，P1.04/P1.05/P1.06/P1.07 完全交给应用（中断输入 / ADC / ANA_EN 输出 / TWIM SDA），冲突解除。

---

## 3. nPM1300 模式与切换策略

| 模式 | 典型电流 | Bucks/VSYS | 唤醒方式 | 本产品用途 |
|---|---|---|---|---|
| **Ship** | ~370 nA | 全关，整机断电 | 长按 SW1 约 0.6 s（ship-to-active-time）或插 VBUS | **"关机"目标状态**；出厂/运输态 |
| **Hibernate** | ~500 nA | Bucks 关（需使能 active discharge 才彻底放干净） | 短按 SW1，或可配置的内部定时器自唤醒 | 预留：需要"快速开机"或定时自检的场景（本期不启用，见 §7.3） |
| **Active** | ~800 nA（PMIC 自身，无负载） | VSYS=电池功率路径直通（**Buck1 未用，SW1 悬空**）、Buck2→VDD_SYS_3V0、LDO1/LDO2 按令 | — | 正常工作态 |
| Buck 调制 | — | Auto（默认，轻载 PFM/重载 PWM 自动切换） | — | 保持 Auto，RF 发射窗口可按需锁 PWM（后期优化项） |

切换路径（状态机见 §5）：

```
Ship ──长按SW1≈0.6s──▶ Active（PMIC 上电，SoC 启动，走开机流程）
Active ──长按SW1 3s松开──▶ 优雅关机流程（关域→LED指示→写 SHIP 任务寄存器）──▶ Ship
Active ──VBUS插入──▶ 充电监护（可继续运行，不强制关机）
任何状态 ──长按SW1 10s──▶ PMIC 硬件 one-button power-cycle（应急恢复，保留默认）
```

要点：
1. **进 Ship 由主机写任务寄存器触发**（BASE 0x0B, offset 0x02, 写 1），写成功后 VSYS 立即切断，SoC 掉电，代码不应再执行（1 s 后仍存活视为故障——沿用 petChoker 判据）。
2. **VBUS 在位时 Ship 请求挂起**：两次间隔 150 ms 采样确认 VBUS 不在位才执行（防刚拔线误判；也避免充电中强制断电）。
3. **Hibernate 实现修正（2026-09-30 固件开发时确认）**：本机 NCS v3.4.0（Zephyr 4.4.0）的 `mfd_npm13xx` 驱动**原生提供** `mfd_npm13xx_hibernate(dev, time_ms)`，无需寄存器级手写；但 DevZone 实测注意事项仍然成立：进 Hibernate 前需使能 Buck/LDSW 的 active discharge，且会被内部定时器唤醒（timed hibernate）。另注意本 SDK 中 nPM1300 归并到 npm13xx 驱动族：Kconfig 符号为 `CONFIG_MFD_NPM13XX`（无 `MFD_NPM1300`），API 为 `mfd_npm13xx_reg_read/write()`、事件枚举 `NPM13XX_EVENT_*`（petChoker 源码是旧版 SDK 命名，不能直接照抄）。

---

## 4. 三路电源域独立控制

| 电源域 | 受电器件 | 控制接口 | 关键时序 |
|---|---|---|---|
| **VDD_SENS_3V0**（LDO1/LDSW1） | 主板 IMU(U4)、双 PDM 麦克风、柔性板 IMU+温度计、QVAR-A 偏置 | LDSW1_SET/CLR（BASE 0x08，offset 0x00/0x01），状态回读 LDSW_STATUS bit0 | I2C 总线（SENS_I2C）上拉 R2/R4 挂在本域：关域后总线无源，需先停传感器再断电 |
| **VDD_STORE_3V0_SW**（LDO2/LDSW2） | SD NAND(U12) | LDSW2_SET/CLR（offset 0x02/0x03），状态回读 bit2 | SPI（P2.01/02/04/05）：**先开电源域、延时 ≥20 ms、再初始化 SPI 并片选**；关域前先 unmount/释放 CSN（R26 47k 上拉在本域，断电后 CSN 自然跟随拉低，防 SD 倒灌） |
| **VDD_ANA_3V0**（TPS7A2030） | OPA4391 四运放（PVDF 心率链） | SoC GPIO P1.06（ANA_EN，R25 100k 下拉保证默认关） | ADC 采样前开、批量间隙可关；**uart20 禁用前 RTS 会把它常开（§2）** |

统一 API（沿用 petChoker `power_control.c` 已验证骨架，改 ANA_EN 引脚）：

```c
enum power_domain { POWER_DOMAIN_SENS, POWER_DOMAIN_STORE, POWER_DOMAIN_ANALOG };
int power_domain_set(enum power_domain d, bool enable);   // 单域开关
int power_domains_all_off(void);                          // 关机序列用
int power_domains_verify_all_off(void);                   // LDSW_STATUS + GPIO 回读校验
int power_control_vbus_present(bool *present);            // VBUS 双采样
int power_control_enter_ship(void);                       // 进 Ship（含 VBUS 防呆）
```

LDSW 全局配置沿用 EVT0 已验证值 `LDSW_CONFIG_SAFE=0xD0`（LS1 10 mA / LS2 20 mA 软启动 + 放电使能），GPISEL/LDOSEL 清零置主机控制模式。

---

## 5. 开关机状态机与按键 UI

### 5.1 状态定义

```
SHIP ──长按0.6s──▶ BOOTING ──LED0闪3s──▶ READY(双灯同闪3s) ──▶ ACTIVE
ACTIVE ──长按3s松开──▶ SHUTTING_DOWN ──LED1闪3s──▶ (关域→进SHIP) ──▶ SHIP
任何运行态 ──VBUS插入──▶ CHARGE_ASSIST（继续运行+充电监护）
任何运行态 ──致命错误──▶ FAULT（红三连闪，2s周期）
任何状态 ──按住10s──▶ PMIC硬件power-cycle（不经过固件）
```

### 5.2 SW1 按键判定（事件源：PMIC IRQ P0.03，NPM1300_EVENT_SHIPHOLD_PRESS/RELEASE）

| 动作 | 判定 | 响应 |
|---|---|---|
| 开机 | Ship 态按住 ≈0.6 s（PMIC 硬件行为） | VSYS 上电、SoC 启动 |
| 关机 | 运行态按住 ≥3 s 后**松开** | 触发优雅关机 |
| 短按 | <1 s 松开 | 状态确认（蓝灯双闪，可选） |
| 武装提示 | 按住 1~3 s | 红灯常亮提示"继续按住将关机"（防误触） |
| 应急 | 按住 10 s | PMIC 硬件复位（默认保留） |

> 关机用"按住 3 s 松开"而非"按住即关"，避免佩戴中误压导致关机——宠物环误触风险高，这是有意设计。

### 5.3 优雅关机序列（顺序不可换）

1. 停传感器/存储业务：SD NAND flush+unmount → 传感器休眠 → 停 ADC/PDM；
2. `power_domains_all_off()`：ANA_EN 拉低 → LDSW1 CLR → LDSW2 CLR；
3. `power_domains_verify_all_off()` 回读校验，失败进 FAULT；
4. **LED1（蓝）闪 3 秒**（关机指示，同时给用户"确实要断了"的视觉确认）；
5. VBUS 双采样：在位 → 挂起关机请求（CHARGE_ASSIST，拔线后续接步骤 6）；不在位 → 直接 6；
6. 熄灯 → 写 TASKENTER_SHIP → VSYS 切断（1 s 后仍运行判 FAULT）。

---

## 6. LED 指示规范

LED0=红（充电/开机域）、LED1=蓝（状态/关机域），经 LEDDRV 主机模式（BASE 0x0A，MODE=0x02 host control）驱动，SET/CLR 任务寄存器开关，带状态缓存省 I2C 流量。

| 场景 | LED0 红 | LED1 蓝 | 时长/周期 |
|---|---|---|---|
| **开机中（BOOTING）** | **闪烁（300 ms on / 300 ms off）** | 灭 | 3 s |
| **就绪/工作正常（READY 一次性确认）** | **同闪（同相同频）** | **同闪** | 3 s（一次性，自检通过后播） |
| **关机中（SHUTTING_DOWN）** | 灭 | **闪烁（300 ms on / 300 ms off）** | 3 s |
| 运行心跳（ACTIVE，可选） | 同闪 80 ms | 同闪 80 ms | 每 3 s 一次（功耗敏感则删除） |
| 关机武装中（按住 1~3 s） | 常亮 | 灭 | 跟随按键 |
| 关机已达时长（按住 ≥3 s） | 常亮 | 常亮 | 跟随按键（提示松手即关） |
| 短按确认 | 灭 | 双闪 80 ms×2 | 一次性 |
| 充电中（VBUS 在位） | 1 s 周期 50% 闪烁 | 灭 | 持续（表示 USB 供电，不代表充电电流） |
| FAULT | 三连闪 120 ms | 灭 | 2 s 周期 |

> 充电指示说明：LED 切到主机模式后 PMIC 自动充电灯效被绕开，充电语义由固件承担（红闪=VBUS 在位）。若产品要求区分"充电中/充满"，需通过 charger 模块读充电状态再细分（本期列为可选项）。

---

## 7. 设备树与软件架构

### 7.1 app.overlay 关键片段（相对 petChoker 的差异已标注）

```dts
/ {
	zephyr,user {
		ana-enable-gpios = <&gpio1 6 GPIO_ACTIVE_HIGH>;  /* ★改：P1.06（EVT0为P1.05）*/
	};
};

&uart20 {                       /* ★新增：解除 P1.04~P1.07 被 DK pinctrl 占用 */
	status = "disabled";
};

/* SENS_I2C(P1.02/P1.03)、PMIC_TWI(P1.07/P1.08=i2c22)、PDM(P1.10/P1.12)：
   pinctrl 覆盖与 petChoker 相同，照搬即可 */

&i2c22 {
	status = "okay";
	npm1300_pmic: pmic@6b {
		compatible = "nordic,npm1300";
		reg = <0x6b>;
		host-int-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;  /* PMIC_IRQ_N，R24 47k上拉 */
		pmic-int-pin = <1>;
		ship-to-active-time-ms = <608>;
		long-press-reset = "one-button";
	};
};
```

### 7.2 模块划分（沿用 petChoker，本板修订点）

| 模块 | 职责 | 本板修订 |
|---|---|---|
| `src/hardware/power_control.c` | LDSW1/LDSW2 寄存器级开关、ANA_EN GPIO、VBUS 检测、进 Ship | ANA_EN P1.05→**P1.06**；其余照搬 |
| `src/hardware/status_led.c` | LEDDRV 主机模式、红/蓝开关、脉冲/闪烁图案 | 照搬；新增 3 秒闪烁图案 API |
| `src/app/main.c` | 电源状态机、SW1 按键 UI、LED 图案调度 | 状态机按 §5/§6 重写（petChoker 是 EVT0 bringup 版，灯效/时长不同） |
| `src/services/`（后续） | 传感器/存储/音频业务 | 电源域开关联动挂接 |

### 7.3 本期范围外（预留）

- Hibernate 模式使能（SDK 已有 `mfd_npm13xx_hibernate()`，接入即可，见 §3 要点 3）；
- 充电状态细分指示（充满/故障灯效）；
- Buck PWM 锁定（RF 窗口降噪）。

### 7.4 固件实现状态（2026-09-30，commit e95ffb0）

基础固件已落地并编译通过（`west build` 无错误；唯一告警 `NRF_PLATFORM_LUMOS deprecated` 来自 DK 板级 defconfig，非应用引入）：
- `prj.conf` / `Kconfig`（APP_POWER_DOMAIN_SELFTEST 等开关）/ `app.overlay`（uart20 disabled、ANA_EN=gpio1.6、PMIC@i2c22@6b）
- `src/hardware/power_control.c`：三域独立开关 + 全关校验 + VBUS 双采样 + 进 Ship
- `src/hardware/status_led.c`：LEDDRV 主机模式 + set/pulse/blink 三类图案 API
- `src/app/main.c`：§5 状态机全量实现（开机红闪 3s → 自检 → 双闪 3s → 主循环；长按 3s 松开 → 关域 → 蓝闪 3s → 进 Ship；充电中挂起）
- 资源占用：FLASH 2.49% / RAM 4.79%
- **待上板验证项**：烧录 + RTT 日志确认状态机行为、三域独立控制、LED2 蓝灯（依赖极性整改）

---

## 8. 风险与待确认项

### 8.1 【硬件·已实锤】LED2（蓝）极性接反 ⚠️⚠️（与"接法一致"的说法不符，见实测证据）

**2026-09-30 通过 EasyEDA 实时 API 对 pw&charge 页逐线追查，三点证据闭环：**

1. **符号引脚定义**：LED1(KT-0603R) 符号 pin1=A、pin2=K；LED2(MHT192DBCT) 符号 **pin1=K、pin2=A**——两个符号库极性定义相反（嘉立创库差异），摆放时又都按 pin1 同侧放置，等效 LED2 旋转 180°。
2. **导线追迹**：LED1.K(490,730)→导线→U7.LED0(375,655) ✓ 正确；**LED2.A(420,730)→导线→U7.LED1(365,655)** ✗ 阳极接到了 PMIC 低边灌电流脚。
3. **网络标志**：VSYS 电源标志位于 (420,785)，恰在 LED 公共轨端点——公共轨连接的是 LED1.A 与 **LED2.K**，即 **LED2 阴极挂在 VSYS**。

结论：LED1 = A→VSYS / K→PMIC.LED0（正确）；**LED2 = K→VSYS / A→PMIC.LED1（接反，恒流低边驱动下永不亮）**。网表导出结果与实时追线一致。

**整改**：EasyEDA 中将 LED2 旋转 180°（使 A→VSYS、K→PMIC.LED1），下一版 PCB 更正；当前板可热风翻转重焊验证。注意旋转后要用网表复核 pin 对应关系，避免再次因符号 pin1/pin2 差异翻车。

### 8.1b 【硬件·已确认】Buck1 未使用，VSYS 直接跟随电池电压

U7.SW1（Buck1 开关节点）悬空、未接电感，Buck1 完全未用；VOUT1 与 VSYS 同网络。因此 **VSYS ≈ VBAT（功率路径直通，Li-ion 3.0~4.2V）**，并非 3.3V 稳压轨。影响：
- LED 阳极挂在 VSYS：红灯 Vf 1.8~2.4V 无压力；
- **蓝灯 Vf 2.5~3.0V + 恒流 sink ≈0.3V headroom → VBAT 低于 ≈3.3V 时蓝灯变暗、低于 ≈2.8~3.0V 点不亮**。整改极性后仍建议换低 Vf 蓝灯（Vf≤2.6V），否则低电量时"关机指示"失效；
- U10(TPS7A2030, IN=VSYS→3.0V) 压差随电池放电增大到 1.2V，注意其功耗与压差裕量（3.0V 输出在 VBAT≥3.1V 时才稳定，LDO dropout≈100mV 尚可）。

### 8.2 【固件】petChoker overlay 不能直接照抄

ANA_EN 已从 P1.05（EVT0）改到 **P1.06**（本板 P1.05 让给 ADC_HEART/AIN1）。直接照抄会把 ANA_EN 配到 ADC 输入脚，模拟域控制失效且可能干扰心率采集。

### 8.3 【固件】uart20 禁用必须双保险

prj.conf 关 SHELL_BACKEND_SERIAL + overlay disable uart20 缺一不可（select 链覆盖问题，§2.2）。

### 8.4 【验证】交付门槛（对齐 AGENTS.md）

1. `ninja -C build` 无错无新增告警；
2. RTT 日志确认：开机→LED0 闪 3s→双灯同闪 3s；长按 3 s 松开→LED1 闪 3s→三域关断回读 OK→进 Ship（掉电）；
3. 三域独立开关交叉验证：单开/单关互不影响（RTT 打印 LDSW_STATUS 与 ana_en 回读）；
4. 长按 10 s 硬件 power-cycle 复测；
5. LED2 蓝灯点亮验证（依赖 §8.1 整改）。

---

## 9. 结论

- **uart20 与本板电源功能冲突成立**（RTS=P1.06/ANA_EN、CTS=P1.07/PMIC_SDA、TX=P1.04/IMU_INT、RX=P1.05/ADC），禁用 uart20 + RTT 控制台后无剩余冲突，VDD_STORE_3V0_SW、VDD_SENS_3V0、ANA_EN 三域可完全独立控制；
- nPM1300 以 **Ship 为关机目标态**（≈370 nA），长按 0.6 s 开机、长按 3 s 松开关机、10 s 硬件应急复位，充电中关机请求自动挂起；
- LED0（红）=开机指示、LED1（蓝）=关机指示、双灯同闪=工作正常，全部 3 秒；
- **唯一硬件级风险是 LED2 极性接反（已实锤：符号库 pin1=K 与 KT-0603R 的 pin1=A 相反，实际 K→VSYS、A→PMIC 低边驱动脚，永不亮），需旋转 180° 整改**；Buck1 未使用、VSYS≈VBAT，蓝灯 Vf 2.5~3V 在低电量时（<3.3V）会变暗/熄灭，建议换低 Vf 蓝灯；其余全部可通过固件实现。
