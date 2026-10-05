/**
 * @file qvar_sensor.c
 * @brief QVAR-A/B 通道采集实现：裸 I2C 寄存器配置 + AH_QVAR_OUT 读取
 *
 * 详见 qvar_sensor.h 头注释与 docs/外设扩展方案_V1.05.md §3。
 *
 * 实现要点：
 *   - 全部走裸 I2C（i2c_write_read），不依赖 Zephyr sensor 驱动 ready；
 *     U4@0x6B 恒在主板，U1@0x6A 在柔性板（FPC 未插时无 ACK → ABSENT）；
 *   - 数据手册时序：改 CTRL7.ah_qvar_en 前 XL/G 必须 power-down，
 *     qvar_channel_enable() 内部完成 保存→关→开→恢复；
 *   - CTRL7 写用 read-modify-write，保留 Zephyr 驱动可能设置的其他位
 *     （当前 NCS 驱动不触碰 CTRL7，但保留兼容性）；
 *   - 读数前查 STATUS_REG.ah_qvarda（bit3），未就绪跳过本帧。
 */
#include "qvar_sensor.h"

#if CONFIG_APP_QVAR_TEST

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "power_control.h"

/* ── LSM6DSV16X 寄存器（NCS v3.4.0 lsm6dsv16x_reg.h 实锤） ── */
#define LSM6DSV16X_REG_WHO_AM_I     0x0FU
#define LSM6DSV16X_WHO_AM_I_VAL     0x70U
#define LSM6DSV16X_REG_CTRL1        0x10U /* accel ODR/FS */
#define LSM6DSV16X_REG_CTRL2        0x11U /* gyro  ODR/FS */
#define LSM6DSV16X_REG_CTRL7        0x16U
#define LSM6DSV16X_REG_STATUS       0x1EU
#define LSM6DSV16X_REG_AH_QVAR_OUT_L 0x3AU /* +H(0x3B) 16bit QVAR */

/* CTRL7 位掩码（小端位域：bit7=ah_qvar_en, bit6=int2_drdy_ah_qvar,
 * bits[5:4]=ah_qvar_c_zin） */
#define CTRL7_AH_QVAR_EN            0x80U
#define CTRL7_AH_QVAR_ZIN_SHIFT     4U
#define CTRL7_AH_QVAR_ZIN_MASK      0x30U

/* STATUS_REG 位掩码 */
#define STATUS_AH_QVARDA            0x08U

/* CTRL1/CTRL2 的 ODR 位段（上板实锤：LSM6DSV16X 的 odr_xl/odr_g 在**低 4 位**，
 * bit3:0=odr、bit6:4=op_mode、bit7 保留——与 LSM6DSO 等老型号"高 4 位 ODR"
 * 布局不同，误用 0xF0 掩码会把值写进 op_mode 且 ODR 恒为 0）。
 * 0 = power-down。 */
#define CTRL_ODR_MASK               0x0FU
#define CTRL_ODR_POWERDOWN          0x00U

/* XL ODR 编码（pid 驱动 lsm6dsv16x_data_rate_t 实锤：30Hz=0x4，低 4 位）。
 * QVAR 数据率跟随 ODR_XL：XL power-down 时 ah_qvarda 永不置位。 */
#define CTRL1_ODR_XL_30HZ           0x04U

/* 通道 → I2C 地址映射：U1 柔性板 0x6A（QVAR-A），U4 主板 0x6B（QVAR-B） */
#define QVAR_A_I2C_ADDR             0x6AU
#define QVAR_B_I2C_ADDR             0x6BU

#define SENS_I2C_NODE               DT_NODELABEL(i2c21)
#define SENS_POWER_SETTLE_MS        25U
#define QVAR_IO_FAIL_THRESHOLD      5U
#define BRINGUP_READ_FRAMES         3U
/* 僵尸 ACTIVE 自愈阈值：连续 EAGAIN（ah_qvarda 不置位且不算 IO 失败）
 * 达此数即强制重跑 enable 序列。30Hz ODR 下 90 帧约 3s。 */
#define QVAR_EAGAIN_REENABLE_LIMIT  90U

static const struct device *const sens_i2c = DEVICE_DT_GET(SENS_I2C_NODE);

/** 每通道运行状态 */
static struct {
	enum qvar_ch_state state;
	enum qvar_zin zin;
	bool sens_powered;
	uint32_t io_fails;
	uint32_t eagain_cnt;
} chans[QVAR_CHANNEL_NUM] = {
	{ .state = QVAR_CH_UNKNOWN, .zin = QVAR_ZIN_2400MOHM,
	  .sens_powered = false, .io_fails = 0, .eagain_cnt = 0 },
	{ .state = QVAR_CH_UNKNOWN, .zin = QVAR_ZIN_2400MOHM,
	  .sens_powered = false, .io_fails = 0, .eagain_cnt = 0 },
};

static uint16_t qvar_addr_of(enum qvar_channel ch)
{
	return ch == QVAR_CHANNEL_A ? QVAR_A_I2C_ADDR : QVAR_B_I2C_ADDR;
}

static int qvar_power_up(void)
{
	static bool powered;
	int ret = power_domain_set(POWER_DOMAIN_SENS, true);

	if (ret != 0) {
		return ret;
	}
	if (!powered) {
		k_msleep(SENS_POWER_SETTLE_MS);
		powered = true;
	}
	return 0;
}

/** 裸 I2C 寄存器读写（read-modify-write 用） */
static int qvar_reg_read(uint16_t addr, uint8_t reg, uint8_t *val, uint8_t len)
{
	return i2c_write_read(sens_i2c, addr, &reg, 1, val, len);
}

static int qvar_reg_write(uint16_t addr, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };

	return i2c_write(sens_i2c, buf, 2, addr);
}

/** WHO_AM_I 探测（0x0F 期望 0x70） */
static bool qvar_present(uint16_t addr)
{
	uint8_t id = 0U;

	if (!device_is_ready(sens_i2c)) {
		return false;
	}
	if (qvar_reg_read(addr, LSM6DSV16X_REG_WHO_AM_I, &id, 1) != 0) {
		return false;
	}
	return id == LSM6DSV16X_WHO_AM_I_VAL;
}

static void qvar_set_state(enum qvar_channel ch, enum qvar_ch_state s)
{
	if (chans[ch].state == s) {
		return;
	}
	printk("QVAR,%s,state,%s->%s\n",
	       ch == QVAR_CHANNEL_A ? "A" : "B",
	       (const char *[]){"UNKNOWN", "ACTIVE", "ABSENT", "DEGRADED"}[chans[ch].state],
	       (const char *[]){"UNKNOWN", "ACTIVE", "ABSENT", "DEGRADED"}[s]);
	chans[ch].state = s;
}

int qvar_channel_enable(enum qvar_channel ch, enum qvar_zin zin)
{
	uint8_t ctrl1, ctrl2, ctrl7;
	uint16_t addr;
	int ret;

	if (ch >= QVAR_CHANNEL_NUM) {
		return -EINVAL;
	}
	addr = qvar_addr_of(ch);

	ret = qvar_power_up();
	if (ret != 0) {
		return ret;
	}

	if (!qvar_present(addr)) {
		qvar_set_state(ch, QVAR_CH_ABSENT);
		return -ENODEV;
	}

	/* 1. 保存 XL/G 当前 ODR（Zephyr 驱动配置的 30Hz） */
	ret = qvar_reg_read(addr, LSM6DSV16X_REG_CTRL1, &ctrl1, 1);
	if (ret == 0) {
		ret = qvar_reg_read(addr, LSM6DSV16X_REG_CTRL2, &ctrl2, 1);
	}
	if (ret != 0) {
		goto io_fail;
	}

	/* 2. 数据手册时序：XL/G 先 power-down 再动 ah_qvar_en */
	ret = qvar_reg_write(addr, LSM6DSV16X_REG_CTRL1,
			     ctrl1 & ~CTRL_ODR_MASK);
	if (ret == 0) {
		ret = qvar_reg_write(addr, LSM6DSV16X_REG_CTRL2,
				     ctrl2 & ~CTRL_ODR_MASK);
	}
	if (ret != 0) {
		goto io_fail;
	}

	/* 3. CTRL7 read-modify-write：置 ah_qvar_en + zin（保留其他位） */
	ret = qvar_reg_read(addr, LSM6DSV16X_REG_CTRL7, &ctrl7, 1);
	if (ret != 0) {
		goto io_fail;
	}
	ctrl7 |= CTRL7_AH_QVAR_EN;
	ctrl7 = (ctrl7 & ~CTRL7_AH_QVAR_ZIN_MASK) |
		((uint8_t)zin << CTRL7_AH_QVAR_ZIN_SHIFT);
	ret = qvar_reg_write(addr, LSM6DSV16X_REG_CTRL7, ctrl7);
	if (ret != 0) {
		goto io_fail;
	}

	/* 4. 恢复 XL/G ODR（QVAR 与六轴可并行输出，ah_qvarda 随 XL drdy）。
	 * 上板实锤的坑：若本模块先于 Zephyr sensor 驱动 device_init 运行
	 * （开机自检阶段），保存的 CTRL1 ODR=0（deferred-init 未配置），
	 * "恢复原值"等于让 XL 继续停摆 → QVAR 永无数据。此时主动给 XL
	 * 配 30Hz（仅 ODR 位段，FS 等低位保持原样；驱动后续 init 会
	 * 按 overlay 全量重配，不冲突）。gyro 不强制开启（省功耗）。 */
	if ((ctrl1 & CTRL_ODR_MASK) == CTRL_ODR_POWERDOWN) {
		ctrl1 = (ctrl1 & ~CTRL_ODR_MASK) | CTRL1_ODR_XL_30HZ;
		printk("QVAR,%s,xl_odr_was_powerdown,force_30hz\n",
		       ch == QVAR_CHANNEL_A ? "A" : "B");
	}
	ret = qvar_reg_write(addr, LSM6DSV16X_REG_CTRL1, ctrl1);
	if (ret == 0) {
		ret = qvar_reg_write(addr, LSM6DSV16X_REG_CTRL2, ctrl2);
	}
	if (ret != 0) {
		goto io_fail;
	}

	chans[ch].zin = zin;
	chans[ch].io_fails = 0;
	chans[ch].eagain_cnt = 0;
	qvar_set_state(ch, QVAR_CH_ACTIVE);
	printk("QVAR,%s,enabled,zin=%u,ctrl7=0x%02x\n",
	       ch == QVAR_CHANNEL_A ? "A" : "B", (unsigned)zin, ctrl7);
	return 0;

io_fail:
	if (++chans[ch].io_fails >= QVAR_IO_FAIL_THRESHOLD) {
		qvar_set_state(ch, QVAR_CH_DEGRADED);
	}
	return ret;
}

int qvar_channel_reconfigure(enum qvar_channel ch)
{
	if (ch >= QVAR_CHANNEL_NUM) {
		return -EINVAL;
	}
	/* 无条件全序列重配：芯片寄存器可能已被 sw_por/POR 清掉而软件
	 * 状态仍 ACTIVE（僵尸 ACTIVE），enable 本身幂等可安全重跑 */
	return qvar_channel_enable(ch, chans[ch].zin);
}

/** 单通道读 16bit QVAR（先查 ah_qvarda 就绪位） */
static int qvar_read_one(enum qvar_channel ch, int16_t *raw)
{
	uint8_t status, buf[2];
	uint16_t addr = qvar_addr_of(ch);
	int ret;

	if (chans[ch].state == QVAR_CH_ABSENT ||
	    chans[ch].state == QVAR_CH_DEGRADED) {
		return -ENODEV;
	}

	ret = qvar_reg_read(addr, LSM6DSV16X_REG_STATUS, &status, 1);
	if (ret != 0) {
		goto io_fail;
	}
	if ((status & STATUS_AH_QVARDA) == 0U) {
		/* task-V1.10 僵尸 ACTIVE 自愈：QVAR 链被 IMU 驱动 sw_por /
		 * SENS 域 POR 静默关闭时，I2C 应答正常、不算 IO 失败，
		 * ah_qvarda 却永不置位，状态机永远停在 ACTIVE。连续无就绪
		 * 达阈值即强制重跑 enable 序列恢复 CTRL7。 */
		if (++chans[ch].eagain_cnt >= QVAR_EAGAIN_REENABLE_LIMIT) {
			chans[ch].eagain_cnt = 0U;
			printk("QVAR,%s,data_stale,reconfig\n",
			       ch == QVAR_CHANNEL_A ? "A" : "B");
			(void)qvar_channel_enable(ch, chans[ch].zin);
		}
		return -EAGAIN; /* 本帧未就绪，调用方按周期重试 */
	}
	chans[ch].eagain_cnt = 0U;

	ret = qvar_reg_read(addr, LSM6DSV16X_REG_AH_QVAR_OUT_L, buf, 2);
	if (ret != 0) {
		goto io_fail;
	}

	*raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
	chans[ch].io_fails = 0;
	return 0;

io_fail:
	if (++chans[ch].io_fails >= QVAR_IO_FAIL_THRESHOLD) {
		qvar_set_state(ch, QVAR_CH_DEGRADED);
	}
	return -EIO;
}

int qvar_read(struct qvar_sample *out)
{
	int16_t raw;
	bool any = false;

	if (out == NULL) {
		return -EINVAL;
	}

	out->a_valid = (qvar_read_one(QVAR_CHANNEL_A, &raw) == 0);
	if (out->a_valid) {
		out->a_raw = raw;
		any = true;
	}

	out->b_valid = (qvar_read_one(QVAR_CHANNEL_B, &raw) == 0);
	if (out->b_valid) {
		out->b_raw = raw;
		any = true;
	}

	return any ? 0 : -ENODEV;
}

int qvar_bringup_test(void)
{
	int16_t raw;
	int active_cnt = 0;
	int ret;

	printk("QVAR_TEST,begin,channels=A(U1@0x6a)+B(U4@0x6b)\n");

	for (enum qvar_channel ch = QVAR_CHANNEL_A; ch < QVAR_CHANNEL_NUM; ch++) {
		const char *name = ch == QVAR_CHANNEL_A ? "A" : "B";
		uint8_t got = 0;

		ret = qvar_channel_enable(ch, chans[ch].zin);
		if (ret == -ENODEV) {
			printk("QVAR_TEST,%s,absent\n", name);
			continue;
		}
		if (ret != 0) {
			printk("QVAR_TEST,%s,fail,enable_rc=%d\n", name, ret);
			continue;
		}

		/* 连读若干帧验证数据路径（QVAR 前端稳定前可能先丢几帧） */
		for (uint8_t i = 0; i < BRINGUP_READ_FRAMES * 4 && got < BRINGUP_READ_FRAMES; i++) {
			k_msleep(40); /* 30Hz ODR 下一帧约 33ms */
			if (qvar_read_one(ch, &raw) == 0) {
				got++;
			}
		}

		if (got > 0) {
			printk("QVAR_TEST,%s,PASS,frames=%u,last_raw=%d\n",
			       name, got, raw);
			active_cnt++;
		} else {
			/* no_data 诊断：dump 关键寄存器一次看全——
			 * CTRL1/2 低 4 位为 ODR（0x4=30Hz），CTRL7 应含 0x80，
			 * STATUS bit0(xlda)/bit3(ah_qvarda) 反映 XL 与 QVAR 链活性 */
			uint8_t c1 = 0, c2 = 0, c7 = 0, st = 0;
			uint16_t a = qvar_addr_of(ch);

			qvar_reg_read(a, LSM6DSV16X_REG_CTRL1, &c1, 1);
			qvar_reg_read(a, LSM6DSV16X_REG_CTRL2, &c2, 1);
			qvar_reg_read(a, LSM6DSV16X_REG_CTRL7, &c7, 1);
			qvar_reg_read(a, LSM6DSV16X_REG_STATUS, &st, 1);
			printk("QVAR_TEST,%s,fail,no_data,ctrl1=0x%02x,ctrl2=0x%02x,"
			       "ctrl7=0x%02x,status=0x%02x\n", name, c1, c2, c7, st);
			qvar_set_state(ch, QVAR_CH_DEGRADED);
		}
	}

	printk("QVAR_TEST,summary,active=%d/2\n", active_cnt);
	if (active_cnt == 0) {
		return 1; /* 全 ABSENT/失败：FPC 未插场景，非致命 */
	}
	return 0;
}

enum qvar_ch_state qvar_channel_state(enum qvar_channel ch)
{
	if (ch >= QVAR_CHANNEL_NUM) {
		return QVAR_CH_ABSENT;
	}
	return chans[ch].state;
}

int qvar_channel_reprobe(enum qvar_channel ch)
{
	if (ch >= QVAR_CHANNEL_NUM) {
		return -EINVAL;
	}
	if (chans[ch].state == QVAR_CH_ACTIVE) {
		/* task-V1.10：不再盲目短路。软件 ACTIVE 不代表寄存器还在
		 * ——IMU 驱动 sw_por / SENS 域 POR 会清 CTRL7.ah_qvar_en
		 * 而状态机不知情（静电 B 恒灰的帮凶）。回读校验，链没了
		 * 就落到下面的全序列重配。 */
		uint8_t ctrl7 = 0U;
		uint16_t addr = qvar_addr_of(ch);

		if (qvar_power_up() == 0 && qvar_present(addr) &&
		    qvar_reg_read(addr, LSM6DSV16X_REG_CTRL7, &ctrl7, 1) == 0 &&
		    (ctrl7 & CTRL7_AH_QVAR_EN) != 0U) {
			return 0; /* 链确实活着 */
		}
	}

	int ret = qvar_channel_enable(ch, chans[ch].zin);

	return ret == 0 ? 0 : (ret == -ENODEV ? 1 : ret);
}

#else /* !CONFIG_APP_QVAR_TEST */

int qvar_bringup_test(void) { return 0; }

int qvar_channel_enable(enum qvar_channel ch, enum qvar_zin zin)
{
	ARG_UNUSED(ch);
	ARG_UNUSED(zin);
	return -ENOTSUP;
}

int qvar_channel_reconfigure(enum qvar_channel ch)
{
	ARG_UNUSED(ch);
	return -ENOTSUP;
}

int qvar_read(struct qvar_sample *out)
{
	ARG_UNUSED(out);
	return -ENOTSUP;
}

enum qvar_ch_state qvar_channel_state(enum qvar_channel ch)
{
	ARG_UNUSED(ch);
	return QVAR_CH_ABSENT;
}

int qvar_channel_reprobe(enum qvar_channel ch)
{
	ARG_UNUSED(ch);
	return 1;
}

#endif /* CONFIG_APP_QVAR_TEST */
