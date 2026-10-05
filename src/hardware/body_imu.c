/**
 * @file body_imu.c
 * @brief 柔性板 U1 LSM6DSV16XTR 驱动实现：presence 状态机 + 六轴读取
 *
 * 详见 body_imu.h 头注释与 docs/外设扩展方案_V1.05.md §2。
 *
 * 关键实现要点：
 *   - WHO_AM_I 用裸 I2C（i2c_write_read）探测，不依赖驱动是否 ready，
 *     因此 ABSENT 判定不受 deferred-init 状态影响；
 *   - device_init 仅 PRESENT 且未初始化时执行一次（-EALREADY 视为成功）；
 *   - IO 连续失败 BODY_IMU_IO_FAIL_THRESHOLD 次 → DEGRADED，避免
 *     排线松动时每次 read 都在总线上空转超时；
 *   - 单位换算与 imu_stream.c 完全一致（int64 定点 + 四舍五入），
 *     两块 IMU 输出的帧语义可直接对比/融合。
 */
#include "body_imu.h"

#if CONFIG_APP_BODY_IMU_TEST

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "power_control.h"
#include "qvar_sensor.h"

/** U1 设备树节点（app.overlay imu_body_u1: lsm6dsv16x@6a，deferred-init） */
#define BODY_IMU_DT_NODE DT_NODELABEL(imu_body_u1)

#if !DT_NODE_HAS_STATUS(BODY_IMU_DT_NODE, okay)
#error "app.overlay 缺少 imu_body_u1 (st,lsm6dsv16x@6a) 节点"
#endif

/** SENS_I2C 总线节点（裸 I2C WHO_AM_I 探测用） */
#define SENS_I2C_NODE DT_NODELABEL(i2c21)

/** LSM6DSV16X 裸寄存器（bring-up 探测，不依赖驱动 ready） */
#define LSM6DSV16X_REG_WHO_AM_I   0x0FU
#define LSM6DSV16X_WHO_AM_I_VAL   0x70U

/** U1 I2C 地址（柔性板 R1 0Ω 到 GND → SA0=0 → 0x6A） */
#define BODY_IMU_I2C_ADDR         0x6AU

/** 上电后到可响应 I2C 的等待（与 imu_sensor.c 对齐） */
#define SENS_POWER_SETTLE_MS      25U

/** 连续 IO 失败达到该次数后转 DEGRADED（停止频繁空转总线） */
#define BODY_IMU_IO_FAIL_THRESHOLD 5U

static const struct device *const body_imu_dev = DEVICE_DT_GET(BODY_IMU_DT_NODE);
static const struct device *const sens_i2c = DEVICE_DT_GET(SENS_I2C_NODE);

/** 模块运行状态（bring-up 时建立，read/reprobe 维护） */
static struct {
	enum body_imu_state state;
	bool dev_inited;      /* device_init 已成功过 */
	bool sens_powered;    /* SENS 域是否由本模块打开（共享域，不私自下电） */
	uint32_t io_fails;    /* 连续 IO 失败计数（读成功清零） */
} body = {
	.state = BODY_IMU_STATE_UNKNOWN,
	.dev_inited = false,
	.sens_powered = false,
	.io_fails = 0,
};

/** m/s²(micro) -> mg（与 imu_stream.c 同源换算，勿改分母） */
static inline int16_t micro_ms2_to_mg(int64_t micro)
{
	return (int16_t)((micro * 101971621LL + 500000000000LL) /
			 1000000000000LL);
}

/** rad/s(micro) -> dps×10（与 imu_stream.c 同源换算） */
static inline int16_t micro_rads_to_dps10(int64_t micro)
{
	return (int16_t)((micro * 5729578LL + 5000000000LL) /
			 10000000000LL);
}

/** SENS 域确保上电（与 imu_stream/temp/mic 共享，幂等） */
static int body_imu_power_up(void)
{
	int ret = power_domain_set(POWER_DOMAIN_SENS, true);

	if (ret != 0) {
		printk("BODY_IMU,power_fail,rc=%d\n", ret);
		return ret;
	}
	if (!body.sens_powered) {
		k_msleep(SENS_POWER_SETTLE_MS);
		body.sens_powered = true;
	}
	return 0;
}

/** 裸 I2C WHO_AM_I 探测（0x6A，期望 0x70） */
static bool body_imu_who_am_i(void)
{
	uint8_t reg = LSM6DSV16X_REG_WHO_AM_I;
	uint8_t id = 0U;

	if (!device_is_ready(sens_i2c)) {
		return false;
	}
	if (i2c_write_read(sens_i2c, BODY_IMU_I2C_ADDR, &reg, 1, &id, 1) != 0) {
		return false; /* 无 ACK：FPC 未插 / 排线断 / 器件虚焊 */
	}
	return id == LSM6DSV16X_WHO_AM_I_VAL;
}

/** PRESENT 状态下确保驱动已初始化（首次手动 device_init） */
static int body_imu_ensure_driver(void)
{
	int ret;

	if (body.dev_inited) {
		return device_is_ready(body_imu_dev) ? 0 : -ENODEV;
	}

	ret = device_init(body_imu_dev);
	if (ret != 0 && ret != -EALREADY) {
		return ret;
	}
	if (!device_is_ready(body_imu_dev)) {
		return -ENODEV;
	}
	body.dev_inited = true;
	/* task-V1.10 兜底：U1 驱动 init_chip（I2C 分支）的 sw_por 同样会
	 * 清 CTRL7.ah_qvar_en（QVAR-A 链）。开机时序上 A 在 4.9 初始化、
	 * 4.10 才使能 QVAR，本就安全；此处覆盖 reprobe / 域掉电重上电等
	 * 任何再次走到 device_init 的路径（-EALREADY 重入无害）。 */
	(void)qvar_channel_reconfigure(QVAR_CHANNEL_A);
	return 0;
}

static void body_imu_set_state(enum body_imu_state new_state)
{
	if (body.state == new_state) {
		return;
	}
	printk("BODY_IMU,state,%s->%s\n",
	       (const char *[]){"UNKNOWN", "PRESENT", "ABSENT", "DEGRADED"}[body.state],
	       (const char *[]){"UNKNOWN", "PRESENT", "ABSENT", "DEGRADED"}[new_state]);
	body.state = new_state;
}

int body_imu_bringup_test(void)
{
	struct body_imu_frame f;
	int ret;

	printk("BODY_IMU_TEST,begin,addr=0x6a,bus=i2c21\n");

	ret = body_imu_power_up();
	if (ret != 0) {
		printk("BODY_IMU_TEST,fail,power_rc=%d\n", ret);
		return ret;
	}

	/* WHO_AM_I 探测：FPC 未插属预期，ABSENT 不算失败 */
	if (!body_imu_who_am_i()) {
		body_imu_set_state(BODY_IMU_STATE_ABSENT);
		printk("BODY_IMU_TEST,absent,reason=no_ack_or_bad_id\n");
		return 1;
	}

	ret = body_imu_ensure_driver();
	if (ret != 0) {
		printk("BODY_IMU_TEST,fail,driver_init_rc=%d\n", ret);
		body_imu_set_state(BODY_IMU_STATE_ABSENT);
		return 1;
	}

	body_imu_set_state(BODY_IMU_STATE_PRESENT);

	/* 读一帧验证数据通路 */
	ret = body_imu_read_frame(&f);
	if (ret != 0) {
		printk("BODY_IMU_TEST,fail,first_frame_rc=%d\n", ret);
		return 1;
	}

	printk("BODY_IMU_TEST,PASS,a=[%d,%d,%d]mg,g=[%d,%d,%d]dps10\n",
	       f.ax_mg, f.ay_mg, f.az_mg, f.gx_dps10, f.gy_dps10, f.gz_dps10);
	return 0;
}

int body_imu_read_frame(struct body_imu_frame *out)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}

	/* ABSENT/DEGRADED 短路：不发 I2C，让系统在排线未接时照常运行 */
	if (body.state == BODY_IMU_STATE_ABSENT ||
	    body.state == BODY_IMU_STATE_DEGRADED) {
		return -ENODEV;
	}

	ret = body_imu_ensure_driver();
	if (ret != 0) {
		if (++body.io_fails >= BODY_IMU_IO_FAIL_THRESHOLD) {
			body_imu_set_state(BODY_IMU_STATE_DEGRADED);
		}
		return ret;
	}

	if (sensor_sample_fetch(body_imu_dev) != 0 ||
	    sensor_channel_get(body_imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) != 0 ||
	    sensor_channel_get(body_imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro) != 0) {
		if (++body.io_fails >= BODY_IMU_IO_FAIL_THRESHOLD) {
			body_imu_set_state(BODY_IMU_STATE_DEGRADED);
			printk("BODY_IMU,degraded,reason=io_fails>%u\n",
			       BODY_IMU_IO_FAIL_THRESHOLD);
		}
		return -EIO;
	}

	body.io_fails = 0;

	out->ax_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[0]));
	out->ay_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[1]));
	out->az_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[2]));
	out->gx_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[0]));
	out->gy_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[1]));
	out->gz_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[2]));
	return 0;
}

enum body_imu_state body_imu_get_state(void)
{
	return body.state;
}

int body_imu_reprobe(void)
{
	int ret;

	if (body.state == BODY_IMU_STATE_PRESENT) {
		return 0;
	}

	ret = body_imu_power_up();
	if (ret != 0) {
		return ret;
	}

	if (!body_imu_who_am_i()) {
		if (body.state != BODY_IMU_STATE_ABSENT) {
			body_imu_set_state(BODY_IMU_STATE_ABSENT);
		}
		return 1;
	}

	ret = body_imu_ensure_driver();
	if (ret != 0) {
		return 1;
	}

	body.io_fails = 0;
	body_imu_set_state(BODY_IMU_STATE_PRESENT);
	printk("BODY_IMU,reprobe,recovered\n");
	return 0;
}

const struct device *body_imu_get_device(void)
{
	return body_imu_dev;
}

#else /* !CONFIG_APP_BODY_IMU_TEST */

int body_imu_bringup_test(void) { return 0; }

int body_imu_read_frame(struct body_imu_frame *out)
{
	ARG_UNUSED(out);
	return -ENOTSUP;
}

enum body_imu_state body_imu_get_state(void) { return BODY_IMU_STATE_ABSENT; }

int body_imu_reprobe(void) { return 1; }

const struct device *body_imu_get_device(void) { return NULL; }

#endif /* CONFIG_APP_BODY_IMU_TEST */
