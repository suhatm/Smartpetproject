/**
 * @file imu_stream.c
 * @brief IMU 六轴数据流实现：deferred 驱动初始化 + drdy 触发 + 环形缓冲
 *
 * 详见 imu_stream.h 头注释与 docs/IMU采集测试方案.md §4。
 *
 * 线程模型：
 *   - drdy 回调跑驱动的 global-thread（系统工作队列），I2C 读取合法；
 *   - 轮询兜底：k_timer(33ms) -> work（ISR 里不做 I2C）；
 *   - 两条路径共用 sample_push()，内部 sensor_sample_fetch + 换算 + 入队。
 *
 * 换算（Zephyr sensor_value = val1 + val2/1e6）：
 *   accel 通道单位 m/s²：mg = micro × 101971621 / 1e9（1 m/s²=101.971621 mg）
 *   gyro  通道单位 rad/s：dps×10 = micro × 572958 / 1e6（1 rad/s=572.9578 dps×10）
 *   均 int64 定点运算 + 四舍五入，无浮点。
 */
#include "imu_stream.h"

#if CONFIG_APP_BLE_IMU_STREAM

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "power_control.h"

/** U4 设备树节点（app.overlay imu_u4: lsm6dsv16x@6b，deferred-init） */
#define IMU_DT_NODE DT_NODELABEL(imu_u4)

#if !DT_NODE_EXISTS(IMU_DT_NODE)
#error "app.overlay 缺少 imu_u4 (st,lsm6dsv16x@6b) 节点"
#endif

static const struct device *const imu_dev = DEVICE_DT_GET(IMU_DT_NODE);

/** 帧环形缓冲：64 帧 × 12B。30Hz 采集 + 50ms 打包，深度足够跨连接间隔 */
#define FRAME_Q_DEPTH 64
K_MSGQ_DEFINE(frame_q, sizeof(struct imu_frame), FRAME_Q_DEPTH, 2);

/** 轮询兜底周期：33ms ≈ 30Hz ODR */
#define POLL_PERIOD_MS 33

/** IMU 上电稳定时间（对齐 imu_sensor.c bring-up 实测值） */
#define SENS_POWER_STABLE_MS 10

/** 运行状态 */
static struct {
	atomic_t active;    /* 0=idle 1=running（imu_stream_start/stop 写） */
	uint8_t src;        /* enum imu_stream_src */
	bool dev_inited;    /* device_init 已成功 */
	uint32_t frames;    /* 累计入队帧数 */
	uint32_t dropped;   /* 缓冲满丢弃帧数 */
	uint32_t errors;    /* fetch/get 失败次数 */
} stream = {
	.active = 0,
	.src = IMU_STREAM_SRC_NONE,
	.dev_inited = false,
	.frames = 0,
	.dropped = 0,
	.errors = 0,
};

/*
 * 坑（2026-10-01 实测踩过）：chan 必须写具体通道。
 * 驱动 lsm6dsv16x_trigger_set() 内部是
 *     if (chan == SENSOR_CHAN_ACCEL_XYZ) {...} else if (chan == SENSOR_CHAN_GYRO_XYZ) {...}
 * 传 SENSOR_CHAN_ALL 两个分支都不进，函数仍返回 0（假成功），
 * 但 INT1 路由没开、handler 没挂 → 永远收不到中断（frames=0）。
 * accel/gyro 同 ODR(30Hz)，只挂 accel 即可，sample_fetch 一次读六轴。
 */
static const struct sensor_trigger drdy_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

/** m/s²(micro) -> mg，int64 + 四舍五入 */
static inline int16_t micro_ms2_to_mg(int64_t micro)
{
	return (int16_t)((micro * 101971621LL + 500000000LL) / 1000000000LL);
}

/** rad/s(micro) -> dps×10，int64 + 四舍五入 */
static inline int16_t micro_rads_to_dps10(int64_t micro)
{
	return (int16_t)((micro * 572958LL + 500000LL) / 1000000LL);
}

/**
 * @brief 采样一帧并入队（drdy 回调线程 / 轮询 work 共用）
 */
static void sample_push(void)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct imu_frame f;

	if (sensor_sample_fetch(imu_dev) != 0) {
		stream.errors++;
		return;
	}
	if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) != 0 ||
	    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro) != 0) {
		stream.errors++;
		return;
	}

	f.ax_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[0]));
	f.ay_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[1]));
	f.az_mg = micro_ms2_to_mg(sensor_value_to_micro(&accel[2]));
	f.gx_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[0]));
	f.gy_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[1]));
	f.gz_dps10 = micro_rads_to_dps10(sensor_value_to_micro(&gyro[2]));

	if (k_msgq_put(&frame_q, &f, K_NO_WAIT) != 0) {
		stream.dropped++; /* 缓冲满：丢最旧意义不大，直接丢新帧计数 */
	} else {
		stream.frames++;
	}
}

/** drdy 触发回调（驱动 global-thread = 系统工作队列，I2C 合法） */
static void drdy_handler(const struct device *dev,
			 const struct sensor_trigger *trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);

	if (atomic_get(&stream.active) == 0) {
		return; /* stop 与 in-flight 回调的竞态：直接忽略 */
	}
	sample_push();
}

/* ---- 轮询兜底：k_timer(ISR) -> work(系统工作队列) ---- */

static void poll_work_handler(struct k_work *work);
static K_WORK_DEFINE(poll_work, poll_work_handler);

static void poll_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&poll_work);
}

static K_TIMER_DEFINE(poll_timer, poll_timer_handler, NULL);

static void poll_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&stream.active) == 0) {
		return;
	}
	sample_push();
}

/*
 * 触发模式看门狗：注册触发成功 ≠ 中断真的来了（INT 走线虚焊 / 引脚复用
 * 冲突都会导致"注册成功但零中断"）。启动后 TRIGGER_WATCHDOG_MS 内若一帧
 * 未入队，自动降级为定时轮询，保证数据流不断。
 */
#define TRIGGER_WATCHDOG_MS 600

static void wd_timer_handler(struct k_timer *timer);

static K_TIMER_DEFINE(trigger_wd_timer, wd_timer_handler, NULL);

static void start_polling_mode(void)
{
	k_timer_start(&poll_timer, K_MSEC(POLL_PERIOD_MS),
		      K_MSEC(POLL_PERIOD_MS));
	stream.src = IMU_STREAM_SRC_POLLING;
	printk("IMU_STREAM,started,src=polling,period_ms=%d\n", POLL_PERIOD_MS);
}

static void wd_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (atomic_get(&stream.active) == 0 ||
	    stream.src != IMU_STREAM_SRC_TRIGGER) {
		return;
	}
	if (stream.frames > 0) {
		return; /* 中断正常，无需降级 */
	}

	/* 摘掉触发，改轮询 */
	(void)sensor_trigger_set(imu_dev, &drdy_trig, NULL);
	start_polling_mode();
	printk("IMU_STREAM,fallback,polling,reason=no_drdy\n");
}

int imu_stream_start(void)
{
	int ret;

	/* 已在跑：幂等，仅清统计重新开始 */
	if (atomic_get(&stream.active) != 0) {
		k_msgq_purge(&frame_q);
		stream.frames = 0;
		stream.dropped = 0;
		stream.errors = 0;
		return 0;
	}

	/* 1. VDD_SENS_3V0 上电（imu_bringup_test 开机已验证过链路） */
	ret = power_domain_set(POWER_DOMAIN_SENS, true);
	if (ret != 0) {
		printk("IMU_STREAM,start_failed,power_rc=%d\n", ret);
		return ret;
	}
	k_msleep(SENS_POWER_STABLE_MS);

	/* 2. deferred 驱动手动初始化（首次；sensor 上电后 whoami 才会过） */
	if (!stream.dev_inited) {
		ret = device_init(imu_dev);
		if (ret != 0 && ret != -EALREADY) {
			printk("IMU_STREAM,start_failed,init_rc=%d\n", ret);
			power_domain_set(POWER_DOMAIN_SENS, false);
			return ret;
		}
		stream.dev_inited = true;
	}
	if (!device_is_ready(imu_dev)) {
		printk("IMU_STREAM,start_failed,not_ready\n");
		return -ENODEV;
	}

	/* 3. drdy 触发；失败降级为定时轮询（自制板 INT 走线兜底） */
	k_msgq_purge(&frame_q);
	stream.frames = 0;
	stream.dropped = 0;
	stream.errors = 0;

	ret = sensor_trigger_set(imu_dev, &drdy_trig, drdy_handler);
	if (ret == 0) {
		stream.src = IMU_STREAM_SRC_TRIGGER;
		printk("IMU_STREAM,started,src=trigger,odr=30Hz,fs=8g|2000dps\n");
		/* 看门狗：600ms 内没帧就降级轮询 */
		k_timer_start(&trigger_wd_timer, K_MSEC(TRIGGER_WATCHDOG_MS),
			      K_NO_WAIT);
	} else {
		printk("IMU_STREAM,trigger_failed_rc=%d,fallback=polling\n", ret);
		start_polling_mode();
	}

	atomic_set(&stream.active, 1);
	return 0;
}

void imu_stream_stop(void)
{
	if (atomic_get(&stream.active) == 0) {
		return; /* 幂等 */
	}

	atomic_set(&stream.active, 0);
	k_timer_stop(&trigger_wd_timer);

	if (stream.src == IMU_STREAM_SRC_TRIGGER) {
		(void)sensor_trigger_set(imu_dev, &drdy_trig, NULL);
	} else if (stream.src == IMU_STREAM_SRC_POLLING) {
		k_timer_stop(&poll_timer);
		(void)k_work_cancel(&poll_work);
	}

	k_msgq_purge(&frame_q);
	printk("IMU_STREAM,stopped,frames=%u,dropped=%u,errors=%u\n",
	       stream.frames, stream.dropped, stream.errors);
	stream.src = IMU_STREAM_SRC_NONE;
}

bool imu_stream_active(void)
{
	return atomic_get(&stream.active) != 0;
}

enum imu_stream_src imu_stream_source(void)
{
	return (enum imu_stream_src)stream.src;
}

size_t imu_stream_drain(struct imu_frame *dst, size_t max)
{
	size_t n = 0;

	while (n < max && k_msgq_get(&frame_q, &dst[n], K_NO_WAIT) == 0) {
		n++;
	}
	return n;
}

#else /* !CONFIG_APP_BLE_IMU_STREAM */

/* 关闭时保留空实现，上层（ble_imu_service）同样受开关控制，不会调用 */

int imu_stream_start(void) { return -ENOTSUP; }
void imu_stream_stop(void) { }
bool imu_stream_active(void) { return false; }
enum imu_stream_src imu_stream_source(void) { return IMU_STREAM_SRC_NONE; }
size_t imu_stream_drain(struct imu_frame *dst, size_t max)
{
	ARG_UNUSED(dst);
	ARG_UNUSED(max);
	return 0;
}

#endif /* CONFIG_APP_BLE_IMU_STREAM */
