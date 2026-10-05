/**
 * @file sensor_hub_bridge.c
 * @brief Sensor Hub 业务桥接实现（协议 V0.5）
 *
 * 调度（全部在系统工作队列，delayable work 链）：
 *   fast_work  33ms：IMU_U4 帧流（imu_stream drain）+ U1 轮询 + QVAR + 阈值判定
 *   pvdf_work  按 rate（默认 100Hz）：PVDF 三路
 *   slow_work   1s：TEMP + MODULE_STATUS（变更时发 EVENT 0x01）+
 *               BATTERY（2s 一拍，低电滞回发 EVENT 0x03）+ WDT 复位事件补报
 *   cmd_work   事件式：指令队列消费 -> 分发 -> CMD_ACK
 *
 * 电源域管理：SENS 域内任一传感器使能 -> 上电；全部关闭 -> 下电。
 *             ANALOG 域由 PVDF 使能/PWR_SET 管理。STORE 由 recorder/sd_store 管理。
 */
#include "sensor_hub_bridge.h"

#if CONFIG_APP_SENSOR_HUB

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ble_sensor_hub.h"
#include "body_imu.h"
#include "batt_gauge.h"
#include "imu_sensor.h"
#include "imu_stream.h"
#include "power_control.h"
#include "pvdf_adc.h"
#include "qvar_sensor.h"
#include "recorder.h"
#include "sd_store.h"
#include "status_led.h"
#include "temp_sensor.h"

/* ---- 指令码（协议 §7.2） ---- */
#define CMD_LED_SET      0x01U
#define CMD_PWR_SET      0x02U
#define CMD_SENSOR_EN    0x03U
#define CMD_RATE_SET     0x04U
#define CMD_QVAR_CFG     0x05U
#define CMD_REPROBE      0x06U
#define CMD_REC_CTRL     0x07U
#define CMD_QVAR_THR_SET 0x08U
#define CMD_REC_LIST     0x09U
#define CMD_REC_READ     0x0AU
#define CMD_REC_DELETE   0x0BU
#define CMD_GET_BATTERY  0x0CU
#define CMD_SD_TEST      0x0DU
#define CMD_GET_STATUS   0x10U
#define CMD_GET_VERSION  0x11U
#define CMD_FACTORY_PING 0x7FU

/** 扩展事件：SD_TEST 完成（协议 V0.5 SD_TEST 应答语义） */
#define EVENT_SD_TEST_DONE 0x07U

/* ---- sensor_id（协议 §7.3） ---- */
enum {
	SENSOR_IMU_U4 = 0,
	SENSOR_IMU_U1 = 1,
	SENSOR_QVAR   = 2,
	SENSOR_PVDF   = 3,
	SENSOR_TEMP   = 4,
	SENSOR_MIC    = 5,
	SENSOR_NUM
};

/* ---- 模块状态编码（协议 6.1） ---- */
#define MOD_ST_UNKNOWN  0U
#define MOD_ST_PRESENT  1U
#define MOD_ST_ABSENT   2U
#define MOD_ST_DEGRADED 3U

#define FW_INFO_STRING "SmartPetRing v1.09;task-V1.09;hubV0.5"

#define FAST_PERIOD_MS  33U   /* ~30Hz：IMU/QVAR */
#define SLOW_PERIOD_MS  1000U /* 1Hz：TEMP/STATUS；2s 分频：BATTERY */

/** 传感器运行配置 */
struct sensor_cfg {
	bool enabled;    /* SENSOR_EN 状态 */
	uint8_t rate_hz; /* RATE_SET 目标（15/30/60；PVDF 默认 100） */
};

static struct sensor_cfg cfg[SENSOR_NUM] = {
	[SENSOR_IMU_U4] = { .enabled = IS_ENABLED(CONFIG_APP_SENSOR_HUB_AUTOSTART), .rate_hz = 30 },
	[SENSOR_IMU_U1] = { .enabled = IS_ENABLED(CONFIG_APP_SENSOR_HUB_AUTOSTART), .rate_hz = 30 },
	[SENSOR_QVAR]   = { .enabled = IS_ENABLED(CONFIG_APP_SENSOR_HUB_AUTOSTART), .rate_hz = 30 },
	[SENSOR_PVDF]   = { .enabled = IS_ENABLED(CONFIG_APP_SENSOR_HUB_AUTOSTART), .rate_hz = 100 },
	[SENSOR_TEMP]   = { .enabled = true, .rate_hz = 1 },
	[SENSOR_MIC]    = { .enabled = false, .rate_hz = 2 },
};

/** 模块实况（MODULE_STATUS 帧 + 变更事件用） */
static uint8_t st_imu_u4 = MOD_ST_UNKNOWN;
static uint8_t st_temp = MOD_ST_UNKNOWN;
static uint8_t st_mic = MOD_ST_UNKNOWN;
static uint8_t last_status[8];
static bool status_sent_once;

/** SENS 域引用计数（域内传感器使能数） */
static uint8_t sens_users;
static bool analog_on;

/** QVAR 阈值判定状态（QVAR_THR_SET，RAM 态掉电不保存） */
struct qvar_thr {
	uint16_t thr;      /* |Δraw| 门限，0=禁用 */
	uint16_t hyst;     /* 滞回 */
	int32_t baseline;  /* EMA 基线（raw 定点，×16） */
	bool crossed;      /* 当前是否处于越限态 */
	int64_t last_evt_ms; /* 限速：每通道每秒最多 4 次 */
};

static struct qvar_thr qvar_thr[QVAR_CHANNEL_NUM];

/** WDT 复位事件待报 */
static uint32_t wdt_reset_cause;
static bool wdt_event_pending;

/** 低电事件边沿 */
static bool low_batt_active;

/* ---- 工作项 ---- */

static void fast_work_handler(struct k_work *work);
static void pvdf_work_handler(struct k_work *work);
static void slow_work_handler(struct k_work *work);
static void cmd_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(fast_work, fast_work_handler);
static K_WORK_DELAYABLE_DEFINE(pvdf_work, pvdf_work_handler);
static K_WORK_DELAYABLE_DEFINE(slow_work, slow_work_handler);
static K_WORK_DEFINE(cmd_work, cmd_work_handler);

void sensor_hub_bridge_cmd_kick(void)
{
	k_work_submit(&cmd_work);
}

/* ---- ACK 发送 ---- */

static void send_ack(uint8_t cmd, uint8_t req_seq, int8_t result,
		     const uint8_t *data, uint8_t data_len)
{
	uint8_t payload[3 + 64];

	if (data_len > 64U) {
		data_len = 64U;
	}
	payload[0] = cmd;
	payload[1] = req_seq;
	payload[2] = (uint8_t)result;
	if (data_len > 0U && data != NULL) {
		memcpy(&payload[3], data, data_len);
	}
	(void)hub_event_frame(HUB_TYPE_CMD_ACK, payload, (uint8_t)(3U + data_len));
}

/* ---- SENS 域电源管理 ---- */

static int sens_acquire(void)
{
	if (sens_users == 0U) {
		int ret = power_domain_set(POWER_DOMAIN_SENS, true);

		if (ret != 0) {
			return ret;
		}
		k_msleep(20); /* LDO 上电 + 传感器 POR */
	}
	sens_users++;
	return 0;
}

static void sens_release(void)
{
	if (sens_users > 0U) {
		sens_users--;
	}
	if (sens_users == 0U) {
		(void)power_domain_set(POWER_DOMAIN_SENS, false);
	}
}

/* ---- IMU_U4 流控 ---- */

static bool u4_streaming;

static int u4_stream_start(void)
{
	if (u4_streaming) {
		return 0;
	}
	int ret = sens_acquire();

	if (ret != 0) {
		st_imu_u4 = MOD_ST_DEGRADED;
		return ret;
	}
	ret = imu_stream_start();
	if (ret != 0) {
		sens_release();
		st_imu_u4 = MOD_ST_DEGRADED;
		printk("BRIDGE,imu_u4_start_fail,rc=%d\n", ret);
		return ret;
	}
	u4_streaming = true;
	st_imu_u4 = MOD_ST_PRESENT;
	return 0;
}

static void u4_stream_stop(void)
{
	if (!u4_streaming) {
		return;
	}
	imu_stream_stop();
	u4_streaming = false;
	st_imu_u4 = MOD_ST_UNKNOWN;
	sens_release();
}

/* ---- 帧组装：各传感器 ---- */

static void imu_u4_tick(void)
{
	struct imu_frame frames[19];
	uint8_t payload[1 + 19 * 12];
	size_t cnt;

	if (!cfg[SENSOR_IMU_U4].enabled || !u4_streaming) {
		return;
	}

	cnt = imu_stream_drain(frames, ARRAY_SIZE(frames));
	if (cnt == 0U) {
		return;
	}

	payload[0] = MOD_ST_PRESENT |
		     ((uint8_t)imu_stream_source() << 4); /* bit7:4=数据来源 */
	for (size_t i = 0; i < cnt; i++) {
		uint8_t *p = &payload[1 + i * 12];
		const int16_t *v = (const int16_t *)&frames[i];

		for (uint8_t k = 0; k < 6U; k++) {
			p[k * 2] = (uint8_t)(v[k] & 0xFF);
			p[k * 2 + 1] = (uint8_t)(v[k] >> 8);
		}
	}
	(void)hub_stream_frame(HUB_TYPE_IMU_U4, payload,
			       (uint8_t)(1U + cnt * 12U));
}

static void imu_u1_tick(void)
{
	struct body_imu_frame f;
	uint8_t payload[13];

	if (!cfg[SENSOR_IMU_U1].enabled) {
		return;
	}
	if (body_imu_get_state() == BODY_IMU_STATE_ABSENT) {
		return; /* ABSENT 不发帧，状态仅在 MODULE_STATUS 体现 */
	}
	if (body_imu_read_frame(&f) != 0) {
		return;
	}

	payload[0] = MOD_ST_PRESENT;
	const int16_t *v = (const int16_t *)&f;

	for (uint8_t k = 0; k < 6U; k++) {
		payload[1 + k * 2] = (uint8_t)(v[k] & 0xFF);
		payload[2 + k * 2] = (uint8_t)(v[k] >> 8);
	}
	(void)hub_stream_frame(HUB_TYPE_BODY_IMU_U1, payload,
			       sizeof(payload));
}

/** QVAR-B 硬件约束：FPC 未插（U1 ABSENT）时电极开路，判 DEGRADED（P0-1） */
static uint8_t qvar_b_effective_state(void)
{
	if (body_imu_get_state() == BODY_IMU_STATE_ABSENT) {
		return MOD_ST_DEGRADED;
	}
	switch (qvar_channel_state(QVAR_CHANNEL_B)) {
	case QVAR_CH_ACTIVE:
		return MOD_ST_PRESENT;
	case QVAR_CH_ABSENT:
		return MOD_ST_ABSENT;
	case QVAR_CH_DEGRADED:
		return MOD_ST_DEGRADED;
	default:
		return MOD_ST_UNKNOWN;
	}
}

static uint8_t qvar_a_effective_state(void)
{
	switch (qvar_channel_state(QVAR_CHANNEL_A)) {
	case QVAR_CH_ACTIVE:
		return MOD_ST_PRESENT;
	case QVAR_CH_ABSENT:
		return MOD_ST_ABSENT;
	case QVAR_CH_DEGRADED:
		return MOD_ST_DEGRADED;
	default:
		return MOD_ST_UNKNOWN;
	}
}

static void qvar_thr_check(enum qvar_channel ch, int16_t raw)
{
	struct qvar_thr *t = &qvar_thr[ch];

	if (t->thr == 0U) {
		return;
	}

	/* EMA 基线（1/16 权重，定点 ×16） */
	t->baseline += ((int32_t)raw * 16 - t->baseline) / 16;
	int32_t delta = (int32_t)raw - (t->baseline / 16);
	int64_t now = k_uptime_get();

	if (!t->crossed && (delta > (int32_t)t->thr || delta < -(int32_t)t->thr)) {
		if ((now - t->last_evt_ms) >= 250) {
			uint8_t payload[5] = {
				HUB_EVENT_QVAR_THR_CROSSED, (uint8_t)ch, 1U,
				(uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8),
			};

			(void)hub_event_frame(HUB_TYPE_EVENT, payload,
					      sizeof(payload));
			t->last_evt_ms = now;
			t->crossed = true;
		}
	} else if (t->crossed &&
		   delta < ((int32_t)t->thr - (int32_t)t->hyst) &&
		   delta > -((int32_t)t->thr - (int32_t)t->hyst)) {
		if ((now - t->last_evt_ms) >= 250) {
			uint8_t payload[5] = {
				HUB_EVENT_QVAR_THR_CROSSED, (uint8_t)ch, 0U,
				(uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8),
			};

			(void)hub_event_frame(HUB_TYPE_EVENT, payload,
					      sizeof(payload));
			t->last_evt_ms = now;
			t->crossed = false;
		}
	}
}

static void qvar_tick(void)
{
	struct qvar_sample s;
	uint8_t payload[5];

	if (!cfg[SENSOR_QVAR].enabled) {
		return;
	}
	if (qvar_read(&s) != 0) {
		return;
	}

	/* FPC 约束：U1 ABSENT 时 B 通道 valid 清 0（电极开路） */
	if (body_imu_get_state() == BODY_IMU_STATE_ABSENT) {
		s.b_valid = false;
	}
	if (!s.a_valid && !s.b_valid) {
		return; /* 双通道都无效不发帧 */
	}

	/* 帧级状态（协议 §6.1，task-V1.09）：任一通道 DEGRADED 时报 DEGRADED，
	 * 上位机按 valid 位图区分——有效通道显示 PRESENT，无效通道落到帧级
	 * DEGRADED（V0.5 补充：QVAR-B 电极经 FPC 引出，FPC 异常判 DEGRADED
	 * 而非 ABSENT）。原实现只报 PRESENT/ABSENT，无效通道被误显示为 ABSENT。 */
	uint8_t st_a = qvar_a_effective_state();
	uint8_t st_b = qvar_b_effective_state();
	uint8_t st = (st_a == MOD_ST_DEGRADED || st_b == MOD_ST_DEGRADED)
			     ? MOD_ST_DEGRADED
			     : MOD_ST_PRESENT;

	payload[0] = st;
	payload[1] = (uint8_t)(s.a_raw & 0xFF);
	payload[2] = (uint8_t)(s.a_raw >> 8);
	payload[3] = (uint8_t)(s.b_raw & 0xFF);
	payload[4] = (uint8_t)((s.b_raw >> 8) & 0xFF);
	/* 注：5B 载荷放不下 valid 位图 -> 协议 6.3 为 5B：状态+2+2+valid。
	 * 修正：a_raw/b_raw/valid 共 1+2+2+1=6B。协议原文 "载荷 5 B：
	 * 状态字节 1 B + a_raw（int16）+ b_raw（int16）+ valid 位图 1 B"
	 * 实为 6B（1+2+2+1），固件按 6B 发（见 docx V0.5 勘误备注）。 */
	uint8_t payload6[6];

	memcpy(payload6, payload, 5);
	payload6[5] = (uint8_t)((s.a_valid ? 0x01U : 0U) |
				(s.b_valid ? 0x02U : 0U));
	(void)hub_stream_frame(HUB_TYPE_QVAR, payload6, sizeof(payload6));

	if (s.a_valid) {
		qvar_thr_check(QVAR_CHANNEL_A, s.a_raw);
	}
	if (s.b_valid) {
		qvar_thr_check(QVAR_CHANNEL_B, s.b_raw);
	}
}

static void pvdf_tick(void)
{
	struct pvdf_sample s;
	uint8_t payload[13];

	if (!cfg[SENSOR_PVDF].enabled) {
		return;
	}
	if (pvdf_read(&s) != 0) {
		return;
	}

	payload[0] = MOD_ST_PRESENT;
	const int32_t vals[3] = { s.heart_mv, s.raw_mv, s.ref_mv };

	for (uint8_t k = 0; k < 3U; k++) {
		uint32_t v = (uint32_t)vals[k];

		payload[1 + k * 4] = (uint8_t)(v & 0xFFU);
		payload[2 + k * 4] = (uint8_t)((v >> 8) & 0xFFU);
		payload[3 + k * 4] = (uint8_t)((v >> 16) & 0xFFU);
		payload[4 + k * 4] = (uint8_t)((v >> 24) & 0xFFU);
	}
	(void)hub_stream_frame(HUB_TYPE_PVDF, payload, sizeof(payload));
}

static void temp_tick(void)
{
	int32_t mdeg;
	uint8_t payload[3];

	if (!cfg[SENSOR_TEMP].enabled) {
		return;
	}
	if (temp_read_mdeg(&mdeg) != 0) {
		if (st_temp != MOD_ST_ABSENT) {
			st_temp = MOD_ST_ABSENT;
		}
		return; /* ABSENT 停发 */
	}
	st_temp = MOD_ST_PRESENT;

	int16_t cdeg = (int16_t)(mdeg / 10); /* mdeg -> 0.01℃ */

	payload[0] = MOD_ST_PRESENT;
	payload[1] = (uint8_t)(cdeg & 0xFF);
	payload[2] = (uint8_t)(cdeg >> 8);
	(void)hub_stream_frame(HUB_TYPE_TEMP, payload, sizeof(payload));
}

static void battery_tick(bool vbus_present)
{
	struct batt_reading b;

	(void)batt_gauge_update(vbus_present);
	batt_gauge_get(&b);

	uint8_t payload[5] = {
		MOD_ST_PRESENT,
		(uint8_t)(b.vbat_mv & 0xFFU), (uint8_t)(b.vbat_mv >> 8),
		b.percent,
		(uint8_t)((b.charging ? 0x01U : 0U) |
			  (b.full ? 0x02U : 0U) | (b.low ? 0x04U : 0U)),
	};

	(void)hub_stream_frame(HUB_TYPE_BATTERY, payload, sizeof(payload));

	/* 低电事件：上升沿发一次（滞回在 batt_gauge 内） */
	if (b.low && !low_batt_active) {
		uint8_t evt[4] = {
			HUB_EVENT_LOW_BATTERY,
			(uint8_t)(b.vbat_mv & 0xFFU),
			(uint8_t)(b.vbat_mv >> 8),
			b.percent,
		};

		(void)hub_event_frame(HUB_TYPE_EVENT, evt, sizeof(evt));
	}
	low_batt_active = b.low;
}

static void module_status_tick(bool vbus_present)
{
	uint8_t st[8] = { 0 };
	uint8_t ldsw;
	bool ana;

	/* 模块状态位图 1：IMU_U4/IMU_U1/QVAR_A/QVAR_B */
	uint8_t st_u1;

	switch (body_imu_get_state()) {
	case BODY_IMU_STATE_PRESENT:
		st_u1 = MOD_ST_PRESENT;
		break;
	case BODY_IMU_STATE_ABSENT:
		st_u1 = MOD_ST_ABSENT;
		break;
	case BODY_IMU_STATE_DEGRADED:
		st_u1 = MOD_ST_DEGRADED;
		break;
	default:
		st_u1 = MOD_ST_UNKNOWN;
		break;
	}

	st[0] = st_imu_u4 | (uint8_t)(st_u1 << 2) |
		(uint8_t)(qvar_a_effective_state() << 4) |
		(uint8_t)(qvar_b_effective_state() << 6);

	/* 模块状态位图 2：PVDF/TEMP/MIC/SD */
	uint8_t st_pvdf = (pvdf_adc_get_state() == PVDF_ADC_READY)
				  ? MOD_ST_PRESENT
				  : ((pvdf_adc_get_state() == PVDF_ADC_DEGRADED)
					     ? MOD_ST_DEGRADED
					     : MOD_ST_UNKNOWN);
	uint8_t st_sd = (uint8_t)sd_get_state();

	st[1] = st_pvdf | (uint8_t)(st_temp << 2) | (uint8_t)(st_mic << 4) |
		(uint8_t)(st_sd << 6);

	/* 电源域实况 */
	if (power_control_get_state(&ldsw, &ana) == 0) {
		st[2] = (uint8_t)(((ldsw & 0x01U) ? 0x01U : 0U) |
				  ((ldsw & 0x04U) ? 0x02U : 0U) |
				  (ana ? 0x04U : 0U));
	}

	/* LED 实况：上报实际生效状态（task-V1.07 起不再直接报 override
	 * 请求值——故障/按键图案优先级更高、override 未必真正生效，
	 * 上报真实值才能保证上位机 UI 与板上实灯一致） */
	bool led_r;
	bool led_b;

	if (status_led_get(&led_r, &led_b) == 0) {
		st[3] = (uint8_t)((led_r ? 0x01U : 0U) | (led_b ? 0x02U : 0U));
	}

	/* 电量 */
	struct batt_reading b;

	batt_gauge_get(&b);
	st[4] = b.percent;

	(void)hub_stream_frame(HUB_TYPE_MODULE_STATUS, st, sizeof(st));

	/* 变更检测：逐模块发 EVENT 0x01 */
	if (status_sent_once) {
		static const char *const names[8] = {
			"IMU_U4", "IMU_U1", "QVAR_A", "QVAR_B",
			"PVDF", "TEMP", "MIC", "SD",
		};
		uint8_t cur[8] = {
			(uint8_t)(st[0] & 0x03U), (uint8_t)((st[0] >> 2) & 0x03U),
			(uint8_t)((st[0] >> 4) & 0x03U), (uint8_t)((st[0] >> 6) & 0x03U),
			(uint8_t)(st[1] & 0x03U), (uint8_t)((st[1] >> 2) & 0x03U),
			(uint8_t)((st[1] >> 4) & 0x03U), (uint8_t)((st[1] >> 6) & 0x03U),
		};
		uint8_t prev[8] = {
			(uint8_t)(last_status[0] & 0x03U),
			(uint8_t)((last_status[0] >> 2) & 0x03U),
			(uint8_t)((last_status[0] >> 4) & 0x03U),
			(uint8_t)((last_status[0] >> 6) & 0x03U),
			(uint8_t)(last_status[1] & 0x03U),
			(uint8_t)((last_status[1] >> 2) & 0x03U),
			(uint8_t)((last_status[1] >> 4) & 0x03U),
			(uint8_t)((last_status[1] >> 6) & 0x03U),
		};

		for (uint8_t i = 0; i < 8U; i++) {
			if (cur[i] != prev[i]) {
				uint8_t evt[3] = {
					HUB_EVENT_MODULE_STATE_CHANGED,
					i, cur[i],
				};

				(void)hub_event_frame(HUB_TYPE_EVENT, evt,
						      sizeof(evt));
				printk("BRIDGE,mod_state,%s,%u->%u\n",
				       names[i], prev[i], cur[i]);
			}
		}
	}
	memcpy(last_status, st, sizeof(st));
	status_sent_once = true;
}

/* ---- 工作项实现 ---- */

static void fast_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	imu_u4_tick();
	imu_u1_tick();
	qvar_tick();
	k_work_reschedule(&fast_work, K_MSEC(FAST_PERIOD_MS));
}

static void pvdf_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	pvdf_tick();
	uint8_t rate = cfg[SENSOR_PVDF].rate_hz;
	uint32_t period = (rate > 0U && rate <= 200U) ? (1000U / rate) : 10U;

	k_work_reschedule(&pvdf_work, K_MSEC(period));
}

static void slow_work_handler(struct k_work *work)
{
	static uint8_t batt_div;
	bool vbus = false;

	ARG_UNUSED(work);

	(void)power_control_vbus_present(&vbus);

	temp_tick();

	if ((batt_div++ % 2U) == 0U) {
		battery_tick(vbus);
	}

	module_status_tick(vbus);

	/* WDT 复位事件补报（订阅后生效） */
	if (wdt_event_pending && hub_event_subscribed()) {
		uint8_t evt[5] = {
			HUB_EVENT_WDT_RESET,
			(uint8_t)(wdt_reset_cause & 0xFFU),
			(uint8_t)((wdt_reset_cause >> 8) & 0xFFU),
			(uint8_t)((wdt_reset_cause >> 16) & 0xFFU),
			(uint8_t)((wdt_reset_cause >> 24) & 0xFFU),
		};

		(void)hub_event_frame(HUB_TYPE_EVENT, evt, sizeof(evt));
		wdt_event_pending = false;
	}

	k_work_reschedule(&slow_work, K_MSEC(SLOW_PERIOD_MS));
}

/* ---- 指令处理 ---- */

static void handle_led_set(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 1U) {
		send_ack(CMD_LED_SET, req_seq, -1, NULL, 0);
		return;
	}
	hub_led_override_set((uint8_t)(p[0] & 0x03U));
	send_ack(CMD_LED_SET, req_seq, 0, NULL, 0);
}

static void handle_pwr_set(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U || p[0] > 2U || p[1] > 1U) {
		send_ack(CMD_PWR_SET, req_seq, -1, NULL, 0);
		return;
	}

	uint8_t dom = p[0];
	bool on = p[1] != 0U;
	int ret;

	/* 录音/监听/传输进行中拒绝断相关域电：麦克风挂 SENS、WAV 文件挂
	 * STORE（task-V1.07：曾可边录音边切电，卡掉电后 FATFS 状态脱钩，
	 * 后续录音静默失败且上位机卡"等待启动"） */
	if (!on && ((dom == POWER_DOMAIN_SENS) ||
		    (dom == POWER_DOMAIN_STORE)) && recorder_busy()) {
		send_ack(CMD_PWR_SET, req_seq, -11, NULL, 0);
		return;
	}

	if (dom == POWER_DOMAIN_SENS) {
		if (on) {
			ret = power_domain_set(POWER_DOMAIN_SENS, true);
			if (ret == 0) {
				/* PWR_SET 是电源权威控制：关断期间域内传感器
				 * 已 POR，须重建引用计数 + 重启数据流 + 重探测
				 * 才能恢复（此前关->开后 IMU 永久哑火） */
				sens_users = 0U;
				if (cfg[SENSOR_IMU_U4].enabled &&
				    (u4_stream_start() != 0)) {
					st_imu_u4 = MOD_ST_DEGRADED;
				}
				if (cfg[SENSOR_IMU_U1].enabled ||
				    cfg[SENSOR_QVAR].enabled ||
				    cfg[SENSOR_TEMP].enabled) {
					(void)sens_acquire();
					(void)body_imu_reprobe();
					(void)qvar_channel_reprobe(
						QVAR_CHANNEL_A);
					(void)qvar_channel_reprobe(
						QVAR_CHANNEL_B);
				}
			}
		} else {
			/* 停 U4 流（释放其引用）后强制清零基线引用，
			 * 确保域真正断电（协议 §7.4） */
			if (u4_streaming) {
				u4_stream_stop();
				st_imu_u4 = MOD_ST_DEGRADED;
			}
			sens_users = 0U;
			ret = power_domain_set(POWER_DOMAIN_SENS, false);
		}
	} else if (dom == POWER_DOMAIN_STORE) {
		if (on) {
			ret = power_domain_set(POWER_DOMAIN_STORE, true);
		} else {
			/* 先卸载 FATFS 再断电（sd_unmount 一并负责）：
			 * 直接切电会让 mounted 标志与卡实际状态脱钩，
			 * 之后 REC_* / SD_TEST 全部静默失败 */
			sd_unmount();
			ret = 0;
		}
	} else { /* POWER_DOMAIN_ANALOG */
		ret = power_domain_set(POWER_DOMAIN_ANALOG, on);
		if (ret == 0) {
			/* 同步桥接状态：否则 SENSOR_EN(PVDF) 的 !analog_on
			 * 判断失真；断电期间 PVDF 读数为垃圾，停帧 */
			analog_on = on;
			cfg[SENSOR_PVDF].enabled = on;
		}
	}

	send_ack(CMD_PWR_SET, req_seq, (int8_t)(ret == 0 ? 0 : -5), NULL, 0);
}

static void handle_sensor_en(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U || p[0] >= SENSOR_NUM || p[1] > 1U) {
		send_ack(CMD_SENSOR_EN, req_seq, -1, NULL, 0);
		return;
	}

	uint8_t sid = p[0];
	bool on = p[1] != 0U;
	int8_t result = 0;

	if (sid == SENSOR_IMU_U4) {
		if (on && !cfg[sid].enabled) {
			result = (u4_stream_start() == 0) ? 0 : -5;
		} else if (!on && cfg[sid].enabled) {
			u4_stream_stop();
		}
	} else if (sid == SENSOR_MIC) {
		if (on && !cfg[sid].enabled) {
			result = (recorder_monitor_start() == REC_OK) ? 0 : -11;
		} else if (!on && cfg[sid].enabled) {
			(void)recorder_monitor_stop();
		}
		st_mic = on ? MOD_ST_PRESENT : MOD_ST_UNKNOWN;
	} else if (sid == SENSOR_PVDF) {
		if (on && !analog_on) {
			if (power_domain_set(POWER_DOMAIN_ANALOG, true) == 0) {
				analog_on = true;
			} else {
				result = -5;
			}
		} else if (!on && analog_on) {
			(void)power_domain_set(POWER_DOMAIN_ANALOG, false);
			analog_on = false;
		}
	} else if (sid == SENSOR_QVAR || sid == SENSOR_IMU_U1 ||
		   sid == SENSOR_TEMP) {
		/* SENS 域内：由 bring-up/读取路径自管理，开关仅控帧流 */
	}

	if (result == 0) {
		cfg[sid].enabled = on;
	}
	send_ack(CMD_SENSOR_EN, req_seq, result, NULL, 0);
}

static void handle_rate_set(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U || p[0] >= SENSOR_NUM) {
		send_ack(CMD_RATE_SET, req_seq, -1, NULL, 0);
		return;
	}

	uint8_t sid = p[0];
	uint8_t rate = p[1];

	/* IMU/QVAR 支持 15/30/60；PVDF 1~200 */
	if (sid == SENSOR_PVDF) {
		if (rate == 0U || rate > 200U) {
			send_ack(CMD_RATE_SET, req_seq, -1, NULL, 0);
			return;
		}
	} else if (rate != 15U && rate != 30U && rate != 60U) {
		send_ack(CMD_RATE_SET, req_seq, -1, NULL, 0);
		return;
	}

	cfg[sid].rate_hz = rate;
	send_ack(CMD_RATE_SET, req_seq, 0, NULL, 0);
}

static void handle_qvar_cfg(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U || p[0] > 1U || p[1] > 3U) {
		send_ack(CMD_QVAR_CFG, req_seq, -1, NULL, 0);
		return;
	}
	int ret = qvar_channel_enable((enum qvar_channel)p[0],
				      (enum qvar_zin)p[1]);

	send_ack(CMD_QVAR_CFG, req_seq, (int8_t)(ret == 0 ? 0 : -2), NULL, 0);
}

static void handle_reprobe(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 1U || p[0] >= SENSOR_NUM) {
		send_ack(CMD_REPROBE, req_seq, -1, NULL, 0);
		return;
	}

	int ret = 1; /* 1=仍不在位（协议：REPROBE 成功=0，不在位=1） */

	switch (p[0]) {
	case SENSOR_IMU_U1:
		ret = body_imu_reprobe();
		break;
	case SENSOR_QVAR:
		{
			int ra = qvar_channel_reprobe(QVAR_CHANNEL_A);
			int rb = qvar_channel_reprobe(QVAR_CHANNEL_B);

			ret = (ra == 0 || rb == 0) ? 0 : 1;
		}
		break;
	case SENSOR_TEMP:
		ret = (temp_read_mdeg(&(int32_t){0}) == 0) ? 0 : 1;
		break;
	case SENSOR_PVDF:
		ret = (pvdf_adc_get_state() == PVDF_ADC_READY) ? 0 : 1;
		break;
	case SENSOR_IMU_U4:
		ret = u4_streaming ? 0 : (u4_stream_start() == 0 ? 0 : 1);
		break;
	default:
		ret = -12;
		break;
	}
	send_ack(CMD_REPROBE, req_seq, (int8_t)ret, NULL, 0);
}

static void handle_qvar_thr(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 4U || p[0] > 1U) {
		send_ack(CMD_QVAR_THR_SET, req_seq, -1, NULL, 0);
		return;
	}

	uint8_t ch = p[0];

	qvar_thr[ch].thr = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
	qvar_thr[ch].hyst = p[3];
	qvar_thr[ch].crossed = false;
	send_ack(CMD_QVAR_THR_SET, req_seq, 0, NULL, 0);
}

static void handle_rec_ctrl(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 4U || p[0] > 1U) {
		send_ack(CMD_REC_CTRL, req_seq, -1, NULL, 0);
		return;
	}

	int ret;

	if (p[0] == 1U) {
		uint8_t ch_mask = p[1];
		uint16_t dur = (uint16_t)(p[2] | ((uint16_t)p[3] << 8));

		if (dur > 3600U) {
			send_ack(CMD_REC_CTRL, req_seq, -1, NULL, 0);
			return;
		}
		ret = recorder_start(ch_mask, dur);
	} else {
		ret = recorder_stop();
	}
	send_ack(CMD_REC_CTRL, req_seq, (int8_t)ret, NULL, 0);
}

static void handle_rec_list(uint8_t req_seq)
{
	uint8_t buf[REC_LIST_MAX * REC_LIST_ENTRY_LEN];
	uint16_t out_len = 0U;
	int ret = recorder_list(buf, sizeof(buf), &out_len);

	send_ack(CMD_REC_LIST, req_seq, (int8_t)ret, buf,
		 (uint8_t)(ret == 0 ? out_len : 0U));
}

static void handle_rec_read(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U) {
		send_ack(CMD_REC_READ, req_seq, -1, NULL, 0);
		return;
	}

	uint16_t file_id = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
	uint32_t offset = 0U;

	/* 断点续传（V0.5 决策）：可选 offset(4B) 追加在 file_id 之后 */
	if (len >= 6U) {
		offset = (uint32_t)p[2] | ((uint32_t)p[3] << 8) |
			 ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
	}

	int ret = recorder_read_start(file_id, offset);

	send_ack(CMD_REC_READ, req_seq, (int8_t)ret, NULL, 0);
}

static void handle_rec_delete(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	if (len < 2U) {
		send_ack(CMD_REC_DELETE, req_seq, -1, NULL, 0);
		return;
	}

	uint16_t file_id = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
	int ret = recorder_delete(file_id);

	send_ack(CMD_REC_DELETE, req_seq, (int8_t)ret, NULL, 0);
}

/** SD_TEST 完成回调（录音线程上下文） */
static void sd_test_done_cb(const struct sd_test_result *res)
{
	/* EVENT 0x07 SD_TEST_DONE：result(1)+w_kbps(2)+r_kbps(2)+bad(2) */
	uint8_t evt[8] = {
		EVENT_SD_TEST_DONE,
		(uint8_t)res->result,
		(uint8_t)(res->w_kbps & 0xFFU),
		(uint8_t)((res->w_kbps >> 8) & 0xFFU),
		(uint8_t)(res->r_kbps & 0xFFU),
		(uint8_t)((res->r_kbps >> 8) & 0xFFU),
		(uint8_t)(res->bad_blocks & 0xFFU),
		(uint8_t)((res->bad_blocks >> 8) & 0xFFU),
	};

	(void)hub_event_frame(HUB_TYPE_EVENT, evt, sizeof(evt));
}

static void handle_sd_test(const uint8_t *p, uint8_t len, uint8_t req_seq)
{
	uint8_t size_mb = (len >= 1U) ? p[0] : 4U;
	uint8_t verify = (len >= 2U) ? p[1] : 1U;
	int ret = recorder_sd_test(size_mb, verify, sd_test_done_cb);

	send_ack(CMD_SD_TEST, req_seq, (int8_t)ret, NULL, 0);
}

static void cmd_work_handler(struct k_work *work)
{
	struct hub_cmd_msg msg;

	ARG_UNUSED(work);

	while (sensor_hub_cmd_fetch(&msg, K_NO_WAIT) == 0) {
		uint8_t cmd = msg.data[0];
		const uint8_t *p = &msg.data[1];
		uint8_t plen = (uint8_t)(msg.len - 1U);

		switch (cmd) {
		case CMD_LED_SET:
			handle_led_set(p, plen, msg.req_seq);
			break;
		case CMD_PWR_SET:
			handle_pwr_set(p, plen, msg.req_seq);
			break;
		case CMD_SENSOR_EN:
			handle_sensor_en(p, plen, msg.req_seq);
			break;
		case CMD_RATE_SET:
			handle_rate_set(p, plen, msg.req_seq);
			break;
		case CMD_QVAR_CFG:
			handle_qvar_cfg(p, plen, msg.req_seq);
			break;
		case CMD_REPROBE:
			handle_reprobe(p, plen, msg.req_seq);
			break;
		case CMD_REC_CTRL:
			handle_rec_ctrl(p, plen, msg.req_seq);
			break;
		case CMD_QVAR_THR_SET:
			handle_qvar_thr(p, plen, msg.req_seq);
			break;
		case CMD_REC_LIST:
			handle_rec_list(msg.req_seq);
			break;
		case CMD_REC_READ:
			handle_rec_read(p, plen, msg.req_seq);
			break;
		case CMD_REC_DELETE:
			handle_rec_delete(p, plen, msg.req_seq);
			break;
		case CMD_GET_BATTERY:
			send_ack(CMD_GET_BATTERY, msg.req_seq, 0, NULL, 0);
			{
				bool vbus = false;

				(void)power_control_vbus_present(&vbus);
				battery_tick(vbus);
			}
			break;
		case CMD_SD_TEST:
			handle_sd_test(p, plen, msg.req_seq);
			break;
		case CMD_GET_STATUS:
			send_ack(CMD_GET_STATUS, msg.req_seq, 0, NULL, 0);
			{
				bool vbus = false;

				(void)power_control_vbus_present(&vbus);
				module_status_tick(vbus);
			}
			break;
		case CMD_GET_VERSION:
			send_ack(CMD_GET_VERSION, msg.req_seq, 0,
				 (const uint8_t *)FW_INFO_STRING,
				 (uint8_t)(sizeof(FW_INFO_STRING) - 1U));
			break;
		case CMD_FACTORY_PING:
			send_ack(CMD_FACTORY_PING, msg.req_seq, 0, p,
				 (plen > 4U) ? 4U : plen);
			break;
		default:
			send_ack(cmd, msg.req_seq, -12, NULL, 0);
			break;
		}
	}
}

/* ---- 初始化 ---- */

int sensor_hub_bridge_init(void)
{
	int ret;

	ret = batt_gauge_init();
	if (ret < 0) {
		printk("BRIDGE,warn,batt_gauge_rc=%d\n", ret);
	}

	(void)recorder_init();

	/* 自动开流（demo/测试默认开，SENSOR_EN 可关） */
	if (cfg[SENSOR_IMU_U4].enabled) {
		if (u4_stream_start() != 0) {
			printk("BRIDGE,warn,u4_autostart_fail\n");
		}
	}
	/* SENS 域基线：U1/QVAR/TEMP 任一使能即保持域上电（帧流不中断；
	 * 与 U4 流的 sens_acquire/release 引用计数互补，域由引用计数管理） */
	if (cfg[SENSOR_IMU_U1].enabled || cfg[SENSOR_QVAR].enabled ||
	    cfg[SENSOR_TEMP].enabled) {
		if (sens_acquire() != 0) {
			printk("BRIDGE,warn,sens_baseline_fail\n");
		}
	}
	if (cfg[SENSOR_PVDF].enabled && !analog_on) {
		if (power_domain_set(POWER_DOMAIN_ANALOG, true) == 0) {
			analog_on = true;
			k_msleep(1200); /* VBIAS 稳定等待（协议 §10.3） */
		} else {
			printk("BRIDGE,warn,analog_on_fail\n");
			cfg[SENSOR_PVDF].enabled = false;
		}
	}
	st_mic = MOD_ST_PRESENT; /* 双麦在主板，bring-up 已验证 */

	ret = sensor_hub_init(NULL);
	if (ret != 0) {
		printk("BRIDGE,warn,hub_init_rc=%d\n", ret);
	}

	k_work_reschedule(&fast_work, K_MSEC(100));
	k_work_reschedule(&pvdf_work, K_MSEC(100));
	k_work_reschedule(&slow_work, K_MSEC(500));

	printk("BRIDGE,init,autostart=%d\n",
	       IS_ENABLED(CONFIG_APP_SENSOR_HUB_AUTOSTART));
	return 0;
}

void sensor_hub_bridge_report_wdt_reset(uint32_t reset_cause)
{
	wdt_reset_cause = reset_cause;
	wdt_event_pending = (reset_cause != 0U);
}

#else /* !CONFIG_APP_SENSOR_HUB */

int sensor_hub_bridge_init(void) { return 0; }
void sensor_hub_bridge_cmd_kick(void) {}
void sensor_hub_bridge_report_wdt_reset(uint32_t reset_cause)
{
	ARG_UNUSED(reset_cause);
}

#endif /* CONFIG_APP_SENSOR_HUB */
