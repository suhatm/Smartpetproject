/**
 * @file batt_gauge.c
 * @brief 电池电量模块实现（nrf_fuel_gauge 库仑计 + 电压映射兜底）
 *
 * 参考 NCS 样例 nrf/samples/pmic/native/npm13xx_fuel_gauge：
 *   - charger 设备（nordic,npm1300-charger，overlay 已加 npm1300_charger 节点）
 *   - SENSOR_CHAN_GAUGE_VOLTAGE / GAUGE_AVG_CURRENT / GAUGE_TEMP 采样
 *   - CHARGER.BCHGCHARGESTATUS 位 -> 充电状态通知库
 * 简化：不做 settings/NVS 持久化（复位后重新估算，几秒后收敛）。
 */
#include "batt_gauge.h"

#include <zephyr/kernel.h>

#if CONFIG_APP_BATT_GAUGE

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/sys/printk.h>

#include <nrf_fuel_gauge.h>

#include "power_control.h"

#define CHARGER_NODE DT_NODELABEL(npm1300_charger)

/* CHARGER.BCHGCHARGESTATUS 位掩码 */
#define CHG_STATUS_COMPLETE_MASK BIT(1)
#define CHG_STATUS_TRICKLE_MASK  BIT(2)
#define CHG_STATUS_CC_MASK       BIT(3)
#define CHG_STATUS_CV_MASK       BIT(4)

/** 电压线性映射兜底窗口（协议 V0.5 注：库仑计启用前允许） */
#define VBAT_EMPTY_MV 3300.0f
#define VBAT_FULL_MV  4200.0f

/** 低电门限与滞回（协议 §6.9/§8.2：≤15% 触发，>20% 复归） */
#define LOW_BATT_ON_PCT  15
#define LOW_BATT_OFF_PCT 20

static const struct device *const charger = DEVICE_DT_GET(CHARGER_NODE);

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

static struct batt_reading latest;
static int64_t ref_time;
static bool coulomb_ok;   /* true=库仑计模式；false=电压映射兜底 */
static bool fg_inited;
static int32_t chg_status_prev = -1;

static int read_sensors(float *voltage, float *current, float *temp,
			int32_t *chg_status)
{
	struct sensor_value value;
	int ret = sensor_sample_fetch(charger);

	if (ret < 0) {
		return ret;
	}

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &value);
	*voltage = (float)value.val1 + ((float)value.val2 / 1000000.f);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &value);
	*temp = (float)value.val1 + ((float)value.val2 / 1000000.f);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &value);
	*current = (float)value.val1 + ((float)value.val2 / 1000000.f);

	sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &value);
	*chg_status = value.val1;

	return 0;
}

static int charge_status_inform(int32_t chg_status)
{
	union nrf_fuel_gauge_ext_state_info_data state_info;

	if (chg_status & CHG_STATUS_COMPLETE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE;
	} else if (chg_status & CHG_STATUS_TRICKLE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE;
	} else if (chg_status & CHG_STATUS_CC_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CC_LIMITED;
	} else if (chg_status & CHG_STATUS_CV_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CV;
	} else {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_IDLE;
	}

	return nrf_fuel_gauge_ext_state_update(
		NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE, &state_info);
}

static uint8_t percent_from_voltage(float v)
{
	float mv = v * 1000.f;

	if (mv <= VBAT_EMPTY_MV) {
		return 0;
	}
	if (mv >= VBAT_FULL_MV) {
		return 100;
	}
	return (uint8_t)((mv - VBAT_EMPTY_MV) * 100.f /
			 (VBAT_FULL_MV - VBAT_EMPTY_MV));
}

int batt_gauge_init(void)
{
	if (!device_is_ready(charger)) {
		printk("BATT,fail,charger_not_ready\n");
		return -ENODEV;
	}

	float v, i, t;
	int32_t chg_status;
	int ret = read_sensors(&v, &i, &t, &chg_status);

	if (ret < 0) {
		printk("BATT,fail,first_read_rc=%d\n", ret);
		return ret;
	}

	struct nrf_fuel_gauge_init_parameters params =
		NRF_FUEL_GAUGE_DEFAULT_INIT_PARAMETERS_SECONDARY(
			v, -i /* Zephyr: 负=放电；库: 负=充电 */, t,
			&battery_model);

	ret = nrf_fuel_gauge_init(&params, NULL);
	if (ret < 0) {
		printk("BATT,warn,fg_init_rc=%d,fallback=voltage_map\n", ret);
		coulomb_ok = false;
	} else {
		/* 充电电流上限/终止电流（ttf 估算需要；读 charger DT 配置） */
		struct sensor_value value;

		sensor_channel_get(charger,
				   SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
				   &value);
		float max_cc = (float)value.val1 + ((float)value.val2 / 1000000.f);

		(void)nrf_fuel_gauge_ext_state_update(
			NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT,
			&(union nrf_fuel_gauge_ext_state_info_data){
				.charge_current_limit = max_cc });
		(void)nrf_fuel_gauge_ext_state_update(
			NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT,
			&(union nrf_fuel_gauge_ext_state_info_data){
				.charge_term_current = max_cc / 10.f });
		(void)charge_status_inform(chg_status);
		chg_status_prev = chg_status;
		coulomb_ok = true;
		ref_time = k_uptime_get();
	}

	fg_inited = true;
	latest.vbat_mv = (uint16_t)(v * 1000.f);
	latest.percent = coulomb_ok ? 0 : percent_from_voltage(v);
	printk("BATT,init,mode=%s,vbat_mv=%u,model=sample_uncalibrated\n",
	       coulomb_ok ? "coulomb" : "voltage_map", latest.vbat_mv);
	return coulomb_ok ? 0 : 1;
}

int batt_gauge_update(bool vbus_present)
{
	if (!fg_inited) {
		return -ENODEV;
	}

	float v, i, t;
	int32_t chg_status;
	int ret = read_sensors(&v, &i, &t, &chg_status);

	if (ret < 0) {
		return ret;
	}

	latest.vbat_mv = (uint16_t)(v * 1000.f);
	latest.charging = (chg_status & (CHG_STATUS_TRICKLE_MASK |
					 CHG_STATUS_CC_MASK |
					 CHG_STATUS_CV_MASK)) != 0;
	latest.full = (chg_status & CHG_STATUS_COMPLETE_MASK) != 0;

	if (coulomb_ok) {
		(void)nrf_fuel_gauge_ext_state_update(
			vbus_present
				? NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED
				: NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
			NULL);

		if (chg_status != chg_status_prev) {
			chg_status_prev = chg_status;
			(void)charge_status_inform(chg_status);
		}

		int64_t now = k_uptime_get();
		float delta = (float)(now - ref_time) / 1000.f;

		ref_time = now;

		float soc = 0.f;

		/* Zephyr: 负电流=放电；库约定相反，取反 */
		ret = nrf_fuel_gauge_process(v, -i, t, delta, &soc, NULL);
		if (ret < 0) {
			printk("BATT,warn,fg_process_rc=%d\n", ret);
			return ret;
		}

		if (soc < 0.f) {
			soc = 0.f;
		} else if (soc > 100.f) {
			soc = 100.f;
		}
		latest.percent = (uint8_t)(soc + 0.5f);
	} else {
		latest.percent = percent_from_voltage(v);
	}

	/* 低电滞回：≤15% 置位，>20% 复归 */
	if (!latest.low && latest.percent <= LOW_BATT_ON_PCT) {
		latest.low = true;
	} else if (latest.low && latest.percent > LOW_BATT_OFF_PCT) {
		latest.low = false;
	}

	return 0;
}

void batt_gauge_get(struct batt_reading *out)
{
	*out = latest;
}

#else /* !CONFIG_APP_BATT_GAUGE */

int batt_gauge_init(void) { return 0; }
int batt_gauge_update(bool vbus_present)
{
	ARG_UNUSED(vbus_present);
	return 0;
}
void batt_gauge_get(struct batt_reading *out)
{
	out->vbat_mv = 0;
	out->percent = 0xFF;
	out->charging = false;
	out->full = false;
	out->low = false;
}

#endif /* CONFIG_APP_BATT_GAUGE */
