/**
 * @file temp_sensor.c
 * @brief TMP112A bring-up 自检实现（ti,tmp112 sensor 驱动）
 */
#include "temp_sensor.h"

#if CONFIG_APP_TEMP_TEST

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

#include "power_control.h"

#define TEMP_NODE DT_NODELABEL(temp_u2)

#if !DT_NODE_HAS_STATUS(TEMP_NODE, okay)
#error "temp_sensor: temp_u2 节点未在 overlay 定义"
#endif

static const struct device *const temp_dev = DEVICE_DT_GET(TEMP_NODE);

int temp_bringup_test(void)
{
	/* SENS 域上电（IMU 自检已开过的话是幂等操作） */
	int ret = power_domain_set(POWER_DOMAIN_SENS, true);

	if (ret != 0) {
		printk("TEMP_TEST,fail,sens_power_rc=%d\n", ret);
		return ret;
	}
	k_msleep(10); /* LDO 稳定 + TMP112 POR */

	/* deferred-init：VDD_SENS 此时才有电，手动初始化驱动 */
	ret = device_init(temp_dev);
	if (ret != 0 && ret != -EALREADY) {
		/* FPC 未插/器件虚焊：init 走 i2c 会 NACK */
		printk("TEMP_TEST,absent,init_rc=%d\n", ret);
		return 1;
	}
	if (!device_is_ready(temp_dev)) {
		printk("TEMP_TEST,absent,not_ready\n");
		return 1;
	}

	ret = sensor_sample_fetch(temp_dev);
	if (ret != 0) {
		printk("TEMP_TEST,absent,fetch_rc=%d\n", ret);
		return 1;
	}

	struct sensor_value t;

	ret = sensor_channel_get(temp_dev, SENSOR_CHAN_AMBIENT_TEMP, &t);
	if (ret != 0) {
		printk("TEMP_TEST,fail,get_rc=%d\n", ret);
		return ret;
	}

	/* val1=整数, val2=小数(微度) */
	int mdeg = t.val1 * 1000 + t.val2 / 1000;

	/* 合理范围：器件 -40~125°C，板内室温附近判定宽一点 */
	if (mdeg < -20000 || mdeg > 85000) {
		printk("TEMP_TEST,fail,range,t=%d.%03dC\n", mdeg / 1000, mdeg % 1000);
		return -ERANGE;
	}

	printk("TEMP_TEST,PASS,t=%d.%03dC\n", mdeg / 1000, mdeg % 1000);
	return 0;
}

int temp_read_mdeg(int32_t *mdeg)
{
	if (!device_is_ready(temp_dev)) {
		return -ENODEV;
	}
	int ret = sensor_sample_fetch(temp_dev);

	if (ret != 0) {
		return ret;
	}
	struct sensor_value t;

	ret = sensor_channel_get(temp_dev, SENSOR_CHAN_AMBIENT_TEMP, &t);
	if (ret != 0) {
		return ret;
	}
	*mdeg = t.val1 * 1000 + t.val2 / 1000;
	return 0;
}

#else

int temp_bringup_test(void)
{
	return 0;
}

#endif /* CONFIG_APP_TEMP_TEST */
