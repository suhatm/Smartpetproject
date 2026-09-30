/**
 * @file status_led.c
 * @brief 状态 LED 实现：通过 nPM1300 LEDDRV 寄存器控制红/蓝两灯
 *
 * 两路 LED 均切换为"主机控制"模式（绕开 PMIC 自动行为），
 * 通过 SET/CLR 任务寄存器开关。内部缓存当前亮灭状态，
 * 状态未变化时跳过寄存器写入，减少 I2C 流量。
 *
 * 硬件注意（产品方案 §8.1）：LED2（蓝）当前极性接反
 * （K→VSYS / A→PMIC.LED1），整改前蓝灯不会点亮；
 * 固件无需感知极性，整改后自动生效。
 */
#include "status_led.h"

#include <errno.h>

#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/kernel.h>

/* nPM1300 LED 驱动模块（LEDDRV）寄存器定义 */
#define BASE_LEDDRV 0x0AU /* LED 驱动模块基地址 */

#define OFF_LED0_MODE 0x00U /* LED0（红）模式选择 */
#define OFF_LED1_MODE 0x01U /* LED1（蓝）模式选择 */
#define OFF_LED0_SET  0x03U /* LED0 点亮任务触发 */
#define OFF_LED0_CLR  0x04U /* LED0 熄灭任务触发 */
#define OFF_LED1_SET  0x05U /* LED1 点亮任务触发 */
#define OFF_LED1_CLR  0x06U /* LED1 熄灭任务触发 */

/** 主机控制模式（模式寄存器写入值） */
#define LED_MODE_HOST 0x02U
/** 任务触发寄存器统一写入值（写 1 触发） */
#define TASK_TRIGGER 0x01U

/** PMIC 设备指针，init 时保存；为 NULL 表示未初始化 */
static const struct device *pmic_dev;
/** 红灯当前缓存状态 */
static bool red_state;
/** 蓝灯当前缓存状态 */
static bool blue_state;
/** 缓存状态是否有效（init 后首次必须实际写寄存器） */
static bool state_valid;

/**
 * @brief 设置单个 LED 通道的亮灭
 *
 * @param red_channel true 操作红灯（LED0），false 操作蓝灯（LED1）
 * @param on true 点亮，false 熄灭
 * @return 0 成功；负值寄存器写失败
 */
static int set_channel(bool red_channel, bool on)
{
	uint8_t offset;

	/* 根据通道与目标状态选择 SET/CLR 任务寄存器 */
	if (red_channel) {
		offset = on ? OFF_LED0_SET : OFF_LED0_CLR;
	} else {
		offset = on ? OFF_LED1_SET : OFF_LED1_CLR;
	}

	return mfd_npm13xx_reg_write(pmic_dev, BASE_LEDDRV, offset,
				      TASK_TRIGGER);
}

/**
 * @brief 设置红/蓝灯组合状态（带变化检测）
 *
 * 仅当缓存无效或目标状态与缓存不同才写寄存器。
 *
 * @return 0 成功；-ENODEV 未初始化；其他负值写失败
 */
int status_led_set(bool red, bool blue)
{
	int ret;

	if (pmic_dev == NULL) {
		return -ENODEV;
	}

	/* 红灯：状态有变化（或缓存无效）才写 */
	if (!state_valid || (red != red_state)) {
		ret = set_channel(true, red);
		if (ret != 0) {
			return ret;
		}
		red_state = red;
	}

	/* 蓝灯：同上 */
	if (!state_valid || (blue != blue_state)) {
		ret = set_channel(false, blue);
		if (ret != 0) {
			return ret;
		}
		blue_state = blue;
	}

	state_valid = true;
	return 0;
}

/**
 * @brief 阻塞式脉冲：点亮 on_ms，再熄灭，可选再等待 off_ms
 *
 * @return 0 成功；负值 LED 设置失败
 */
int status_led_pulse(bool red, bool blue, uint32_t on_ms, uint32_t off_ms)
{
	/* 先点亮 */
	int ret = status_led_set(red, blue);

	if (ret != 0) {
		return ret;
	}

	/* 维持点亮时长 */
	k_sleep(K_MSEC(on_ms));
	/* 熄灭 */
	ret = status_led_set(false, false);
	if ((ret == 0) && (off_ms != 0U)) {
		/* 熄灭后额外间隔，用于连续脉冲之间的节奏控制 */
		k_sleep(K_MSEC(off_ms));
	}

	return ret;
}

/**
 * @brief 阻塞式闪烁：指定通道按 on/off 半周期闪烁指定总时长
 *
 * 剩余不足一个完整周期时仍保证最后的熄灭收尾，避免图案结束时残亮。
 *
 * @return 0 成功；负值 LED 设置失败
 */
int status_led_blink(bool red, bool blue, uint32_t duration_ms,
		     uint32_t on_ms, uint32_t off_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)duration_ms;
	int ret = 0;

	if ((on_ms == 0U) || (off_ms == 0U)) {
		return -EINVAL;
	}

	while (k_uptime_get() < deadline) {
		uint32_t remaining = (uint32_t)(deadline - k_uptime_get());

		/* 亮半周期（末尾截断） */
		ret = status_led_set(red, blue);
		if (ret != 0) {
			return ret;
		}
		k_sleep(K_MSEC(MIN(on_ms, remaining)));
		if ((uint32_t)(deadline - k_uptime_get()) == 0U) {
			break;
		}

		/* 灭半周期（末尾截断） */
		ret = status_led_set(false, false);
		if (ret != 0) {
			return ret;
		}
		k_sleep(K_MSEC(MIN(off_ms, (uint32_t)(deadline - k_uptime_get()))));
	}

	/* 收尾熄灭，防残亮 */
	return status_led_set(false, false);
}

/**
 * @brief LED 模块初始化
 *
 * 两通道切换为主机控制模式，随后熄灯一次以建立有效的缓存状态。
 *
 * @param pmic PMIC 设备句柄
 * @return 0 成功；-ENODEV 设备未就绪；其他负值失败
 */
int status_led_init(const struct device *pmic)
{
	int ret;

	if ((pmic == NULL) || !device_is_ready(pmic)) {
		return -ENODEV;
	}

	pmic_dev = pmic;
	state_valid = false;

	/* LED0（红）与 LED1（蓝）均切换为主机控制模式 */
	ret = mfd_npm13xx_reg_write(pmic_dev, BASE_LEDDRV,
				     OFF_LED0_MODE, LED_MODE_HOST);
	if (ret == 0) {
		ret = mfd_npm13xx_reg_write(pmic_dev, BASE_LEDDRV,
					     OFF_LED1_MODE, LED_MODE_HOST);
	}
	if (ret == 0) {
		/* 初始熄灯，同时使状态缓存生效 */
		ret = status_led_set(false, false);
	}

	return ret;
}
