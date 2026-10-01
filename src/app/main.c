/**
 * @file main.c
 * @brief 系统主入口：电源状态机 + SW1 开关机 + LED 指示
 *
 * 实现产品方案（docs/电源开关机与LED指示产品方案.md）§5/§6：
 *
 * 状态机：
 *   SHIP --长按0.6s(PMIC硬件)--> BOOTING --LED0红闪3s--> READY(双灯同闪3s)
 *        --> ACTIVE/CHARGE_ONLY
 *   ACTIVE --长按3s松开--> SHUTTING_DOWN --LED1蓝闪3s--> 关域->进SHIP
 *
 * 按键 UI（SW1 接 nPM1300 SHPHLD，事件经 PMIC IRQ P0.03 上报）：
 *  - 短按（<1s）：状态确认，蓝灯双闪，无破坏性操作；
 *  - 按住 1~3s：关机武装，红灯常亮；
 *  - 按住 >=3s 松开：执行优雅关机（关域->蓝闪3s->进Ship）；
 *  - 按住 10s：PMIC 硬件 power-cycle（应急复位，不经固件）。
 *
 * 硬件注意：LED2（蓝）极性当前接反（方案 §8.1），整改前蓝灯不亮，
 * 固件逻辑不受影响，整改后自动生效。
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "power_control.h"
#include "status_led.h"
#include "app_watchdog.h"
#if CONFIG_APP_IMU_TEST
#include "imu_sensor.h"
#endif
#if CONFIG_APP_TEMP_TEST
#include "temp_sensor.h"
#include "ble_imu_service.h"
#endif
#if CONFIG_APP_SD_TEST
#include "sd_store.h"
#endif
#if CONFIG_APP_MIC_TEST
#include "mic_pdm.h"
#endif
#if CONFIG_APP_BLE_LED_TEST
#include "ble_led_service.h"
#endif
#if CONFIG_APP_POWER_DOMAIN_TEST_MB
#include "test_mailbox.h"
#endif
#if CONFIG_APP_HW_WATCHDOG
#include <nrfx.h> /* NRF_RESET->RESETREAS（nRF54L 无 hwinfo 驱动，裸读） */
#endif

/** nPM1300 PMIC 的设备树节点 */
#define PMIC_NODE DT_NODELABEL(npm1300_pmic)

/** 短按判定上限（ms）：松开时按住时长小于此值视为短按 */
#define SHORT_PRESS_MAX_MS CONFIG_APP_SHORT_PRESS_MAX_MS
/** 关机武装时长（ms）：按住达到该时长后松开即触发关机 */
#define SHUTDOWN_ARM_MS    CONFIG_APP_SHUTDOWN_HOLD_MS
/** 主循环轮询周期（ms） */
#define LOOP_PERIOD_MS     20U
/** 指示图案时长（ms）：开机/关机/就绪均为 3 秒 */
#define INDICATION_MS      CONFIG_APP_LED_INDICATION_MS
/** 指示图案闪烁半周期（ms）：300 on / 300 off */
#define BLINK_HALF_MS      CONFIG_APP_LED_BLINK_PERIOD_MS
/** 心跳指示周期（ms）：正常工作期间每该时长双灯快闪一次 */
#define HEARTBEAT_PERIOD_MS CONFIG_APP_HEARTBEAT_PERIOD_MS
/** 心跳单次点亮时长（ms） */
#define HEARTBEAT_FLASH_MS  CONFIG_APP_HEARTBEAT_FLASH_MS
/** 电源域开关后等待生效时间（ms） */
#define DOMAIN_SETTLE_MS   20U

/** LDSW 状态寄存器导通位（bit0=LDSW1/SENS, bit2=LDSW2/STORE） */
#define LDSW_SENS_ON_BIT  0x01U
#define LDSW_STORE_ON_BIT 0x04U

/** 应用运行状态 */
enum app_state {
	APP_STATE_ACTIVE,       /* 电池供电，系统就绪工作 */
	APP_STATE_CHARGE_ONLY,  /* USB 已插入，充电监护（系统保持运行） */
	APP_STATE_SHUTTING_DOWN, /* 关机流程执行中（阻塞流程内使用） */
	APP_STATE_FAULT,        /* 故障状态：LED 报错图案 */
};

/** PMIC 设备句柄（设备树静态获取） */
static const struct device *const pmic = DEVICE_DT_GET(PMIC_NODE);
/** PMIC 事件 GPIO 回调对象 */
static struct gpio_callback pmic_event_cb;
/** PMIC 事件位图：中断上下文置位，主循环消费后清零 */
static atomic_t pending_events;

/** 当前应用状态 */
static enum app_state app_state;
/** 按键当前是否处于按下状态 */
static bool button_pressed;
/** 标志位：充电中请求了关机，等拔掉 USB 后再执行进 Ship */
static bool pending_ship_on_vbus_remove;
/** 按键按下时刻的系统 uptime（ms），用于计算按住时长 */
static int64_t press_started_ms;

/**
 * @brief PMIC 事件中断回调
 *
 * 中断上下文中只做一件事：把触发的事件引脚位图原子地并入
 * pending_events，具体处理延迟到主循环 handle_events() 中完成。
 */
static void pmic_event_callback(const struct device *dev,
				struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	atomic_or(&pending_events, (atomic_val_t)pins);
}

/**
 * @brief 注册 PMIC 关注的四种事件回调
 *
 * 关注：ShipHold 按键按下/松开、VBUS 插入/拔出。
 *
 * @return 0 成功；负值失败
 */
static int configure_pmic_events(void)
{
	const uint32_t mask = BIT(NPM13XX_EVENT_SHIPHOLD_PRESS) |
			      BIT(NPM13XX_EVENT_SHIPHOLD_RELEASE) |
			      BIT(NPM13XX_EVENT_VBUS_DETECTED) |
			      BIT(NPM13XX_EVENT_VBUS_REMOVED);

	gpio_init_callback(&pmic_event_cb, pmic_event_callback, mask);
	return mfd_npm13xx_add_callback(pmic, &pmic_event_cb);
}

/**
 * @brief 进入故障状态
 *
 * 关闭所有电源轨，切换到 FAULT 状态（LED 会报三连红闪图案），
 * 并通过 RTT 打印故障原因与错误码。
 */
static void enter_fault(const char *reason, int rc)
{
	(void)power_domains_all_off();
	app_state = APP_STATE_FAULT;
	printk("FAULT,reason=%s,rc=%d\n", reason, rc);
}

#if CONFIG_APP_POWER_DOMAIN_SELFTEST
/**
 * @brief 开机电源域独立控制自检
 *
 * 对三路电源域逐一执行 开->回读校验->关->回读校验，
 * 验证域间互不影响（产品方案 §4）。任一失败打印 FAIL 但不阻断启动。
 *
 * @return 0 全部通过；负值失败域数（取负）
 */
static int power_domain_selftest(void)
{
	static const struct {
		enum power_domain domain;
		const char *name;
		uint8_t ldsw_bit; /* ANALOG 域无 LDSW 位，传 0 */
	} cases[] = {
		{ POWER_DOMAIN_SENS, "SENS", LDSW_SENS_ON_BIT },
		{ POWER_DOMAIN_STORE, "STORE", LDSW_STORE_ON_BIT },
		{ POWER_DOMAIN_ANALOG, "ANALOG", 0U },
	};
	int failures = 0;

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		uint8_t ldsw;
		bool ana;
		bool ok = true;
		int ret;

		/* 开域 -> 校验导通 */
		ret = power_domain_set(cases[i].domain, true);
		if (ret != 0) {
			printk("SELFTEST,%s,on=FAIL,rc=%d\n", cases[i].name, ret);
			failures++;
			continue;
		}
		k_sleep(K_MSEC(DOMAIN_SETTLE_MS));
		ret = power_control_get_state(&ldsw, &ana);
		if (ret != 0) {
			printk("SELFTEST,%s,read=FAIL,rc=%d\n", cases[i].name, ret);
			failures++;
			continue;
		}
		if (cases[i].ldsw_bit != 0U) {
			ok = (ldsw & cases[i].ldsw_bit) != 0U;
		} else {
			ok = ana;
		}
		if (!ok) {
			printk("SELFTEST,%s,on=FAIL,ldsw=0x%02x,ana=%u\n",
			       cases[i].name, ldsw, (unsigned)ana);
			failures++;
		}

		/* 关域 -> 校验关断 */
		ret = power_domain_set(cases[i].domain, false);
		if (ret != 0) {
			printk("SELFTEST,%s,off=FAIL,rc=%d\n", cases[i].name, ret);
			failures++;
			continue;
		}
		k_sleep(K_MSEC(DOMAIN_SETTLE_MS));
		ret = power_control_get_state(&ldsw, &ana);
		if (ret == 0) {
			if (cases[i].ldsw_bit != 0U) {
				ok = (ldsw & cases[i].ldsw_bit) == 0U;
			} else {
				ok = !ana;
			}
			if (!ok) {
				printk("SELFTEST,%s,off=FAIL,ldsw=0x%02x,ana=%u\n",
				       cases[i].name, ldsw, (unsigned)ana);
				failures++;
			}
		}
	}

	printk("SELFTEST,domains=%s,failures=%d\n",
	       failures == 0 ? "PASS" : "FAIL", failures);
	return -failures;
}
#endif /* CONFIG_APP_POWER_DOMAIN_SELFTEST */

/**
 * @brief 执行优雅关机流程（产品方案 §5.3，顺序不可换）
 *
 * 流程：关所有电源轨 -> 回读校验全关 -> LED1 蓝闪 3 秒 ->
 * 检查 VBUS：若在充电则推迟关机（等拔线后再进 Ship），
 * 否则直接进入 Ship 模式（PMIC 切断 VSYS，整机断电）。
 * 若 1s 后代码仍能执行到末尾，说明没断成电，判为故障。
 */
static void request_graceful_shutdown(void)
{
	bool vbus_present = false;
	int ret;

	printk("POWER_BUTTON,action=shutdown_requested\n");

#if CONFIG_APP_BLE_LED_TEST
	/* 关机前断开蓝牙连接并停止广播（方案 §3.3）：
	 * 蓝闪指示与进 Ship 之后设备不再对外可见 */
	ble_led_test_shutdown();
#endif

	/* 1. 关闭所有电源轨并回读校验 */
	ret = power_domains_all_off();
	if (ret == 0) {
		k_sleep(K_MSEC(20));
		ret = power_domains_verify_all_off();
	}
	if (ret != 0) {
		enter_fault("rails_not_off", ret);
		return;
	}

	/* 2. 关机指示：LED1（蓝）闪烁 3 秒 */
	ret = status_led_blink(false, true, INDICATION_MS,
			       BLINK_HALF_MS, BLINK_HALF_MS);
	if (ret != 0) {
		enter_fault("shutdown_led", ret);
		return;
	}

	/* 3. 读取 VBUS 状态，决定立即进 Ship 还是推迟 */
	ret = power_control_vbus_present(&vbus_present);
	if (ret != 0) {
		enter_fault("vbus_read", ret);
		return;
	}

	if (vbus_present) {
		/* 正在充电：先保持充电监护模式，等拔掉 USB 再真正进 Ship */
		pending_ship_on_vbus_remove = true;
		app_state = APP_STATE_CHARGE_ONLY;
		printk("POWER_BUTTON,ship_deferred=1,reason=vbus_present\n");
		return;
	}

	/* 4. 无 USB：熄灯并进入 Ship 模式（成功后 VSYS 被切断） */
	(void)status_led_set(false, false);
	printk("POWER_BUTTON,enter_ship=1,wake_hold_ms=608\n");
	ret = power_control_enter_ship();
	if (ret != 0) {
		enter_fault("enter_ship", ret);
		return;
	}

	/* 成功进入 Ship 后 VSYS 被切断，代码不应走到这里；
	 * 1s 后还能执行说明电源没有断掉，判定为故障。 */
	k_sleep(K_SECONDS(1));
	enter_fault("ship_did_not_remove_power", -EIO);
}

/**
 * @brief 处理按键松开事件，按住时长分三档处理
 *
 *  - >=3s：触发优雅关机；
 *  - <1s：短按，闪蓝灯两次作为状态确认；
 *  - 1~3s：取消关机（武装中途松手），闪红灯一下。
 */
static void handle_button_release(int64_t now_ms)
{
	uint32_t held_ms;

	/* 没有对应的按下记录（可能上电前就按着），忽略 */
	if (!button_pressed) {
		printk("POWER_BUTTON,event=release_without_press\n");
		return;
	}

	held_ms = (uint32_t)(now_ms - press_started_ms);
	button_pressed = false;
	printk("POWER_BUTTON,event=release,held_ms=%u\n", held_ms);

	if (held_ms >= SHUTDOWN_ARM_MS) {
		/* 长按达标：执行关机 */
		app_state = APP_STATE_SHUTTING_DOWN;
		request_graceful_shutdown();
	} else if (held_ms < SHORT_PRESS_MAX_MS) {
		/* 短按刻意做成无破坏性操作：仅状态确认 */
		(void)status_led_pulse(false, true, 80U, 70U);
		(void)status_led_pulse(false, true, 80U, 0U);
		printk("POWER_BUTTON,action=status_ack\n");
	} else {
		/* 1~3s 之间松手：取消关机武装 */
		(void)status_led_pulse(true, false, 120U, 0U);
		printk("POWER_BUTTON,action=shutdown_cancelled\n");
	}
}

/**
 * @brief 消费并处理 PMIC 事件（VBUS 插拔、按键按下/松开）
 */
static void handle_events(int64_t now_ms)
{
	/* 原子取出全部待处理事件并清零 */
	atomic_val_t events = atomic_set(&pending_events, 0);

	if ((events & BIT(NPM13XX_EVENT_VBUS_DETECTED)) != 0U) {
		/* 插入 USB：进入充电监护模式（系统保持运行） */
		app_state = APP_STATE_CHARGE_ONLY;
		printk("POWER_EVENT,vbus=present,mode=charge_only\n");
	}

	if ((events & BIT(NPM13XX_EVENT_VBUS_REMOVED)) != 0U) {
		printk("POWER_EVENT,vbus=removed,pending_ship=%u\n",
		       pending_ship_on_vbus_remove);
		if (pending_ship_on_vbus_remove) {
			/* 之前充电中推迟的关机请求：现在拔线了，执行进 Ship */
			pending_ship_on_vbus_remove = false;
			app_state = APP_STATE_SHUTTING_DOWN;
			request_graceful_shutdown();
		} else {
			/* 正常拔线：回到就绪状态 */
			app_state = APP_STATE_ACTIVE;
		}
	}

	if ((events & BIT(NPM13XX_EVENT_SHIPHOLD_PRESS)) != 0U) {
		/* 记录按下时刻（去重：避免重复按下事件覆盖计时起点） */
		if (!button_pressed) {
			button_pressed = true;
			press_started_ms = now_ms;
			printk("POWER_BUTTON,event=press\n");
		}
	}

	if ((events & BIT(NPM13XX_EVENT_SHIPHOLD_RELEASE)) != 0U) {
		handle_button_release(now_ms);
	}
}

/**
 * @brief 根据当前状态与按键情况计算并刷新 LED 图案（产品方案 §6）
 *
 *  - FAULT：每 2s 周期闪三次 120ms 红灯（错误图案）；
 *  - 按键按住 >=3s：红蓝同亮，提示"松手即关机"；
 *  - 按键按住 1~3s：红灯常亮，提示关机武装中；
 *  - CHARGE_ONLY：红灯 1s 周期闪烁（表示 USB 供电，非充电电流证明）；
 *  - ACTIVE：心跳指示——每 APP_HEARTBEAT_PERIOD_MS 双灯同亮
 *    APP_HEARTBEAT_FLASH_MS（默认 5s/100ms，可关，见《可配置项.txt》）。
 *
 * @return 0 成功；负值 LED 驱动失败
 */
static int update_led_pattern(int64_t now_ms)
{
	bool red = false;
	bool blue = false;

	if (app_state == APP_STATE_FAULT) {
		/* 故障图案：每 2s 帧内三次 120ms 红脉冲（0/240/480ms 处） */
		uint32_t phase = (uint32_t)(now_ms % 2000U);
		red = (phase < 120U) || ((phase >= 240U) && (phase < 360U)) ||
		      ((phase >= 480U) && (phase < 600U));
	} else if (button_pressed) {
		uint32_t held_ms = (uint32_t)(now_ms - press_started_ms);

		if (held_ms >= SHUTDOWN_ARM_MS) {
			/* 已达关机时长：红蓝同亮，提示现在松手即执行关机 */
			red = true;
			blue = true;
		} else if (held_ms >= SHORT_PRESS_MAX_MS) {
			/* 关机武装中：红灯常亮 */
			red = true;
		}
	} else if (app_state == APP_STATE_CHARGE_ONLY) {
		/* 注意：只表示 USB/仅充电模式，不代表实际充电电流 */
		red = ((uint32_t)now_ms % 1000U) < 500U;
	} else {
#if CONFIG_APP_BLE_LED_TEST
		/* BLE 测试 override（方案 §3.3 仲裁第 4 级）：
		 * 仅在 ACTIVE 且无按键/充电/故障抢占时可见，
		 * 覆盖心跳层；断开连接后 override 自动清除回落原逻辑 */
		uint8_t ble_mask;

		if (ble_led_override_get(&ble_mask)) {
			return status_led_set(
				(ble_mask & 0x01U) != 0U,
				(ble_mask & 0x02U) != 0U);
		}
#endif
#if CONFIG_APP_HEARTBEAT_LED
		/* ACTIVE：心跳指示——每 HEARTBEAT_PERIOD_MS 双灯同亮 HEARTBEAT_FLASH_MS */
		uint32_t beat = (uint32_t)now_ms % HEARTBEAT_PERIOD_MS;

		red = beat < HEARTBEAT_FLASH_MS;
		blue = red;
#else
		/* ACTIVE：无周期图案，静默运行（最省电） */
		red = false;
		blue = false;
#endif
	}

	return status_led_set(red, blue);
}

/**
 * @brief 主函数：初始化各模块，随后进入 20ms 周期的主循环
 *
 * 启动序列（产品方案 §5.1）：
 *  1. 电源控制初始化（三域全关 + LDSW 主机模式）；
 *  2. LED 初始化（主机控制模式）+ 开机指示：LED0 红闪 3 秒；
 *  3. （可选）三路电源域独立控制自检；
 *  4. 就绪指示：LED0+LED1 同闪 3 秒（一次性）；
 *  5. 注册 PMIC 事件，按 VBUS 状态确定初始模式，进入主循环。
 */
int main(void)
{
	bool vbus_present;
	int ret;

	printk("BOOT,SMARTPET_POWER_UI,v1.0\n");
	printk("POWER_POLICY,wake_hold_ms=608,shutdown_hold_ms=%u,"
	       "emergency_reset_s=10\n", SHUTDOWN_ARM_MS);
	printk("HW_NOTE,led2_polarity_reversed_expect_no_blue\n");

#if CONFIG_APP_HW_WATCHDOG
	{
		/*
		 * 复位原因（RESET.RESETREAS 裸读：nRF54L 无 hwinfo 驱动）。
		 * 看门狗复位（DOG0/DOG1）会在此留痕——WDT 兜底触发的自愈
		 * 在日志上可辨识，不与正常上电混淆。
		 */
		uint32_t cause = NRF_RESET->RESETREAS;

		printk("RESET,cause=0x%08x%s%s%s%s%s\n", cause,
		       (cause & RESET_RESETREAS_DOG0_Msk) ? ",wdt0" : "",
		       (cause & RESET_RESETREAS_DOG1_Msk) ? ",wdt31" : "",
		       (cause & RESET_RESETREAS_RESETPIN_Msk) ? ",pin" : "",
		       (cause & RESET_RESETREAS_CTRLAPSOFT_Msk) ? ",ctrl_soft" : "",
		       (cause & RESET_RESETREAS_CTRLAPHARD_Msk) ? ",ctrl_hard" : "");
		/* 写 1 清除，避免下次启动重复报告 */
		NRF_RESET->RESETREAS = cause;
	}
#endif

	/* 0. 尽早武装硬件看门狗：覆盖后面所有初始化与运行期；
	 * main 主循环喂狗（20ms/拍），任何线程/闭源库挂死 -> 复位恢复 */
	(void)app_watchdog_init();

	/* 1. 检查 PMIC 是否就绪 */
	if (!device_is_ready(pmic)) {
		printk("FATAL,pmic_not_ready\n");
		return -ENODEV;
	}

	/* 2. 顺序初始化：电源控制 -> LED */
	ret = power_control_init(pmic);
	if (ret == 0) {
		ret = status_led_init(pmic);
	}
	if (ret != 0) {
		printk("FATAL,initialization_failed,rc=%d\n", ret);
		return ret;
	}

#if CONFIG_APP_POWER_DOMAIN_TEST_MB
	/* 2.5 SWD 测试邮箱：上位机（tools/power_domain_test.py）经调试器
	 * 直接读写 RAM 邮箱下发三电源域独立开关命令 */
	test_mailbox_init();
#endif

#if CONFIG_APP_BOOT_LED_INDICATION
	/* 3. 开机指示：LED0（红）闪烁 3 秒 */
	ret = status_led_blink(true, false, INDICATION_MS,
			       BLINK_HALF_MS, BLINK_HALF_MS);
	if (ret != 0) {
		enter_fault("boot_led", ret);
		return ret;
	}
#endif

#if CONFIG_APP_POWER_DOMAIN_SELFTEST
	/* 4. 三路电源域独立控制自检（SENS/STORE/ANALOG 逐一开关校验） */
	(void)power_domain_selftest();
#endif

#if CONFIG_APP_IMU_TEST
	/* 4.5 IMU 连通性自检（task-V1.03）：NFC pad 归还复核 + i2c21 扫描
	 * + U4 WHO_AM_I 校验；失败仅打印 FAIL 不阻断启动（同 BLE 策略），
	 * 便于把电源 UI 主流程与 IMU 问题分开定位。 */
	if (imu_bringup_test() != 0) {
		printk("WARN,imu_bringup_failed\n");
	}
#endif

#if CONFIG_APP_TEMP_TEST
	/* 4.6 TMP112 温度自检（task-V1.04）：柔性板经 FPC，不在位仅
	 * 打印 ABSENT 不算故障（temp_sensor.c 返回值 1） */
	if (temp_bringup_test() < 0) {
		printk("WARN,temp_bringup_failed\n");
	}
#endif

#if CONFIG_APP_SD_TEST
	/* 4.7 SD NAND 自检（task-V1.04）：disk 初始化 + FATFS 读写比对；
	 * 无文件系统的裸卡会被格式化（bring-up 阶段允许） */
	if (sd_bringup_test() != 0) {
		printk("WARN,sd_bringup_failed\n");
	}
#endif

#if CONFIG_APP_MIC_TEST
	/* 4.8 双 PDM 麦自检（task-V1.04）：2s 采集 + 左右声道 RMS/峰值 */
	if (mic_bringup_test() != 0) {
		printk("WARN,mic_bringup_failed\n");
	}
#endif

#if CONFIG_APP_BOOT_LED_INDICATION
	/* 5. 就绪指示：LED0+LED1 同闪 3 秒（自检通过的一次性确认） */
	ret = status_led_blink(true, true, INDICATION_MS,
			       BLINK_HALF_MS, BLINK_HALF_MS);
	if (ret != 0) {
		enter_fault("ready_led", ret);
		return ret;
	}
#endif

	/* 6. 注册 PMIC 事件回调 */
	ret = configure_pmic_events();
	if (ret != 0) {
		enter_fault("pmic_events", ret);
		return ret;
	}

	/* 7. 根据 VBUS 状态确定初始运行模式 */
	ret = power_control_vbus_present(&vbus_present);
	if (ret != 0) {
		enter_fault("initial_vbus_read", ret);
		return ret;
	}
	if (vbus_present) {
		app_state = APP_STATE_CHARGE_ONLY;
		printk("SYSTEM_STATE,mode=charge_only,vbus=1,rails=off\n");
	} else {
		app_state = APP_STATE_ACTIVE;
		printk("SYSTEM_STATE,mode=active,vbus=0,rails=off\n");
	}

#if CONFIG_APP_BLE_LED_TEST
	/* 7.5 蓝牙从机初始化（异步：栈就绪后自动开始广播）。
	 * 失败仅告警不进 FAULT——蓝牙为附加功能，电源 UI 主流程
	 * 必须继续（方案 §7 风险 6）。 */
	ret = ble_led_test_init();
	if (ret != 0) {
		printk("WARN,ble_init_failed,rc=%d\n", ret);
	}
#endif

	printk("LED_MAP,red=nPM1300_LED0,blue=nPM1300_LED1\n");
	printk("LED_PATTERN,boot=red_blink_3s,ready=both_blink_3s,"
	       "shutdown=blue_blink_3s\n");
#if CONFIG_APP_HEARTBEAT_LED
	printk("LED_PATTERN,heartbeat=both_flash_%ums_every_%ums\n",
	       HEARTBEAT_FLASH_MS, HEARTBEAT_PERIOD_MS);
#endif
#if CONFIG_APP_BLE_LED_TEST
	printk("BLE,svc=e5a00001,chr=e5a00002,name=SmartPet\n");
#endif
	printk("BUTTON_UI,short=status_ack,hold_1s=arming_red,"
	       "hold_3s=release_to_ship_red_blue\n");

	/* 8. 主循环：20ms 周期轮询处理 */
	int64_t last_temp_ms = -1000;
	int temp_print_div = 0;

	while (true) {
		int64_t now_ms = k_uptime_get();

		app_watchdog_kick();           /* 心跳：sysworkq 门控喂狗 */
		handle_events(now_ms);          /* 按键/VBUS 事件 */
		ret = update_led_pattern(now_ms);  /* LED 状态指示 */
		if ((ret != 0) && (app_state != APP_STATE_FAULT)) {
			enter_fault("led_update", ret);
		}
		/* 温度 1Hz 读取 + BLE 通知（TMP112 U2；ABSENT 时静默跳过） */
#if CONFIG_APP_TEMP_TEST
		if ((now_ms - last_temp_ms) >= 1000) {
			last_temp_ms = now_ms;
			int32_t mdeg;

			if (temp_read_mdeg(&mdeg) == 0) {
				ble_imu_service_notify_temp(mdeg);
				if ((temp_print_div++ % 10) == 0) {
					printk("TEMP,now=%d.%02dC\n",
					       (int)(mdeg / 1000),
					       (int)((mdeg % 1000) / 10));
				}
			}
		}
#endif
#if CONFIG_APP_POWER_DOMAIN_TEST_MB
		test_mailbox_poll();            /* SWD 测试邮箱命令 */
#endif
		k_sleep(K_MSEC(LOOP_PERIOD_MS));
	}
}
