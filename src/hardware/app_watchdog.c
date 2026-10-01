/**
 * @file app_watchdog.c
 * @brief 硬件看门狗兜底实现（wdt31，nordic,nrf-wdt / wdt_nrfx 驱动）
 *
 * 喂狗采用"活性门控"：sysworkq 周期 work 检查 main 心跳，双活才喂。
 * 任一环节停摆（sysworkq 被 MPSL 死锁饿死 / main 主循环卡死）都会
 * 停止喂狗 -> 30s 硬件复位 -> 恢复广播。
 */
#include "app_watchdog.h"

#if CONFIG_APP_HW_WATCHDOG

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/sys/printk.h>

#define WDT_DEV_NODE DT_NODELABEL(wdt31)

#if !DT_NODE_HAS_STATUS(WDT_DEV_NODE, okay)
#error "app_watchdog: wdt31 未在 overlay 启用（&wdt31 { status = \"okay\"; }）"
#endif

/* 喂狗检查周期：远小于 30s 超时，又足够稀疏不打断睡眠 */
#define FEED_CHECK_PERIOD_SEC 5U

static const struct device *const wdt_dev = DEVICE_DT_GET(WDT_DEV_NODE);
static int wdt_ch = -1;

/* main 主循环心跳（app_watchdog_kick 递增） */
static volatile uint32_t main_heartbeat;
static uint32_t last_seen_heartbeat;

static void feed_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(feed_work, feed_work_handler);

static void feed_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (wdt_ch >= 0 && main_heartbeat != last_seen_heartbeat) {
		last_seen_heartbeat = main_heartbeat;
		(void)wdt_feed(wdt_dev, wdt_ch);
	}
	/* 心跳不动则故意不喂：main 卡死 -> 超时复位。
	 * 本 work 不运行（sysworkq 饿死）同理 -> 超时复位。
	 */
	(void)k_work_reschedule(&feed_work, K_SECONDS(FEED_CHECK_PERIOD_SEC));
}

int app_watchdog_init(void)
{
	if (!device_is_ready(wdt_dev)) {
		printk("WDT,not_ready\n");
		return -ENODEV;
	}

	/*
	 * 单窗口超时（min=0），SOC 复位。nRF WDT 计数器 32768Hz / 25bit，
	 * 上限约 1024s，30s 远在范围内。
	 */
	const struct wdt_timeout_cfg cfg = {
		.window.min = 0,
		.window.max = (uint32_t)CONFIG_APP_WATCHDOG_TIMEOUT_SEC * 1000U,
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	wdt_ch = wdt_install_timeout(wdt_dev, &cfg);
	if (wdt_ch < 0) {
		printk("WDT,install_rc=%d\n", wdt_ch);
		return wdt_ch;
	}

	int ret = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);

	if (ret != 0) {
		printk("WDT,setup_rc=%d\n", ret);
		wdt_ch = -1;
		return ret;
	}

	/* 武装时先喂一次，随后交给 sysworkq 周期 work 门控喂狗 */
	(void)wdt_feed(wdt_dev, wdt_ch);
	(void)k_work_schedule(&feed_work, K_SECONDS(FEED_CHECK_PERIOD_SEC));

	printk("WDT,armed,timeout_s=%u,ch=%d,pause_dbg=1,gate=sysworkq+main_hb\n",
	       CONFIG_APP_WATCHDOG_TIMEOUT_SEC, wdt_ch);
	return 0;
}

void app_watchdog_kick(void)
{
	main_heartbeat++;
}

#else /* !CONFIG_APP_HW_WATCHDOG */

int app_watchdog_init(void)
{
	printk("WDT,disabled\n");
	return 0;
}

void app_watchdog_kick(void)
{
}

#endif /* CONFIG_APP_HW_WATCHDOG */
