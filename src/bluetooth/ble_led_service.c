/**
 * @file ble_led_service.c
 * @brief BLE 从机 LED 测试服务实现（nRF Connect App 可控 LED0/LED1）
 *
 * 详见 docs/蓝牙驱动测试方案.md（task-V1.02）与 ble_led_service.h 头注释。
 *
 * 不照搬 NCS peripheral_lbs 例程的 CONFIG_BT_LBS：LBS 写回调直驱 SoC GPIO，
 * 而本板 LED0/LED1 挂在 nPM1300 LEDDRV（I2C）后，且须与电源 UI 主循环的
 * LED 状态机仲裁。故自定义 GATT 服务，写回调仅置原子标志，点灯全部
 * 延迟到主循环 update_led_pattern() 完成（与 pending_events 模式同构）。
 */
#include "ble_led_service.h"

#if CONFIG_APP_BLE_LED_TEST

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/** 广播设备名（prj.conf CONFIG_BT_DEVICE_NAME 需一致） */
#define DEVICE_NAME     "SmartPet"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/** LED 控制位图定义（协议 §3.2：高 6 位保留忽略） */
#define LED_MASK_RED   BIT(0) /* LED0（红） */
#define LED_MASK_BLUE  BIT(1) /* LED1（蓝） */
#define LED_MASK_VALID 0x03U

/**
 * override 状态（原子）：
 *   -1 = 未激活（未连接 / 断开后清除 / 尚未写入）
 *   0~3 = App 最后写入的有效显示位图
 */
static atomic_t override_mask = ATOMIC_INIT(-1);

/** 关机中标志：置位后 disconnected 回调不再重启广播 */
static atomic_t shutting_down;

/** 当前连接句柄（BT 线程读写，持引用计数；NULL 表示未连接） */
static struct bt_conn *current_conn;

/*
 * 自定义 UUID（128-bit，小端字节序）：
 *   服务   e5a00001-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   特征值 e5a00002-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 */
static const struct bt_uuid_128 led_svc_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x01, 0x00, 0xa0, 0xe5);

static const struct bt_uuid_128 led_chr_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x02, 0x00, 0xa0, 0xe5);

/**
 * @brief GATT 读回调：返回当前 override 显示字节（未激活返回 0x00）
 */
static ssize_t read_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	atomic_val_t v = atomic_get(&override_mask);
	uint8_t val = (v < 0) ? 0U : (uint8_t)v;

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &val, sizeof(val));
}

/**
 * @brief GATT 写回调（BT 线程）：仅原子置位图 + 打印，不做任何 I2C 操作
 *
 * 写入值按低 2 位掩码处理，高 6 位保留位忽略（方案 §3.2）。
 */
static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	const uint8_t *val = buf;
	uint8_t mask;

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	mask = *val & LED_MASK_VALID;
	atomic_set(&override_mask, mask);
	printk("BLE,led_write,mask=0x%02x\n", mask);

	return len;
}

/** SmartPet LED 测试服务（链接期静态注册，无需显式 init 调用） */
BT_GATT_SERVICE_DEFINE(led_test_svc,
	BT_GATT_PRIMARY_SERVICE(&led_svc_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&led_chr_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_led, write_led, NULL),
);

/**
 * @brief 开始可连接广播（Flags + 128-bit 服务 UUID / Scan Response 设备名）
 *
 * 注意：BT_DATA_BYTES 展开含复合字面量，不能用作 static 初始化器，
 * ad/sd 须为函数内局部 const（与 NCS 官方例程一致）。
 */
static int adv_start(void)
{
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS,
			      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA_BYTES(BT_DATA_UUID128_SOME,
			      0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			      0x8f, 0x4b, 0x5c, 0x1e, 0x01, 0x00, 0xa0, 0xe5),
	};
	const struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	};
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));

	if (ret == 0) {
		printk("BLE,adv_started\n");
	} else {
		printk("BLE,adv_start_failed,rc=%d\n", ret);
	}
	return ret;
}

/**
 * @brief 广播重启工作项（系统工作队列，delayable）
 *
 * 2026-10-01 上板实测：在 disconnected 回调（BT RX 线程）里同步调
 * bt_le_adv_start 会因控制器尚未释放连接资源而失败（rc=-12 ENOMEM），
 * 导致断开后设备无法再被连接。故延迟 20ms 到系统工作队列重启，
 * 失败再以 100ms 间隔重试（最多 5 次）。
 */
static void adv_restart_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_handler);

/** 广播重启重试计数（adv_restart_handler 私有） */
static uint8_t adv_restart_retry;

static void adv_restart_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* 关机流程已停广播，不得复活（与 ble_led_test_shutdown 互斥） */
	if (atomic_get(&shutting_down) != 0) {
		adv_restart_retry = 0;
		return;
	}

	if (adv_start() != 0 && adv_restart_retry < 5) {
		adv_restart_retry++;
		k_work_reschedule(&adv_restart_work, K_MSEC(100));
	} else {
		adv_restart_retry = 0;
	}
}

/**
 * @brief 连接建立回调（BT 线程）
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		printk("BLE,connect_failed,err=0x%02x\n", err);
		return;
	}

	if (current_conn == NULL) {
		current_conn = bt_conn_ref(conn);
	}
	printk("BLE,connected\n");
}

/**
 * @brief 连接断开回调（BT 线程）：清除 override，非关机场景自动重新广播
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	/* 断开即清除 override：LED 显示权立即交还原有状态机（方案 §3.3） */
	atomic_set(&override_mask, -1);
	printk("BLE,disconnected,reason=0x%02x\n", reason);

	if (atomic_get(&shutting_down) == 0) {
		/* 不能在此回调里直接 adv_start（rc=-12），延迟到工作队列 */
		k_work_reschedule(&adv_restart_work, K_MSEC(20));
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/**
 * @brief bt_enable 完成回调（BT 线程）：栈就绪后开始广播
 */
static void bt_ready(int ret)
{
	if (ret != 0) {
		printk("BLE,enable_failed,rc=%d\n", ret);
		return;
	}
	printk("BLE,enabled,name=%s\n", DEVICE_NAME);
	(void)adv_start();
}

int ble_led_test_init(void)
{
	atomic_set(&override_mask, -1);
	atomic_set(&shutting_down, 0);

	/* 异步使能：结果经 bt_ready 回调报告，错误不阻塞电源 UI 主流程 */
	return bt_enable(bt_ready);
}

bool ble_led_override_get(uint8_t *mask)
{
	atomic_val_t v = atomic_get(&override_mask);

	if (v < 0) {
		return false;
	}
	*mask = (uint8_t)v;
	return true;
}

void ble_led_test_shutdown(void)
{
	/* 先置关机标志：避免随后的断开回调/延迟工作把广播重新拉起来 */
	atomic_set(&shutting_down, 1);
	(void)k_work_cancel_delayable(&adv_restart_work);
	(void)bt_le_adv_stop();
	printk("BLE,adv_stopped\n");

	if (current_conn != NULL) {
		if (bt_conn_disconnect(current_conn,
				       BT_HCI_ERR_REMOTE_USER_TERM_CONN) == 0) {
			printk("BLE,disconnecting\n");
		}
	}

	atomic_set(&override_mask, -1);
}

#endif /* CONFIG_APP_BLE_LED_TEST */
