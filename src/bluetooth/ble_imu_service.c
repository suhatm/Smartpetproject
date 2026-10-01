/**
 * @file ble_imu_service.c
 * @brief BLE 从机 IMU 数据流服务实现（Notify 帧流 + 控制点启停）
 *
 * 详见 ble_imu_service.h 头注释与 docs/IMU采集测试方案.md §4。
 *
 * 线程模型（对齐 ble_led_service 的"回调不做事"原则，此处适度放宽）：
 *   - 控制点写回调（BT 线程）：仅记录命令字节 + 提交 ctrl_work；
 *   - ctrl_work（系统工作队列）：调 imu_stream_start/stop（含 I2C 上电
 *     与驱动初始化，毫秒级，不放 BT 线程里做）；
 *   - notify_work（系统工作队列，50ms 周期）：从 imu_stream 缓冲取帧
 *     打包 -> bt_gatt_notify。未订阅/未连接时只空转一拍再试。
 *
 * MTU 适配：bt_gatt_get_mtu(conn) 动态算每包最大帧数，App 未请求大 MTU
 * 时自动缩小包（默认 MTU 23 时 1 帧/包也能工作，只是慢）。
 */
#include "ble_imu_service.h"

#if CONFIG_APP_BLE_IMU_STREAM

#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "imu_stream.h"

/** 打包周期：50ms（30Hz 采集 ≈ 每 1~2 帧，延迟远小于人手感知） */
#define NOTIFY_PERIOD_MS 50

/** 单包帧数上限（MTU 247 时 (247-3-2)/12 = 20 帧） */
#define FRAMES_PER_PACKET_MAX 20

/** 控制点命令 */
#define IMU_CMD_STOP  0x00
#define IMU_CMD_START 0x01

/*
 * 自定义 UUID（128-bit 小端）：
 *   服务   e5a00010-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   数据流 e5a00011-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   控制点 e5a00012-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 */
static const struct bt_uuid_128 imu_svc_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x10, 0x00, 0xa0, 0xe5);

static const struct bt_uuid_128 imu_data_chr_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x11, 0x00, 0xa0, 0xe5);

static const struct bt_uuid_128 imu_ctrl_chr_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x12, 0x00, 0xa0, 0xe5);

/** 最新包序号（uint8 wrap） */
static uint8_t packet_seq;

/** 首包成功日志一次性标志（bring-up 诊断） */
static bool notify_seen;

/** 当前连接句柄（connected/disconnected 回调维护，NULL 表示未连接） */
static struct bt_conn *current_conn;

/*
 * 坑（2026-10-01 实测踩过）：BT_GATT_CCC 第二个参数是【权限位】不是 CCC 值。
 * 写成 BT_GATT_CCC(ccc_cfg_changed, BT_GATT_CCC_NOTIFY) 时，
 * BT_GATT_CCC_NOTIFY(0x0001) 恰好等于 BT_GATT_PERM_READ(0x01)，
 * 描述符成了只读 → 手机写 CCC 被栈拒绝（Write Not Permitted），
 * ccc_cfg_changed 永远不会回调，Notify 订阅也就永远开不起来。
 * 正确写法见官方样例 peripheral_power_profiling：READ | WRITE。
 */

/** CCC 订阅状态变化（App 打开/关闭 Notify） */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	printk("IMU_BLE,ccc,value=0x%04x%s\n", value,
	       (value == BT_GATT_CCC_NOTIFY) ? ",notify_on" : ",notify_off");
}

/* ---- 前向声明（BT_GATT_SERVICE_DEFINE 引用在先） ---- */

static ssize_t read_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset);
static ssize_t read_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset);
static ssize_t write_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset,
			  uint8_t flags);

/** IMU 数据流服务（链接期静态注册；attrs[2] = 数据流特征值，notify 用） */
BT_GATT_SERVICE_DEFINE(imu_stream_svc,
	BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&imu_data_chr_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_data, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&imu_ctrl_chr_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_ctrl, write_ctrl, NULL),
);

/* ---- Notify 打包：50ms 周期从 imu_stream 缓冲取帧发送 ---- */

static void notify_work_handler(struct k_work *work);

/** notify 周期工作项（delayable，ctrl_work 启动成功后首次调度） */
static K_WORK_DELAYABLE_DEFINE(notify_work, notify_work_handler);

static void notify_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* 单连接产品：connected 回调维护的 current_conn，NULL=未连接 */
	struct bt_conn *conn = current_conn;
	struct imu_frame frames[FRAMES_PER_PACKET_MAX];
	uint8_t pkt[2 + sizeof(frames)];
	size_t max_frames = FRAMES_PER_PACKET_MAX;
	size_t cnt;

	if (conn != NULL) {
		bool subscribed = bt_gatt_is_subscribed(
			conn, &imu_stream_svc.attrs[2], BT_GATT_CCC_NOTIFY);
		/* MTU-3(ATT 头) -2(包内 seq/cnt) 再除 12B/帧 */
		uint16_t mtu = bt_gatt_get_mtu(conn);

		if (mtu > 5U) {
			size_t by_mtu =
				(mtu - 3U - 2U) / sizeof(struct imu_frame);
			max_frames = MIN(max_frames, by_mtu);
		}

		if (!subscribed || max_frames == 0U) {
			goto resched; /* 未订阅 / MTU 太小：等下一拍 */
		}
	}

	cnt = imu_stream_drain(frames, max_frames);
	if (cnt == 0U) {
		goto resched; /* 本拍无新帧 */
	}

	pkt[0] = ++packet_seq;
	pkt[1] = (uint8_t)cnt;
	memcpy(&pkt[2], frames, cnt * sizeof(struct imu_frame));

	int ret = bt_gatt_notify(NULL, &imu_stream_svc.attrs[2],
				 pkt, 2U + cnt * sizeof(struct imu_frame));

	if (ret != 0) {
		/* 典型 -ENOMEM：ACL TX 缓冲耗尽，本包丢，下拍继续 */
		printk("IMU_BLE,notify_rc=%d,frames_lost=%u\n", ret,
		       (unsigned)cnt);
	} else if (!notify_seen) {
		/* 首包成功：确认 Notify 通路真的通了（bring-up 诊断用） */
		notify_seen = true;
		printk("IMU_BLE,notify_ok,mtu=%u,frames=%u\n",
		       (unsigned)bt_gatt_get_mtu(conn), (unsigned)cnt);
	}

resched:
	/* 采集中持续调度；stop 后不再续期，自然停转 */
	if (imu_stream_active()) {
		k_work_reschedule(&notify_work, K_MSEC(NOTIFY_PERIOD_MS));
	}
}

/* ---- 控制点：写回调只记录命令，实活在 ctrl_work 做 ---- */

/** 待处理命令（0=无 1=start 2=stop，写回调与 work 之间单向传递） */
static atomic_t pending_cmd = ATOMIC_INIT(0);

static void ctrl_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int cmd = (int)atomic_set(&pending_cmd, 0);

	if (cmd == 1) {
		int ret = imu_stream_start();

		printk("IMU_BLE,start_rc=%d\n", ret);
		if (ret == 0) {
			/* 启动成功：立即打包一拍（App 马上能看到数据） */
			notify_seen = false;
			k_work_reschedule(&notify_work, K_NO_WAIT);
		}
	} else if (cmd == 2) {
		imu_stream_stop();
		printk("IMU_BLE,stopped\n");
	}
}

static K_WORK_DEFINE(ctrl_work, ctrl_work_handler);

/* ---- GATT 回调实现 ---- */

/**
 * @brief 读回调（数据流特征值）：3 字节状态 [active][src][seq]
 */
static ssize_t read_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t state[3] = {
		imu_stream_active() ? 1U : 0U,
		(uint8_t)imu_stream_source(),
		packet_seq,
	};

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 state, sizeof(state));
}

/**
 * @brief 读回调（控制点）：1 字节（0=停止 1=采集中）
 */
static ssize_t read_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t val = imu_stream_active() ? 1U : 0U;

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &val, sizeof(val));
}

/**
 * @brief 写回调（控制点，BT 线程）：仅记命令 + 提交工作项，不做 I2C
 */
static ssize_t write_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset,
			  uint8_t flags)
{
	const uint8_t *val = buf;

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	switch (*val) {
	case IMU_CMD_START:
		atomic_set(&pending_cmd, 1);
		break;
	case IMU_CMD_STOP:
		atomic_set(&pending_cmd, 2);
		break;
	default:
		printk("IMU_BLE,ctrl_unknown,0x%02x\n", *val);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	k_work_submit(&ctrl_work);
	printk("IMU_BLE,ctrl_cmd=%u\n", *val);

	return len;
}

/* ---- 连接管理：维护连接句柄 + 断开自动停采集（省电/防孤儿流） ---- */

static void imu_connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		return;
	}
	if (current_conn == NULL) {
		current_conn = bt_conn_ref(conn);
	}
}

static void imu_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	if (imu_stream_active()) {
		printk("IMU_BLE,disconnect_stop,reason=0x%02x\n", reason);
		atomic_set(&pending_cmd, 2);
		k_work_submit(&ctrl_work);
	}
}

BT_CONN_CB_DEFINE(imu_conn_callbacks) = {
	.connected = imu_connected,
	.disconnected = imu_disconnected,
};

int ble_imu_service_init(void)
{
	/* GATT 服务静态注册，无运行时初始化；保留入口供未来扩展 */
	return 0;
}

#endif /* CONFIG_APP_BLE_IMU_STREAM */
