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
#include <stdint.h>
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

/* 温度特征值 e5a00013（int16 小端，单位 0.01°C；TMP112 U2） */
static const struct bt_uuid_128 temp_chr_uuid =
	BT_UUID_INIT_128(0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a,
			 0x8f, 0x4b, 0x5c, 0x1e, 0x13, 0x00, 0xa0, 0xe5);

/** 最新包序号（uint8 wrap） */
static uint8_t packet_seq;

/** 首包成功日志一次性标志（bring-up 诊断） */
static bool notify_seen;

/** 当前连接句柄（connected/disconnected 回调维护，NULL 表示未连接） */
static struct bt_conn *current_conn;

#if CONFIG_APP_BLE_IDLE_TIMEOUT_SEC > 0
/** CCC 订阅态（1=Notify 开）——空闲看门狗用，订阅中的连接永不断开 */
static atomic_t notify_subscribed = ATOMIC_INIT(0);
/** 最近一次对端活动时刻（k_uptime_get_32，32位ms约49天回绕，够用） */
static atomic_t last_peer_activity_ms = ATOMIC_INIT(0);
#else
/* 看门狗关闭时仍保留变量，让 ccc_cfg_changed 等处的引用无需条件编译 */
static atomic_t notify_subscribed = ATOMIC_INIT(0);
static atomic_t last_peer_activity_ms = ATOMIC_INIT(0);
#endif

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

	atomic_set(&notify_subscribed,
		   (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
	atomic_set(&last_peer_activity_ms, (atomic_val_t)k_uptime_get_32());

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
static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset);

/** 温度 CCC 订阅回调（与 IMU 流 CCC 分开，语义独立） */
static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	printk("TEMP_BLE,ccc,value=0x%04x%s\n", value,
	       (value == BT_GATT_CCC_NOTIFY) ? ",notify_on" : ",notify_off");
}

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
	/* 温度：attrs[6]=decl attrs[7]=value(notify 用) attrs[8]=CCC */
	BT_GATT_CHARACTERISTIC(&temp_chr_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_temp, NULL, NULL),
	BT_GATT_CCC(temp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
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

	atomic_set(&last_peer_activity_ms, (atomic_val_t)k_uptime_get_32());

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

	atomic_set(&last_peer_activity_ms, (atomic_val_t)k_uptime_get_32());

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
	atomic_set(&last_peer_activity_ms, (atomic_val_t)k_uptime_get_32());

	return len;
}

/* ---- 僵尸连接看门狗（APP_BLE_IDLE_TIMEOUT_SEC，0=禁用） ----
 *
 * 背景（2026-10-01 实测踩坑）：链路层监督超时只救得了"手机走远/
 * 关蓝牙"（prj.conf 里 0.42s 就会断开恢复广播）；但"App 被杀、
 * 手机蓝牙还开着、人在旁边"时，链路层 PDUs 仍在正常交换，监督超时
 * 永远不触发——板子就一直不广播，任何新设备都搜不到它。
 *
 * 策略：连接后若【未订阅通知】且【超过阈值时间没有任何读写/订阅】，
 * 判定为僵尸连接，主动断开恢复广播。订阅中的活跃连接不受影响；
 * 任何一次读、写、订阅/取消订阅都会刷新活动时间戳。
 */
#if CONFIG_APP_BLE_IDLE_TIMEOUT_SEC > 0
#define IDLE_CHECK_PERIOD_SEC 15

static void idle_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(idle_work, idle_work_handler);

static void idle_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (current_conn == NULL) {
		return; /* 已断开：停止自调度（下次连接时重新启动） */
	}

	if (atomic_get(&notify_subscribed) != 0) {
		/* 活跃订阅中：永不主动断开，继续观察 */
		k_work_reschedule(&idle_work,
				  K_SECONDS(IDLE_CHECK_PERIOD_SEC));
		return;
	}

	uint32_t idle_ms = k_uptime_get_32() -
			   (uint32_t)atomic_get(&last_peer_activity_ms);

	if (idle_ms >= (uint32_t)CONFIG_APP_BLE_IDLE_TIMEOUT_SEC * 1000U) {
		printk("IMU_BLE,idle_disconnect,idle_s=%u\n",
		       idle_ms / 1000U);
		int ret = bt_conn_disconnect(
			current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

		if (ret != 0) {
			/* 断开请求失败（罕见）：继续观察，别死循环 */
			printk("IMU_BLE,idle_disconnect_rc=%d\n", ret);
			k_work_reschedule(&idle_work,
					  K_SECONDS(IDLE_CHECK_PERIOD_SEC));
		}
		/* 成功路径：disconnected 回调会取消本 work 并重启广播 */
		return;
	}

	k_work_reschedule(&idle_work, K_SECONDS(IDLE_CHECK_PERIOD_SEC));
}
#endif /* APP_BLE_IDLE_TIMEOUT_SEC > 0 */

/* ---- 连接管理：维护连接句柄 + 断开自动停采集（省电/防孤儿流） ---- */

static void imu_connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		return;
	}
	if (current_conn == NULL) {
		current_conn = bt_conn_ref(conn);
	}
	atomic_set(&last_peer_activity_ms, (atomic_val_t)k_uptime_get_32());
#if CONFIG_APP_BLE_IDLE_TIMEOUT_SEC > 0
	k_work_reschedule(&idle_work, K_SECONDS(IDLE_CHECK_PERIOD_SEC));
#endif

#if CONFIG_APP_IMU_AUTOSTART
	/*
	 * 连接即开采集：手机 App 只需"订阅通知"一步就能看到数据，
	 * 不用先写控制点。避开实测踩到的顺序坑——控制点与订阅是两步，
	 * 用户若在订阅后又点了取消（CCC 0x0001->0x0000），采集照跑但
	 * 数据全丢（frames=64,dropped=11122），App 端一片空白。
	 * imu_stream_start() 幂等，随后的显式 0x01 写入也不会出错。
	 */
	atomic_set(&pending_cmd, 1);
	k_work_submit(&ctrl_work);
	printk("IMU_BLE,autostart\n");
#endif
}

static void imu_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	atomic_set(&notify_subscribed, 0);
#if CONFIG_APP_BLE_IDLE_TIMEOUT_SEC > 0
	(void)k_work_cancel_delayable(&idle_work);
#endif

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

/* ---- 温度通知（TMP112 U2，e5a00013） ---- */

/** 最新温度缓存（0.01°C 单位；INT16_MIN=无效） */
static int16_t latest_temp_centi = INT16_MIN;

ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &latest_temp_centi, sizeof(latest_temp_centi));
}

int ble_imu_service_notify_temp(int32_t mdeg)
{
	struct bt_conn *conn = current_conn;

	if (conn == NULL) {
		return -ENOTCONN;
	}
	latest_temp_centi = (int16_t)(mdeg / 10); /* 0.001°C -> 0.01°C */
	if (!bt_gatt_is_subscribed(conn, &imu_stream_svc.attrs[7],
				   BT_GATT_CCC_NOTIFY)) {
		return -EACCES;
	}
	return bt_gatt_notify(NULL, &imu_stream_svc.attrs[7],
			      &latest_temp_centi, sizeof(latest_temp_centi));
}
