/**
 * @file ble_sensor_hub.c
 * @brief BLE Sensor Hub 服务实现（协议 V0.5）
 *
 * 线程模型：
 *   - GATT 回调（BT RX 线程）：写指令 -> k_msgq -> bridge 工作项消费；
 *     订阅状态原子记录；任何 ATT 活动喂僵尸看门狗。
 *   - pack_work（系统工作队列，50ms 周期/文件传输立即）：从两个帧缓冲
 *     （stream/ack）取出已封装帧，按 MTU-3 上限串联，bt_gatt_notify。
 *
 * 连接管理沿用 ble_led_service 的实测结论：
 *   - disconnected 回调里不能同步 bt_le_adv_start（rc=-12），延迟 20ms 重试；
 *   - 关机流程置 shutting_down 后广播不再复活；
 *   - 僵尸连接看门狗：连接后未订阅且无活动超时主动断开（APP_BLE_IDLE_TIMEOUT_SEC）。
 */
#include "ble_sensor_hub.h"

#if CONFIG_APP_SENSOR_HUB

#include <errno.h>
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

#define DEVICE_NAME     "SmartPet"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* 帧结构常量（协议 §4.1） */
#define FRAME_SYNC0      0x55U
#define FRAME_SYNC1      0xAAU
#define FRAME_HDR_LEN    5U /* SYNC2 + TYPE + SEQ + LEN */
#define FRAME_CRC_LEN    1U
#define FRAME_MAX_PAYLOAD 240U

/** 帧环形缓冲（已封装整帧字节流）：stream 与 ack 各一个 */
#define FRAME_BUF_SIZE   2048U

/** 打包周期（ms）：缓冲区非空即组包；文件传输时立即 */
#define PACK_PERIOD_MS   50U

/** 指令接收队列深度 */
#define CMD_MSGQ_DEPTH   8U

/* 128-bit UUID（小端字节序），基座 e5a0xxxx-1e5c-4b8f-9a2d-6c0f7e8d9a0b */
#define HUB_UUID_BYTES(x) \
	0x0b, 0x9a, 0x8d, 0x7e, 0x0f, 0x6c, 0x2d, 0x9a, \
	0x8f, 0x4b, 0x5c, 0x1e, (x), 0x00, 0xa0, 0xe5

static const struct bt_uuid_128 hub_svc_uuid =
	BT_UUID_INIT_128(HUB_UUID_BYTES(0x20));
static const struct bt_uuid_128 hub_data_uuid =
	BT_UUID_INIT_128(HUB_UUID_BYTES(0x21));
static const struct bt_uuid_128 hub_cmd_uuid =
	BT_UUID_INIT_128(HUB_UUID_BYTES(0x22));
static const struct bt_uuid_128 hub_ack_uuid =
	BT_UUID_INIT_128(HUB_UUID_BYTES(0x23));
static const struct bt_uuid_128 hub_info_uuid =
	BT_UUID_INIT_128(HUB_UUID_BYTES(0x24));

/** 固件信息字符串（e5a00024 / GET_VERSION 共用） */
#define FW_INFO_STRING "SmartPetRing v1.07;task-V1.07;hubV0.5"

/* ---- 帧缓冲 ---- */

struct frame_buf {
	uint8_t data[FRAME_BUF_SIZE];
	uint16_t head; /**< 待发送起点 */
	uint16_t tail; /**< 写入终点 */
	uint16_t used;
	uint8_t seq[256]; /**< 每个 TYPE 独立序号 */
	struct k_mutex lock;
};

static struct frame_buf stream_buf = { .lock = Z_MUTEX_INITIALIZER(stream_buf.lock) };
static struct frame_buf ack_buf    = { .lock = Z_MUTEX_INITIALIZER(ack_buf.lock) };

K_MSGQ_DEFINE(cmd_msgq, sizeof(struct hub_cmd_msg), CMD_MSGQ_DEPTH, 4);

/** 指令处理回调（bridge 注册） */
static hub_cmd_handler_t cmd_handler;

/** 设备端指令接收计数（ACK.req_seq 回显用） */
static uint8_t cmd_rx_seq;

/** CCC 订阅状态（原子） */
static atomic_t stream_ccc;
static atomic_t ack_ccc;

/** 关机互斥标志 */
static atomic_t shutting_down;

/** 当前连接（持引用计数） */
static struct bt_conn *current_conn;

/** 僵尸连接看门狗：最后 ATT 活动时间戳（ms，原子存 uptime/1000 秒） */
static atomic_t last_activity_sec;

/** LED override 位图（-1=未激活） */
static atomic_t led_override = ATOMIC_INIT(-1);

/** 文件传输模式 */
static atomic_t file_xfer;

/* ---- CRC8（多项式 0x07，初值 0x00，非反射，覆盖 TYPE~PAYLOAD） ---- */

static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
	crc ^= byte;
	for (uint8_t i = 0; i < 8U; i++) {
		crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
				    : (uint8_t)(crc << 1);
	}
	return crc;
}

/* ---- 帧封装入缓冲 ---- */

static int frame_buf_put(struct frame_buf *fb, uint8_t type,
			 const uint8_t *payload, uint8_t len)
{
	uint16_t frame_len = FRAME_HDR_LEN + len + FRAME_CRC_LEN;

	k_mutex_lock(&fb->lock, K_FOREVER);
	if ((uint16_t)(fb->used + frame_len) > FRAME_BUF_SIZE) {
		k_mutex_unlock(&fb->lock);
		return -ENOMEM;
	}

	uint16_t w = fb->tail;
	uint8_t crc = 0U;
	uint8_t seq = fb->seq[type]++;

	fb->data[w] = FRAME_SYNC0;
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);
	fb->data[w] = FRAME_SYNC1;
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);

	fb->data[w] = type;
	crc = crc8_update(crc, type);
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);
	fb->data[w] = seq;
	crc = crc8_update(crc, seq);
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);
	fb->data[w] = len;
	crc = crc8_update(crc, len);
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);

	for (uint8_t i = 0; i < len; i++) {
		fb->data[w] = payload[i];
		crc = crc8_update(crc, payload[i]);
		w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);
	}
	fb->data[w] = crc;
	w = (uint16_t)((w + 1U) % FRAME_BUF_SIZE);

	fb->tail = w;
	fb->used = (uint16_t)(fb->used + frame_len);
	k_mutex_unlock(&fb->lock);
	return 0;
}

/**
 * @brief 从帧缓冲取出最多 max_len 字节（不跨帧）
 *
 * @return 取出字节数（0=缓冲空或剩余空间放不下下一整帧）
 */
static uint16_t frame_buf_take(struct frame_buf *fb, uint8_t *out, uint16_t max_len)
{
	uint16_t copied = 0U;

	k_mutex_lock(&fb->lock, K_FOREVER);
	while (fb->used > 0U) {
		/* 整帧长度 = 头5 + LEN + CRC1；帧布局 SYNC0/1 TYPE SEQ LEN，LEN 在 head+4 */
		uint16_t avail = fb->used;
		uint8_t plen = fb->data[(fb->head + 4U) % FRAME_BUF_SIZE];
		uint16_t flen = FRAME_HDR_LEN + plen + FRAME_CRC_LEN;

		if ((avail < flen) || ((uint16_t)(copied + flen) > max_len)) {
			break;
		}
		for (uint16_t i = 0; i < flen; i++) {
			out[copied++] = fb->data[fb->head];
			fb->head = (uint16_t)((fb->head + 1U) % FRAME_BUF_SIZE);
		}
		fb->used = (uint16_t)(fb->used - flen);
	}
	k_mutex_unlock(&fb->lock);
	return copied;
}

/* ---- 打包工作项 ---- */

/** GATT 服务前向声明（BT_GATT_SERVICE_DEFINE 在本文件后段展开） */
extern const struct bt_gatt_service_static hub_svc;

static void pack_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(pack_work, pack_work_handler);

/** 单包发送缓冲（MTU 上限 247-3） */
static uint8_t pkt_buf[244];

/** 发送挂起槽：notify 失败时保留未发出字节，下次先补发（保序不丢帧）。
 *  上板实测：TX 缓冲耗尽时 bt_gatt_notify 返错，直接丢弃已取出的包
 *  会导致文件传输缺帧（REC_READ 长度/CRC 失败）。 */
static uint8_t ack_pend[FRAME_HDR_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN];
static uint16_t ack_pend_len;
static uint8_t stream_pend[FRAME_HDR_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN];
static uint16_t stream_pend_len;

/** 单通道发送：先补挂起包，再按 room 取新帧；失败保数据并停发（由调用方重调度） */
static void flush_channel(struct frame_buf *fb, const struct bt_gatt_attr *attr,
			  uint8_t *pend, uint16_t *pend_len, uint16_t room,
			  const char *tag)
{
	for (;;) {
		if (*pend_len == 0U) {
			*pend_len = frame_buf_take(fb, pend, room);
			if (*pend_len == 0U) {
				return; /* 缓冲空 */
			}
		}
		if (bt_gatt_notify(NULL, attr, pend, *pend_len) == 0) {
			*pend_len = 0U;
			continue;
		}
		printk("HUB,%s_notify_fail,n=%u\n", tag, *pend_len);
		return; /* 保留 pend，等待重试 */
	}
}

static void pack_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if ((current_conn == NULL) ||
	    ((atomic_get(&stream_ccc) == 0) && (atomic_get(&ack_ccc) == 0))) {
		/* 未连接/未订阅：清空缓冲避免堆积过期数据 */
		uint8_t dump[64];

		while (frame_buf_take(&stream_buf, dump, sizeof(dump)) > 0U) {
		}
		while (frame_buf_take(&ack_buf, dump, sizeof(dump)) > 0U) {
		}
		stream_pend_len = 0U;
		ack_pend_len = 0U;
		return;
	}

	uint16_t mtu = bt_gatt_get_mtu(current_conn);
	uint16_t room = (mtu > (3U + FRAME_HDR_LEN + FRAME_CRC_LEN))
				? (uint16_t)(mtu - 3U)
				: 20U;

	if (room > sizeof(pkt_buf)) {
		room = sizeof(pkt_buf);
	}

	/* ack 通道优先（应答/事件实时性高于数据流） */
	flush_channel(&ack_buf, &hub_svc.attrs[7], ack_pend, &ack_pend_len,
		      room, "ack");
	flush_channel(&stream_buf, &hub_svc.attrs[2], stream_pend,
		      &stream_pend_len, room, "data");

	/* 还有未发完的数据（缓冲剩余或挂起包）：继续调度 */
	bool pending;

	k_mutex_lock(&stream_buf.lock, K_FOREVER);
	pending = stream_buf.used > 0U;
	k_mutex_unlock(&stream_buf.lock);
	k_mutex_lock(&ack_buf.lock, K_FOREVER);
	pending = pending || (ack_buf.used > 0U);
	k_mutex_unlock(&ack_buf.lock);
	pending = pending || (stream_pend_len > 0U) || (ack_pend_len > 0U);

	if (pending) {
		/* 有挂起包=TX 缓冲满：短退避；否则按常规周期/文件传输立即 */
		k_timeout_t delay = K_MSEC(PACK_PERIOD_MS);

		if ((stream_pend_len > 0U) || (ack_pend_len > 0U)) {
			delay = K_MSEC(5);
		} else if (atomic_get(&file_xfer) != 0) {
			delay = K_NO_WAIT;
		}
		k_work_reschedule(&pack_work, delay);
	}
}

static void pack_kick(void)
{
	k_work_reschedule(&pack_work,
			  (atomic_get(&file_xfer) != 0) ? K_NO_WAIT
							: K_MSEC(PACK_PERIOD_MS));
}

/* ---- GATT 回调 ---- */

void hub_activity_kick(void)
{
	atomic_set(&last_activity_sec,
		   (atomic_val_t)(k_uptime_get() / 1000));
}

static ssize_t read_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	static const char info[] = FW_INFO_STRING;

	hub_activity_kick();
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 info, sizeof(info) - 1U);
}

static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	struct hub_cmd_msg msg;

	hub_activity_kick();

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if ((len < 1U) || (len > HUB_CMD_MAX_LEN)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	msg.req_seq = cmd_rx_seq++;
	msg.len = (uint8_t)len;
	memcpy(msg.data, buf, len);

	if (k_msgq_put(&cmd_msgq, &msg, K_NO_WAIT) != 0) {
		printk("HUB,cmdq_full,cmd=0x%02x\n", msg.data[0]);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* 唤醒 bridge 指令处理（复用 pack_work 所在工作队列由 bridge 自取） */
	extern void sensor_hub_bridge_cmd_kick(void);

	sensor_hub_bridge_cmd_kick();
	return len;
}

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	atomic_set(&stream_ccc, (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
	hub_activity_kick();
	printk("HUB,data_ccc=%u\n", (value == BT_GATT_CCC_NOTIFY) ? 1U : 0U);
}

static void ack_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	atomic_set(&ack_ccc, (value == BT_GATT_CCC_NOTIFY) ? 1 : 0);
	hub_activity_kick();
	printk("HUB,ack_ccc=%u\n", (value == BT_GATT_CCC_NOTIFY) ? 1U : 0U);
}

/** Sensor Hub 服务（属性索引见下方布局注释） */
BT_GATT_SERVICE_DEFINE(hub_svc,
	BT_GATT_PRIMARY_SERVICE(&hub_svc_uuid.uuid),
	/* 数据流 e5a00021：decl[1] value[2] ccc[3] */
	BT_GATT_CHARACTERISTIC(&hub_data_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* 指令 e5a00022：decl[4] value[5] */
	BT_GATT_CHARACTERISTIC(&hub_cmd_uuid.uuid,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, write_cmd, NULL),
	/* 应答/事件 e5a00023：decl[6] value[7] ccc[8] */
	BT_GATT_CHARACTERISTIC(&hub_ack_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(ack_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* 信息 e5a00024：decl[9] value[10] */
	BT_GATT_CHARACTERISTIC(&hub_info_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_info, NULL, NULL),
);

/*
 * 属性索引布局（BT_GATT_SERVICE_DEFINE 宏展开，decl/value/ccc 各占一项）：
 *   [0] 主服务  [1] 数据 decl  [2] 数据 value  [3] CCC
 *   [4] 指令 decl  [5] 指令 value
 *   [6] 应答 decl  [7] 应答 value  [8] CCC
 *   [9] 信息 decl  [10] 信息 value
 * notify 目标：数据 = attrs[2]，应答 = attrs[7]。
 * 修改服务布局时须同步 pack_work_handler 中的索引。
 */

/* ---- 连接管理与广播（沿用实测结论） ---- */

static int adv_start(void)
{
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS,
			      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA_BYTES(BT_DATA_UUID128_SOME, HUB_UUID_BYTES(0x20)),
	};
	const struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	};
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));

	if (ret == 0) {
		printk("HUB,adv_started\n");
	} else {
		printk("HUB,adv_start_failed,rc=%d\n", ret);
	}
	return ret;
}

static void adv_restart_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_handler);

static uint8_t adv_restart_retry;

static void adv_restart_handler(struct k_work *work)
{
	ARG_UNUSED(work);

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

/** 僵尸连接看门狗工作项（1s 周期） */
static void zombie_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(zombie_work, zombie_work_handler);

static void zombie_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

#if CONFIG_APP_BLE_IDLE_TIMEOUT_SEC > 0
	if (current_conn != NULL) {
		int64_t idle = (k_uptime_get() / 1000) -
			       (int64_t)atomic_get(&last_activity_sec);
		bool any_ccc = (atomic_get(&stream_ccc) != 0) ||
			       (atomic_get(&ack_ccc) != 0);

		if (!any_ccc && (idle >= CONFIG_APP_BLE_IDLE_TIMEOUT_SEC)) {
			printk("HUB,zombie_disconnect,idle_s=%lld\n", idle);
			(void)bt_conn_disconnect(current_conn,
						 BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			return;
		}
	}
#endif
	k_work_reschedule(&zombie_work, K_SECONDS(1));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		printk("HUB,connect_failed,err=0x%02x\n", err);
		return;
	}
	if (current_conn == NULL) {
		current_conn = bt_conn_ref(conn);
	}
	hub_activity_kick();
	atomic_set(&stream_ccc, 0);
	atomic_set(&ack_ccc, 0);
	printk("HUB,connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (current_conn != NULL) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	atomic_set(&stream_ccc, 0);
	atomic_set(&ack_ccc, 0);
	atomic_set(&led_override, -1); /* 断开即归还 LED 状态机 */
	printk("HUB,disconnected,reason=0x%02x\n", reason);

	if (atomic_get(&shutting_down) == 0) {
		k_work_reschedule(&adv_restart_work, K_MSEC(20));
	}
}

BT_CONN_CB_DEFINE(hub_conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int ret)
{
	if (ret != 0) {
		printk("HUB,enable_failed,rc=%d\n", ret);
		return;
	}
	printk("HUB,enabled,name=%s,svc=e5a00020\n", DEVICE_NAME);
	(void)adv_start();
	k_work_reschedule(&zombie_work, K_SECONDS(1));
}

/* ---- 对外 API ---- */

int sensor_hub_init(hub_cmd_handler_t handler)
{
	cmd_handler = handler;
	atomic_set(&shutting_down, 0);
	hub_activity_kick();
	return bt_enable(bt_ready);
}

void sensor_hub_shutdown(void)
{
	atomic_set(&shutting_down, 1);
	(void)k_work_cancel_delayable(&adv_restart_work);
	(void)k_work_cancel_delayable(&zombie_work);
	(void)k_work_cancel_delayable(&pack_work);
	(void)bt_le_adv_stop();
	printk("HUB,adv_stopped\n");

	if (current_conn != NULL) {
		(void)bt_conn_disconnect(current_conn,
					 BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	atomic_set(&led_override, -1);
}

int hub_stream_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	if ((payload == NULL && len > 0U) || len > FRAME_MAX_PAYLOAD) {
		return -EINVAL;
	}
	if ((current_conn == NULL) || (atomic_get(&stream_ccc) == 0)) {
		return 0; /* 未订阅丢弃，不算错误 */
	}
	int ret = frame_buf_put(&stream_buf, type, payload, len);

	if (ret == 0) {
		pack_kick();
	}
	return ret;
}

int hub_event_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	if ((payload == NULL && len > 0U) || len > FRAME_MAX_PAYLOAD) {
		return -EINVAL;
	}
	if ((current_conn == NULL) || (atomic_get(&ack_ccc) == 0)) {
		return 0;
	}
	int ret = frame_buf_put(&ack_buf, type, payload, len);

	if (ret == 0) {
		pack_kick();
	}
	return ret;
}

bool hub_stream_subscribed(void)
{
	return atomic_get(&stream_ccc) != 0;
}

bool hub_event_subscribed(void)
{
	return atomic_get(&ack_ccc) != 0;
}

bool hub_connected(void)
{
	return current_conn != NULL;
}

void hub_file_xfer_mode(bool on)
{
	atomic_set(&file_xfer, on ? 1 : 0);
}

int hub_stream_flush_wait(k_timeout_t timeout)
{
	int64_t deadline = k_uptime_get() +
			   k_ticks_to_ms_floor64(timeout.ticks);

	for (;;) {
		uint16_t used;

		k_mutex_lock(&stream_buf.lock, K_FOREVER);
		used = stream_buf.used;
		k_mutex_unlock(&stream_buf.lock);
		if (used == 0U && stream_pend_len == 0U) {
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -EAGAIN;
		}
		k_msleep(5);
	}
}

bool hub_led_override_get(uint8_t *mask)
{
	atomic_val_t v = atomic_get(&led_override);

	if (v < 0) {
		return false;
	}
	*mask = (uint8_t)v;
	return true;
}

void hub_led_override_set(uint8_t mask)
{
	atomic_set(&led_override,
		   (mask > 0x03U) ? -1 : (atomic_val_t)mask);
}

/* ---- 指令分发（bridge 调用） ---- */

int sensor_hub_cmd_fetch(struct hub_cmd_msg *msg, k_timeout_t timeout)
{
	return k_msgq_get(&cmd_msgq, msg, timeout);
}

#else /* !CONFIG_APP_SENSOR_HUB */

int sensor_hub_init(hub_cmd_handler_t handler)
{
	ARG_UNUSED(handler);
	return 0;
}
void sensor_hub_shutdown(void) {}
int hub_stream_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	ARG_UNUSED(type); ARG_UNUSED(payload); ARG_UNUSED(len);
	return 0;
}
int hub_event_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	ARG_UNUSED(type); ARG_UNUSED(payload); ARG_UNUSED(len);
	return 0;
}
bool hub_stream_subscribed(void) { return false; }
bool hub_event_subscribed(void) { return false; }
bool hub_connected(void) { return false; }
void hub_file_xfer_mode(bool on) { ARG_UNUSED(on); }
int hub_stream_flush_wait(k_timeout_t timeout)
{
	ARG_UNUSED(timeout);
	return 0;
}
bool hub_led_override_get(uint8_t *mask)
{
	ARG_UNUSED(mask);
	return false;
}
void hub_led_override_set(uint8_t mask) { ARG_UNUSED(mask); }
void hub_activity_kick(void) {}

#endif /* CONFIG_APP_SENSOR_HUB */
