/**
 * @file ble_sensor_hub.h
 * @brief BLE Sensor Hub 服务（task-V1.06 统一通信入口）
 *
 * 对应《宠物环传感器控制_通信协议.docx》V0.5：
 *
 *   GATT 服务  e5a00020-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   ├── e5a00021  数据流   Notify          （TYPE 0x01~0x10 帧）
 *   ├── e5a00022  指令     Write/WriteCmd  （CMD 1B + PARAMS 0~n B）
 *   ├── e5a00023  应答/事件 Notify          （TYPE 0x20 CMD_ACK / 0x21 EVENT）
 *   └── e5a00024  信息     Read            （固件信息 ASCII 字符串）
 *
 * 帧格式：SYNC(0x55,0xAA) + TYPE + SEQ + LEN + PAYLOAD + CRC8(0x07/0x00/非反射)。
 * 本模块负责：GATT 注册、连接管理、广播、帧封装、MTU 感知组包、
 * CCC 订阅跟踪、僵尸连接看门狗。业务指令解析在 sensor_hub_bridge。
 */
#ifndef BLE_SENSOR_HUB_H
#define BLE_SENSOR_HUB_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

/** 指令最大长度（CMD+PARAMS） */
#define HUB_CMD_MAX_LEN 33U

/** 指令消息（GATT 写 -> msgq -> bridge 消费） */
struct hub_cmd_msg {
	uint8_t req_seq;           /**< 设备端接收计数，ACK 回显 */
	uint8_t len;               /**< data 有效长度 */
	uint8_t data[HUB_CMD_MAX_LEN]; /**< data[0]=CMD，其后 PARAMS */
};

/**
 * @brief bridge 从指令队列取一条指令
 *
 * @param msg     输出
 * @param timeout 等待超时（bridge 用 K_NO_WAIT 轮询）
 * @return 0 取到；负值空
 */
int sensor_hub_cmd_fetch(struct hub_cmd_msg *msg, k_timeout_t timeout);

/** 帧类型（协议 §5） */
enum hub_frame_type {
	HUB_TYPE_IMU_U4        = 0x01,
	HUB_TYPE_BODY_IMU_U1   = 0x02,
	HUB_TYPE_QVAR          = 0x03,
	HUB_TYPE_PVDF          = 0x04,
	HUB_TYPE_TEMP          = 0x05,
	HUB_TYPE_MIC           = 0x06,
	HUB_TYPE_AUDIO_FILE    = 0x07,
	HUB_TYPE_BATTERY       = 0x08,
	HUB_TYPE_MODULE_STATUS = 0x10,
	HUB_TYPE_CMD_ACK       = 0x20,
	HUB_TYPE_EVENT         = 0x21,
};

/** 事件 ID（协议 §8.2） */
enum hub_event_id {
	HUB_EVENT_MODULE_STATE_CHANGED = 0x01,
	HUB_EVENT_WDT_RESET            = 0x02,
	HUB_EVENT_LOW_BATTERY          = 0x03,
	HUB_EVENT_QVAR_THR_CROSSED     = 0x04,
	HUB_EVENT_REC_STATE            = 0x05,
	HUB_EVENT_REC_FILE_DONE        = 0x06,
};

/**
 * @brief 指令回调原型（bridge 实现）
 *
 * @param cmd     CMD 字节
 * @param params  参数区（可为 NULL）
 * @param len     参数长度
 * @param req_seq 设备端接收计数（0~255 循环），应答时原样回显
 */
typedef void (*hub_cmd_handler_t)(uint8_t cmd, const uint8_t *params,
				  uint8_t len, uint8_t req_seq);

/**
 * @brief 初始化 Sensor Hub（异步使能蓝牙，就绪后自动广播）
 *
 * @param handler 指令处理回调（NULL 则收到指令一律应答 -12）
 * @return 0 成功；负值失败
 */
int sensor_hub_init(hub_cmd_handler_t handler);

/**
 * @brief 关机前调用：停止广播并断开连接（与关机流程互斥，不再复活广播）
 */
void sensor_hub_shutdown(void);

/**
 * @brief 发送数据流帧（e5a00021 Notify，TYPE 0x01~0x10）
 *
 * 线程安全；未订阅/未连接时帧被丢弃（返回 0，不视为错误）。
 *
 * @return 0 已入包/已丢弃；负值参数错误
 */
int hub_stream_frame(uint8_t type, const uint8_t *payload, uint8_t len);

/**
 * @brief 发送应答/事件帧（e5a00023 Notify，TYPE 0x20/0x21）
 *
 * 语义同 hub_stream_frame，走应答通道。
 */
int hub_event_frame(uint8_t type, const uint8_t *payload, uint8_t len);

/** @brief 数据流特征是否被订阅 */
bool hub_stream_subscribed(void);

/** @brief 应答特征是否被订阅 */
bool hub_event_subscribed(void);

/** @brief 是否有活动连接 */
bool hub_connected(void);

/**
 * @brief 文件传输模式开关（REC_READ 期间置 true）
 *
 * true 时打包不再等 50ms 周期，缓冲区非空立即组包发送（尽快发满 MTU）。
 */
void hub_file_xfer_mode(bool on);

/**
 * @brief 等待数据流通道排空（帧缓冲与挂起包均已交给控制器）
 *
 * REC_READ 发 DONE 事件前调用，保证"事件到=数据已全部发出"，
 * 消除 0x21 数据 / 0x23 事件双通道竞态（上板实测尾部缺帧根因）。
 *
 * @param timeout 最长等待
 * @return 0 已排空；-EAGAIN 超时未排空
 */
int hub_stream_flush_wait(k_timeout_t timeout);

/**
 * @brief LED override 查询（main 循环 LED 仲裁第 4 级用）
 *
 * @param mask 输出显示位图（bit0=LED0 bit1=LED1）
 * @return true=override 激活；false=未激活（归还状态机）
 */
bool hub_led_override_get(uint8_t *mask);

/**
 * @brief 设置 LED override（bridge 处理 LED_SET 时调用）
 *
 * @param mask 0~3；传 0xFF 清除 override
 */
void hub_led_override_set(uint8_t mask);

/**
 * @brief 喂僵尸连接看门狗（任何读/写/订阅活动都会自动调用，业务无需管）
 */
void hub_activity_kick(void);

#endif /* BLE_SENSOR_HUB_H */
