/**
 * @file ble_led_service.h
 * @brief BLE 从机 LED 测试服务：手机 nRF Connect App 控制板载 LED0/LED1
 *
 * 详见 docs/蓝牙驱动测试方案.md（task-V1.02）。
 *
 * 服务设计（对齐方案 §3.2）：
 *   Service    e5a00001-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   特征值     e5a00002-1e5c-4b8f-9a2d-6c0f7e8d9a0b（Read + Write 带响应）
 *   协议       1 字节位图：bit0=LED0(红) bit1=LED1(蓝)，高 6 位保留忽略
 *
 * 并发模型（方案 §3.4）：
 *   GATT 写回调（BT 线程）只做原子变量写 + printk；
 *   实际点灯由主循环 update_led_pattern() 统一仲裁执行，
 *   status_led 模块保持单一 I2C 写入点，零改动。
 */
#ifndef BLE_LED_SERVICE_H
#define BLE_LED_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 蓝牙从机初始化：使能协议栈，就绪后自动开始广播（异步）
 *
 * GATT 服务本体经 BT_GATT_SERVICE_DEFINE() 静态注册，无需显式调用。
 * 参考流程（PDF §5.1）：bt_enable -> 服务已注册 -> advertising_start。
 *
 * @return 0 已提交（结果经 bt_ready 回调异步报告）；
 *         负值 bt_enable 立即失败（调用方仅告警，不进 FAULT，方案 §7 风险 6）
 */
int ble_led_test_init(void);

/**
 * @brief 读取 BLE override 显示位图（原子，主循环 20ms 周期调用）
 *
 * @param mask 输出：bit0=红 bit1=蓝
 * @return true override 激活（已连接且 App 写入过有效值）；
 *         false 未激活（显示权交还原有 LED 状态机）
 */
bool ble_led_override_get(uint8_t *mask);

/**
 * @brief 关机前调用：断开蓝牙连接并停止广播（方案 §3.3）
 *
 * 同时清除 override，之后主循环不再出现 BLE 显示层。
 */
void ble_led_test_shutdown(void);

#endif /* BLE_LED_SERVICE_H */
