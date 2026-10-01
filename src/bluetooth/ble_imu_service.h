/**
 * @file ble_imu_service.h
 * @brief BLE 从机 IMU 数据流服务：六轴数据经 GATT Notify 推到手机 App
 *
 * task-V1.03 第二步（docs/IMU采集测试方案.md §4）。
 *
 * 服务设计（对齐方案，沿用 e5a000xx UUID 风格）：
 *   Service  e5a00010-1e5c-4b8f-9a2d-6c0f7e8d9a0b
 *   数据流   e5a00011-...（Read + Notify，帧流，CCC 订阅）
 *   控制点   e5a00012-...（Read + Write，启停采集）
 *
 * 控制点协议（1 字节，Write）：
 *   0x00 = 停止采集
 *   0x01 = 启动采集（30Hz，±8g/2000dps；上报经 e5a00011）
 *
 * 数据包格式（Notify，小端）：
 *   [seq:1B 包序号 wrap][cnt:1B 帧数][cnt × 12B 帧]
 *   帧 = ax,ay,az(mg int16) + gx,gy,gz(dps×10 int16)
 *   帧数按协商 MTU 自适应（MTU 247 时最多 20 帧/包）
 *
 * 状态字节（Read e5a00011 返回 3 字节）：
 *   [active:1=采集中][src:1=trigger/2=polling][seq 当前包序号]
 *
 * 使用（nRF Connect for Mobile）：
 *   连接 SmartPet -> 展开 e5a00010 -> 订阅 e5a00011（Enable Notify）
 *   -> 写 01 到 e5a00012 -> e5a00011 开始持续收到数据包
 */
#ifndef BLE_IMU_SERVICE_H
#define BLE_IMU_SERVICE_H

#include <stdint.h>

/**
 * @brief IMU 数据流服务就绪检查（GATT 静态注册，无需显式 init）
 *
 * 本函数仅做编译期占位说明：服务经 BT_GATT_SERVICE_DEFINE 链接期注册，
 * 广播/连接管理复用 ble_led_service（同一个 BT_CONN_CB 体系）。
 *
 * @return 0 恒定（保留返回值便于未来扩展）
 */
int ble_imu_service_init(void);

#endif /* BLE_IMU_SERVICE_H */
