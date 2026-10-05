/**
 * @file sensor_hub_bridge.h
 * @brief Sensor Hub 业务桥接（task-V1.06）：指令分发 + 传感器采集 + 状态上报
 *
 * 职责（协议 V0.5 全部指令/帧/事件）：
 *   - 指令：LED_SET/PWR_SET/SENSOR_EN/RATE_SET/QVAR_CFG/REPROBE/REC_CTRL/
 *     QVAR_THR_SET/REC_LIST/REC_READ/REC_DELETE/GET_BATTERY/SD_TEST/
 *     GET_STATUS/GET_VERSION/FACTORY_PING；
 *   - 数据帧：IMU_U4(0x01)/BODY_IMU_U1(0x02)/QVAR(0x03)/PVDF(0x04)/
 *     TEMP(0x05)/MIC(0x06 经 recorder)/BATTERY(0x08)/MODULE_STATUS(0x10)；
 *   - 事件：MODULE_STATE_CHANGED/WDT_RESET/LOW_BATTERY/QVAR_THR_CROSSED/
 *     REC_STATE/REC_FILE_DONE/SD_TEST_DONE(0x07 扩展)；
 *   - QVAR-B 硬件约束（原理图核对 P0-1）：FPC 未插（U1 ABSENT）时
 *     QVAR-B 判 DEGRADED，valid 位清 0。
 */
#ifndef SENSOR_HUB_BRIDGE_H
#define SENSOR_HUB_BRIDGE_H

#include <stdint.h>

/**
 * @brief 初始化桥接（注册指令回调、启动采集调度）
 *
 * @return 0 成功；负值失败
 */
int sensor_hub_bridge_init(void);

/**
 * @brief 由 ble_sensor_hub 在收到指令入队后调用（唤醒指令处理工作项）
 */
void sensor_hub_bridge_cmd_kick(void);

/**
 * @brief 上报看门狗复位原因（main 在清除 RESETREAS 前传入）
 *
 * 桥接层缓存，待上位机订阅应答特征后经 EVENT 0x02 上报。
 */
void sensor_hub_bridge_report_wdt_reset(uint32_t reset_cause);

#endif /* SENSOR_HUB_BRIDGE_H */
