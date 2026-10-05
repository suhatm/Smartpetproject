/**
 * @file imu_sensor.h
 * @brief IMU（U4 LSM6DSV16XTR）采集接口：bring-up 自检 + 后续数据采集
 *
 * task-V1.03 第一步：SENS_I2C（i2c21）连通性自检
 *   - NFC pad（P1.02/P1.03）归还为 GPIO
 *   - VDD_SENS_3V0 上电
 *   - 总线扫描 + U4 WHO_AM_I 校验（期望 0x70）
 */
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>

/**
 * @brief IMU 连通性自检（开机一次性执行）
 *
 * 流程：NFC pad 归还检查 -> SENS 电源域上电 -> I2C 总线扫描
 * （0x08~0x77）-> 对两个候选地址（0x6A/0x6B）读 WHO_AM_I 校验
 * 0x70 -> SENS 电源域下电（保持原有电源行为）。
 *
 * @return 0 至少一个地址 WHO_AM_I 校验通过；-ENODEV 总线无设备或
 *         WHO_AM_I 不符；其他负值见 printk 输出
 */
int imu_bringup_test(void);

#endif /* IMU_SENSOR_H */
