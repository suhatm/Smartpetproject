/**
 * @file temp_sensor.h
 * @brief TMP112A 温度传感器（柔性板 U2，SENS_I2C @0x48）bring-up 自检
 *
 * 注意：器件经 FPC 在柔性板上，未插排线时探测不到属预期（ABSENT），
 * 与真正的总线故障区分报告。
 */
#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>

/**
 * @brief TMP112 bring-up：上电(SENS 域) -> device_init -> 读温 -> 范围判定
 *
 * @return 0 PASS；1 ABSENT（不在位，非故障）；负值 总线/驱动故障
 */
int temp_bringup_test(void);

/**
 * @brief 读当前温度（毫°C）。SENS 域须已上电且 bringup 通过
 * @return 0 成功；负值 失败（不在位/总线错）
 */
int temp_read_mdeg(int32_t *mdeg);

#endif /* TEMP_SENSOR_H */
