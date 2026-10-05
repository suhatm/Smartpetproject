/**
 * @file batt_gauge.h
 * @brief 电池电量模块（task-V1.06，协议 0x08 BATTERY 帧数据源）
 *
 * percent 算法（协议 V0.4/V0.5 定稿）：nPM1300 库仑计 SOC ——
 * NCS nrf_fuel_gauge 算法库 + 电池模型（battery_model.inc，
 * 未标定前沿用 NCS 样例模型，实配电芯标定后替换）。
 * 库仑计初始化失败时降级为电压线性映射（3300~4200 mV）兜底，
 * 帧格式不变（协议允许）。
 */
#ifndef BATT_GAUGE_H
#define BATT_GAUGE_H

#include <stdbool.h>
#include <stdint.h>

struct batt_reading {
	uint16_t vbat_mv;  /**< 电池电压 mV（nPM1300 实测） */
	uint8_t percent;   /**< SOC 0~100 */
	bool charging;     /**< flags.bit0：充电中 */
	bool full;         /**< flags.bit1：已充满 */
	bool low;          /**< flags.bit2：低电（percent ≤ 15） */
};

/**
 * @brief 初始化电量计（含 charger 设备检查 + nrf_fuel_gauge 初始化）
 *
 * @return 0 成功（库仑计模式）；1 降级（电压线性映射）；负值失败
 */
int batt_gauge_init(void);

/**
 * @brief 周期更新（建议 2s 一拍，充电期间可加密到 1s）
 *
 * 内部完成：charger 采样 -> nrf_fuel_gauge_process -> 缓存最新读数。
 * 可在任意线程周期调用（I2C 约数 ms）。
 *
 * @param vbus_present 当前 VBUS 状态（库仑计充电状态机用）
 * @return 0 成功；负值采样失败（读数保持上次值）
 */
int batt_gauge_update(bool vbus_present);

/**
 * @brief 取最新读数（非阻塞，副本返回）
 */
void batt_gauge_get(struct batt_reading *out);

#endif /* BATT_GAUGE_H */
