/**
 * @file imu_stream.h
 * @brief IMU（U4 LSM6DSV16XTR）六轴数据流：驱动初始化 + 采样 + 环形缓冲
 *
 * task-V1.03 第二步（docs/IMU采集测试方案.md §4）：
 *
 * 架构：
 *   [U4 --INT1/P0.00--> drdy 触发] -> sensor_sample_fetch -> k_msgq 环形缓冲
 *   -> ble_imu_service 每 50ms 打包 notify（seq + cnt + N×12B 帧）
 *   触发建立失败时自动降级为 33ms 定时轮询（自制板 INT 走线异常的兜底）。
 *
 * 电源时序（关键）：
 *   U4 由 VDD_SENS_3V0（nPM1300 LDSW1）供电，复位后默认关闭。
 *   驱动节点在 overlay 中标记 zephyr,deferred-init，开机不自动初始化；
 *   本模块 start() 先上电再 device_init()，避免无电状态下 whoami 失败。
 *
 * 帧格式（12 字节，小端）：
 *   ax/ay/az: int16，单位 mg（±8g 量程 → ±8000，无裁剪）
 *   gx/gy/gz: int16，单位 dps×10（±2000dps → ±20000，无裁剪）
 */
#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 一帧六轴数据（12 字节） */
struct imu_frame {
	int16_t ax_mg;
	int16_t ay_mg;
	int16_t az_mg;
	int16_t gx_dps10;
	int16_t gy_dps10;
	int16_t gz_dps10;
};

/** 采样来源（状态字节 bit1） */
enum imu_stream_src {
	IMU_STREAM_SRC_NONE = 0,     /**< 未启动 */
	IMU_STREAM_SRC_TRIGGER = 1,  /**< INT1 drdy 触发（正常路径） */
	IMU_STREAM_SRC_POLLING = 2,  /**< 定时轮询（触发失败的降级路径） */
};

/**
 * @brief 启动数据流（幂等）
 *
 * 流程：VDD_SENS_3V0 上电 -> device_init(U4)（首次）-> drdy 触发建立
 * （失败则降级 33ms 定时轮询）。重复调用仅复位统计计数。
 *
 * @return 0 成功（trigger 或 polling 任一模式）；
 *         负值失败（上电失败 / 驱动初始化失败 / 总线无响应）
 */
int imu_stream_start(void);

/**
 * @brief 停止数据流（幂等）：撤销触发 + 清空缓冲
 *
 * 注意：VDD_SENS_3V0 保持上电（同总线还有柔性板器件，统一由
 * 电源策略管理，本模块不越权下电）。
 */
void imu_stream_stop(void);

/** @brief 数据流是否在跑 */
bool imu_stream_active(void);

/** @brief 采样来源（NONE / TRIGGER / POLLING） */
enum imu_stream_src imu_stream_source(void);

/**
 * @brief 批量取帧（BLE 服务打包用，K_NO_WAIT）
 *
 * @param dst 输出数组
 * @param max 最大取多少帧
 * @return 实际取出的帧数（0 = 缓冲空）
 */
size_t imu_stream_drain(struct imu_frame *dst, size_t max);

#endif /* IMU_STREAM_H */
