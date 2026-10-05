/**
 * @file body_imu.h
 * @brief 柔性板 U1 LSM6DSV16XTR（felxboard，经 FPC1）驱动与连接状态管理
 *
 * task-V1.05（docs/外设扩展方案_V1.05.md §2）：
 *
 * 硬件定位：
 *   柔性板（felxboard）板载六轴，I2C 地址 0x6A（柔性板 R1 0Ω 到 GND，SA0=0），
 *   与主板 U4@0x6B 共用 SENS_I2C（i2c21，400kHz）。
 *   INT1→P0.01（IMU_BODY_INT1，FPC1 pin8↔13），INT2→P1.11（IMU_BODY_INT2）。
 *   电极 AQVAR1/AQVAR2 为片内 QVAR 前端引脚，不占 MCU 引脚（见 qvar_sensor.c）。
 *
 * 连接状态机（关键——硬件未插 FPC 时系统必须正常运行）：
 *   UNKNOWN →（bring-up 或 read 探测）→ PRESENT / ABSENT
 *   PRESENT →（连续 IO 失败）→ DEGRADED（周期性重探测）
 *   ABSENT →（定时重探测）→ PRESENT（允许带电插排线）
 *   所有状态迁移仅打印日志，绝不向调用方抛致命错误；ABSENT/DEGRADED
 *   状态下 read 直接返回 -ENODEV，不发起 I2C 事务。
 *
 * 电源：VDD_SENS_3V0（nPM1300 LDSW1）经 FPC 供电，与主板传感域同轨；
 *   上电/下电由本模块内部按引用计数与主板 imu_stream 共存（域统一开启）。
 */
#ifndef BODY_IMU_H
#define BODY_IMU_H

#include <stdbool.h>
#include <stdint.h>

/** 柔性板 U1 连接状态 */
enum body_imu_state {
	BODY_IMU_STATE_UNKNOWN = 0, /**< 尚未探测 */
	BODY_IMU_STATE_PRESENT,     /**< 在位（WHO_AM_I=0x70） */
	BODY_IMU_STATE_ABSENT,      /**< 不在位（FPC 未插/排线断） */
	BODY_IMU_STATE_DEGRADED,    /**< 曾在位但近期 IO 失败，等重探测 */
};

/** 一帧六轴数据（与主板 imu_frame 布局一致，单位同源） */
struct body_imu_frame {
	int16_t ax_mg;    /**< accel X，单位 mg（±8g 量程） */
	int16_t ay_mg;
	int16_t az_mg;
	int16_t gx_dps10; /**< gyro X，单位 dps×10（±2000dps 量程） */
	int16_t gy_dps10;
	int16_t gz_dps10;
};

/**
 * @brief 柔性板 U1 连通性自检（开机一次性，幂等）
 *
 * 流程：SENS 域上电 → 0x6A WHO_AM_I（0x0F 期望 0x70）→ deferred
 * 驱动 device_init → 读一帧六轴验证数据通路。
 * ABSENT（FPC 未插）为预期结果，返回 1 而非错误码，与 TMP112 同策略。
 *
 * @return 0 PASS（在位且一帧读取成功）；1 ABSENT（不在位）；
 *         负值 上电/总线故障
 */
int body_imu_bringup_test(void);

/**
 * @brief 读取一帧六轴数据
 *
 * ABSENT/DEGRADED 状态直接返回 -ENODEV，不发 I2C。
 * PRESENT 下 IO 失败累计到阈值自动转入 DEGRADED。
 *
 * @param out 输出帧
 * @return 0 成功；-ENODEV 不在位；其他负值 IO 失败
 */
int body_imu_read_frame(struct body_imu_frame *out);

/**
 * @brief 当前连接状态（快查，无 IO）
 */
enum body_imu_state body_imu_get_state(void);

/**
 * @brief 主动重探测（热插拔场景 / DEGRADED 恢复）
 *
 * ABSENT/DEGRADED 状态下调用：重做一次 WHO_AM_I 探测，
 * 在位则自动 device_init 并转 PRESENT。
 *
 * @return 0 已在位或恢复成功；1 仍不在位；负值 总线故障
 */
int body_imu_reprobe(void);

/**
 * @brief 柔性板 U1 设备句柄（qvar_sensor 模块复用做片内寄存器配置）
 *
 * 仅用于需要直接寄存器访问的兄弟模块；常规调用方用 read_frame。
 * 不在位时返回的设备可能未 ready，调用方需自行 device_is_ready 判断。
 *
 * @return U1 device 指针（恒非空，编译期静态绑定）
 */
const struct device *body_imu_get_device(void);

#endif /* BODY_IMU_H */
