/**
 * @file qvar_sensor.h
 * @brief QVAR 人体准静态感应：QVAR-A（柔性板 U1）/ QVAR-B（主板 U4）采集
 *
 * task-V1.05（docs/外设扩展方案_V1.05.md §3）：
 *
 * 硬件定位（《元器件引脚及功能模块对应表》§一.8/§一.11）：
 *   LSM6DSV16X 片内 AH/QVAR 模拟前端，每颗芯片提供 **1 路差分 QVAR 通道**
 *   （引脚 pin2=SDx/AH1/Qvar1 与 pin3=SCx/AH2/Qvar2 为差分对，不占 MCU 引脚）：
 *
 *   ┌─ QVAR-A：柔性板 U1 → 电极 AQ1/AQ2（皮肤接触，10MΩ 中点偏置 + 499Ω
 *   │           + 110pF + ESD）→ AQVAR1/AQVAR2 → U1.Qvar1/Qvar2
 *   └─ QVAR-B：主板 U4   → 电极 BQ1/BQ2（经 FPC 排线 → ESD → 499Ω + 110pF）
 *               → BQVAR1/BQVAR2 → U4.Qvar1/Qvar2
 *
 * 寄存器依据（NCS v3.4.0 ST pid 驱动 lsm6dsv16x_reg.h/.c 实锤）：
 *   CTRL7 (0x16) bit7 ah_qvar_en    AH/QVAR 链使能
 *   CTRL7 (0x16) bits[5:4] ah_qvar_c_zin  输入阻抗（2400/730/300/255 MΩ）
 *   CTRL7 (0x16) bit6 int2_drdy_ah_qvar   QVAR 数据就绪路由到 INT2（预留）
 *   STATUS_REG (0x1E) bit3 ah_qvarda      QVAR 数据就绪
 *   AH_QVAR_OUT_L/H (0x3A/0x3B)           16bit QVAR 原始输出（单差分值）
 *
 * 关键时序（数据手册强制）：
 *   置 ah_qvar_en=1 之前，加速度计与陀螺仪必须先配置为 power-down
 *   （ODR=0），使能后再恢复 ODR。本模块在 qvar_channel_enable() 内部
 *   自动完成 保存 ODR→关→开 QVAR→恢复 的完整序列。
 *
 * 降级策略：
 *   通道所在的 IMU 不在位（柔性板 FPC 未插 / U4 故障）时，该通道标记
 *   QVAR_CH_ABSENT，读取返回 -ENODEV；另一通道不受影响（双 IMU 独立
 *   供电同轨但探测独立）。
 *
 * 注意：Zephyr sensor 驱动（st,lsm6dsv16x）不触碰 CTRL7（已 grep 确认），
 *   裸寄存器配置 QVAR 与 Zephyr 驱动的 XL/G 数据通路无冲突。
 */
#ifndef QVAR_SENSOR_H
#define QVAR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/** QVAR 通道标识（一颗 IMU = 一路差分 QVAR） */
enum qvar_channel {
	QVAR_CHANNEL_A = 0, /**< QVAR-A：柔性板 U1（电极 AQ1/AQ2 → AQVAR1/AQVAR2） */
	QVAR_CHANNEL_B = 1, /**< QVAR-B：主板 U4（电极 BQ1/BQ2 → BQVAR1/BQVAR2） */
	QVAR_CHANNEL_NUM
};

/** 通道连接状态（与 body_imu 状态机风格一致） */
enum qvar_ch_state {
	QVAR_CH_UNKNOWN = 0, /**< 尚未探测/配置 */
	QVAR_CH_ACTIVE,      /**< 已使能，可读 */
	QVAR_CH_ABSENT,      /**< 所在 IMU 不在位（FPC 未插 / 器件故障） */
	QVAR_CH_DEGRADED,    /**< 配置成功但近期 IO 失败，待重配置 */
};

/** QVAR 输入阻抗选择（CTRL7.ah_qvar_c_zin，大阻抗匹配皮肤电极） */
enum qvar_zin {
	QVAR_ZIN_2400MOHM = 0, /**< 2.4GΩ（默认，皮肤准静态场景首选） */
	QVAR_ZIN_730MOHM  = 1, /**< 730MΩ */
	QVAR_ZIN_300MOHM  = 2, /**< 300MΩ */
	QVAR_ZIN_255MOHM  = 3, /**< 255MΩ */
};

/** 一帧双通道 QVAR 数据（含各自有效性） */
struct qvar_sample {
	int16_t a_raw;    /**< QVAR-A 原始值（有效时有效） */
	int16_t b_raw;    /**< QVAR-B 原始值 */
	bool a_valid;     /**< A 通道本帧可读 */
	bool b_valid;     /**< B 通道本帧可读 */
};

/**
 * @brief QVAR 模块自检（开机一次性，幂等）
 *
 * 对 A/B 两通道分别执行：所在 IMU presence 探测 → QVAR 链使能
 * （含 XL/G power-down 时序）→ 连读 3 帧验证 ah_qvarda 置位。
 * 某通道所在 IMU 不在位 → 该通道 ABSENT，不影响另一通道结果。
 *
 * @return 0 至少一通道 ACTIVE；1 两通道均 ABSENT；负值 总线故障
 */
int qvar_bringup_test(void);

/**
 * @brief 使能指定 QVAR 通道（幂等）
 *
 * 内部序列：读保存 CTRL1/CTRL2（XL/G ODR）→ ODR=0 → 置 CTRL7
 * ah_qvar_en + 指定 zin → 恢复 ODR。
 *
 * @param ch 通道
 * @param zin 输入阻抗（皮肤电极建议 QVAR_ZIN_2400MOHM）
 * @return 0 成功；-ENODEV 所在 IMU 不在位；其他负值 IO 失败
 */
int qvar_channel_enable(enum qvar_channel ch, enum qvar_zin zin);

/**
 * @brief 强制全序列重配指定通道（task-V1.10 僵尸 ACTIVE 修复）
 *
 * 与 qvar_channel_reprobe() 的区别：不检查软件状态、不短路，无条件
 * 重跑 qvar_channel_enable() 完整序列。用于芯片寄存器被外部复位但
 * 软件状态仍为 ACTIVE 的场景：
 *   - U4/U1 驱动 device_init() 的 lsm6dsv16x_sw_por() 会把 CTRL7
 *     .ah_qvar_en 清零（U4 deferred-init 首次发生在 imu_stream_start，
 *     晚于开机 QVAR 自检——静电 B 恒灰的根因）；
 *   - SENS 域断电重上电（POR）。
 *
 * @param ch 通道
 * @return 0 成功；-ENODEV 所在 IMU 不在位；其他负值 IO 失败
 */
int qvar_channel_reconfigure(enum qvar_channel ch);

/**
 * @brief 读取一帧双通道 QVAR 数据
 *
 * ABSENT 通道不发 I2C，a_valid/b_valid 标记各自可用性。
 * 输出为带符号 16bit 原始值（LSB），物理量换算（nV/电荷）
 * 留待标定阶段按前端增益确定。
 *
 * @param out 输出帧
 * @return 0 至少一通道有效；-ENODEV 两通道均不可用；-EINVAL 参数空
 */
int qvar_read(struct qvar_sample *out);

/**
 * @brief 查询指定通道当前状态（快查，无 IO）
 */
enum qvar_ch_state qvar_channel_state(enum qvar_channel ch);

/**
 * @brief 重探测 + 重配置指定通道（热插拔/降级恢复）
 *
 * @return 0 恢复 ACTIVE；1 仍不在位；负值 总线故障
 */
int qvar_channel_reprobe(enum qvar_channel ch);

#endif /* QVAR_SENSOR_H */
