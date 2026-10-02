/**
 * @file pvdf_adc.h
 * @brief PVDF 压电信号链采集：U8 OPA4391 调理 → nRF54L15 SAADC
 *
 * task-V1.05（docs/外设扩展方案_V1.05.md §4）：
 *
 * 硬件链路（《元器件引脚及功能模块对应表》§一.7 pvdf 页）：
 *   H2 PVDF 膜 ──R12 47k──> U8-CH2 电荷放大（R18 250MΩ//C34 4.7nF 反馈）
 *       → CHARGEOUT ──C32 2.2uF──> CH4 高通增益级（R16 402k, 增益≈-4）
 *       → R17 22k → HEART_AA → R15 1k → **ADC_HEART = P1.05/AIN1**
 *   CHARGEOUT ──R19/R20 低通──> CH3 缓冲 → R13 1k
 *       → **ADC_RAW = P1.13/AIN6**
 *   R9/R10 分压 → CH1 缓冲 → **VBIAS（虚地，≈VDD_ANA/2）**
 *       → R14 1k → **ADC_REF = P1.14/AIN7**
 *
 * 电源：U8 由 VDD_ANA_3V0（U10 TPS7A2030，EN=ANA_EN=P1.06）供电，
 *   复位后默认关断（R25 100k 下拉）。读取前必须
 *   power_domain_set(POWER_DOMAIN_ANALOG, true)，模块内部处理。
 *
 * ADC 配置（app.overlay &adc）：
 *   gain=1/6 + 内部 0.6V 基准 → 满量程 3.6V，覆盖 0~3.0V 模拟摆幅；
 *   12bit → 1LSB ≈ 0.879mV。
 *
 * 降级策略：
 *   ANALOG 域上电失败（PMIC/GPIO 故障）→ 模块标记 PVD_ADC_DEGRADED，
 *   读取返回 -EIO；ADC 驱动未 ready → -ENODEV；均不阻断系统运行。
 *   PVDF 膜未接（H2 悬空）不是可软件检测的状态——ADC 仍会读到 VBIAS
 *   附近的静态值，由上层应用按信号幅度判活。
 */
#ifndef PVDF_ADC_H
#define PVDF_ADC_H

#include <stdbool.h>
#include <stdint.h>

/** PVDF 模块状态 */
enum pvdf_adc_state {
	PVDF_ADC_UNKNOWN = 0, /**< 尚未初始化 */
	PVDF_ADC_READY,       /**< 可采集 */
	PVDF_ADC_DEGRADED,    /**< 上电/初始化失败，待重试 */
};

/** 一帧 PVDF 信号链采样（单位 mV，int32 保留换算余量） */
struct pvdf_sample {
	int32_t heart_mv; /**< ADC_HEART：心率信号（高通增益级输出，含 VBIAS 偏置） */
	int32_t raw_mv;   /**< ADC_RAW：低通原始信号（电荷放大直接缓冲） */
	int32_t ref_mv;   /**< ADC_REF：VBIAS 虚地实测（链路自检基准，≈1500mV） */
};

/**
 * @brief PVDF 信号链自检（开机一次性，幂等）
 *
 * 流程：ANALOG 域上电 → ADC 驱动初始化 → 连续采 4 帧：
 *   - ADC_REF 必须在 1200~1800mV（VBIAS=3.0/2 容差 ±20%），
 *     偏离说明运放未供电/偏置链断裂；
 *   - 三路均在 0~3600mV 量程内。
 * 失败打印 FAIL 不阻断启动（与其他 bring-up 同策略）。
 *
 * @return 0 PASS；负值 失败（上电/驱动/量程）
 */
int pvdf_bringup_test(void);

/**
 * @brief 采集一帧 PVDF 信号（HEART/RAW/REF 三路）
 *
 * 首次调用自动完成 ANALOG 上电 + ADC 初始化（懒加载，
 * 非开机自检路径也可独立工作）。DEGRADED 状态下每次调用
 * 尝试重新初始化一次（简单恢复策略）。
 *
 * @param out 输出帧（mV）
 * @return 0 成功；-EINVAL 参数空；-ENODEV ADC 未就绪；-EIO 上电/转换失败
 */
int pvdf_read(struct pvdf_sample *out);

/**
 * @brief 当前模块状态（快查，无 IO）
 */
enum pvdf_adc_state pvdf_adc_get_state(void);

#endif /* PVDF_ADC_H */
