/**
 * @file power_control.h
 * @brief 电源控制接口：nPM1300 寄存器级负载开关管理 + 模拟域 GPIO
 *
 * 三路电源域（产品方案 §4）：
 *  - VDD_SENS_3V0      传感器域（nPM1300 LDSW1/LDO1，LSOUT1）
 *  - VDD_STORE_3V0_SW  存储域（nPM1300 LDSW2/LDO2，LSOUT2）
 *  - VDD_ANA_3V0       模拟域（TPS7A2030 LDO，EN=ANA_EN=SoC P1.06）
 */
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/** 可独立控制的电源域 */
enum power_domain {
	POWER_DOMAIN_SENS,   /**< VDD_SENS_3V0（IMU/麦克风/柔性板） */
	POWER_DOMAIN_STORE,  /**< VDD_STORE_3V0_SW（SD NAND） */
	POWER_DOMAIN_ANALOG, /**< VDD_ANA_3V0（PVDF 运放） */
};

/**
 * @brief 打开/关闭指定电源域（域间互不影响）
 *
 * @param domain 电源域
 * @param enable true 打开，false 关闭
 * @return 0 成功；-EINVAL 域非法；其他负值寄存器/GPIO 操作失败
 */
int power_domain_set(enum power_domain domain, bool enable);

/**
 * @brief 关闭所有电源轨（关机序列用）
 *
 * 顺序：ANA_EN 拉低 -> LDSW1 关 -> LDSW2 关。尽力而为，返回首个错误。
 *
 * @return 0 全部成功；否则返回首个错误码
 */
int power_domains_all_off(void);

/**
 * @brief 回读校验所有电源轨确实已关闭
 *
 * 检查 LDSW 状态寄存器导通位与 ANA_EN GPIO 实际电平。
 *
 * @return 0 全部关闭；-EIO 仍有轨导通；其他负值寄存器读失败
 */
int power_domains_verify_all_off(void);

/**
 * @brief 查询当前电源状态
 *
 * @param ldsw_status 输出 LDSW 状态原始值（bit0=LDSW1, bit2=LDSW2）
 * @param analog_enabled 输出模拟域是否使能
 * @return 0 成功；-EINVAL 参数为空；其他负值失败
 */
int power_control_get_state(uint8_t *ldsw_status, bool *analog_enabled);

/**
 * @brief 查询 VBUS（USB）是否在位
 *
 * @param present 输出 true=USB 已插入
 * @return 0 成功；-EINVAL 参数为空；其他负值寄存器读失败
 */
int power_control_vbus_present(bool *present);

/**
 * @brief 进入 Ship 模式（切断 VSYS，整机断电，约 370 nA）
 *
 * 安全措施：间隔 150 ms 两次采样 VBUS，均不在位才执行。
 *
 * @return 0 已触发进 Ship；-EBUSY VBUS 在位；其他负值失败
 */
int power_control_enter_ship(void);

/**
 * @brief 电源控制模块初始化
 *
 * 配置 ANA_EN 为输出低 -> 关全部电源轨 -> LDSW 主机控制模式 +
 * EVT0 已验证的软启动/放电策略 -> 回读校验全关。
 *
 * @param pmic nPM1300 MFD 设备句柄
 * @return 0 成功；-ENODEV 设备未就绪；其他负值失败
 */
int power_control_init(const struct device *pmic);

#endif /* POWER_CONTROL_H */
