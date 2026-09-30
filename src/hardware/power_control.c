/**
 * @file power_control.c
 * @brief 电源控制实现：nPM1300 寄存器级负载开关管理 + 模拟域 GPIO
 *
 * 三路电源域：
 *  - LDSW1 -> SENS   传感器域（VDD_SENS_3V0）
 *  - LDSW2 -> STORE  存储域（VDD_STORE_3V0_SW）
 *  - GPIO  -> ANALOG 模拟域（VDD_ANA_3V0，TPS7A2030 EN）
 *
 * 所有对 PMIC 的操作走 mfd_npm13xx 驱动的寄存器读写接口；
 * 关键时序（软启动、放电策略）沿用 petChoker EVT0 硬件已验证配置。
 * 本板差异：ANA_EN 由 EVT0 的 P1.05 改为 P1.06（设备树获取）。
 */
#include "power_control.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/** zephyr_user 节点：devicetree 中声明的 ana-enable GPIO（P1.06） */
#define USER_NODE DT_PATH(zephyr_user)

/* nPM1300 寄存器基地址 */
#define BASE_VBUS 0x02U /* VBUS 检测模块 */
#define BASE_LDSW 0x08U /* 负载开关模块 */
#define BASE_SHIP 0x0BU /* Ship 模式模块 */

/* VBUS 模块内偏移 */
#define OFF_VBUS_STATUS 0x07U /* VBUS 状态寄存器 */

/* 负载开关（LDSW）模块内偏移 */
#define OFF_LDSW1_SET    0x00U /* LDSW1 打开触发 */
#define OFF_LDSW1_CLR    0x01U /* LDSW1 关闭触发 */
#define OFF_LDSW2_SET    0x02U /* LDSW2 打开触发 */
#define OFF_LDSW2_CLR    0x03U /* LDSW2 关闭触发 */
#define OFF_LDSW_STATUS  0x04U /* 两路 LDSW 开关状态 */
#define OFF_LDSW1_GPISEL 0x05U /* LDSW1 GPIO 选择模式 */
#define OFF_LDSW2_GPISEL 0x06U /* LDSW2 GPIO 选择模式 */
#define OFF_LDSW_CONFIG  0x07U /* LDSW 全局配置（软启动/放电） */
#define OFF_LDSW1_LDOSEL 0x08U /* LDSW1 LDO 模式选择 */
#define OFF_LDSW2_LDOSEL 0x09U /* LDSW2 LDO 模式选择 */

/* Ship 模式模块内偏移 */
#define OFF_TASK_ENTER_SHIP 0x02U /* 进入 Ship 模式任务触发寄存器 */

/** 任务触发寄存器统一写入值（写 1 触发任务） */
#define TASK_TRIGGER 0x01U
/** VBUS 状态寄存器中"VBUS 在位"的位掩码 */
#define VBUS_PRESENT_MASK 0x01U
/** LDSW 状态寄存器中"开关导通"的位掩码（bit0=LDSW1, bit2=LDSW2） */
#define LDSW_ON_MASK 0x05U

/* EVT0 硬件验证通过的 LDSW 配置：LS1 10mA 软启动、LS2 20mA、放电使能 */
#define LDSW_CONFIG_SAFE 0xD0U

/** 模拟电源域使能 GPIO（设备树获取，P1.06） */
static const struct gpio_dt_spec ana_enable =
	GPIO_DT_SPEC_GET(USER_NODE, ana_enable_gpios);
/** PMIC 设备指针，init 时保存 */
static const struct device *pmic_dev;

/** @brief 写 PMIC 寄存器（封装基地址+偏移） */
static int reg_write(uint8_t base, uint8_t offset, uint8_t value)
{
	return mfd_npm13xx_reg_write(pmic_dev, base, offset, value);
}

/** @brief 读 PMIC 寄存器（封装基地址+偏移） */
static int reg_read(uint8_t base, uint8_t offset, uint8_t *value)
{
	return mfd_npm13xx_reg_read(pmic_dev, base, offset, value);
}

/**
 * @brief 关闭所有电源轨
 *
 * 顺序：先拉低模拟域 GPIO，再依次关 LDSW1/LDSW2。
 * 记录首个发生的错误并继续执行后续关闭动作（尽力而为）。
 *
 * @return 0 全部成功；否则返回首个错误码
 */
int power_domains_all_off(void)
{
	int first_error = 0;
	int ret;

	ret = gpio_pin_set_dt(&ana_enable, 0);
	if (ret != 0) {
		first_error = ret;
	}

	ret = reg_write(BASE_LDSW, OFF_LDSW1_CLR, TASK_TRIGGER);
	if ((ret != 0) && (first_error == 0)) {
		first_error = ret;
	}

	ret = reg_write(BASE_LDSW, OFF_LDSW2_CLR, TASK_TRIGGER);
	if ((ret != 0) && (first_error == 0)) {
		first_error = ret;
	}

	return first_error;
}

/**
 * @brief 回读校验所有电源轨确实已关闭
 *
 * 检查两处：LDSW 状态寄存器的导通位、模拟域 GPIO 实际电平。
 *
 * @return 0 全部关闭；-EIO 有电源轨仍在导通；其他负值寄存器读失败
 */
int power_domains_verify_all_off(void)
{
	uint8_t status;
	int ret = reg_read(BASE_LDSW, OFF_LDSW_STATUS, &status);

	if (ret != 0) {
		return ret;
	}

	/* 任一负载开关仍导通 -> 校验失败 */
	if ((status & LDSW_ON_MASK) != 0U) {
		printk("POWER_VERIFY,all_off=0,ldsw=0x%02x\n", status);
		return -EIO;
	}

	/* 模拟域 GPIO 仍为高 -> 校验失败 */
	if (gpio_pin_get_raw(ana_enable.port, ana_enable.pin) != 0) {
		printk("POWER_VERIFY,all_off=0,ana_en=1\n");
		return -EIO;
	}

	return 0;
}

/**
 * @brief 查询当前电源状态（负载开关寄存器 + 模拟域 GPIO 电平）
 *
 * @param ldsw_status 输出 LDSW 状态原始值
 * @param analog_enabled 输出模拟域是否使能
 * @return 0 成功；-EINVAL 参数为空；其他负值失败
 */
int power_control_get_state(uint8_t *ldsw_status, bool *analog_enabled)
{
	int ret;
	int ana_raw;

	if ((ldsw_status == NULL) || (analog_enabled == NULL)) {
		return -EINVAL;
	}

	ret = reg_read(BASE_LDSW, OFF_LDSW_STATUS, ldsw_status);
	if (ret != 0) {
		return ret;
	}

	ana_raw = gpio_pin_get_raw(ana_enable.port, ana_enable.pin);
	if (ana_raw < 0) {
		return ana_raw;
	}

	*analog_enabled = ana_raw != 0;
	return 0;
}

/**
 * @brief 打开/关闭指定电源域
 *
 * @param domain 电源域（SENS/STORE/ANALOG）
 * @param enable true 打开，false 关闭
 * @return 0 成功；-EINVAL 域非法；其他负值寄存器/GPIO 操作失败
 */
int power_domain_set(enum power_domain domain, bool enable)
{
	switch (domain) {
	case POWER_DOMAIN_SENS:
		/* 传感器域走 LDSW1 */
		return reg_write(BASE_LDSW,
				 enable ? OFF_LDSW1_SET : OFF_LDSW1_CLR,
				 TASK_TRIGGER);
	case POWER_DOMAIN_STORE:
		/* 存储域走 LDSW2 */
		return reg_write(BASE_LDSW,
				 enable ? OFF_LDSW2_SET : OFF_LDSW2_CLR,
				 TASK_TRIGGER);
	case POWER_DOMAIN_ANALOG:
		/* 模拟域由普通 GPIO 控制 */
		return gpio_pin_set_dt(&ana_enable, enable ? 1 : 0);
	default:
		return -EINVAL;
	}
}

/**
 * @brief 查询 VBUS（USB）是否在位
 *
 * @param present 输出 true=USB 已插入
 * @return 0 成功；-EINVAL 参数为空；其他负值寄存器读失败
 */
int power_control_vbus_present(bool *present)
{
	uint8_t status;
	int ret;

	if (present == NULL) {
		return -EINVAL;
	}

	ret = reg_read(BASE_VBUS, OFF_VBUS_STATUS, &status);
	if (ret == 0) {
		*present = (status & VBUS_PRESENT_MASK) != 0U;
	}

	return ret;
}

/**
 * @brief 进入 Ship 模式（切断 VSYS，整机断电）
 *
 * 安全措施：间隔 150ms 两次采样 VBUS，都确认不在位才执行，
 * 避免刚拔掉 USB 时 VBUS 还未放电完毕导致误判。
 *
 * @return 0 已触发进 Ship；-EBUSY VBUS 在位；其他负值失败
 */
int power_control_enter_ship(void)
{
	bool present;
	int ret;

	/* 要求两次采样均不在位，给刚移除的 VBUS 留出放电时间 */
	ret = power_control_vbus_present(&present);
	if ((ret != 0) || present) {
		return (ret != 0) ? ret : -EBUSY;
	}

	k_sleep(K_MSEC(150));
	ret = power_control_vbus_present(&present);
	if ((ret != 0) || present) {
		return (ret != 0) ? ret : -EBUSY;
	}

	/* 写任务触发寄存器，PMIC 切断 VSYS */
	return reg_write(BASE_SHIP, OFF_TASK_ENTER_SHIP, TASK_TRIGGER);
}

/**
 * @brief 电源控制模块初始化
 *
 * 流程：检查设备就绪 -> 配置模拟域 GPIO 为输出低 -> 关闭全部电源轨 ->
 * 配置 LDSW 为主机控制模式并写入 EVT0 验证过的软启动/放电策略 ->
 * 等待 20ms 后回读校验全关。
 *
 * @param pmic PMIC 设备句柄
 * @return 0 成功；-ENODEV 设备未就绪；其他负值失败
 */
int power_control_init(const struct device *pmic)
{
	int ret;

	if ((pmic == NULL) || !device_is_ready(pmic)) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&ana_enable)) {
		return -ENODEV;
	}

	pmic_dev = pmic;

	/* 模拟域 GPIO 配置为输出、初始低电平（默认关闭） */
	ret = gpio_pin_configure_dt(&ana_enable, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	/* 先关闭全部电源轨，确保初始状态干净 */
	ret = power_domains_all_off();
	if (ret != 0) {
		return ret;
	}

	/* 设为主机控制、负载开关模式，随后写入 EVT0 验证的软启动/放电策略 */
	ret = reg_write(BASE_LDSW, OFF_LDSW1_GPISEL, 0U);
	if (ret == 0) {
		ret = reg_write(BASE_LDSW, OFF_LDSW2_GPISEL, 0U);
	}
	if (ret == 0) {
		ret = reg_write(BASE_LDSW, OFF_LDSW1_LDOSEL, 0U);
	}
	if (ret == 0) {
		ret = reg_write(BASE_LDSW, OFF_LDSW2_LDOSEL, 0U);
	}
	if (ret == 0) {
		ret = reg_write(BASE_LDSW, OFF_LDSW_CONFIG, LDSW_CONFIG_SAFE);
	}
	if (ret != 0) {
		return ret;
	}

	/* 等配置生效后校验最终状态 */
	k_sleep(K_MSEC(20));
	return power_domains_verify_all_off();
}
