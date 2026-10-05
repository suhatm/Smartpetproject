/**
 * @file imu_sensor.c
 * @brief IMU（U4 LSM6DSV16XTR）bring-up 自检：NFC pad 归还 + I2C 扫描 + WHO_AM_I
 *
 * task-V1.03 第一步（连通性验证，docs/IMU采集测试方案.md §3）：
 *
 * 1. NFC pad 归还（PRE_KERNEL_1，必须早于 i2c21 TWIM pinctrl 应用）：
 *    nRF54L15 的 P1.02/P1.03 复位后为 NFC 天线 pad（NFCT.PADCONFIG
 *    复位值 0x1=Enabled，每次复位重新加载）。nRF54L 的归还机制与
 *    nRF52 不同：不是 UICR.NFCPINS（本芯片 UICR 无此寄存器），
 *    而是运行时直接写 NFCT.PADCONFIG=0，立即生效、无需重启。
 *
 * 2. VDD_SENS_3V0 上电（nPM1300 LDSW1），等待 IMU 启动。
 *
 * 3. SENS_I2C（i2c21）总线扫描 + U4 WHO_AM_I（0x0F，期望 0x70，
 *    参考 NCS v3.4.0 lsm6dsv16x_reg.h：LSM6DSV16X_ID=0x70）。
 *    U4 的 I2C 地址由 R6 0Ω 决定（SA0=0 -> 0x6A，SA0=1 -> 0x6B），
 *    bring-up 两个地址都探测。
 *
 * 4. 自检完成后 SENS 电源域下电，保持原有电源行为不变
 *    （后续正式采集功能再统一管理电源）。
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <nrfx.h>

#include "imu_sensor.h"
#include "power_control.h"

/** SENS_I2C 总线设备树节点（app.overlay &i2c21） */
#define SENS_I2C_NODE DT_NODELABEL(i2c21)

/** LSM6DSV16X 寄存器/ID（lsm6dsv16x_reg.h，bring-up 用裸寄存器） */
#define LSM6DSV16X_REG_WHO_AM_I 0x0FU
#define LSM6DSV16X_WHO_AM_I_VAL 0x70U

/** U4 候选地址：R6 0Ω 决定 SA0（0 -> 0x6A，1 -> 0x6B） */
static const uint16_t imu_candidate_addrs[] = { 0x6AU, 0x6BU };

/** 电源域上电后到 IMU 可响应 I2C 的等待时间（含 LDO 稳定 + IMU boot） */
#define SENS_POWER_SETTLE_MS 25U

/** SENS_I2C 总线设备 */
static const struct device *const sens_i2c = DEVICE_DT_GET(SENS_I2C_NODE);

/**
 * @brief NFC pad 归还（P1.02/P1.03 -> GPIO）
 *
 * PRE_KERNEL_1 早期执行，必须早于 i2c21 TWIM 驱动初始化
 * （TWIM pinctrl 会写 PIN_CNF，若 pad 仍被 NFC 占用则引脚不生效）。
 * 幂等：已归还时跳过。
 */
static int imu_nfct_pads_reclaim(void)
{
	if ((NRF_NFCT->PADCONFIG & NFCT_PADCONFIG_ENABLE_Msk) != 0U) {
		NRF_NFCT->PADCONFIG &= ~NFCT_PADCONFIG_ENABLE_Msk;
		if ((NRF_NFCT->PADCONFIG & NFCT_PADCONFIG_ENABLE_Msk) != 0U) {
			return -EIO;
		}
		printk("IMU_TEST,nfc_pads=reclaimed(P1.02/P1.03->GPIO)\n");
	}
	return 0;
}

SYS_INIT(imu_nfct_pads_reclaim, PRE_KERNEL_1, 30);

/**
 * @brief 读指定地址的 WHO_AM_I 并校验是否为 LSM6DSV16X
 *
 * @param addr 7 位 I2C 地址
 * @param id_out 非空时回读的 WHO_AM_I 值
 *
 * @return true 地址有应答且 WHO_AM_I=0x70；false 无应答或 ID 不符
 */
static bool imu_who_am_i_check(uint16_t addr, uint8_t *id_out)
{
	uint8_t reg = LSM6DSV16X_REG_WHO_AM_I;
	uint8_t id = 0U;
	int ret;

	ret = i2c_write_read(sens_i2c, addr, &reg, 1, &id, 1);
	if (ret != 0) {
		/* 无 ACK：该地址无设备 */
		return false;
	}
	if (id_out != NULL) {
		*id_out = id;
	}
	return id == LSM6DSV16X_WHO_AM_I_VAL;
}

int imu_bringup_test(void)
{
	uint8_t found[8];
	uint8_t found_cnt = 0U;
	uint8_t imu_addr = 0U;
	uint8_t imu_id = 0U;
	int ret;
	int failures = 0;

	printk("IMU_TEST,begin,bus=i2c21,sda=P1.02,scl=P1.03,rate=400kHz\n");

	/* 1. 总线设备就绪检查 */
	if (!device_is_ready(sens_i2c)) {
		printk("IMU_TEST,i2c21,not_ready\n");
		return -ENODEV;
	}

	/* 2. NFC pad 归还状态确认（PRE_KERNEL 已做，这里复核） */
	if ((NRF_NFCT->PADCONFIG & NFCT_PADCONFIG_ENABLE_Msk) != 0U) {
		printk("IMU_TEST,nfc_pads,still_enabled=FAIL\n");
		failures++;
	}

	/* 3. SENS 电源域上电 */
	ret = power_domain_set(POWER_DOMAIN_SENS, true);
	if (ret != 0) {
		printk("IMU_TEST,sens_rail,on=FAIL,rc=%d\n", ret);
		return ret;
	}
	k_sleep(K_MSEC(SENS_POWER_SETTLE_MS));

	/* 4. 总线扫描（0x08~0x77，用 WHO_AM_I 寄存器读做探测事务） */
	for (uint16_t addr = 0x08U; addr <= 0x77U; addr++) {
		uint8_t reg = LSM6DSV16X_REG_WHO_AM_I;
		uint8_t dummy = 0U;

		if (i2c_write_read(sens_i2c, addr, &reg, 1, &dummy, 1) == 0) {
			if (found_cnt < ARRAY_SIZE(found)) {
				found[found_cnt] = (uint8_t)addr;
			}
			printk("IMU_TEST,scan,addr=0x%02X,ACK\n", addr);
			found_cnt++;
		}
	}

	if (found_cnt == 0U) {
		printk("IMU_TEST,scan,no_device=FAIL\n");
		(void)power_domain_set(POWER_DOMAIN_SENS, false);
		return -ENODEV;
	}

	/* 5. U4 WHO_AM_I 校验（两个候选地址） */
	for (size_t i = 0; i < ARRAY_SIZE(imu_candidate_addrs); i++) {
		uint8_t id = 0U;

		if (imu_who_am_i_check(imu_candidate_addrs[i], &id)) {
			imu_addr = (uint8_t)imu_candidate_addrs[i];
			imu_id = id;
			break;
		}
		if (id != 0U) {
			/* 有应答但 ID 不符：记录异常 */
			printk("IMU_TEST,whoami,addr=0x%02X,id=0x%02X,expect=0x%02X,MISMATCH\n",
			       imu_candidate_addrs[i], id, LSM6DSV16X_WHO_AM_I_VAL);
		}
	}

	if (imu_addr != 0U) {
		printk("IMU_TEST,u4,PASS,addr=0x%02X,whoami=0x%02X\n",
		       imu_addr, imu_id);
	} else {
		printk("IMU_TEST,u4,whoami=FAIL,found=%u,addr=0x%02X\n",
		       found_cnt, imu_addr);
		failures++;
	}

	/* 6. SENS 电源域下电，保持原有电源行为（正式采集功能另行管理） */
	ret = power_domain_set(POWER_DOMAIN_SENS, false);
	if (ret != 0) {
		printk("IMU_TEST,sens_rail,off=FAIL,rc=%d\n", ret);
		failures++;
	}

	printk("IMU_TEST,summary,%s,devices=%u,failures=%d\n",
	       (failures == 0) ? "PASS" : "FAIL", found_cnt, failures);

	return (failures == 0) ? 0 : -ENODEV;
}
