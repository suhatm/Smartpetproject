/**
 * @file test_mailbox.c
 * @brief SWD 测试邮箱实现：三电源域独立开关命令的接收与执行
 *
 * 详见 test_mailbox.h 头注释的协议说明。
 */
#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "power_control.h"
#include "test_mailbox.h"

/** 电源域开关后等待 LDSW 软启动/状态寄存器更新的时间（ms） */
#define DOMAIN_SETTLE_MS 20U

/** 测试邮箱实例：放 .noinit 段（不占初始化数据段，地址链接期固定） */
struct power_test_mb power_test_mb __attribute__((section(".noinit")));

/**
 * @brief 回读三域实际状态并打包为位图
 *
 * @param mask 输出 TEST_MB_RESULT_* 位或
 * @return 0 成功；负值 power_control_get_state 错误码
 */
static int read_domain_states(uint32_t *mask)
{
	uint8_t ldsw = 0U;
	bool ana = false;
	int ret;

	ret = power_control_get_state(&ldsw, &ana);
	if (ret != 0) {
		return ret;
	}

	/* bit0=LDSW1/SENS，bit2=LDSW2/STORE（main.c LDSW_*_ON_BIT 同源定义） */
	*mask = 0U;
	if ((ldsw & 0x01U) != 0U) {
		*mask |= TEST_MB_RESULT_SENS;
	}
	if ((ldsw & 0x04U) != 0U) {
		*mask |= TEST_MB_RESULT_STORE;
	}
	if (ana) {
		*mask |= TEST_MB_RESULT_ANA;
	}
	return 0;
}

/**
 * @brief 执行单条命令：操作电源域 + 回读状态
 *
 * @param cmd 命令码
 * @param result 输出三域状态位图
 * @param rc 输出执行返回码（0 成功）
 */
static void execute_command(uint32_t cmd, uint32_t *result, int *rc)
{
	int ret = 0;

	switch (cmd) {
	case TEST_MB_CMD_SENS_ON:
		ret = power_domain_set(POWER_DOMAIN_SENS, true);
		break;
	case TEST_MB_CMD_SENS_OFF:
		ret = power_domain_set(POWER_DOMAIN_SENS, false);
		break;
	case TEST_MB_CMD_STORE_ON:
		ret = power_domain_set(POWER_DOMAIN_STORE, true);
		break;
	case TEST_MB_CMD_STORE_OFF:
		ret = power_domain_set(POWER_DOMAIN_STORE, false);
		break;
	case TEST_MB_CMD_ANA_ON:
		ret = power_domain_set(POWER_DOMAIN_ANALOG, true);
		break;
	case TEST_MB_CMD_ANA_OFF:
		ret = power_domain_set(POWER_DOMAIN_ANALOG, false);
		break;
	case TEST_MB_CMD_ALL_ON:
		/* 三域顺序开启，域间互不影响 */
		ret = power_domain_set(POWER_DOMAIN_SENS, true);
		if (ret == 0) {
			ret = power_domain_set(POWER_DOMAIN_STORE, true);
		}
		if (ret == 0) {
			ret = power_domain_set(POWER_DOMAIN_ANALOG, true);
		}
		break;
	case TEST_MB_CMD_ALL_OFF:
		ret = power_domains_all_off();
		break;
	case TEST_MB_CMD_STATUS:
		/* 仅查询，无动作 */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/* 无论命令成败都回读实际状态（上位机据此判断域独立性）；
	 * 先等 LDSW 软启动完成，否则状态位尚未置起（对齐自检逻辑） */
	*rc = ret;
	k_sleep(K_MSEC(DOMAIN_SETTLE_MS));
	(void)read_domain_states(result);
}

void test_mailbox_init(void)
{
	power_test_mb.magic = POWER_TEST_MB_MAGIC;
	power_test_mb.cmd = TEST_MB_CMD_NONE;
	power_test_mb.seq = 0U;
	power_test_mb.status = TEST_MB_STATUS_IDLE;
	power_test_mb.result = 0U;
	power_test_mb.rc = 0;

	printk("TESTMB,mailbox=0x%08x,protocol=swd_mailbox_v1\n",
	       (uint32_t)(uintptr_t)&power_test_mb);
}

void test_mailbox_poll(void)
{
	if (power_test_mb.magic != POWER_TEST_MB_MAGIC) {
		/* 段内容异常（理论上不会发生），重新初始化 */
		test_mailbox_init();
		return;
	}

	if (power_test_mb.status != TEST_MB_STATUS_PENDING) {
		return;
	}

	uint32_t cmd = power_test_mb.cmd;
	uint32_t seq = power_test_mb.seq;
	uint32_t result = 0U;
	int rc = 0;

	execute_command(cmd, &result, &rc);

	printk("TESTMB,cmd=%u,seq=%u,rc=%d,result=0x%02x\n",
	       cmd, seq, rc, result);

	power_test_mb.result = result;
	power_test_mb.rc = (uint32_t)rc;
	power_test_mb.status = TEST_MB_STATUS_DONE;
}
