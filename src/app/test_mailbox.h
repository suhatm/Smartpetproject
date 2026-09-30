/**
 * @file test_mailbox.h
 * @brief SWD 测试邮箱：上位机经调试器直接读写 RAM 邮箱下发电源域命令
 *
 * 背景：本板 uart20 因引脚冲突禁用、无 USB CDC，上位机与固件的唯一
 * 通信通道是 SWD。协议采用"共享内存邮箱"：
 *
 *  - 固件启动时初始化邮箱（写入魔数），主循环每 20ms 轮询一次；
 *  - 上位机（tools/power_domain_test.py，经 nrfutil device read/write）
 *    按顺序写 cmd -> seq -> status=1（status 最后写，作为触发）；
 *  - 固件看到 status==1 后执行命令，回填 result（三域状态位图）与
 *    rc（返回码），最后写 status=2 表示完成；
 *  - 上位机轮询 status==2 即取走结果。
 *
 * 邮箱地址在链接期固定，可从 build/zephyr/zephyr.map 中解析
 * 符号 power_test_mb 获得；固件启动时也会通过 RTT 打印该地址。
 */
#ifndef TEST_MAILBOX_H
#define TEST_MAILBOX_H

#include <stdint.h>

/** 邮箱魔数："PRT3"（Power Rail Test v3），固件启动时写入 */
#define POWER_TEST_MB_MAGIC 0x50525433UL

/** 上位机命令集（写 mb.cmd） */
enum power_test_cmd {
	TEST_MB_CMD_NONE = 0,  /**< 空命令 */
	TEST_MB_CMD_SENS_ON = 1,  /**< 打开 VDD_SENS_3V0 */
	TEST_MB_CMD_SENS_OFF = 2, /**< 关闭 VDD_SENS_3V0 */
	TEST_MB_CMD_STORE_ON = 3, /**< 打开 VDD_STORE_3V0_SW */
	TEST_MB_CMD_STORE_OFF = 4,/**< 关闭 VDD_STORE_3V0_SW */
	TEST_MB_CMD_ANA_ON = 5,   /**< 打开 VDD_ANA_3V0（ANA_EN） */
	TEST_MB_CMD_ANA_OFF = 6,  /**< 关闭 VDD_ANA_3V0（ANA_EN） */
	TEST_MB_CMD_ALL_ON = 7,  /**< 三域全开 */
	TEST_MB_CMD_ALL_OFF = 8, /**< 三域全关 */
	TEST_MB_CMD_STATUS = 9,  /**< 仅查询三域状态 */
};

/** 命令状态机（mb.status） */
#define TEST_MB_STATUS_IDLE    0U /**< 空闲，等待上位机命令 */
#define TEST_MB_STATUS_PENDING 1U /**< 上位机已写入命令，待执行 */
#define TEST_MB_STATUS_DONE    2U /**< 固件已执行完毕，结果有效 */

/** mb.result 三域状态位 */
#define TEST_MB_RESULT_SENS  0x01U /**< bit0: VDD_SENS_3V0 导通 */
#define TEST_MB_RESULT_STORE 0x02U /**< bit1: VDD_STORE_3V0_SW 导通 */
#define TEST_MB_RESULT_ANA   0x04U /**< bit2: VDD_ANA_3V0 导通 */

/**
 * @brief 测试邮箱结构（实例位于 .noinit 段，链接期地址固定）
 *
 * 字段布局（小端 32 位字，上位机按 4 字节对齐访问）：
 *  +0x00 magic   魔数，固件启动时写 0x50525433
 *  +0x04 cmd     命令码（上位机写）
 *  +0x08 seq     命令序号，每条命令递增（上位机写）
 *  +0x0C status  0=空闲 1=待执行 2=完成（触发位，上位机最后写）
 *  +0x10 result  三域状态位图（固件回填）
 *  +0x14 rc      执行返回码，0 成功（固件回填）
 */
struct power_test_mb {
	volatile uint32_t magic;
	volatile uint32_t cmd;
	volatile uint32_t seq;
	volatile uint32_t status;
	volatile uint32_t result;
	volatile uint32_t rc;
};

/** 邮箱全局实例（map 符号名 power_test_mb，.noinit 段） */
extern struct power_test_mb power_test_mb;

/**
 * @brief 初始化测试邮箱（魔数 + 清零），并打印邮箱地址
 */
void test_mailbox_init(void);

/**
 * @brief 主循环轮询：发现待执行命令则执行并回填结果
 *
 * 非中断上下文调用；单条命令执行时间 <1ms（不含 PMIC 寄存器应答）。
 */
void test_mailbox_poll(void);

#endif /* TEST_MAILBOX_H */
