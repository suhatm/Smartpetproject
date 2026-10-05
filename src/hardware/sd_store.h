/**
 * @file sd_store.h
 * @brief SD NAND（CSNP1GCR01-BOW）存储：bring-up 自检 + 挂载管理 + SD_TEST
 *
 * 硬件事实（原理图核对，docs/原理图核对与评审建议.md P0-2）：
 *   无硬件卡检测脚（SPI 模式下 CD/SDD3 引脚用作 CS），
 *   SD 状态靠试挂载判定：挂载成功=PRESENT、初始化/挂载失败=ABSENT、
 *   挂载成功但读写失败=DEGRADED；事件驱动重判，不做周期轮询。
 *
 * 注意：卡上无 FAT 文件系统时会被 fs_mkfs 格式化（清卡）——bring-up/
 * SD_TEST 阶段允许；产品化时此路径要改成"挂载失败仅报错"。
 */
#ifndef SD_STORE_H
#define SD_STORE_H

#include <stdbool.h>
#include <stdint.h>

/** SD 连接状态（与协议 6.1 连接状态编码一致） */
enum sd_state {
	SD_STATE_UNKNOWN  = 0,
	SD_STATE_PRESENT  = 1,
	SD_STATE_ABSENT   = 2,
	SD_STATE_DEGRADED = 3,
};

/** SD_TEST 结果（CMD 0x0D / EVENT 上报用） */
struct sd_test_result {
	int8_t result;       /**< 0 通过；-1 无卡或初始化失败；-2 挂载失败；
			      *  -3 写失败；-4 读失败；-5 校验不一致 */
	uint32_t w_kbps;     /**< 写入速度 KB/s */
	uint32_t r_kbps;     /**< 读取速度 KB/s */
	uint32_t bad_blocks; /**< 校验不一致的块数 */
};

/**
 * @brief SD bring-up：上电(STORE 域) -> disk 初始化 -> 容量 -> FATFS
 *        挂载/格式化 -> 文件写入读回逐字节比对 -> 删除 -> 卸载
 *
 * @return 0 PASS；负值失败（阶段见 RTT 日志）
 */
int sd_bringup_test(void);

/**
 * @brief 确保 SD 已挂载（STORE 域上电 + disk init + FATFS mount）
 *
 * 幂等：已挂载直接返回 0。失败时自动尝试 mkfs 一次
 * （与 bring-up 同策略：裸卡允许格式化）。
 *
 * @return 0 成功；-ENODEV 无卡/初始化失败；-EIO 挂载失败
 */
int sd_ensure_mounted(void);

/**
 * @brief 卸载并下电 STORE 域（长时间不用时省电）
 */
void sd_unmount(void);

/** @brief 当前 SD 状态（试挂载判定结果） */
enum sd_state sd_get_state(void);

/**
 * @brief SD 卡自测（协议 CMD 0x0D）：写伪随机图样 -> 读回 -> 逐块校验 + 测速
 *
 * 阻塞执行（调用方在工作线程跑），典型 4MB 约 3~6s。
 * 图样：xorshift32 伪随机（种子=块号），写 size_mb MB 到临时文件，
 * 读回逐块比对，结束删除临时文件。
 *
 * @param size_mb 测试数据量（1~32，越界按 4MB）
 * @param verify  0=只写不校验（测写速）；非 0=写后读回校验
 * @param out     结果输出
 * @return 0=执行完成（结果见 out->result）；负值=参数/执行环境错误
 */
int sd_run_test(uint8_t size_mb, uint8_t verify, struct sd_test_result *out);

#endif /* SD_STORE_H */
