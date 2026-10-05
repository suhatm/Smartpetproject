/**
 * @file sd_store.c
 * @brief SD NAND bring-up 自检实现（DISK_ACCESS + Elm FatFs）
 */
#include "sd_store.h"

#if CONFIG_APP_SD_TEST

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <errno.h>

#include "power_control.h"

#define DISK_NAME      "SD"
#define MNT_POINT      "/SD:"
#define TEST_FILE      MNT_POINT "/wbtest.txt"
#define SECTOR_BUF_LEN 512

/** SD_TEST 临时文件与块大小 */
#define SD_TEST_FILE   MNT_POINT "/sdtest.tmp"
#define SD_TEST_BLK    4096U

static FATFS fat_fs;
static struct fs_mount_t mnt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = MNT_POINT,
};

static uint8_t sector_buf[SECTOR_BUF_LEN];

/** 挂载互斥（recorder / SD_TEST / bridge 可能并发触发） */
static struct k_mutex mnt_lock = Z_MUTEX_INITIALIZER(mnt_lock);

/** 当前挂载状态与判定 */
static bool mounted;
static enum sd_state state = SD_STATE_UNKNOWN;

static int verify_file_io(void);

static void set_state(enum sd_state s)
{
	if (state != s) {
		printk("SD,state,%u->%u\n", state, s);
		state = s;
	}
}

enum sd_state sd_get_state(void)
{
	return state;
}

int sd_ensure_mounted(void)
{
	int ret = 0;

	k_mutex_lock(&mnt_lock, K_FOREVER);
	if (mounted) {
		k_mutex_unlock(&mnt_lock);
		return 0;
	}

	/* STORE 域上电（PMIC LDO2 -> VDD_STORE_3V0_SW），给卡充足 POR 时间 */
	ret = power_domain_set(POWER_DOMAIN_STORE, true);
	if (ret != 0) {
		printk("SD,mount_fail,store_power_rc=%d\n", ret);
		set_state(SD_STATE_ABSENT);
		goto out;
	}
	k_msleep(50);

	ret = disk_access_init(DISK_NAME);
	if (ret != 0) {
		printk("SD,mount_fail,disk_init_rc=%d\n", ret);
		set_state(SD_STATE_ABSENT);
		ret = -ENODEV;
		goto out;
	}

	ret = fs_mount(&mnt);
	if (ret != 0) {
		printk("SD,mount_rc=%d,mkfs...\n", ret);
		ret = fs_mkfs(FS_FATFS, (uintptr_t)DISK_NAME, NULL, 0);
		if (ret == 0) {
			ret = fs_mount(&mnt);
		}
		if (ret != 0) {
			printk("SD,mount_fail,rc=%d\n", ret);
			set_state(SD_STATE_ABSENT);
			ret = -EIO;
			goto out;
		}
	}

	mounted = true;
	set_state(SD_STATE_PRESENT);
	printk("SD,mounted\n");
	ret = 0;
out:
	k_mutex_unlock(&mnt_lock);
	return ret;
}

void sd_unmount(void)
{
	k_mutex_lock(&mnt_lock, K_FOREVER);
	if (mounted) {
		(void)fs_unmount(&mnt);
		mounted = false;
	}
	(void)power_domain_set(POWER_DOMAIN_STORE, false);
	set_state(SD_STATE_UNKNOWN);
	k_mutex_unlock(&mnt_lock);
}

int sd_bringup_test(void)
{
	int ret = sd_ensure_mounted();

	if (ret != 0) {
		printk("SD_TEST,fail,mount_rc=%d\n", ret);
		return ret;
	}

	uint32_t sector_count = 0;
	uint32_t sector_size = 0;

	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	if (ret != 0) {
		printk("SD_TEST,fail,ioctl_cnt_rc=%d\n", ret);
		goto fail;
	}
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	if (ret != 0) {
		printk("SD_TEST,fail,ioctl_sz_rc=%d\n", ret);
		goto fail;
	}

	uint32_t cap_mb = (uint32_t)(((uint64_t)sector_count * sector_size) / (1024U * 1024U));

	printk("SD_TEST,init_ok,sectors=%u,sector_sz=%u,cap_mb=%u\n",
	       sector_count, sector_size, cap_mb);

	/* 1Gb NAND 预期 ~110-130MB；偏离说明识别错误 */
	if (cap_mb < 100 || cap_mb > 140) {
		printk("SD_TEST,fail,cap_unexpected\n");
		ret = -ERANGE;
		goto fail;
	}

	/* 读扇区 0 头 16 字节（识别 MBR/FAT 引导，纯诊断打印） */
	ret = disk_access_read(DISK_NAME, sector_buf, 0, 1);
	if (ret != 0) {
		printk("SD_TEST,fail,read0_rc=%d\n", ret);
		goto fail;
	}
	printk("SD_TEST,sec0=%02x%02x%02x%02x%02x%02x%02x%02x"
	       "%02x%02x%02x%02x%02x%02x%02x%02x\n",
	       sector_buf[0], sector_buf[1], sector_buf[2], sector_buf[3],
	       sector_buf[4], sector_buf[5], sector_buf[6], sector_buf[7],
	       sector_buf[8], sector_buf[9], sector_buf[10], sector_buf[11],
	       sector_buf[12], sector_buf[13], sector_buf[14], sector_buf[15]);

	ret = verify_file_io();
	if (ret != 0) {
		goto fail;
	}

	printk("SD_TEST,PASS,cap_mb=%u,io=OK\n", cap_mb);
	return 0;

fail:
	sd_unmount();
	return ret;
}

/* ---- SD_TEST（协议 CMD 0x0D） ---- */

/** xorshift32 伪随机填充（种子=块号，写读两侧可复现） */
static void pattern_fill(uint8_t *buf, uint32_t len, uint32_t seed)
{
	uint32_t x = seed | 1U;
	uint32_t i = 0;

	while (i + 4U <= len) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		buf[i] = (uint8_t)x;
		buf[i + 1U] = (uint8_t)(x >> 8);
		buf[i + 2U] = (uint8_t)(x >> 16);
		buf[i + 3U] = (uint8_t)(x >> 24);
		i += 4U;
	}
	while (i < len) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		buf[i++] = (uint8_t)x;
	}
}

static uint8_t blk_buf[SD_TEST_BLK];
static uint8_t rd_buf[SD_TEST_BLK];

int sd_run_test(uint8_t size_mb, uint8_t verify, struct sd_test_result *out)
{
	struct fs_file_t f;
	uint32_t blocks, i;
	int64_t t0, t1;
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	out->result = -1;

	if (size_mb < 1U || size_mb > 32U) {
		size_mb = 4U;
	}
	blocks = (uint32_t)size_mb * 1024U * 1024U / SD_TEST_BLK;

	ret = sd_ensure_mounted();
	if (ret == -ENODEV) {
		out->result = -1; /* 无卡或初始化失败 */
		return 0;
	}
	if (ret != 0) {
		out->result = -2; /* 挂载失败 */
		return 0;
	}

	fs_file_t_init(&f);

	/* ---- 写 ---- */
	ret = fs_open(&f, SD_TEST_FILE, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		printk("SD_TEST2,fail,open_w_rc=%d\n", ret);
		out->result = -3;
		set_state(SD_STATE_DEGRADED);
		return 0;
	}

	t0 = k_uptime_get();
	for (i = 0; i < blocks; i++) {
		pattern_fill(blk_buf, SD_TEST_BLK, i);
		ssize_t wr = fs_write(&f, blk_buf, SD_TEST_BLK);

		if (wr != (ssize_t)SD_TEST_BLK) {
			printk("SD_TEST2,fail,write_blk=%u,rc=%d\n", i, (int)wr);
			out->result = -3;
			(void)fs_close(&f);
			(void)fs_unlink(SD_TEST_FILE);
			set_state(SD_STATE_DEGRADED);
			return 0;
		}
	}
	(void)fs_close(&f);
	t1 = k_uptime_get();
	out->w_kbps = (t1 > t0)
		? (uint32_t)(((uint64_t)blocks * SD_TEST_BLK) * 1000ULL /
			     (uint64_t)(t1 - t0) / 1024ULL)
		: 0;

	if (verify == 0U) {
		(void)fs_unlink(SD_TEST_FILE);
		out->result = 0;
		printk("SD_TEST2,done,w_kbps=%u,verify=skip\n", out->w_kbps);
		return 0;
	}

	/* ---- 读回校验 ---- */
	ret = fs_open(&f, SD_TEST_FILE, FS_O_READ);
	if (ret != 0) {
		printk("SD_TEST2,fail,open_r_rc=%d\n", ret);
		out->result = -4;
		(void)fs_unlink(SD_TEST_FILE);
		return 0;
	}

	t0 = k_uptime_get();
	for (i = 0; i < blocks; i++) {
		ssize_t rd = fs_read(&f, rd_buf, SD_TEST_BLK);

		if (rd != (ssize_t)SD_TEST_BLK) {
			printk("SD_TEST2,fail,read_blk=%u,rc=%d\n", i, (int)rd);
			out->result = -4;
			(void)fs_close(&f);
			(void)fs_unlink(SD_TEST_FILE);
			return 0;
		}
		pattern_fill(blk_buf, SD_TEST_BLK, i);
		if (memcmp(rd_buf, blk_buf, SD_TEST_BLK) != 0) {
			out->bad_blocks++;
		}
	}
	(void)fs_close(&f);
	(void)fs_unlink(SD_TEST_FILE);
	t1 = k_uptime_get();
	out->r_kbps = (t1 > t0)
		? (uint32_t)(((uint64_t)blocks * SD_TEST_BLK) * 1000ULL /
			     (uint64_t)(t1 - t0) / 1024ULL)
		: 0;

	out->result = (out->bad_blocks == 0U) ? 0 : -5;
	printk("SD_TEST2,done,result=%d,w_kbps=%u,r_kbps=%u,bad=%u\n",
	       out->result, out->w_kbps, out->r_kbps, out->bad_blocks);
	return 0;
}

#else

int sd_bringup_test(void)
{
	return 0;
}

int sd_ensure_mounted(void)
{
	return -ENOTSUP;
}

void sd_unmount(void) {}

enum sd_state sd_get_state(void)
{
	return SD_STATE_UNKNOWN;
}

int sd_run_test(uint8_t size_mb, uint8_t verify, struct sd_test_result *out)
{
	ARG_UNUSED(size_mb);
	ARG_UNUSED(verify);
	ARG_UNUSED(out);
	return -ENOTSUP;
}

#endif /* CONFIG_APP_SD_TEST */

static int verify_file_io(void)
{
	static const char payload[] =
		"SmartPet SD NAND bring-up test 2026-10-01. The quick brown fox jumps over the lazy dog. 0123456789.";
	char rbuf[sizeof(payload)];
	struct fs_file_t f;
	int ret;

	fs_file_t_init(&f);

	ret = fs_open(&f, TEST_FILE, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		printk("SD_TEST,fail,open_w_rc=%d\n", ret);
		return ret;
	}
	ssize_t wr = fs_write(&f, payload, sizeof(payload));

	(void)fs_close(&f);
	if (wr != (ssize_t)sizeof(payload)) {
		printk("SD_TEST,fail,write_len=%d\n", (int)wr);
		return -EIO;
	}

	ret = fs_open(&f, TEST_FILE, FS_O_READ);
	if (ret != 0) {
		printk("SD_TEST,fail,open_r_rc=%d\n", ret);
		return ret;
	}
	memset(rbuf, 0, sizeof(rbuf));
	ssize_t rd = fs_read(&f, rbuf, sizeof(rbuf));

	(void)fs_close(&f);
	if (rd != (ssize_t)sizeof(rbuf) || memcmp(rbuf, payload, sizeof(payload)) != 0) {
		printk("SD_TEST,fail,verify_mismatch,rd=%d\n", (int)rd);
		return -EIO;
	}

	(void)fs_unlink(TEST_FILE);
	return 0;
}
