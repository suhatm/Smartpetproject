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

static FATFS fat_fs;
static struct fs_mount_t mnt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = MNT_POINT,
};

static uint8_t sector_buf[SECTOR_BUF_LEN];

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

int sd_bringup_test(void)
{
	/* STORE 域上电（PMIC LDO2 -> VDD_STORE_3V0_SW），给卡充足 POR 时间 */
	int ret = power_domain_set(POWER_DOMAIN_STORE, true);

	if (ret != 0) {
		printk("SD_TEST,fail,store_power_rc=%d\n", ret);
		return ret;
	}
	k_msleep(50);

	ret = disk_access_init(DISK_NAME);
	if (ret != 0) {
		printk("SD_TEST,fail,disk_init_rc=%d\n", ret);
		return ret;
	}

	uint32_t sector_count = 0;
	uint32_t sector_size = 0;

	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	if (ret != 0) {
		printk("SD_TEST,fail,ioctl_cnt_rc=%d\n", ret);
		return ret;
	}
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	if (ret != 0) {
		printk("SD_TEST,fail,ioctl_sz_rc=%d\n", ret);
		return ret;
	}

	uint32_t cap_mb = (uint32_t)(((uint64_t)sector_count * sector_size) / (1024U * 1024U));

	printk("SD_TEST,init_ok,sectors=%u,sector_sz=%u,cap_mb=%u\n",
	       sector_count, sector_size, cap_mb);

	/* 1Gb NAND 预期 ~110-130MB；偏离说明识别错误 */
	if (cap_mb < 100 || cap_mb > 140) {
		printk("SD_TEST,fail,cap_unexpected\n");
		return -ERANGE;
	}

	/* 读扇区 0 头 16 字节（识别 MBR/FAT 引导，纯诊断打印） */
	ret = disk_access_read(DISK_NAME, sector_buf, 0, 1);
	if (ret != 0) {
		printk("SD_TEST,fail,read0_rc=%d\n", ret);
		return ret;
	}
	printk("SD_TEST,sec0=%02x%02x%02x%02x%02x%02x%02x%02x"
	       "%02x%02x%02x%02x%02x%02x%02x%02x\n",
	       sector_buf[0], sector_buf[1], sector_buf[2], sector_buf[3],
	       sector_buf[4], sector_buf[5], sector_buf[6], sector_buf[7],
	       sector_buf[8], sector_buf[9], sector_buf[10], sector_buf[11],
	       sector_buf[12], sector_buf[13], sector_buf[14], sector_buf[15]);

	/* 挂 FATFS；无文件系统则格式化（bring-up 阶段允许清卡） */
	ret = fs_mount(&mnt);
	if (ret != 0) {
		printk("SD_TEST,mount_rc=%d,mkfs...\n", ret);
		ret = fs_mkfs(FS_FATFS, (uintptr_t)DISK_NAME, NULL, 0);
		if (ret != 0) {
			printk("SD_TEST,fail,mkfs_rc=%d\n", ret);
			return ret;
		}
		ret = fs_mount(&mnt);
		if (ret != 0) {
			printk("SD_TEST,fail,mount2_rc=%d\n", ret);
			return ret;
		}
	}
	printk("SD_TEST,mounted\n");

	ret = verify_file_io();
	if (ret != 0) {
		(void)fs_unmount(&mnt);
		return ret;
	}

	(void)fs_unmount(&mnt);
	printk("SD_TEST,PASS,cap_mb=%u,io=OK\n", cap_mb);
	return 0;
}

#else

int sd_bringup_test(void)
{
	return 0;
}

#endif /* CONFIG_APP_SD_TEST */
