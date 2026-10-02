/**
 * @file recorder.c
 * @brief 录音与文件管理实现（协议 REC_* / 0x07 / SD_TEST）
 *
 * 单工作线程串行执行：录音、文件上传、SD_TEST 互斥（REC_ERR_BUSY）。
 * dmic 采集：16kHz/16bit，ch_mask 决定单/双声道；录音块 4096B。
 * 上传：按 MTU 友好的块长经 hub_stream_frame(0x07) 发送，
 *       文件传输模式开启（hub_file_xfer_mode），完成发 EVENT 0x06。
 */
#include "recorder.h"

#if CONFIG_APP_RECORDER

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include "ble_sensor_hub.h"
#include "power_control.h"
#include "sd_store.h"

#define PCM_RATE      16000U
#define PCM_WIDTH     16U
#define BLOCK_SIZE    4096U /* 双声道 64ms/块 */
#define NUM_BLOCKS    4U
#define READ_TIMEOUT  1000U

#define REC_DIR       "/SD:/REC"
#define REC_PATH_FMT  "/SD:/REC/REC%04u.wav"
#define REC_ID_MAX    9999U

/** 单声道（16k*2B=32B/ms）：4096B = 128ms/块 */
#define BLOCK_SIZE_MONO 4096U

/** 上传块长：帧载荷 6B 头 + 数据；MTU 244 下单帧 ≤ 238B 数据，取 224 对齐 */
#define XFER_CHUNK     224U

/** RMS 上报窗口：每累计 500ms 发一帧 0x06 */
#define RMS_WINDOW_MS  500U

K_MEM_SLAB_DEFINE_STATIC(rec_mem_slab, BLOCK_SIZE, NUM_BLOCKS, 4);

static const struct device *const dmic = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

/* ---- WAV 头 ---- */

struct wav_header {
	char riff[4];
	uint32_t riff_size;
	char wave[4];
	char fmt[4];
	uint32_t fmt_size;
	uint16_t audio_format;
	uint16_t channels;
	uint32_t sample_rate;
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bits;
	char data_id[4];
	uint32_t data_size;
} __packed;

static void wav_header_fill(struct wav_header *h, uint16_t channels,
			    uint32_t data_size)
{
	memcpy(h->riff, "RIFF", 4);
	h->riff_size = 36U + data_size;
	memcpy(h->wave, "WAVE", 4);
	memcpy(h->fmt, "fmt ", 4);
	h->fmt_size = 16U;
	h->audio_format = 1U; /* PCM */
	h->channels = channels;
	h->sample_rate = PCM_RATE;
	h->bits = PCM_WIDTH;
	h->block_align = (uint16_t)(channels * (PCM_WIDTH / 8U));
	h->byte_rate = PCM_RATE * h->block_align;
	memcpy(h->data_id, "data", 4);
	h->data_size = data_size;
}

/* ---- 线程命令 ---- */

enum rec_cmd {
	REC_CMD_NONE = 0,
	REC_CMD_START,      /* ch_mask/duration 有效 */
	REC_CMD_STOP,
	REC_CMD_MONITOR_ON,
	REC_CMD_MONITOR_OFF,
	REC_CMD_READ,       /* file_id/offset 有效 */
	REC_CMD_SD_TEST,    /* sd_size_mb/sd_verify 有效 */
};

struct rec_msg {
	enum rec_cmd cmd;
	uint8_t ch_mask;
	uint16_t duration_s;
	uint16_t file_id;
	uint32_t offset;
	uint8_t sd_size_mb;
	uint8_t sd_verify;
};

K_MSGQ_DEFINE(rec_msgq, sizeof(struct rec_msg), 4, 4);

/* ---- 状态 ---- */

static volatile bool recording;
static volatile bool monitoring;
static volatile bool xfering;
static volatile bool stop_req;
static volatile bool sens_powered; /* 本模块是否拉起了 SENS 域 */
static uint16_t cur_file_id;
static int64_t rec_started_ms;
static rec_sd_test_done_cb sd_test_cb;

/* ---- 工具 ---- */

static int sens_ensure(void)
{
	if (sens_powered) {
		return 0;
	}
	int ret = power_domain_set(POWER_DOMAIN_SENS, true);

	if (ret == 0) {
		sens_powered = true;
		k_msleep(10);
	}
	return ret;
}

static void sens_release(void)
{
	if (sens_powered) {
		(void)power_domain_set(POWER_DOMAIN_SENS, false);
		sens_powered = false;
	}
}

static int find_free_file_id(void)
{
	struct fs_dir_t dir;
	struct fs_dirent ent;
	/* 位图压缩（10000 位 = 1250B，避免大数组撑爆线程栈） */
	static uint8_t used[(REC_ID_MAX + 8U) / 8U];

	memset(used, 0, sizeof(used));

	(void)fs_mkdir(REC_DIR);

	fs_dir_t_init(&dir);
	if (fs_opendir(&dir, REC_DIR) == 0) {
		while (fs_readdir(&dir, &ent) == 0 && ent.name[0] != '\0') {
			unsigned int id;

			if (sscanf(ent.name, "REC%4u.wav", &id) == 1 &&
			    id >= 1U && id <= REC_ID_MAX) {
				used[id / 8U] |= (uint8_t)(1U << (id % 8U));
			}
		}
		(void)fs_closedir(&dir);
	}

	for (uint16_t id = 1U; id <= REC_ID_MAX; id++) {
		if ((used[id / 8U] & (uint8_t)(1U << (id % 8U))) == 0U) {
			return (int)id;
		}
	}
	return -1;
}

static bool file_exists(uint16_t file_id, uint32_t *size_out)
{
	char path[32];
	struct fs_dirent ent;

	(void)snprintk(path, sizeof(path), REC_PATH_FMT, file_id);
	if (fs_stat(path, &ent) != 0 || ent.type != FS_DIR_ENTRY_FILE) {
		return false;
	}
	if (size_out != NULL) {
		*size_out = ent.size;
	}
	return true;
}

/* ---- 录音/监听引擎 ---- */

static int dmic_setup(uint8_t ch_mask)
{
	uint8_t num_ch = (ch_mask == 0x03U) ? 2U : 1U;
	struct pcm_stream_cfg stream = {
		.pcm_rate = PCM_RATE,
		.pcm_width = PCM_WIDTH,
		.block_size = BLOCK_SIZE,
		.mem_slab = &rec_mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_chan = num_ch,
			.req_num_streams = 1,
			.req_chan_map_lo =
				(num_ch == 2U)
					? (dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
					   dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT))
					: dmic_build_channel_map(0, 0,
						(ch_mask & 0x01U) ? PDM_CHAN_LEFT
								  : PDM_CHAN_RIGHT),
		},
	};

	return dmic_configure(dmic, &cfg);
}

/**
 * @brief RMS 统计器：500ms 窗累计，到窗发 0x06 MIC 帧
 */
struct rms_acc {
	int64_t sum_l;
	int64_t sum_r;
	uint32_t n;
	uint32_t window_bytes; /* 窗口字节数（按声道数换算） */
};

static void rms_feed(struct rms_acc *acc, const int16_t *s, uint32_t size,
		     uint8_t num_ch)
{
	if (num_ch == 2U) {
		uint32_t pairs = size / (2U * sizeof(int16_t));

		for (uint32_t i = 0; i < pairs; i++) {
			int16_t l = s[2U * i];
			int16_t r = s[2U * i + 1U];

			acc->sum_l += (int32_t)l * l;
			acc->sum_r += (int32_t)r * r;
		}
		acc->n += pairs;
	} else {
		uint32_t cnt = size / sizeof(int16_t);

		for (uint32_t i = 0; i < cnt; i++) {
			acc->sum_l += (int32_t)s[i] * s[i];
		}
		acc->n += cnt;
	}
}

static void rms_emit(struct rms_acc *acc, uint8_t num_ch)
{
	if (acc->n == 0U) {
		return;
	}
	uint16_t rms_l = (uint16_t)sqrt((double)(acc->sum_l / (int64_t)acc->n));
	uint16_t rms_r = (num_ch == 2U)
				 ? (uint16_t)sqrt((double)(acc->sum_r / (int64_t)acc->n))
				 : rms_l;
	uint8_t payload[5] = {
		0x01U, /* 状态：PRESENT */
		(uint8_t)(rms_l & 0xFFU), (uint8_t)(rms_l >> 8),
		(uint8_t)(rms_r & 0xFFU), (uint8_t)(rms_r >> 8),
	};

	(void)hub_stream_frame(HUB_TYPE_MIC, payload, sizeof(payload));
	acc->sum_l = 0;
	acc->sum_r = 0;
	acc->n = 0;
}

static void send_rec_state(uint8_t state, uint16_t file_id, uint16_t elapsed_s)
{
	uint8_t payload[5] = {
		HUB_EVENT_REC_STATE, state,
		(uint8_t)(file_id & 0xFFU), (uint8_t)(file_id >> 8),
		(uint8_t)(elapsed_s & 0xFFU),
	};

	(void)hub_event_frame(HUB_TYPE_EVENT, payload, sizeof(payload));
}

/** 录音/监听主流程（线程上下文，阻塞直到停止/时长到） */
static void record_run(uint8_t ch_mask, uint16_t duration_s)
{
	uint8_t num_ch = (ch_mask == 0x03U) ? 2U : 1U;
	struct fs_file_t wav;
	bool to_file = (duration_s != 0xFFFFU); /* monitor 用 0xFFFF 哨兵 */
	uint32_t data_bytes = 0U;
	struct rms_acc acc = { 0 };
	uint32_t bytes_per_ms = PCM_RATE / 1000U * num_ch * (PCM_WIDTH / 8U);
	uint32_t acc_bytes = 0U;
	int ret = 0;

	if (!device_is_ready(dmic)) {
		printk("REC,fail,dmic_not_ready\n");
		goto out;
	}

	ret = sens_ensure();
	if (ret != 0) {
		printk("REC,fail,sens_rc=%d\n", ret);
		goto out;
	}

	fs_file_t_init(&wav);
	if (to_file) {
		struct wav_header hdr;

		ret = sd_ensure_mounted();
		if (ret != 0) {
			printk("REC,fail,sd_rc=%d\n", ret);
			goto out_sens;
		}

		char path[32];

		(void)snprintk(path, sizeof(path), REC_PATH_FMT, cur_file_id);
		ret = fs_open(&wav, path, FS_O_CREATE | FS_O_WRITE);
		if (ret != 0) {
			printk("REC,fail,open_rc=%d\n", ret);
			goto out_sens;
		}
		wav_header_fill(&hdr, num_ch, 0U);
		(void)fs_write(&wav, &hdr, sizeof(hdr));
	}

	ret = dmic_setup(ch_mask);
	if (ret != 0) {
		printk("REC,fail,dmic_cfg_rc=%d\n", ret);
		goto out_file;
	}
	ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
	if (ret != 0) {
		printk("REC,fail,dmic_trig_rc=%d\n", ret);
		goto out_file;
	}

	recording = to_file;
	monitoring = !to_file;
	rec_started_ms = k_uptime_get();
	stop_req = false;
	if (to_file) {
		send_rec_state(1U, cur_file_id, 0U);
	}
	printk("REC,start,file=%u,ch=%u,dur=%u\n",
	       to_file ? cur_file_id : 0U, num_ch, duration_s);

	while (!stop_req) {
		void *buf = NULL;
		uint32_t size = 0;

		ret = dmic_read(dmic, 0, &buf, &size, READ_TIMEOUT);
		if (ret != 0 || buf == NULL) {
			printk("REC,warn,read_rc=%d\n", ret);
			break;
		}

		rms_feed(&acc, (const int16_t *)buf, size, num_ch);
		acc_bytes += size;
		if (acc_bytes >= bytes_per_ms * RMS_WINDOW_MS) {
			rms_emit(&acc, num_ch);
			acc_bytes = 0U;
		}

		if (to_file) {
			ssize_t wr = fs_write(&wav, buf, size);

			if (wr != (ssize_t)size) {
				printk("REC,fail,write_rc=%d\n", (int)wr);
				k_mem_slab_free(&rec_mem_slab, buf);
				break;
			}
			data_bytes += size;
		}
		k_mem_slab_free(&rec_mem_slab, buf);

		if (to_file && duration_s > 0U) {
			uint32_t elapsed = (uint32_t)((k_uptime_get() -
						       rec_started_ms) / 1000);

			if (elapsed >= duration_s) {
				break;
			}
		}
	}

	(void)dmic_trigger(dmic, DMIC_TRIGGER_STOP);
	rms_emit(&acc, num_ch);

out_file:
	if (to_file) {
		/* 回填 WAV 头 */
		struct wav_header hdr;

		wav_header_fill(&hdr, num_ch, data_bytes);
		(void)fs_seek(&wav, 0, FS_SEEK_SET);
		(void)fs_write(&wav, &hdr, sizeof(hdr));
		(void)fs_close(&wav);

		uint16_t elapsed_s = (uint16_t)((k_uptime_get() -
						 rec_started_ms) / 1000);

		send_rec_state(2U, cur_file_id, elapsed_s);
		printk("REC,done,file=%u,bytes=%u,elapsed=%us\n",
		       cur_file_id, data_bytes, elapsed_s);
	}
out_sens:
	sens_release();
out:
	recording = false;
	monitoring = false;
}

/* ---- 文件上传 ---- */

static void xfer_run(uint16_t file_id, uint32_t offset)
{
	char path[32];
	struct fs_file_t f;
	uint32_t file_size = 0U;
	uint8_t buf[6U + XFER_CHUNK];
	uint32_t crc = 0U;

	xfering = true;
	hub_file_xfer_mode(true);

	if (!file_exists(file_id, &file_size)) {
		printk("XFER,fail,no_file,id=%u\n", file_id);
		goto out;
	}
	if (offset >= file_size && file_size > 0U) {
		offset = 0U; /* 非法 offset 从头发，上位机可自恢复 */
	}

	(void)snprintk(path, sizeof(path), REC_PATH_FMT, file_id);
	fs_file_t_init(&f);
	if (fs_open(&f, path, FS_O_READ) != 0) {
		printk("XFER,fail,open,id=%u\n", file_id);
		goto out;
	}
	if (offset > 0U) {
		(void)fs_seek(&f, offset, FS_SEEK_SET);
	}

	/* CRC32 覆盖整个文件（含 offset 之前部分）：先扫前段 */
	crc = 0U;
	{
		uint8_t tmp[128];
		uint32_t remain = offset;

		(void)fs_seek(&f, 0, FS_SEEK_SET);
		while (remain > 0U) {
			uint32_t want = MIN(remain, sizeof(tmp));
			ssize_t rd = fs_read(&f, tmp, want);

			if (rd <= 0) {
				break;
			}
			crc = crc32_ieee_update(crc, tmp, (size_t)rd);
			remain -= (uint32_t)rd;
		}
		(void)fs_seek(&f, offset, FS_SEEK_SET);
	}

	printk("XFER,start,id=%u,offset=%u,size=%u\n", file_id, offset,
	       file_size);

	uint32_t pos = offset;

	while (pos < file_size) {
		uint32_t want = MIN((uint32_t)XFER_CHUNK, file_size - pos);
		ssize_t rd = fs_read(&f, buf + 6U, want);

		if (rd <= 0) {
			printk("XFER,fail,read,pos=%u\n", pos);
			break;
		}

		buf[0] = (uint8_t)(file_id & 0xFFU);
		buf[1] = (uint8_t)(file_id >> 8);
		buf[2] = (uint8_t)(pos & 0xFFU);
		buf[3] = (uint8_t)((pos >> 8) & 0xFFU);
		buf[4] = (uint8_t)((pos >> 16) & 0xFFU);
		buf[5] = (uint8_t)((pos >> 24) & 0xFFU);

		crc = crc32_ieee_update(crc, buf + 6U, (size_t)rd);

		/* 缓冲满时短暂等待（hub pack 以 ASAP 模式发送） */
		for (uint8_t retry = 0; retry < 50U; retry++) {
			if (hub_stream_frame(HUB_TYPE_AUDIO_FILE, buf,
					     (uint8_t)(6U + rd)) == 0) {
				break;
			}
			k_msleep(10);
		}

		pos += (uint32_t)rd;
	}
	(void)fs_close(&f);

	if (pos >= file_size) {
		/* EVENT 0x06 REC_FILE_DONE：file_id(2)+total_bytes(4)+crc32(4) */
		uint8_t done[11] = {
			HUB_EVENT_REC_FILE_DONE,
			(uint8_t)(file_id & 0xFFU), (uint8_t)(file_id >> 8),
			(uint8_t)(file_size & 0xFFU),
			(uint8_t)((file_size >> 8) & 0xFFU),
			(uint8_t)((file_size >> 16) & 0xFFU),
			(uint8_t)((file_size >> 24) & 0xFFU),
			(uint8_t)(crc & 0xFFU), (uint8_t)((crc >> 8) & 0xFFU),
			(uint8_t)((crc >> 16) & 0xFFU),
			(uint8_t)((crc >> 24) & 0xFFU),
		};

		(void)hub_event_frame(HUB_TYPE_EVENT, done, sizeof(done));
		printk("XFER,done,id=%u,bytes=%u,crc=%08x\n", file_id, pos, crc);
	}

out:
	hub_file_xfer_mode(false);
	xfering = false;
}

/* ---- SD_TEST ---- */

static void sd_test_run(uint8_t size_mb, uint8_t verify)
{
	struct sd_test_result res;

	(void)sd_run_test(size_mb, verify, &res);
	if (sd_test_cb != NULL) {
		sd_test_cb(&res);
	}
}

/* ---- 线程主循环 ---- */

static void recorder_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct rec_msg msg;

	for (;;) {
		if (k_msgq_get(&rec_msgq, &msg, K_FOREVER) != 0) {
			continue;
		}

		switch (msg.cmd) {
		case REC_CMD_START:
			record_run(msg.ch_mask, msg.duration_s);
			break;
		case REC_CMD_MONITOR_ON:
			record_run(msg.ch_mask, 0xFFFFU); /* 哨兵=监听 */
			break;
		case REC_CMD_READ:
			xfer_run(msg.file_id, msg.offset);
			break;
		case REC_CMD_SD_TEST:
			sd_test_run(msg.sd_size_mb, msg.sd_verify);
			break;
		default:
			break;
		}
	}
}

K_THREAD_DEFINE(recorder_tid, 4096, recorder_thread, NULL, NULL, NULL,
		5, 0, 0);

/* ---- 对外 API（bridge 线程上下文调用） ---- */

int recorder_init(void)
{
	return 0;
}

int recorder_start(uint8_t ch_mask, uint16_t duration_s)
{
	if ((ch_mask == 0U) || (ch_mask > 0x03U)) {
		return REC_ERR_PARAM;
	}
	if (recording || monitoring || xfering) {
		return REC_ERR_BUSY;
	}
	if (sd_get_state() == SD_STATE_ABSENT) {
		return REC_ERR_NO_CARD;
	}

	int id = find_free_file_id();

	if (id < 0) {
		return REC_ERR_NO_SPACE;
	}

	struct rec_msg msg = {
		.cmd = REC_CMD_START,
		.ch_mask = ch_mask,
		.duration_s = duration_s,
	};

	cur_file_id = (uint16_t)id;
	if (k_msgq_put(&rec_msgq, &msg, K_NO_WAIT) != 0) {
		return REC_ERR_BUSY;
	}
	return REC_OK;
}

int recorder_stop(void)
{
	if (!recording && !monitoring) {
		return REC_ERR_PARAM;
	}
	stop_req = true;
	return REC_OK;
}

bool recorder_active(void)
{
	return recording;
}

int recorder_monitor_start(void)
{
	if (recording || monitoring || xfering) {
		return REC_ERR_BUSY;
	}
	struct rec_msg msg = {
		.cmd = REC_CMD_MONITOR_ON,
		.ch_mask = 0x03U,
	};

	if (k_msgq_put(&rec_msgq, &msg, K_NO_WAIT) != 0) {
		return REC_ERR_BUSY;
	}
	return REC_OK;
}

int recorder_monitor_stop(void)
{
	return recorder_stop();
}

int recorder_list(uint8_t *buf, uint16_t max_len, uint16_t *out_len)
{
	struct fs_dir_t dir;
	struct fs_dirent ent;
	uint16_t n = 0U;

	*out_len = 0U;

	if (sd_ensure_mounted() != 0) {
		return REC_ERR_NO_CARD;
	}
	(void)fs_mkdir(REC_DIR);

	fs_dir_t_init(&dir);
	if (fs_opendir(&dir, REC_DIR) != 0) {
		return REC_ERR_IO;
	}

	while (fs_readdir(&dir, &ent) == 0 && ent.name[0] != '\0' &&
	       n < REC_LIST_MAX &&
	       (uint32_t)(*out_len + REC_LIST_ENTRY_LEN) <= max_len) {
		unsigned int id;

		if (ent.type != FS_DIR_ENTRY_FILE ||
		    sscanf(ent.name, "REC%4u.wav", &id) != 1) {
			continue;
		}

		/* 时长估算：数据字节 / byte_rate */
		uint16_t ch = (ent.size >= 24U) ? 2U : 1U;
		uint32_t byte_rate = PCM_RATE * ch * (PCM_WIDTH / 8U);
		uint32_t dur = (ent.size > 44U)
				       ? (ent.size - 44U) / byte_rate
				       : 0U;
		uint16_t size_kb = (uint16_t)(ent.size / 1024U);

		buf[(*out_len)++] = (uint8_t)(id & 0xFFU);
		buf[(*out_len)++] = (uint8_t)(id >> 8);
		buf[(*out_len)++] = (uint8_t)(dur & 0xFFU);
		buf[(*out_len)++] = (uint8_t)(dur >> 8);
		buf[(*out_len)++] = (uint8_t)(size_kb & 0xFFU);
		buf[(*out_len)++] = (uint8_t)(size_kb >> 8);
		n++;
	}
	(void)fs_closedir(&dir);
	return REC_OK;
}

int recorder_read_start(uint16_t file_id, uint32_t offset)
{
	if (recording || monitoring || xfering) {
		return REC_ERR_BUSY;
	}
	if (sd_get_state() == SD_STATE_ABSENT) {
		return REC_ERR_NO_CARD;
	}
	if (!file_exists(file_id, NULL)) {
		return REC_ERR_NO_FILE;
	}

	struct rec_msg msg = {
		.cmd = REC_CMD_READ,
		.file_id = file_id,
		.offset = offset,
	};

	if (k_msgq_put(&rec_msgq, &msg, K_NO_WAIT) != 0) {
		return REC_ERR_BUSY;
	}
	return REC_OK;
}

int recorder_delete(uint16_t file_id)
{
	if (recording || monitoring || xfering) {
		return REC_ERR_BUSY;
	}
	char path[32];

	if (!file_exists(file_id, NULL)) {
		return REC_ERR_NO_FILE;
	}
	(void)snprintk(path, sizeof(path), REC_PATH_FMT, file_id);
	if (fs_unlink(path) != 0) {
		return REC_ERR_IO;
	}
	printk("REC,deleted,id=%u\n", file_id);
	return REC_OK;
}

int recorder_sd_test(uint8_t size_mb, uint8_t verify, rec_sd_test_done_cb cb)
{
	if (recording || monitoring || xfering) {
		return REC_ERR_BUSY;
	}
	sd_test_cb = cb;

	struct rec_msg msg = {
		.cmd = REC_CMD_SD_TEST,
		.sd_size_mb = size_mb,
		.sd_verify = verify,
	};

	if (k_msgq_put(&rec_msgq, &msg, K_NO_WAIT) != 0) {
		return REC_ERR_BUSY;
	}
	return REC_OK;
}

bool recorder_xfer_active(void)
{
	return xfering;
}

#else /* !CONFIG_APP_RECORDER */

int recorder_init(void) { return 0; }
int recorder_start(uint8_t ch_mask, uint16_t duration_s)
{
	ARG_UNUSED(ch_mask); ARG_UNUSED(duration_s);
	return -12;
}
int recorder_stop(void) { return -1; }
bool recorder_active(void) { return false; }
int recorder_monitor_start(void) { return -12; }
int recorder_monitor_stop(void) { return -1; }
int recorder_list(uint8_t *buf, uint16_t max_len, uint16_t *out_len)
{
	ARG_UNUSED(buf); ARG_UNUSED(max_len); ARG_UNUSED(out_len);
	return -12;
}
int recorder_read_start(uint16_t file_id, uint32_t offset)
{
	ARG_UNUSED(file_id); ARG_UNUSED(offset);
	return -12;
}
int recorder_delete(uint16_t file_id)
{
	ARG_UNUSED(file_id);
	return -12;
}
int recorder_sd_test(uint8_t size_mb, uint8_t verify, rec_sd_test_done_cb cb)
{
	ARG_UNUSED(size_mb); ARG_UNUSED(verify); ARG_UNUSED(cb);
	return -12;
}
bool recorder_xfer_active(void) { return false; }

#endif /* CONFIG_APP_RECORDER */
