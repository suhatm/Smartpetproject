/**
 * @file mic_pdm.c
 * @brief 双 PDM 麦 bring-up 自检实现（nordic,nrf-pdm + Zephyr dmic API）
 *
 * 接线（原理图 mic 页）：两麦共享 CLK(P1.12)；DATA 经 100Ω 汇至 DIN(P1.10)；
 * U5 LR=VDD（右时隙），U6 LR=GND（左时隙）——PDM 标准立体声分时隙。
 * PCM 交错格式：L,R,L,R...（dmic 声道映射默认 L 先）。
 */
#include "mic_pdm.h"

#if CONFIG_APP_MIC_TEST

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <stdint.h>

#include "power_control.h"

#define PCM_RATE        16000U
#define PCM_WIDTH       16U
#define NUM_CHAN        2U
#define CAPTURE_MS      2000U
#define BLOCK_SIZE      4096U /* 字节：16k*2ch*2B=64B/ms -> 每块 64ms */
#define NUM_BLOCKS      4U
#define READ_TIMEOUT_MS 1000U

K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, BLOCK_SIZE, NUM_BLOCKS, 4);

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

int mic_bringup_test(void)
{
	if (!device_is_ready(dmic_dev)) {
		printk("MIC_TEST,fail,dev_not_ready\n");
		return -ENODEV;
	}

	/* 麦克风在 VDD_SENS_3V0 域 */
	int ret = power_domain_set(POWER_DOMAIN_SENS, true);

	if (ret != 0) {
		printk("MIC_TEST,fail,sens_power_rc=%d\n", ret);
		return ret;
	}
	k_msleep(10);

	struct pcm_stream_cfg stream = {
		.pcm_rate = PCM_RATE,
		.pcm_width = PCM_WIDTH,
		.block_size = BLOCK_SIZE,
		.mem_slab = &rx_mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* IM69D128 时钟范围约 1.0~3.3MHz，占空比 40~60% */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_chan = NUM_CHAN,
			.req_num_streams = 1,
			/* ch0=左(U6 LR=GND)、ch1=右(U5 LR=VDD)，默认映射 */
			.req_chan_map_lo =
				dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
				dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT),
		},
	};

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret != 0) {
		printk("MIC_TEST,fail,configure_rc=%d\n", ret);
		return ret;
	}
	printk("MIC_TEST,cfg,rate=%u,ch=%u\n",
	       cfg.streams[0].pcm_rate, cfg.channel.act_num_chan);

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret != 0) {
		printk("MIC_TEST,fail,trigger_rc=%d\n", ret);
		return ret;
	}

	int64_t sum_l = 0, sum_r = 0;
	uint32_t n_l = 0, n_r = 0;
	int16_t peak_l = 0, peak_r = 0;
	uint32_t captured_ms = 0;
	uint32_t frames = 0;

	while (captured_ms < CAPTURE_MS) {
		void *buf = NULL;
		uint32_t size = 0;

		ret = dmic_read(dmic_dev, 0, &buf, &size, READ_TIMEOUT_MS);
		if (ret != 0 || buf == NULL) {
			printk("MIC_TEST,fail,read_rc=%d\n", ret);
			(void)dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
			return (ret != 0) ? ret : -EIO;
		}

		const int16_t *s = (const int16_t *)buf;
		uint32_t pairs = size / (2U * sizeof(int16_t));

		for (uint32_t i = 0; i < pairs; i++) {
			int16_t l = s[2U * i];
			int16_t r = s[2U * i + 1U];

			sum_l += (int32_t)l * l;
			sum_r += (int32_t)r * r;
			if (l > peak_l) { peak_l = l; }
			if (-l > peak_l) { peak_l = (int16_t)-l; }
			if (r > peak_r) { peak_r = r; }
			if (-r > peak_r) { peak_r = (int16_t)-r; }
		}
		n_l += pairs;
		n_r += pairs;
		frames++;
		captured_ms += size / 64U; /* 64 字节 = 1ms 立体声 16bit */

		k_mem_slab_free(&rx_mem_slab, buf);
	}

	(void)dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);

	if (n_l == 0 || n_r == 0) {
		printk("MIC_TEST,fail,no_samples\n");
		return -EIO;
	}

	/* 均方近似 RMS（避免开方，输出 rms^2 与线性 RMS 估算） */
	uint32_t ms_l = (uint32_t)(sum_l / (int64_t)n_l);
	uint32_t ms_r = (uint32_t)(sum_r / (int64_t)n_r);

	printk("MIC_TEST,stat,frames=%u,n=%u,lms=%u,rms=%u,lpeak=%d,rpeak=%d\n",
	       frames, n_l, ms_l, ms_r, peak_l, peak_r);

	/* 判定：两路都必须有非零活动（恒 0 = 无时钟/数据；虚焊/LR 接反） */
	if (ms_l == 0 || ms_r == 0) {
		printk("MIC_TEST,fail,silent,lms=%u,rms=%u\n", ms_l, ms_r);
		return -EIO;
	}

	printk("MIC_TEST,PASS,lms=%u,rms=%u\n", ms_l, ms_r);
	return 0;
}

#else

int mic_bringup_test(void)
{
	return 0;
}

#endif /* CONFIG_APP_MIC_TEST */
