/**
 * @file pvdf_adc.c
 * @brief PVDF 压电信号链采集实现：ANALOG 域上电 + SAADC 三路读
 *
 * 详见 pvdf_adc.h 头注释与 docs/外设扩展方案_V1.05.md §4。
 *
 * 实现要点：
 *   - 懒初始化：首次 pvdf_read() 才上电 ANALOG 域 + adc_channel_setup，
 *     避免开机自检关闭时（量产省电）无谓初始化；
 *   - VDD_ANA 上电后等待 OPA4391 稳定（VBIAS 建立，R9/R10 100k 分压
 *     + C24 4.7uF 充电时间常数约 0.47s，取保守 600ms 仅首次等待，
 *     后续读不重等）；
 *   - 三路转换复用同一 adc_sequence，逐通道顺序采样；
 *   - DEGRADED 恢复：每次 read 重试一次 init，成功后自动回 READY。
 */
#include "pvdf_adc.h"

#if CONFIG_APP_PVDF_TEST

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "power_control.h"

/** &adc 节点的三路 channel@N（app.overlay：0=HEART/AIN1, 1=RAW/AIN6, 2=REF/AIN7） */
#define PVDF_ADC_NODE   DT_NODELABEL(adc)
#define CH_HEART_IDX    0U
#define CH_RAW_IDX      1U
#define CH_REF_IDX      2U
#define PVDF_CH_NUM     3U

/** OPA4391 上电稳定（VBIAS 建立：R=50k 等效 × C24 4.7uF ≈ 235ms 时间常数，
 *  5τ 保守取 1200ms；仅首次上电等待） */
#define PVDF_POWER_SETTLE_MS 1200U

/** VBIAS 合理窗口（VDD_ANA=3.0V 中点 ±20%） */
#define VBIAS_MIN_MV    1200
#define VBIAS_MAX_MV    1800

/** ADC 满量程（gain 1/6 × 0.6V ref → 3.6V），用于合理性检查 */
#define ADC_FULL_SCALE_MV 3600

static const struct device *const adc_dev = DEVICE_DT_GET(PVDF_ADC_NODE);

/* 通道 DT 描述（DT_CHILD 依次取 channel@0/1/2） */
static const struct adc_dt_spec ch_spec[PVDF_CH_NUM] = {
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
};

static struct {
	enum pvdf_adc_state state;
	bool inited;        /* 通道已 setup */
	bool power_waited;  /* 首次上电稳定等待已完成 */
} pvdf = {
	.state = PVDF_ADC_UNKNOWN,
	.inited = false,
	.power_waited = false,
};

/** 初始化（幂等）：ANALOG 上电 → 通道 setup。DEGRADED 恢复也走这里 */
static int pvdf_init(void)
{
	int ret;

	if (pvdf.inited) {
		return 0;
	}

	if (!device_is_ready(adc_dev)) {
		printk("PVDF,adc_not_ready\n");
		pvdf.state = PVDF_ADC_DEGRADED;
		return -ENODEV;
	}

	ret = power_domain_set(POWER_DOMAIN_ANALOG, true);
	if (ret != 0) {
		printk("PVDF,power_fail,rc=%d\n", ret);
		pvdf.state = PVDF_ADC_DEGRADED;
		return ret;
	}

	if (!pvdf.power_waited) {
		k_msleep(PVDF_POWER_SETTLE_MS);
		pvdf.power_waited = true;
	}

	for (uint8_t i = 0; i < PVDF_CH_NUM; i++) {
		ret = adc_channel_setup_dt(&ch_spec[i]);
		if (ret != 0) {
			printk("PVDF,ch%u_setup_fail,rc=%d\n", i, ret);
			pvdf.state = PVDF_ADC_DEGRADED;
			return ret;
		}
	}

	pvdf.inited = true;
	if (pvdf.state != PVDF_ADC_READY) {
		printk("PVDF,state->ready\n");
	}
	pvdf.state = PVDF_ADC_READY;
	return 0;
}

/** 单通道读原始码并换算 mV */
static int pvdf_read_channel(const struct adc_dt_spec *spec, int32_t *mv)
{
	int16_t buf;
	int ret;
	struct adc_sequence seq = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

	ret = adc_sequence_init_dt(spec, &seq);
	if (ret != 0) {
		return ret;
	}

	ret = adc_read_dt(spec, &seq);
	if (ret != 0) {
		return ret;
	}

	/* adc_raw_to_millivolts_dt：in-out 语义——输入原始码，就地转 mV
	 * （NCS v3.4.0 签名为 (spec, int32_t *valp)，两参） */
	*mv = (int32_t)buf;
	return adc_raw_to_millivolts_dt(spec, mv);
}

int pvdf_read(struct pvdf_sample *out)
{
	int32_t mv[PVDF_CH_NUM];
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}

	ret = pvdf_init(); /* DEGRADED 时自动重试 */
	if (ret != 0) {
		return ret;
	}

	for (uint8_t i = 0; i < PVDF_CH_NUM; i++) {
		ret = pvdf_read_channel(&ch_spec[i], &mv[i]);
		if (ret != 0) {
			printk("PVDF,read_fail,ch=%u,rc=%d\n", i, ret);
			return -EIO;
		}
	}

	out->heart_mv = mv[CH_HEART_IDX];
	out->raw_mv = mv[CH_RAW_IDX];
	out->ref_mv = mv[CH_REF_IDX];
	return 0;
}

int pvdf_bringup_test(void)
{
	struct pvdf_sample s;
	int ret;

	printk("PVDF_TEST,begin,channels=HEART(AIN1)/RAW(AIN6)/REF(AIN7)\n");

	ret = pvdf_init();
	if (ret != 0) {
		printk("PVDF_TEST,fail,init_rc=%d\n", ret);
		return ret;
	}

	/* 连续采 4 帧，VBIAS 必须在合理窗口（运放链路是否活着的直接证据） */
	for (uint8_t i = 0; i < 4; i++) {
		ret = pvdf_read(&s);
		if (ret != 0) {
			printk("PVDF_TEST,fail,read_rc=%d\n", ret);
			return ret;
		}
		k_msleep(20);
	}

	if (s.ref_mv < VBIAS_MIN_MV || s.ref_mv > VBIAS_MAX_MV) {
		printk("PVDF_TEST,fail,vbias=%dmV,expect=%d~%dmV\n",
		       s.ref_mv, VBIAS_MIN_MV, VBIAS_MAX_MV);
		return -ERANGE;
	}
	if (s.heart_mv < 0 || s.heart_mv > ADC_FULL_SCALE_MV ||
	    s.raw_mv < 0 || s.raw_mv > ADC_FULL_SCALE_MV) {
		printk("PVDF_TEST,fail,range,heart=%d,raw=%d\n",
		       s.heart_mv, s.raw_mv);
		return -ERANGE;
	}

	printk("PVDF_TEST,PASS,heart=%dmV,raw=%dmV,vbias=%dmV\n",
	       s.heart_mv, s.raw_mv, s.ref_mv);
	return 0;
}

enum pvdf_adc_state pvdf_adc_get_state(void)
{
	return pvdf.state;
}

#else /* !CONFIG_APP_PVDF_TEST */

int pvdf_bringup_test(void) { return 0; }

int pvdf_read(struct pvdf_sample *out)
{
	ARG_UNUSED(out);
	return -ENOTSUP;
}

enum pvdf_adc_state pvdf_adc_get_state(void) { return PVDF_ADC_UNKNOWN; }

#endif /* CONFIG_APP_PVDF_TEST */
