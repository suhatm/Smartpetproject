/**
 * @file mic_pdm.h
 * @brief 双 IM69D128 PDM 麦克风（U5 右/U6 左，pdm20 立体声）bring-up 自检
 */
#ifndef MIC_PDM_H
#define MIC_PDM_H

/**
 * @brief 麦克风 bring-up：上电(SENS 域) -> dmic 配置 16kHz/16bit/2ch
 *        -> 采集约 2s -> 左右声道分离 -> 各算 RMS 与峰值并判定
 *
 * @return 0 PASS；负值失败（阶段见 RTT 日志）
 */
int mic_bringup_test(void);

#endif /* MIC_PDM_H */
