/**
 * @file recorder.h
 * @brief 录音与文件管理（task-V1.06，协议 REC_* 指令 + 0x07 AUDIO_FILE 帧）
 *
 * 功能：
 *   - REC_CTRL(0x07)：双 PDM 麦 -> WAV（PCM 16kHz/16bit）落盘 SD；
 *   - 录音/监听期间上报 0x06 MIC RMS 帧（电平监视，2Hz）；
 *   - REC_LIST(0x09)/REC_READ(0x0A)/REC_DELETE(0x0B) 文件管理；
 *   - REC_READ 支持 offset 断点续传（V0.5 决策：offset 重发起）；
 *   - SD_TEST(0x0D)：在录音线程执行 SD 自测，避免阻塞主循环。
 *
 * 文件命名：/SD:/REC/REC%04u.wav，file_id = 序号（1~9999）。
 */
#ifndef RECORDER_H
#define RECORDER_H

#include <stdbool.h>
#include <stdint.h>

/** REC_CTRL / REC_READ 错误码（协议 §9 子集） */
#define REC_OK            0
#define REC_ERR_BUSY     -11 /* 录音/传输/测试进行中 */
#define REC_ERR_NO_CARD   -2 /* SD 不在位（模块 ABSENT） */
#define REC_ERR_NO_SPACE  -9 /* 存储不足 */
#define REC_ERR_NO_FILE  -13 /* file_id 不存在 */
#define REC_ERR_PARAM     -1 /* 参数错误 */
#define REC_ERR_IO        -5 /* IO 失败 */

/** REC_LIST 单条记录：file_id(2) + duration_s(2) + size_kb(2) */
#define REC_LIST_ENTRY_LEN 6U
#define REC_LIST_MAX       20U

/** 录音线程 SD_TEST 完成回调（bridge 注册，用于 EVENT 上报） */
struct sd_test_result;
typedef void (*rec_sd_test_done_cb)(const struct sd_test_result *res);

/**
 * @brief 初始化录音模块（创建工作线程）
 */
int recorder_init(void);

/**
 * @brief 开始录音
 *
 * @param ch_mask    bit0=左 bit1=右（至少一路）
 * @param duration_s 1~3600；0=不限（手动停止）
 * @return REC_OK / REC_ERR_*
 */
int recorder_start(uint8_t ch_mask, uint16_t duration_s);

/**
 * @brief 手动停止录音（未在录音返回 REC_ERR_PARAM）
 */
int recorder_stop(void);

/** @brief 是否正在录音 */
bool recorder_active(void);

/** @brief 录音/监听/文件传输任一进行中（电源域断电互斥判断用） */
bool recorder_busy(void);

/** @brief 开始 MIC 电平监听（不录音，仅 0x06 RMS 帧；SENSOR_EN(MIC,1) 用） */
int recorder_monitor_start(void);

/** @brief 停止 MIC 电平监听 */
int recorder_monitor_stop(void);

/**
 * @brief 枚举录音文件
 *
 * @param buf      输出缓冲（REC_LIST_ENTRY_LEN 的倍数）
 * @param max_len  缓冲长度
 * @param out_len  实际写入长度
 * @return REC_OK / REC_ERR_NO_CARD
 */
int recorder_list(uint8_t *buf, uint16_t max_len, uint16_t *out_len);

/**
 * @brief 开始上传录音文件（0x07 AUDIO_FILE 帧流，完成发 EVENT 0x06）
 *
 * @param file_id 文件标识
 * @param offset  断点续传起点（0=从头；协议 V0.5：丢块后按 offset 重发）
 * @return REC_OK（ACK 后开始帧流）/ REC_ERR_*
 */
int recorder_read_start(uint16_t file_id, uint32_t offset);

/**
 * @brief 删除录音文件
 */
int recorder_delete(uint16_t file_id);

/**
 * @brief 请求执行 SD_TEST（异步，完成经回调上报）
 *
 * @param size_mb 测试数据量
 * @param verify  是否读回校验
 * @param cb      完成回调（录音线程上下文）
 * @return REC_OK / REC_ERR_BUSY / REC_ERR_NO_CARD
 */
int recorder_sd_test(uint8_t size_mb, uint8_t verify, rec_sd_test_done_cb cb);

/** @brief 上传/录音是否进行中（桥接层互斥判断用） */
bool recorder_xfer_active(void);

#endif /* RECORDER_H */
