/**
 * @file status_led.h
 * @brief 状态 LED 接口：通过 nPM1300 LEDDRV 寄存器控制红/蓝两灯
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief 设置红/蓝灯组合状态（带变化检测，减少 I2C 流量）
 *
 * @param red true 点亮 LED0（红）
 * @param blue true 点亮 LED1（蓝）
 * @return 0 成功；-ENODEV 未初始化；其他负值写失败
 */
int status_led_set(bool red, bool blue);

/**
 * @brief 读取当前实际生效的 LED 状态（status_led_set 缓存值）
 *
 * 主循环每 20ms 调 status_led_set 刷新，缓存即当前物理输出。
 * MODULE_STATUS 帧上报 LED 实况用（上报真实值而非 override 请求值，
 * 避免充电模式等"override 未生效"场景下 UI 与实灯不一致）。
 *
 * @param red 输出 LED0（红）当前状态
 * @param blue 输出 LED1（蓝）当前状态
 * @return 0 成功；-ENODEV 尚未初始化（缓存无效）
 */
int status_led_get(bool *red, bool *blue);

/**
 * @brief 阻塞式脉冲：点亮 on_ms，再熄灭，可选再等待 off_ms
 *
 * @return 0 成功；负值 LED 设置失败
 */
int status_led_pulse(bool red, bool blue, uint32_t on_ms, uint32_t off_ms);

/**
 * @brief 阻塞式闪烁：指定通道按 on/off 半周期闪烁指定总时长
 *
 * 产品方案的 3 秒指示图案（开机红闪/关机蓝闪/就绪双闪）共用此 API：
 *   - 开机：status_led_blink(true,  false, 3000, 300, 300)
 *   - 关机：status_led_blink(false, true,  3000, 300, 300)
 *   - 就绪：status_led_blink(true,  true,  3000, 300, 300)
 *
 * @param red 红灯参与闪烁
 * @param blue 蓝灯参与闪烁
 * @param duration_ms 闪烁总时长
 * @param on_ms 亮半周期
 * @param off_ms 灭半周期
 * @return 0 成功；负值 LED 设置失败
 */
int status_led_blink(bool red, bool blue, uint32_t duration_ms,
		     uint32_t on_ms, uint32_t off_ms);

/**
 * @brief LED 模块初始化：两通道切换为主机控制模式并熄灯
 *
 * @param pmic PMIC 设备句柄
 * @return 0 成功；-ENODEV 设备未就绪；其他负值失败
 */
int status_led_init(const struct device *pmic);

#endif /* STATUS_LED_H */
