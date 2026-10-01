/**
 * @file app_watchdog.h
 * @brief 硬件看门狗兜底（wdt31）
 *
 * 背景（2026-10-01 SWD 实锤，见 docs/IMU采集测试方案.md §6.7）：
 *   手机断开连接后，MPSL 私有工作队列线程（协作优先级 -10）在
 *   Nordic 闭源库内部死锁（calibration_work 卡在 WFI 等待循环），
 *   饿死所有低优先级线程——sysworkq 里排队的广播重启 work 永远
 *   不执行、main 线程停摆，板子变成"不广播、无日志"的假死。
 *   应用层任何手段（重试、空闲看门狗）都救不了高优先级线程挂死，
 *   唯一可靠兜底是硬件看门狗：超时 -> 芯片复位 -> 恢复广播。
 *
 * 喂狗策略（活性门控，2026-10-01 二版）：main 主循环每拍调
 *   app_watchdog_kick() 仅递增心跳；sysworkq 周期 work（5s）检查心跳
 *   有变化才真正喂狗。实测证明：MPSL 死锁时 main 循环仍然活着（一版
 *   由 main 直接喂狗，永远喂得上，死锁不复位），sysworkq 才被饿死——
 *   所以喂狗权必须交给 sysworkq，且用 main 心跳做第二道门。
 *   - sysworkq 停摆 -> feed work 不执行 -> 30s 复位
 *   - main 停摆 -> 心跳不动 -> feed work 拒绝喂 -> 30s 复位
 * 调试安全：WDT_OPT_PAUSE_HALTED_BY_DBG——调试器 halt 时暂停计数，
 *   SWD 长时间读 RTT / 断点调试不会误触发复位。
 */
#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

/**
 * @brief 初始化并武装硬件看门狗（尽早调用，覆盖后续所有初始化）
 *
 * @return 0 成功；负值失败（非致命，仅打印，不阻断启动）
 */
int app_watchdog_init(void);

/** main 主循环每拍调用：仅递增心跳，真正喂狗由 sysworkq 门控执行 */
void app_watchdog_kick(void);

#endif /* APP_WATCHDOG_H */
