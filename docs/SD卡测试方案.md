# SD 卡（CSNP1GCR01-BOW）测试方案

> 对应分支 task-V1.06 · 配套上位机 `host/petring_console_demo.py` v0.3「SD 卡测试」页签
> 芯片：CSNP1GCR01-BOW —— CS（创世）SD NAND，128MB（1Gbit SLC），LGA-8(6×8mm)，
> 标准 SD 2.0 协议，支持 SDIO / SPI 4 线，内置 ECC + 坏块管理 + 磨损均衡，
> 免驱动（行为等同 TF 卡），支持 FAT 文件系统，擦写寿命 5~10 万次。

## 0. 结论

**是的，推荐加"自己写数据 → 再读出来比对"的自测。** 这是验证焊接、SPI 走线、
电源（STORE 域）、驱动配置最直接的手段。分五层做，由底到上：

## 1. 识别层（先确认"看得见"）

- STORE 域上电 → SPI 初始化 → SD 初始化序列（CMD0 → CMD8 → ACMD41 → CMD2/CMD3）。
- 读 CID/CSD 寄存器，核对**容量 = 128MB**（CSD 计算值），不符即判失败。
- Zephyr 侧：`CONFIG_DISK_DRIVER_SDMMC=y` + `CONFIG_SDMMC_OVER_SPI=y`，
  overlay 里挂 `sdhc@x { status="okay"; spi-max-frequency = <25000000>; mmc { ... }; };`
- 期望现象：驱动 probe 成功，日志打印卡容量/块大小。

## 2. 裸块读写层（核心自测）

- 选测试区（避开文件系统区，或先卸载）：写入已知图样 → 读回 → **逐字节比对**。
- 图样建议 4 种轮换：`0x55`、`0xAA`、地址递增 `(addr & 0xFF)`、伪随机（LFSR 种子固定）。
- 块大小 512B，每次测试 1~4MB（demo 默认 4MB），记录错误块数与首错地址。
- 这一步能抓：虚焊/连锡（全错）、数据线错位（图样错位规律）、时序过快（降频后错误消失）。

## 3. 文件系统层

- `CONFIG_FAT_FILESYSTEM_ELM=y`，挂载 `/SD:`，格式化 FAT32。
- 建文件写 N KB → 关闭 → 重开读回 → CRC32 比对。
- **断电保持测试**：写完断电重启，重挂载后读回校验（验证掉电安全）。
- 录音应用路径：`/REC/REC_xxxx.WAV` 创建/追加/关闭/列举/删除全流程。

## 4. 性能层（参考值）

- SPI 25MHz 模式下实测预期：连续读 ~2.5~3 MB/s，连续写 ~1~1.5 MB/s
  （SDIO 模式才能到 Class10 的 23.5/12.3 MB/s，nRF54L 无 SDIO 外设，用 SPI 即可）。
- 测速口径：固定大小连续写/读计时，报 MB/s，低于 0.5MB/s 判异常（检查 SPI 分频配置）。

## 5. 应用层（随系统联调）

- 录音落盘：REC_CTRL 开始后持续写 WAV，停止后文件大小 = 时长×32KB/s×声道数 ±1 块。
- 低电/断电保护：录音中途断电，文件可关闭或标记损坏，不拖垮文件系统。
- 满盘处理：剩余空间不足时按策略删最旧文件或停止录音并上报。

## 6. BLE 协议（V0.5 提案，待评审冻结）

| 指令 | 名称 | 参数 | 应答/事件 |
|---|---|---|---|
| 0x0D | SD_TEST | size_mb(u8), verify(u8) | ACK 立即返回；完成后 EVENT 上报 result/w_speed/r_speed |
| 0x09/0x0A/0x0B | REC_LIST / REC_READ / REC_DELETE | file_id | 沿用 V0.4 已定义 |

- 测试结果语义：result=0 通过；result=-5 校验失败（附错误块数）。
- SD 不在位（MODULE_STATUS SD=ABSENT）时 SD_TEST 应答 result=-2。

## 7. 上位机 demo 操作（v0.3）

1. 「SD 卡测试」页签可见芯片信息（128MB / SPI 4 线 / FAT32 / 内置 ECC）。
2. 设测试大小（1~64MB）→ 勾"写后读回校验"→「开始测试」：
   进度条前半=写入、后半=读回；结束显示写速/读速/校验结果（绿 PASS / 红 FAIL）。
3. 勾"注入校验错误（演示）"可演示 FAIL 路径。
4. 「模拟插/拔 SD 卡」演示 ABSENT：不在位时测试与录音均被拒绝并记日志。
5. 录音完成后文件自动出现在「SD 录音文件」列表，可下载（0x0A）/删除（0x0B）。
6. 指令终端快捷按钮 `SD_TEST`（CMD 0x0D）可直接发指令看 ACK。

## 8. 固件落地清单（task-V1.06 后续）

- [ ] `src/hardware/sd_store.c`（头文件已立）：SDMMC-over-SPI 初始化 + FATFS 挂载
- [ ] prj.conf：`CONFIG_DISK_DRIVER_SDMMC/SDMMC_OVER_SPI/FAT_FILESYSTEM_ELM/FS_FATFS_MOUNT_MKFS`
- [ ] overlay：SD 的 SPI 节点 + CS 引脚 + `spi-max-frequency=<25000000>`（从 8MHz 起步验证再提频）
- [ ] `sd_test()`：图样写→读→比对 + 测速，结果经 sensor_hub 应答/事件上报
- [ ] 上板验证矩阵：识别 → 4 图样裸块 → FAT 断电保持 → 录音落盘 → 满盘
