# AGENTS.md — 项目规范（AI 助手与协作者必读）

> 本文件面向在本仓库工作的 AI 编码助手与人类协作者，规定了开发环境、Git 工作流与交付标准。
> 任何代码改动都必须遵守以下四条核心规范。

## 1. 开发环境与工具链

| 项目 | 配置 |
|---|---|
| IDE | **VS Code** + nRF Connect for Visual Studio Code 扩展 |
| SDK | nRF Connect SDK **v3.4.0**（`C:\ncs\v3.4.0`，Zephyr 4.4） |
| 工具链 | **v3.4.0 配套工具链**（`C:\ncs\toolchains\dcbdc366a1`） |
| 构建目标 | `nrf54l15dk/nrf54l15/cpuapp` |
| 烧录器 | **J-Link（不做限制：不限探针序列号，任意已连接的 J-Link 均可）** |
| 烧录接口 | **SW（SWD）烧录** |
| 下载速度 | **100 kHz** |
| 控制台 | SEGGER RTT（与烧录共用同一 J-Link 探针） |

VS Code 工程配置已写入 `.vscode/settings.json`（SDK 路径、工具链路径、west 环境变量）。
若 SDK / 工具链路径发生变化，须同步更新该文件。

## 2. Git 提交规范（强制）

每次改动完成后，**必须**创建一个对应的 git commit，以便后续追踪和回滚。

- 一次改动 = 一个 commit：不要把多个无关改动混进同一 commit，也不要改动后长时间不提交。
- commit message 用祈使句简述改动（例：`Add battery monitoring shell command`），便于回溯定位。
- 严禁跳过钩子提交（`--no-verify`）；严禁改写已推送的历史提交。

## 3. 测试与交付标准（强制）

每次改动后**必须**编写或更新相关测试，并在交付给用户前**确保所有测试和验证全部通过**。

- 嵌入式改动的最低验证门槛：`west build`（或对已有 build 目录 `ninja -C <build目录>` 增量编译）
  无错误、无新增警告。
- 涉及外设 / 驱动 / 硬件行为的改动，须在目标板（`nrf54l15dk/nrf54l15/cpuapp`）上完成烧录验证，
  并通过 SEGGER RTT 日志确认行为符合预期后，方可视为通过。
- 测试不通过不得交付；若因客观条件无法立即验证，必须在交付说明中明确标注未验证项与影响范围。

## 4. 资源占用报告（强制）

固件编译通过后，**必须**报告 FLASH / RAM 资源占用情况（取自编译输出末尾的统计行）：

- 格式示例：`FLASH 2.49% / RAM 4.79%`；
- 在交付说明、验证记录及 commit 信息中均可引用，便于跟踪固件体积变化趋势；
- 若某次改动导致占用显著增长（如 FLASH 增幅超过 1 个百分点），需在交付说明中说明原因。

## 5. 编译 / 烧录 / 推送标准流程（强制，勿自行摸索）

> 以下命令均已实测验证。AI 助手每次会话的 shell 状态不保留（PATH、环境变量每次归零），
> 因此**每条命令必须在同一条命令内先 export PATH 再执行**，不要指望上一次的设置仍然生效。
> 严格按照本节执行，不要尝试其他编译方式，避免重复试错。

### 5.1 环境前置（每条编译命令都要带）

```bash
export PATH="/c/ncs/toolchains/dcbdc366a1/opt/bin:/c/ncs/toolchains/dcbdc366a1/opt/bin/Scripts:$PATH"
```

缺少此步会报 `'ccache' 不是内部或外部命令`（链接器用 ccache 前缀）。

### 5.2 编译

- **日常增量编译（首选，最快）**：

  ```bash
  export PATH="/c/ncs/toolchains/dcbdc366a1/opt/bin:/c/ncs/toolchains/dcbdc366a1/opt/bin/Scripts:$PATH" && ninja -C build
  ```

  sysbuild 顶层 build.ninja 会级联构建主应用。

- **全量重建（仅以下情况使用）**：改板型/大幅改 prj.conf 或 app.overlay 后；或报
  Kconfig `malformed string literal` / `Aborting due to Kconfig warnings` 时
  （CMakeCache 被 `-DCONFIG_XXX=` 无引号条目污染，删条目会复活，唯一解法是 pristine 重建）：

  ```bash
  export PATH="/c/ncs/toolchains/dcbdc366a1/opt/bin:/c/ncs/toolchains/dcbdc366a1/opt/bin/Scripts:$PATH" && export ZEPHYR_BASE="C:/ncs/v3.4.0/zephyr" && west build --pristine=always -b nrf54l15dk/nrf54l15/cpuapp -d build
  ```

- **产物路径（sysbuild 结构，注意不是 build/zephyr/）**：
  `build/Smartpetproject/zephyr/zephyr.elf`（烧录用同目录 `merged.hex`）。

### 5.3 烧录与 RTT

- 烧录/复位用 nrfutil device 系列（SWD，任意 J-Link）：
  `nrfutil device reset`、`nrfutil device program --firmware build/Smartpetproject/zephyr/merged.hex`。
- **坑：烧新固件后 RTT 日志静默 ≠ 固件没跑**。`.rtt_buff_data`（NOLOAD 段）跨复位存活，
  旧固件留下的满缓冲区会让新固件日志被 NO_BLOCK_SKIP 丢弃。修法：从 map 文件取
  `_SEGGER_RTT` 地址，SWD 清零控制块首字再复位：

  ```bash
  nrfutil device write --address <_SEGGER_RTT地址> --value 0 --direct && nrfutil device reset
  ```

  判断固件是否真在跑：看控制块 aUp[0].sName 指针是否指向当前固件 flash 的字符串地址。
- 寄存器直读（批量读勿超 256B，会超时）：
  `nrfutil device read --address <addr> --bytes 4 --width 32 --direct --family nrf54l`。

### 5.4 git push（GitHub 需走本机代理）

沙箱代理会拦截 github.com 主站（502/Empty reply），推送必须显式走 v2rayN 代理并
unset 环境代理变量（凭据已存 Git Credential Manager，无需交互）：

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY git -c http.proxy=http://127.0.0.1:10808 push origin task-V1.03
```

### 5.5 上下文卫生

- 编译日志一律 `| tail -N` 截取末尾（资源统计行在末尾），不要把全量编译输出灌进对话。
- 根目录的 `build_*.log` / `*_log.txt` 等历史日志不要读取；新日志写入系统临时目录或
  `tools/`，不要在仓库根目录继续堆积。
