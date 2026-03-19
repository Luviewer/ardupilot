# STM32F427 fmuv2 专用 RTT BSP

## 简介

本 BSP 为 **fmuv2** 专用，在 pogo-apm 仓库内**独立维护**，不依赖 stm32f429-atk-apollo 或 stm32f427-robomaster-a。基于 STM32F427 与 RT-Thread 官方 stm32 BSP 结构，用于与 ArduPilot waf 构建流程配合：**waf 调 scons 先编 RTT**，得到 `librtthread.a` 后由 waf 链接。

## 使用方式（与 waf 配合）

1. **部署**：waf 构建时会自动将本 BSP 复制到 `RTT_ROOT/bsp/stm32/stm32f427-fmuv2`（仅当目标目录不存在时）。
2. **首次构建前**：在部署后的 BSP 目录执行一次 `pkgs --update`（若 scons 报缺包再执行）：
   ```bash
   cd $RTT_ROOT/bsp/stm32/stm32f427-fmuv2
   pkgs --update
   ```
3. **编译**：在 pogo-apm 根目录执行 waf 构建（如 `waf configure --board rtt_fmuv2` 与 `waf build`）。waf 会在该 BSP 目录调用 scons 生成 `librtthread.a` 并链接。

## 硬件

- MCU：STM32F427IIH6（与 robomaster-a 同芯片），主频 180MHz，2048KB FLASH，256KB RAM。
- 本 BSP 源文件从 stm32f427-robomaster-a 复刻而来，板级代码与链接脚本可直接沿用；若 fmuv2 硬件有差异，可在本目录下单独修改 `board/` 等并提交到 pogo-apm。

## 维护说明

- BSP 源位于：`libraries/AP_HAL_RTT/rtt_bsp_fmuv2/`。
- 不复制 `build/`、`packages/`、IDE 工程文件；`packages` 由用户在部署后的目录中通过 `pkgs --update` 获取。

### 其他板子复用本 BSP

- **仍用本 BSP（stm32f427-fmuv2）**：在 [Tools/ardupilotwaf/rtt.py](Tools/ardupilotwaf/rtt.py) 的 `RTT_BSP_MAP` 中增加「板名 → `stm32/stm32f427-fmuv2`」即可；新板子共用同一 BSP 与部署目录。
- **需独立 BSP**：拷贝本目录（`rtt_bsp_fmuv2`）为新 BSP 名（如 `rtt_bsp_xxx`），修改 `board/` 与 hwdef 对应关系；在 `rtt.py` 中增加「板名 → `stm32/新BSP名`」（部署目标为 `RTT_ROOT/bsp/stm32/新BSP名`），并在 rtt.py 中为该板配置 BSP 源路径（类似 `RTT_BSP_FMUV2_SRC`）。部署与 `pkgs --update` 流程同本 BSP。
