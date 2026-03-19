# RT-Thread fmuv2 从零到可链接固件

## 1) fmuv2 专用 RTT BSP（仓库内独立维护）

- **MCU**：fmuv2 对应 **STM32F427**。
- **RTT 子模块路径**：`modules/rt-thread`（官方仓库子模块）。
- **BSP 源**：fmuv2 使用 **pogo-apm 仓库内独立维护**的 BSP：`libraries/AP_HAL_RTT/rtt_bsp_fmuv2`。不依赖 stm32f429-atk-apollo 或 stm32f427-robomaster-a。
- **部署**：waf 构建时，若 `RTT_ROOT/bsp/stm32/stm32f427-fmuv2` 不存在，会自动从 `rtt_bsp_fmuv2` 复制到该目录，以便 scons 在 RTT 标准 bsp 树下运行。
- **构建方式**：**waf 只调 scons**：在 BSP 目录执行 scons，再用 `ar` 将 `build/*.o` 打成 `librtthread.a`，waf 仅负责链接。不再在 waf 中从源编 RTT。

## 2) rtt.py 中的行为

- 当 **BOARD=rtt_fmuv2** 或 **fmuv2** 且 **RTT_ROOT** 指向 `modules/rt-thread` 时：
  - 先执行 **部署**：若 `RTT_ROOT/bsp/stm32/stm32f427-fmuv2` 不存在，从 `libraries/AP_HAL_RTT/rtt_bsp_fmuv2` 复制过去。
  - BSP 路径为 `RTT_ROOT/bsp/stm32/stm32f427-fmuv2`。
  - 若已存在 `librtthread.a`，则将其所在目录加入 `env.LIBPATH`。
  - 若不存在，则自动在 BSP 目录运行 `scons`，再用 `ar rcs librtthread.a` 打包 `build/*.o`，并将该目录加入 `env.LIBPATH`。
  - 若自动构建失败（如缺 packages、无 scons 等），会在日志中打印**手动步骤**，并仍将 BSP 目录加入 `LIBPATH`。

手动构建说明（`_manual_rtt_build_instructions()`）大致为：

1. `cd <RTT_ROOT>/bsp/stm32/stm32f427-fmuv2`
2. `export RTT_ROOT=<RTT_ROOT>`
3. （若 scons 报缺包）在 BSP 目录执行 `pkgs --update`
4. `scons`
5. `ar rcs librtthread.a $(find build -name '*.o')`

## 3) 从零到 fmuv2 RTT 固件可链接的最小操作清单

| 步骤 | 操作 |
|------|------|
| 1. 子模块 | `git submodule update --init modules/rt-thread` |
| 2. waf configure | `./waf configure --board rtt_fmuv2`（默认 `RTT_ROOT=modules/rt-thread`；首次会部署 BSP 到 `bsp/stm32/stm32f427-fmuv2`） |
| 3. 首次 BSP 依赖（若需要） | 若 scons 报缺包，在部署后的 BSP 目录执行：`cd $RTT_ROOT/bsp/stm32/stm32f427-fmuv2`，`pkgs --update` |
| 4. waf build | `./waf build`。若无 `librtthread.a`，会先在该 BSP 目录自动 scons+ar；失败则按日志中的手动步骤执行后再次 `./waf build`。 |

**依赖**：本机需已安装 **scons**、**arm-none-eabi-gcc**（与 waf 所用一致）。BSP 需要 RTT 软件包时，在部署后的 BSP 目录执行 `pkgs --update`。

**可选**：若交叉编译器不在 PATH，可设置 `RTT_EXEC_PATH` 为工具链目录；rtt.py 在 configure 时会尽量从 waf 的 `CC` 推断并设置 `RTT_EXEC_PATH`。
