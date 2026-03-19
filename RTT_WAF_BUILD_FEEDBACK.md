# RTT 构建说明（根目录 SCons）

## 当前构建方式

RTT（RT-Thread）目标**不再使用 waf**，统一在 **pogo-apm 根目录** 用 **SCons** 完成配置、部署与编译。

- **BSP 唯一维护位置**：`libraries/AP_HAL_RTT/` 下的各 BSP（如 `rtt_bsp_pixhawk6c_mini`、`rtt_bsp_cuav_v5`）。
- **rt-thread 子模块保持纯净**：不在 `modules/rt-thread` 内常驻 BSP；构建时由 `rtt_bsp_deploy.py` 将 BSP 部署到 `build/rtt_deploy/<target>/` 再在该目录执行 SCons。
- **产物**：`build/rtt_deploy/<target>/rtthread.bin`、`rt-thread.elf`，并复制到 `build/rtt_<target>/rtthread.bin` 便于烧录。

---

## 支持的目标与别名

| 规范名         | 板级标识           | 可用的 --target 写法示例 |
|----------------|--------------------|---------------------------|
| `pixhawk6c_mini` | `rtt_pixhawk6c_mini` | `pixhawk6c-mini`、`pixhawk6c_mini`、`rtt_pixhawk6c_mini` 等 |
| `cuav_v5`      | `rtt_cuav_v5`     | `cuav-v5`、`cuav_v5`、`cuav v5`、`rtt_cuav_v5` 等 |

---

## 常用命令

```bash
# 编译（从仓库根目录执行）
scons --v=ArduCopter --target=pixhawk6c-mini
scons --v=ArduCopter --target=cuav_v5

# 清理指定 target 的部署与构建
scons -c --target=pixhawk6c-mini
scons -c --target=cuav_v5

# 编译后烧录（需连接飞控）
scons --v=ArduCopter --target=pixhawk6c-mini --upload
scons --target=cuav_v5 --upload

# 指定串口烧录
scons --target=pixhawk6c-mini --upload --port=/dev/ttyACM0
```

---

## 构建流程简述

1. **解析 target**：根据 `--target` 得到规范名（如 `pixhawk6c_mini`、`cuav_v5`）。
2. **Mavlink 头生成**：`Tools/scripts/rtt_mavgen.py` 生成头文件到 `build/<board>/libraries/GCS_MAVLink/include/mavlink/v2.0/`。
3. **BSP 部署**：`Tools/scripts/rtt_bsp_deploy.py` 将对应 BSP 全量拷贝到 `build/rtt_deploy/<canonical_target>/`。
4. **在部署目录执行 SCons**：以 `ARDUPILOT_FULL=1`、`AP_ROOT`、`RTT_ROOT` 等环境变量在该目录执行 SCons，生成 `rtthread.bin`、`rt-thread.elf`。
5. **复制固件**：将 `rtthread.bin` 复制到 `build/rtt_<target>/`；烧录时使用该目录下的 `.apj`/bin。

---

## 烧录与 bootloader 注意事项（Pixhawk6C Mini）

- 应用区 Flash 基址为 **0x08020000**（前 128KB 为 bootloader）。BSP 中已通过 `SCB->VTOR = 0x08020000` 及 `stm32h7xx_hal_conf.h` 中的 `USER_VECT_TAB_ADDRESS`/`VECT_TAB_BASE_ADDRESS` 正确设置向量表。
- 若烧录后 USB 串口不出现，需确认：VTOR 已在 `rt_hw_board_init` 最早阶段设置、`HAL_Init()` 已调用、板级 USB 中断（如 `OTG_FS_IRQHandler`）在 BSP 中有安全实现（见 `libraries/AP_HAL_RTT/rtt_bsp_pixhawk6c_mini/board/ports/cherryusb/usb_irq.c`）。

---

## 历史：waf 审查反馈（仅供参考）

以下为早期基于 **waf** 的 RTT 构建审查记录，当前主流程已改为上述根目录 SCons，仅作存档。

- **审查轮次**：Agent 5 第 1 轮及多轮后更新。
- **当时命令**：`./waf configure --board=rtt_fmuv2`、`./waf build`。
- **曾遇到的问题**：缺少 `stm32f4xx.h`/`stm32f4xx_hal.h`（需在 BSP 目录执行 `pkgs --update`）；RTT 静态库与 rtconfig.h 已处理；asm 输出路径与 BSP/packages 不一致等。
- **当前建议**：若仍需使用 waf 的 RTT 板型（如 `rtt_fmuv2`、stm32f427-robomaster-a），需在对应 `modules/rt-thread/bsp/stm32/...` 下执行 `pkgs --update` 再构建；**推荐优先使用根目录 `scons --target=pixhawk6c-mini` 或 `scons --target=cuav_v5`**。
