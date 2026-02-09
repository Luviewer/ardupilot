# Repository Guidelines

## Project Structure & Module Organization
ArduPilot is a multi-vehicle codebase. Vehicle applications live in top-level
directories like `ArduCopter/`, `ArduPlane/`, `Rover/`, `ArduSub/`,
`AntennaTracker/`, and `Blimp/`. Shared C++ libraries live under `libraries/`
(commonly prefixed `AP_`), with hardware support under `libraries/AP_HAL*/`.
Developer tooling and scripts are in `Tools/`, third-party submodules in
`modules/`, documentation in `docs/`, and build outputs in `build/<board>/`.

## Build, Test, and Development Commands
- Configure a board once (avoid `sudo`): `./waf configure --board CubeBlack`
- Build a vehicle target: `./waf copter` or `./waf plane`
- List supported boards: `./waf list_boards`
- List build targets: `./waf list`
- Build a single target: `./waf --targets bin/arducopter`
- Run tests: `./waf check` (relevant tests) or `./waf check --alltests`
- SITL builds: `./waf configure --board sitl` then `./waf copter`

## Coding Style & Naming Conventions
Match the surrounding file’s style (indentation, braces, and naming) and avoid
reformatting unrelated code. Library modules are typically prefixed `AP_`, and
vehicle apps use CamelCase directory names (e.g., `ArduCopter/`). If you touch
`libraries/AP_DDS`, run the formatter: `./Tools/scripts/run_astyle.py`. The
AP_DDS README also references pre-commit hooks for Python/XML formatting.

## Testing Guidelines
Unit tests are built under the `tests` program group and can be invoked via
`./waf check` or a specific target like `./waf --targets tests/test_vectors`.
SITL and regression tests live under `Tools/autotest/`; follow that README for
scenario-specific runs.

## Commit & Pull Request Guidelines
Recent commits are short, single-line summaries (often English or Chinese) and
don’t enforce a prefix. Use a concise imperative subject and put details in the
body when needed. For pull requests, include a clear description, the tests you
ran (commands + results), and link any related issue. The main contributing
guide is linked from `README.md` if additional review requirements apply.

# 说明
- 每次回答都使用中文。

# AP_QuadRuped 代码阅读笔记（来自 `libraries/AP_QuadRuped/`）

## 1) 调用/依赖的主要库与模块
- AP_HAL / AP_HAL_Boards：硬件抽象、时间（`AP_HAL::millis()`）、RC输入、控制台输出等。
- AP_AHRS_View：姿态解算（roll/pitch/gyro/accel），用于平衡与姿态控制。
- AP_Motors：电机/伺服系统接口（armed状态等）。
- AP_Param：参数系统（配置、保存、加载、分组）。
- AP_Math：向量/四元数/数学工具（向量、欧拉角、约束等）。
- AC_PID：姿态控制 PID（roll/pitch）。
- AC_TD：输入微分/滤波（roll/pitch 平滑）。
- AP_RangeFinder：测距传感器（飞行姿态/爪形态切换逻辑中用到）。
- AP_RCMapper、RC_Channel：遥控通道映射与读取。
- GCS_MAVLink：MAVLink 通信与自定义状态上报。
- AP_DroneCAN + SRV_Channels：CAN 总线伺服指令与PWM输出。

## 2) 整体架构与控制流程
- **前端控制器：** `AP_QuadRuped` 统一管理参数、RC输入、模式切换与后端步态切换。
- **后端接口：** `AP_QuadRuped_Backend` 定义通用运动学、轨迹生成、PWM输出与硬件发送。
- **具体步态后端：**
  - `AP_QuadRuped_Diag`：对角步态（trot）。
  - `AP_QuadRuped_Wave`：波浪步态（单腿依次抬起）。
  - `AP_QuadRuped_Wave_COG`：波浪步态 + 重心偏移。
  - `AP_QuadRuped_HengXiang`：横向工字步态（侧向为主）。
  - `AP_QuadRuped_ZongXiang`：纵向工字步态（前后为主，继承横向实现并改轴向）。
- **主循环：** `AP_QuadRuped::update()`
  1) 检查使能与后端有效性。
  2) MAVLink状态上报。
  3) 读取RC输入并更新模式/控制量。
  4) 按后端步态频率调度 `backend->update()`。

## 3) 运动学与轨迹生成
- **逆运动学：** `AP_QuadRuped_Backend::leg_inverse_kinematics()` 使用三连杆（COXA/FEMUR/TIBIA）几何计算关节角度。
- **正向运动学（机体补偿）：** `body_forward_kinematics()` 将步态位移 + 机体框架 + 重心偏移 + roll/pitch/yaw 旋转后得到目标腿端位置。
- **轨迹模式：** 摆线轨迹（cycloid）与三次贝塞尔轨迹两种模式可选（后端参数控制）。
- **步态相位：**
  - 对角步态：对角腿同相位，另一对腿相位差半周期。
  - 波浪/工字步态：单腿依次摆动，采用 1/4 或 1/8 周期摆动比例。
- **偏航轨迹：** `yaw_trajectory_generation()` 通过相位分段给每条腿一个平滑的yaw补偿。
- **重心偏移（COG）：**
  - `Wave_COG`/`HengXiang` 通过相位生成重心偏移，减少单腿摆动时的倾覆风险。

## 4) 控制输入与模式
- RC 输入转为标准化控制量（-1~1）：前后/横移/偏航 + roll/pitch 由 TD 滤波器平滑。
- **主模式：** 行走模式 / 飞行模式（特殊姿态/爪形态）。
- **飞行子模式：** 普通飞行、纵向爪、横向爪；通过测距高度决定收腿/展开。

## 5) 伺服输出与硬件接口（实物部分）
- 输出流程：`main_inverse_kinematics()` → `output_leg_angle()` 计算PWM → `send_servo_cmd()`
- **CAN伺服指令：** 使用 `AP_DroneCAN` 广播 `com_usl_ServoCmd`。
- **本地PWM输出：** `SRV_Channels::set_output_pwm()` 同步设置每个关节的PWM通道。
- **参数校准：** `AP_QuadRuped_Params` 提供每条腿关节方向、偏移量。
- **机体尺寸参数：** `AP_QuadRuped_SYS_Params` 定义腿长、机身尺寸，用于运动学。

## 6) 仿真部分（SITL/Gazebo）
- 代码侧仿真/实物共用同一套控制逻辑与后端；差别由 HAL/SITL 和传感器/执行器模型提供。
- `readme.md` 提到仿真目录 `qruped_sim`，示例启动方式：
  - `cd qruped_sim`
  - `../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --add-param=quadruped.param`
- 在仿真中，RC/RangeFinder/Motors 由 SITL/Gazebo 模型提供；PWM/伺服命令会映射到模拟执行器。

## 7) 实物 vs 仿真关键差异（我的理解）
- **实物：** DroneCAN 总线发送伺服命令，真实 RC、IMU、测距传感器参与闭环；参数需要与真实机械结构一致。
- **仿真：** 相同代码驱动 SITL 的模拟传感器/执行器，重点验证步态时序、轨迹与控制逻辑。
