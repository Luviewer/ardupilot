# attitude_from_thrust_vector 说明

本文说明 `AC_AttitudeControl::attitude_from_thrust_vector()` 的几何含义、坐标系/符号约定，以及与 `cal_alloc_tri.m` 的一致性。

## 1) 坐标系与符号约定

- **NED 地理系**（ArduPilot 常用）：`+X` 为北、`+Y` 为东、`+Z` 向下。
- **机体系**（Body frame）：`+X` 向前、`+Y` 向右、`+Z` 向下（与 NED 轴向一致，但随机体转动）。
- **推力矢量** `thrust_vector`：函数中按 **NED 地理系**理解。向上推力对应 `-Z` 方向（即 `z < 0`）。
- **航向角** `heading_angle_rad`：绕 `+Z`（向下）轴的偏航角（Yaw）。

> 备注：若你的推力矢量在机体系，需要先用当前姿态将其旋转到 NED 再传入。

## 2) 几何意义

该函数做两件事：

1. **把机体推力轴（默认指向 NED 的 `-Z`）旋转到期望推力方向** `thrust_vector`；
2. **再叠加航向（Yaw）角**，得到完整姿态四元数。

也就是说，它先“让推力方向对齐”，再“绕 Z 轴设定航向”。

## 3) 核心公式（与代码一致）

设：

- `u = [0, 0, -1]` 为“推力向上”的基准方向（NED 中向上）。
- `t = normalize(thrust_vector)` 为期望推力方向（若输入为零向量则用 `u`）。

则：

- 旋转轴（未归一化）：`a = u × t`
- 旋转角：`θ = acos( clamp(u · t, -1, 1) )`
- 若 `|a|` 或 `θ` 接近 0，则用 `a = u` 作为退化轴（避免数值问题）

用轴角生成四元数：

- `q_tv = quat(axis = a/|a|, angle = θ)`
- `q_yaw = quat(axis = [0,0,1], angle = heading_angle_rad)`

最终姿态四元数（与源码一致）：

- `q = q_tv * q_yaw`

> 注：四元数乘法顺序与具体库定义有关，这里严格跟随源码顺序 `thrust_vec_quat * yaw_quat`。

## 4) 与 cal_alloc_tri.m 的一致性

`cal_alloc_tri.m` 的推导使用 **机体系 Z 轴向下为正**（NED 风格），因此“向上推力”对应 **`Fz < 0`**。

`attitude_from_thrust_vector()` 中也使用 `u = [0,0,-1]` 作为“推力向上”的参考方向，二者符号一致：

- 上推力：`z < 0`
- 下为正：`z > 0`

因此在你的三旋翼倾转混控中，`Fx/Fz` 的符号约定与姿态/推力矢量控制是**自洽的**。

## 5) 相关源码位置

- 函数本体：`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
- 典型调用：`input_thrust_vector_heading_rad()` / `input_thrust_vector_rate_heading_rads()`
