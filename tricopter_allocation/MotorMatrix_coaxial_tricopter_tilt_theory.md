# 共轴三旋翼倾转电机混控理论依据

## 0. 概述

本文档描述了共轴三旋翼倾转电机的混控理论依据。该设计包含三个电机位置（前左、前右、后置），每个位置配备共轴双桨（上下两个电机，旋转方向相反）。**所有三个电机均可通过舵机实现倾转**，用于增强pitch控制能力。

**重要说明**：
- 倾转角度通过pitch控制输出计算，而非从舵机读取（舵机一般无法读取当前角度）
- 倾转角度动态影响roll、pitch、throttle混控因子
- 根据MATLAB推导的控制分配矩阵，所有混控因子都考虑了倾转的影响

## 1. 坐标系定义

### 1.1 机体坐标系（Body Frame）

采用右手坐标系，符合ArduPilot标准：
- **X轴**：机头方向（向前为正）
- **Y轴**：右侧方向（向右为正）
- **Z轴**：向下方向（向下为正）

### 1.2 电机位置定义

设机体坐标系原点O在重心位置，电机位置如下：

- **前左电机对**（FL）：位置 `(x_FL, y_FL, 0)`，其中 `x_FL > 0`，`y_FL < 0`
- **前右电机对**（FR）：位置 `(x_FR, y_FR, 0)`，其中 `x_FR > 0`，`y_FR > 0`
- **后电机对**（R）：位置 `(x_R, y_R, 0)`，其中 `x_R < 0`，`y_R = 0`

对于对称布局的三旋翼：
```
x_FL = x_FR = x_F > 0  （前电机X坐标）
y_FL = -y_FR = -y_F < 0  （前左Y坐标为负）
x_R < 0  （后电机X坐标为负）
y_R = 0  （后电机在Y轴上）
```

## 2. 推力与力矩分析

### 2.1 单个电机的推力模型

对于第 `i` 个电机，其推力向量为：

```
F_i = [0, 0, -T_i]^T
```

其中 `T_i` 为推力大小（向上为正，Z轴向下，所以推力为负Z方向）。

### 2.2 力矩计算

根据刚体力学，电机产生的力矩为：

```
τ_i = r_i × F_i
```

其中 `r_i` 为电机位置向量，`F_i` 为推力向量。

展开为：

```
τ_i = |  i    j    k   |
      | x_i  y_i   0   |
      |  0    0  -T_i  |
    
    = [y_i·T_i, -x_i·T_i, 0]^T
```

因此：
- **Roll力矩**：`τ_roll = y_i · T_i`
- **Pitch力矩**：`τ_pitch = -x_i · T_i`
- **Yaw力矩**：`τ_yaw = 0`（假设无倾转时）

### 2.3 倾转电机的推力分解

当电机倾转角度为 `θ` 时（绕Y轴旋转，向前倾转为正），推力向量变为：

```
F_tilt = T · [sin(θ), 0, -cos(θ)]^T
```

在机体坐标系中：
- **X方向分量**：`F_x = T · sin(θ)` （产生pitch力矩）
- **Z方向分量**：`F_z = -T · cos(θ)` （产生升力）

### 2.4 倾转电机的力矩

对于倾转电机，位置向量 `r = [x, y, 0]^T`，推力向量 `F_tilt = T · [sin(θ), 0, -cos(θ)]^T`：

```
τ_tilt = r × F_tilt

      = |  i    j    k   |
        |  x    y    0   |
        | sin(θ) 0 -cos(θ) | · T

      = T · [y·cos(θ), x·cos(θ), -y·sin(θ)]^T
```

因此：
- **Roll力矩**：`τ_roll = y · T · cos(θ)` （倾转减小roll控制能力）
- **Pitch力矩**：`τ_pitch = x · T · cos(θ) + 0 · T · sin(θ) = x · T · cos(θ)` （主要来自X方向推力分量）
- **Yaw力矩**：`τ_yaw = -y · T · sin(θ)` （倾转产生额外yaw力矩，通常很小）

**关键发现**：倾转电机的pitch力矩主要来自X方向推力分量，与 `sin(θ)` 成正比。

## 3. 混控矩阵推导

### 3.1 控制输入

定义控制输入向量：

```
u = [τ_roll_demand, τ_pitch_demand, τ_yaw_demand, T_total]^T
```

其中：
- `τ_roll_demand`：期望roll力矩
- `τ_pitch_demand`：期望pitch力矩
- `τ_yaw_demand`：期望yaw力矩
- `T_total`：总推力需求

### 3.2 电机输出

定义电机推力向量：

```
T = [T_0, T_1, T_2, T_3, T_4, T_5]^T
```

其中：
- `T_0, T_1`：前左共轴对（上、下）
- `T_2, T_3`：前右共轴对（上、下）
- `T_4, T_5`：后共轴对（上、下）

### 3.3 混控矩阵

根据力矩平衡方程：

```
τ_roll = Σ(y_i · T_i · cos(θ_i))
τ_pitch = Σ(x_i · T_i · cos(θ_i) + F_x_i)
τ_yaw = Σ(τ_yaw_i)
T_total = Σ(T_i · cos(θ_i))
```

对于共轴三旋翼，假设：
- 前左和前右倾转角度：`θ_FL = θ_FR = θ`
- 后电机无倾转：`θ_R = 0`

展开为矩阵形式：

```
[τ_roll  ]   [y_FL  y_FL  y_FR  y_FR   0    0  ]   [T_0]
[τ_pitch ] = [x_FL  x_FL  x_FR  x_FR  x_R  x_R] · [T_1] · cos(θ)
[τ_yaw   ]   [  0     0     0     0    0    0  ]   [T_2]
[T_total ]   [  1     1     1     1    1    1  ]   [T_3]
                                                      [T_4]
                                                      [T_5]
```

加上倾转产生的X方向推力分量：

```
[τ_pitch] += [sin(θ)  sin(θ)  -sin(θ)  -sin(θ)  0  0] · [T_0]
                                                          [T_1]
                                                          [T_2]
                                                          [T_3]
                                                          [T_4]
                                                          [T_5]
```

**注意**：前右电机的X方向推力为负（因为倾转方向与前左相反）。

### 3.4 归一化混控因子

为了简化控制，将混控矩阵归一化。定义混控因子：

```
M = [M_roll, M_pitch, M_yaw, M_throttle]^T
```

对于前左电机（电机0和1）：
- **Roll因子**：`M_roll = y_FL / y_max = -0.5`（假设对称布局）
- **Pitch因子**：`M_pitch = (x_FL · cos(θ) + sin(θ)) / x_max`
- **Yaw因子**：`M_yaw = 0`（共轴对不产生净yaw）
- **Throttle因子**：`M_throttle = 1.0`

对于前右电机（电机2和3）：
- **Roll因子**：`M_roll = y_FR / y_max = +0.5`
- **Pitch因子**：`M_pitch = (x_FR · cos(θ) - sin(θ)) / x_max`（注意负号）
- **Yaw因子**：`M_yaw = 0`
- **Throttle因子**：`M_throttle = 1.0`

对于后电机（电机4和5）：
- **Roll因子**：`M_roll = 0`（在Y轴上）
- **Pitch因子**：`M_pitch = x_R / x_max = 0.5`（固定值）
- **Yaw因子**：`M_yaw = 0`
- **Throttle因子**：`M_throttle = 1.0`

## 4. 倾转角度与Pitch因子的关系

### 4.1 简化假设

在小角度倾转时（`θ < 30°`），可以近似：
- `cos(θ) ≈ 1`
- `sin(θ) ≈ θ`（弧度）

因此，pitch因子简化为：

```
M_pitch_FL ≈ (x_FL + θ) / x_max
M_pitch_FR ≈ (x_FR - θ) / x_max
```

### 4.2 实际实现

在脚本中，我们使用：

```lua
pitch_factor_left = sin(θ_left) * TILT_PITCH_GAIN * 0.5
pitch_factor_right = -sin(θ_right) * TILT_PITCH_GAIN * 0.5
pitch_factor_rear = cos(θ_rear) * 0.5
```

**理论依据**：
1. **sin(θ)项**：来自倾转产生的X方向推力分量，与倾转角度正弦值成正比
2. **符号相反**：前左和前右倾转方向相反，所以符号相反
3. **增益系数0.5**：归一化因子，确保pitch控制力矩在合理范围内
4. **TILT_PITCH_GAIN**：可调增益，用于平衡倾转控制灵敏度
5. **后置电机**：主要使用cos项（位置力矩），因为后置电机在X轴负方向

### 4.4 倾转角度计算方式

**重要**：舵机一般无法读取当前倾转角度，因此需要通过控制输出计算目标倾转角度。

**计算方式**：
1. 获取pitch控制输出：`pitch_demand = vehicle:get_control_output(CONTROL_OUTPUT_PITCH)`（范围：-1.0 到 1.0）
2. 转换为倾转角度：`tilt_angle = pitch_demand * TILT_ANGLE_MAX`（度）
3. 设置舵机PWM输出：根据倾转角度计算PWM值并输出到舵机

**倾转策略**：
- **前左和前右**：对称倾转（相反方向），`tilt_angle_left = pitch_demand * TILT_ANGLE_MAX`，`tilt_angle_right = -pitch_demand * TILT_ANGLE_MAX`
- **后置**：根据 `REAR_TILT_RATIO` 决定是否倾转，`tilt_angle_rear = pitch_demand * REAR_TILT_RATIO * TILT_ANGLE_MAX`（默认REAR_TILT_RATIO=0.0，不倾转）

### 4.3 完整公式

考虑倾转后的完整pitch因子：

```
M_pitch_FL = (x_FL · cos(θ) + sin(θ)) / x_max
           ≈ (x_FL + sin(θ)) / x_max  （小角度近似）
           = x_FL/x_max + sin(θ)/x_max
           = M_pitch_base + sin(θ) · gain
```

其中：
- `M_pitch_base = x_FL/x_max`：基础pitch因子（无倾转时）
- `gain = 1/x_max`：倾转增益

在脚本实现中，我们假设 `M_pitch_base = 0`（前电机无倾转时无pitch控制），所以：

```
M_pitch_FL = sin(θ) · gain · 0.5
M_pitch_FR = -sin(θ) · gain · 0.5
```

## 5. 共轴对的Yaw因子

### 5.1 共轴对力矩分析

对于共轴双桨，上下桨旋转方向相反：
- 上桨：角速度 `ω_upper`，产生yaw力矩 `τ_yaw_upper = k · ω_upper²`
- 下桨：角速度 `ω_lower`，产生yaw力矩 `τ_yaw_lower = -k · ω_lower²`

如果上下桨转速相等（`ω_upper = ω_lower = ω`），则：

```
τ_yaw_total = τ_yaw_upper + τ_yaw_lower = k · ω² - k · ω² = 0
```

因此，**共轴对的净yaw力矩为零**，yaw因子为0。

### 5.2 实际考虑

在实际飞行中，如果上下桨转速不完全匹配，可能产生小的yaw力矩。但通常可以忽略，或通过PID控制器补偿。

## 6. Roll因子配置

### 6.1 几何分析

对于对称布局的三旋翼：
- 前左电机：`y_FL < 0`，产生负roll力矩
- 前右电机：`y_FR > 0`，产生正roll力矩
- 后电机：`y_R = 0`，不产生roll力矩

归一化后：
```
M_roll_FL = y_FL / |y_max| = -0.5
M_roll_FR = y_FR / |y_max| = +0.5
M_roll_R = 0
```

### 6.2 倾转对Roll的影响

当电机倾转时，roll控制能力会减小（因为 `cos(θ) < 1`）。

**根据MATLAB推导**：Roll力矩 = `-Fi*ly*(cos(a1) - cos(a2))`

**实际实现**：
```lua
roll_factor_left = -0.5 * cos(θ_left)
roll_factor_right = 0.5 * cos(θ_right)
roll_factor_rear = 0  -- 后置ly=0，不贡献roll
```

倾转会减小roll控制能力，需要在实际飞行中考虑这一影响。

## 7. 动态更新策略

### 7.1 更新阈值

为了避免频繁更新混控矩阵，设置阈值：

```
Δθ_threshold = 0.5° = 0.0087 rad
```

只有当倾转角度变化超过此阈值时才更新混控矩阵。

### 7.2 更新频率

更新频率设置为100Hz（10ms周期），确保：
1. 及时响应倾转角度变化
2. 不会过度消耗CPU资源
3. 与飞行控制器更新频率匹配

## 8. 参数调优指南

### 8.1 TILT_PITCH_GAIN

根据理论分析，pitch因子为：

```
M_pitch = sin(θ) · gain
```

其中 `gain = TILT_PITCH_GAIN · 0.5`。

**调优方法**：
- 如果pitch响应迟钝：增大 `TILT_PITCH_GAIN`
- 如果pitch响应过于敏感：减小 `TILT_PITCH_GAIN`
- 建议范围：0.5 - 2.0

### 8.2 TILT_ANGLE_MAX

最大倾转角度影响：
1. **控制能力**：角度越大，pitch控制能力越强
2. **稳定性**：角度过大可能导致控制不稳定
3. **效率**：倾转会损失部分升力（`cos(θ) < 1`）

**建议值**：
- 初始测试：30°
- 正常飞行：45°
- 极限测试：60°（需谨慎）

### 8.3 后电机Pitch因子

后电机的pitch因子用于：
1. 提供基础pitch控制能力
2. 平衡前倾转电机的pitch力矩

**调优方法**：
- 如果pitch控制不足：增大后电机pitch因子（当前0.5）
- 如果pitch控制过度：减小后电机pitch因子

## 9. 理论验证

### 9.1 边界条件检查

1. **无倾转（θ = 0）**：
   - `sin(0) = 0`
   - 前电机pitch因子 = 0 ✓
   - 仅后电机提供pitch控制 ✓

2. **最大倾转（θ = 45°）**：
   - `sin(45°) = 0.707`
   - 前电机pitch因子 = ±0.707 · gain · 0.5
   - 提供最大pitch控制能力 ✓

3. **对称倾转（θ_FL = θ_FR）**：
   - 前左和前右pitch因子大小相等，符号相反 ✓
   - 不产生额外roll力矩 ✓

### 9.2 力矩平衡验证

在悬停状态下（无pitch需求）：
- 前左和前右倾转角度应相等
- 前左和前右pitch因子应相等但符号相反
- 总pitch力矩应为零 ✓

## 10. 总结

### 10.1 核心公式

**Pitch因子计算**：
```
M_pitch_FL = sin(θ_FL) · TILT_PITCH_GAIN · 0.5
M_pitch_FR = -sin(θ_FR) · TILT_PITCH_GAIN · 0.5
M_pitch_R = cos(θ_R) · 0.5  （后置主要用cos项）
```

**Roll因子**（受倾转影响）：
```
M_roll_FL = -0.5 · cos(θ_FL)
M_roll_FR = +0.5 · cos(θ_FR)
M_roll_R = 0  （后置ly=0，不贡献roll）
```

**Yaw因子**（共轴对）：
```
M_yaw = 0  （所有电机，共轴对相互抵消）
```

**Throttle因子**（受倾转影响）：
```
M_throttle_FL = cos(θ_FL)  （倾转损失升力）
M_throttle_FR = cos(θ_FR)
M_throttle_R = cos(θ_R)
```

### 10.2 理论优势

1. **数学严谨**：基于刚体力学和力矩平衡
2. **物理合理**：符合实际飞行器动力学
3. **可调参数**：提供增益和角度限制等可调参数
4. **实时更新**：动态响应倾转角度变化

### 10.3 实际应用建议

1. **初始配置**：使用默认参数进行地面测试
2. **逐步调优**：根据实际飞行效果调整增益
3. **安全限制**：设置合理的倾转角度限制
4. **监控验证**：通过日志和遥测验证混控效果

## 参考文献

1. ArduPilot官方文档：Motor Mixer Theory
2. 多旋翼飞行器动力学与控制（相关教材）
3. 刚体力学基础（力矩分析）
4. 控制分配理论（Control Allocation）
