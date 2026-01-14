# AP_MotorsTri_Tilt - 共轴倾转三旋翼控制系统

## 概述

AP_MotorsTri_Tilt 是为共轴Y6B倾转三旋翼设计的电机混控类，基于静态控制分配矩阵实现5自由度控制。

## 系统架构

### 电机布局

- **电机 0-1**: 前右共轴对 (上桨CW顺时针, 下桨CCW逆时针)
- **电机 2-3**: 后置共轴对 (上桨CW顺时针, 下桨CCW逆时针)
- **电机 4-5**: 前左共轴对 (上桨CW顺时针, 下桨CCW逆时针)

### 舵机配置

- **舵机 1（前右）**：默认 `SERVO7`，Function = `k_tiltMotorRight (76)`
- **舵机 2（后置）**：默认 `SERVO8`，Function = `k_tiltMotorRear (45)`
- **舵机 3（前左）**：默认 `SERVO9`，Function = `k_tiltMotorLeft (75)`

说明：这里采用与 `AP_MotorsTailsitter` 一致的方式，通过 `SRV_Channels::set_aux_channel_default()` 设置默认输出口；如需更换输出口请改 `SERVOx_FUNCTION`，而不是改电机类内部参数。
默认 PWM 端点按你的硬件设置为 **500–2500us**（只作为默认值，不会覆盖你已经保存的 `SERVOx_MIN/MAX`）。

## 控制原理

### 5自由度控制分配

系统使用静态控制分配矩阵 F_alloc[5×6]，将5个期望控制量映射到6个中间变量：

**输入（5DOF）**:
- Fx: 前进力
- Fz: 垂直升力
- Mx: 滚转力矩
- My: 俯仰力矩
- Mz: 偏航力矩

**中间变量（6维）**:
- [F1×sin(a1), F1×cos(a1), F2×sin(a2), F2×cos(a2), F3×sin(a3), F3×cos(a3)]

**输出**:
- F1, F2, F3: 三个转子组的推力
- a1, a2, a3: 三个转子组的倾转角度

### 混控矩阵

基于MATLAB推导（tricopter_allocation/cal_alloc_tri.m）的静态矩阵：

```
F_alloc = 
[      -1,         0,        -1,      0,        -1,         0]  % Fx
[       0,        -1,         0,     -1,         0,        -1]  % Fz
[       0, -lfront_y,         0,      0,         0,  lfront_y]  % Mx
[       0,  lfront_x,         0, -lrear,         0, -lfront_x]  % My
[lfront_y,         0,         0,      0, -lfront_y,         0]  % Mz
```

### 偏航控制

通过共轴上下桨差动推力实现，不依赖倾转角度，简化控制逻辑。

## 参数配置

### 必需参数

| 参数名 | 说明 | 默认值 | 范围 |
|--------|------|--------|------|
| MOT_TILT_LX | 前臂X方向归一化长度 | 0.5 | 0.1-2.0 |
| MOT_TILT_LY | 前臂Y方向归一化长度 | 0.5 | 0.1-2.0 |
| MOT_TILT_LREAR | 后臂归一化长度 | 1.0 | 0.1-2.0 |
| MOT_TILT_ANG_MAX | 最大倾转角度（度） | 30 | 5-60 |

### 舵机输出映射（标准SERVOx_FUNCTION）

默认映射（可按需覆盖）：
- `SERVO7_FUNCTION = 76`  (k_tiltMotorRight，前右倾转)
- `SERVO8_FUNCTION = 45`  (k_tiltMotorRear，后置倾转)
- `SERVO9_FUNCTION = 75`  (k_tiltMotorLeft，前左倾转)

### 高级参数

| 参数名 | 说明 | 默认值 | 范围 |
|--------|------|--------|------|
| MOT_TILT_YAW_FAC | 偏航力矩因子 | 0.15 | 0.0-1.0 |
| MOT_TILT_YAW_DIR | 偏航方向（-1反向/1正常） | 1 | -1/1 |
| MOT_TILT_SVO_FR_REV | 前右倾转舵机方向反转 | 0 | 0/1 |
| MOT_TILT_SVO_REAR_REV | 后置倾转舵机方向反转 | 0 | 0/1 |
| MOT_TILT_SVO_FL_REV | 前左倾转舵机方向反转 | 0 | 0/1 |

## 使用方法

### 1. 固件编译

确保在 `AP_Motors_config.h` 中启用：

```cpp
#define AP_MOTORS_TRI_TILT_ENABLED 1
```

### 2. 参数配置

在地面站中设置以下参数：

```
FRAME_CLASS = 7 (TRI)
FRAME_TYPE = 20 (TriTilt)

# 几何参数（根据实际飞行器调整）
MOT_TILT_LX = 0.5
MOT_TILT_LY = 0.5
MOT_TILT_LREAR = 1.0

# 倾转角度限制
MOT_TILT_ANG_MAX = 30

# 舵机功能映射（如需自定义输出口，改 SERVOx_FUNCTION）
SERVO7_FUNCTION = 76   # k_tiltMotorRight
SERVO8_FUNCTION = 45   # k_tiltMotorRear
SERVO9_FUNCTION = 75   # k_tiltMotorLeft

# 偏航控制
MOT_TILT_YAW_FAC = 0.15
```

### 3. 舵机校准

1. 在地面站中进入舵机输出设置
2. 为通道7-9设置舵机行程范围（通常1000-2000μs）
3. 校准舵机中位（1500μs对应0°倾转）
4. 测试最大倾转角度是否符合设定值

### 4. 电机测试

使用地面站的电机测试功能，按顺序测试6个电机：
1. 电机1 (前右上)
2. 电机2 (前右下)
3. 电机3 (后置上)
4. 电机4 (后置下)
5. 电机5 (前左上)
6. 电机6 (前左下)

确认：
- 转向正确（上桨CW，下桨CCW）
- 油门响应线性
- 无异常振动

## 技术细节

### 继承关系

```
AP_Motors
  └─ AP_MotorsMulticopter
      └─ AP_MotorsMatrix
          └─ AP_MotorsTri_Tilt
```

### 关键函数

1. **calculate_allocation_matrix()**: 计算静态混控矩阵F_alloc
2. **calculate_allocation_matrix_pinv()**: 计算伪逆矩阵F_alloc†
3. **output_armed_stabilizing()**: 主混控逻辑
   - 输入: 5DOF期望控制量
   - 输出: 推力和倾转角
4. **output_to_motors()**: PWM输出
   - 电机: 共轴差动偏航
   - 舵机: 倾转角度控制

### 与Lua脚本的对比

| 特性 | C++实现 | Lua脚本 |
|------|---------|---------|
| 执行频率 | 400Hz+ | ~100Hz |
| 延迟 | <1ms | 5-10ms |
| CPU占用 | 低 | 中 |
| 调试便利性 | 中 | 高 |
| 生产环境 | 推荐 | 原型测试 |

## 调试与诊断

### 启用日志

```
LOG_BITMASK = 65535 (Full logging)
```

关键日志消息：
- CTUN: 控制输出
- RATE: 姿态速率
- ATT: 姿态角度
- MOTB: 电机输出

### 常见问题

**问题1: 舵机未响应**
- 检查 `SERVO7/8/9_FUNCTION` 是否设置为 `k_tiltMotorRight/Rear/Left`
- 验证舵机PWM范围

**问题2: 推力分配不均**
- 检查几何参数lx, ly, lrear是否准确
- 验证电机方向和转向
- 校准ESC和电机

**问题3: 偏航控制弱**
- 增大TRI_TILT_YAW_FAC
- 检查共轴电机差动是否正常
- 验证上下桨反转方向正确

## 理论基础

详细推导参见：
- MATLAB脚本: `tricopter_allocation/cal_alloc_tri.m`
- Lua参考实现: `tricopter/scripts/MotorMatrix_coaxial_tricopter_tilt.lua`

控制分配算法基于：
1. 机体动力学建模
2. 倾转推力矢量分解
3. Moore-Penrose伪逆求解
4. 实时约束优化

## 版本历史

- v1.0.0 (2026-01-14): 初始实现
  - 静态混控矩阵
  - 5自由度控制
  - 共轴差动偏航

## 作者与许可

基于ArduPilot开源项目
许可: GPLv3

## 参考资料

- [ArduPilot Motors Library](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Motors)
- [控制分配理论](https://arc.aiaa.org/doi/10.2514/1.19869)
