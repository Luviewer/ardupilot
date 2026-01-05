# 共轴三旋翼倾转电机配置说明

## 概述

此脚本为共轴三旋翼（6个电机，3个共轴对）配置动态混控，实现前左和前右电机的倾转控制，用于X轴（pitch）方向自由度解耦。

## 硬件配置

### 电机布局
- **电机0-1**: 前左共轴双桨（上桨和下桨旋转方向相反）
- **电机2-3**: 前右共轴双桨（上桨和下桨旋转方向相反）
- **电机4-5**: 后共轴双桨（上桨和下桨旋转方向相反）

### 倾转机构
- 前左和前右电机通过舵机实现倾转
- 倾转角度范围：-45° 到 +45°（可配置）
- 倾转舵机连接到飞控的舵机输出通道

## 参数配置

### 脚本内参数（可在脚本开头修改）

```lua
local TILT_ANGLE_MAX = 45      -- 最大倾转角度（度）
local TILT_SERVO_LEFT = 9      -- 前左倾转舵机功能号
local TILT_SERVO_RIGHT = 10    -- 前右倾转舵机功能号
local TILT_PITCH_GAIN = 1.0   -- Pitch控制增益
local UPDATE_RATE_MS = 10      -- 更新频率（毫秒，100Hz）
```

### ArduPilot参数设置

1. **FRAME_CLASS**: 设置为 `17`（动态脚本混控）
   ```
   FRAME_CLASS = 17
   ```

2. **SCR_ENABLE**: 启用Lua脚本
   ```
   SCR_ENABLE = 1
   ```

3. **舵机功能分配**: 配置倾转舵机
   ```
   SERVO9_FUNCTION = 70  (或自定义功能号，需与脚本中TILT_SERVO_LEFT一致)
   SERVO10_FUNCTION = 71 (或自定义功能号，需与脚本中TILT_SERVO_RIGHT一致)
   ```

4. **舵机范围**: 设置倾转舵机的PWM范围
   ```
   SERVO9_MIN = 1000  (对应-45度)
   SERVO9_MAX = 2000  (对应+45度)
   SERVO9_TRIM = 1500 (对应0度，中立位置)
   
   SERVO10_MIN = 1000
   SERVO10_MAX = 2000
   SERVO10_TRIM = 1500
   ```

## 工作原理

### 混控逻辑

1. **Roll控制**: 
   - 前左电机：负roll因子（-0.5）
   - 前右电机：正roll因子（+0.5）
   - 后电机：零roll因子

2. **Pitch控制（动态）**:
   - 前左电机：`pitch_factor = sin(倾转角度) × 增益 × 0.5`
   - 前右电机：`pitch_factor = -sin(倾转角度) × 增益 × 0.5`
   - 后电机：固定pitch因子（0.5），用于平衡

3. **Yaw控制**:
   - 所有共轴对的yaw因子为0（上下桨旋转方向相反，不产生净yaw力矩）

4. **Throttle控制**:
   - 所有电机throttle因子为1.0（等量分配）

### 倾转角度计算

脚本通过读取舵机输出PWM值，转换为倾转角度：
- PWM 1000 → -45度
- PWM 1500 → 0度（中立）
- PWM 2000 → +45度

倾转角度通过 `sin()` 函数转换为pitch控制因子，实现非线性控制。

## 测试步骤

### 1. 地面测试

1. **安全检查**:
   - 确保所有电机旋转方向正确
   - 检查倾转舵机运动范围
   - 验证舵机中立位置（1500 PWM）对应0度倾转

2. **静态测试**:
   - 上电但不解锁
   - 手动控制倾转舵机，观察pitch因子变化
   - 检查GCS日志中的混控更新信息

3. **电机测试**:
   - 使用电机测试功能，验证每个电机响应
   - 检查roll/pitch/yaw控制方向是否正确

### 2. 悬停测试

1. **基础悬停**:
   - 在倾转角度为0时进行悬停
   - 检查姿态稳定性
   - 观察PID调参需求

2. **倾转测试**:
   - 缓慢改变倾转角度
   - 观察pitch响应
   - 检查是否有振荡或不稳定

3. **极限测试**:
   - 测试最大倾转角度（±45度）
   - 验证控制能力
   - 检查是否有控制饱和

### 3. 参数调优

#### TILT_PITCH_GAIN 调优

- **值过小**: Pitch响应迟钝，需要更大倾转角度才能产生足够控制力矩
- **值过大**: Pitch响应过于敏感，可能导致振荡
- **建议**: 从1.0开始，根据实际飞行效果调整（范围：0.5 - 2.0）

#### 倾转角度限制

- **TILT_ANGLE_MAX**: 控制最大倾转角度
- **建议**: 
  - 初始测试：30度
  - 正常飞行：45度
  - 极限测试：60度（需谨慎）

#### 更新频率

- **UPDATE_RATE_MS**: 控制混控更新频率
- **建议**: 
  - 默认10ms（100Hz）通常足够
  - 如果CPU负载高，可增加到20ms（50Hz）
  - 不建议低于50Hz

## 故障排除

### 问题1: 倾转舵机无响应

**可能原因**:
- 舵机功能号配置错误
- 舵机通道未正确连接
- PWM范围设置不正确

**解决方法**:
- 检查 `SERVOx_FUNCTION` 参数
- 验证舵机物理连接
- 检查PWM范围（1000-2000）

### 问题2: Pitch控制方向错误

**可能原因**:
- 前左和前右倾转方向相反
- Pitch因子符号错误

**解决方法**:
- 检查脚本中 `pitch_factor_right` 的符号
- 验证倾转舵机安装方向
- 可能需要交换前左和前右的配置

### 问题3: 控制不稳定或振荡

**可能原因**:
- `TILT_PITCH_GAIN` 过大
- 倾转角度变化过快
- PID参数需要调优

**解决方法**:
- 降低 `TILT_PITCH_GAIN`
- 增加倾转舵机的响应时间（通过舵机速度限制）
- 重新调优Pitch PID参数

### 问题4: 共轴对产生Yaw力矩

**可能原因**:
- 上下桨旋转方向配置错误
- 上下桨转速不一致

**解决方法**:
- 检查电机旋转方向配置
- 确保上下桨转速匹配
- 验证yaw因子为0

## 高级配置

### 自定义倾转控制

如果需要根据飞行状态自动控制倾转角度，可以修改 `update()` 函数：

```lua
function update()
    -- 根据pitch需求自动计算倾转角度
    local pitch_demand = vehicle:get_control_output(1)  -- Pitch control output
    
    -- 计算目标倾转角度
    local target_tilt = pitch_demand * TILT_ANGLE_MAX
    
    -- 设置舵机输出
    SRV_Channels:set_output_pwm_chan_timeout(tilt_servo_left_ch, 
        SERVO_PWM_CENTER + (target_tilt / TILT_ANGLE_MAX) * (SERVO_PWM_RANGE / 2), 
        100)
    
    -- 更新混控矩阵...
end
```

### 非线性倾转曲线

可以修改 `pwm_to_tilt_angle()` 函数，实现非线性映射：

```lua
local function pwm_to_tilt_angle(pwm)
    local normalized = (pwm - SERVO_PWM_CENTER) / (SERVO_PWM_RANGE / 2)
    -- 使用平方曲线实现非线性响应
    local angle_rad = math.rad(normalized * math.abs(normalized) * TILT_ANGLE_MAX)
    return angle_rad
end
```

## 注意事项

1. **安全**: 首次飞行建议在开阔场地，保持低高度
2. **备份**: 修改脚本前备份原文件
3. **测试**: 充分进行地面测试后再进行飞行测试
4. **调参**: 逐步调整参数，避免大幅修改
5. **监控**: 飞行时密切监控姿态和电机输出

## 技术支持

如遇问题，请检查：
- ArduPilot日志文件
- GCS地面站消息
- 脚本运行状态（通过GCS查看）

更多信息请参考：
- ArduPilot Lua脚本文档
- 动态混控API文档
- ArduPilot社区论坛
