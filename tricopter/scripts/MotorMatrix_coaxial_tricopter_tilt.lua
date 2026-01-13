-- 共轴三旋翼Y6B电机配置
-- 此脚本配置共轴三旋翼（共6个电机：3个共轴对）
-- 使用标准Y6B混控配置，无倾转控制
--
-- ============================================================================
-- ArduPilot Roll/Pitch/Yaw 因子方向约定说明
-- ============================================================================
--
-- 【Roll 因子约定】
--   - 正 roll 需求 = 向左滚转（left roll，从机尾看，左侧向下）
--   - 向左滚转需要：增加左侧电机推力，减少右侧电机推力
--   - 因此：左侧电机 roll 因子为正，右侧电机 roll 因子为负
--   - 示例：左前电机 roll_factor = +1.0，右前电机 roll_factor = -1.0
--
-- 【Pitch 因子约定】
--   - 正 pitch 需求 = 机头上仰（nose up，向前倾斜）
--   - 机头上仰需要：增加前电机推力，减少后电机推力
--   - 因此：前电机 pitch 因子为正，后电机 pitch 因子为负
--   - 示例：前电机 pitch_factor = +0.5，后电机 pitch_factor = -1.0
--
-- 【Yaw 因子约定】
--   - 正 yaw 需求 = 顺时针旋转（clockwise，从上方看）
--   - 顺时针旋转需要：增加逆时针（CCW）电机推力，减少顺时针（CW）电机推力
--   - 因此：CCW 电机 yaw 因子为正，CW 电机 yaw 因子为负
--   - 注意：对于共轴对，上下桨旋转方向相反，yaw 因子也相反，净 yaw 力矩为零
--   - 示例：CCW 电机 yaw_factor = +1.0，CW 电机 yaw_factor = -1.0
--
-- 【混控公式】
--   每个电机的最终推力 = throttle_factor * throttle
--                        + roll_factor * roll_thrust
--                        + pitch_factor * pitch_thrust
--                        + yaw_factor * yaw_thrust
--                        + forward_factor * forward_thrust
--                        + right_factor * right_thrust
--
-- ============================================================================
--
-- 电机布局：
--   电机0-1：前右共轴（上桨CW，下桨CCW）
--   电机2-3：后共轴（上桨CW，下桨CCW）
--   电机4-5：前左共轴（上桨CW，下桨CCW）

-- ============================================================================
-- 参数表配置
-- ============================================================================
local PARAM_TABLE_KEY = 100
local PARAM_TABLE_PREFIX = "TRI_"

-- 添加参数并绑定到变量的辅助函数
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
        string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 创建参数表（14个参数）
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 14),
    'could not add param table')

--[[
  // @Param: TRI_ENABLE
  // @DisplayName: 启用倾转
  // @Description: 启用共轴三旋翼倾转控制
  // @Values: 0:禁用,1:启用
  // @User: Standard
--]]
local TRI_ENABLE_PARAM = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: TRI_ANGLE_MAX
  // @DisplayName: 最大倾转角度
  // @Description: 最大倾转角度（度）
  // @Range: 0 90
  // @Units: deg
  // @User: Standard
--]]
local TRI_ANGLE_MAX_PARAM = bind_add_param('ANGLE_MAX', 2, 45.0)

--[[
  // @Param: TRI_PITCH_GAIN
  // @DisplayName: 倾转Pitch增益
  // @Description: 倾转控制的Pitch增益
  // @Range: 0 2
  // @User: Standard
--]]
local TRI_PITCH_GAIN_PARAM = bind_add_param('PITCH_GAIN', 3, 1.0)

--[[
  // @Param: TRI_UPDATE_THR
  // @DisplayName: 倾转更新阈值
  // @Description: 倾转角度变化阈值（度），小于此值时不更新
  // @Range: 0 10
  // @Units: deg
  // @User: Standard
--]]
local TRI_UPDATE_THR_PARAM = bind_add_param('UPDATE_THR', 4, 0.5)

--[[
  // @Param: TRI_LX
  // @DisplayName: 控制分配LX
  // @Description: X轴方向距离（前电机到后电机的距离，归一化值）
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_LX_PARAM = bind_add_param('LX', 5, 1.0)

--[[
  // @Param: TRI_LY
  // @DisplayName: 控制分配LY
  // @Description: Y轴方向距离（左电机到右电机的距离，归一化值）
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_LY_PARAM = bind_add_param('LY', 6, 1.0)

--[[
  // @Param: TRI_FORCE_SCALE
  // @DisplayName: 力缩放系数
  // @Description: 控制分配的力缩放系数
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_FORCE_SCALE_PARAM = bind_add_param('FORCE_SCALE', 7, 1.0)

--[[
  // @Param: TRI_MOMENT_SCALE
  // @DisplayName: 力矩缩放系数
  // @Description: 控制分配的力矩缩放系数
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_MOMENT_SCALE_PARAM = bind_add_param('MOMENT_SCALE', 8, 1.0)

--[[
  // @Param: TRI_DIR_FR
  // @DisplayName: 前右倾转方向
  // @Description: 前右电机倾转方向（1.0=向前，-1.0=向后）
  // @Range: -1 1
  // @User: Standard
--]]
local TRI_DIR_FR_PARAM = bind_add_param('DIR_FR', 9, 1.0)

--[[
  // @Param: TRI_DIR_FL
  // @DisplayName: 前左倾转方向
  // @Description: 前左电机倾转方向（1.0=向前，-1.0=向后）
  // @Range: -1 1
  // @User: Standard
--]]
local TRI_DIR_FL_PARAM = bind_add_param('DIR_FL', 10, -1.0)

--[[
  // @Param: TRI_DIR_REAR
  // @DisplayName: 后置倾转方向
  // @Description: 后置电机倾转方向（1.0=向前，-1.0=向后）
  // @Range: -1 1
  // @User: Standard
--]]
local TRI_DIR_REAR_PARAM = bind_add_param('DIR_REAR', 11, 1.0)

--[[
  // @Param: TRI_SERVO_FR
  // @DisplayName: 前右倾转舵机
  // @Description: 前右倾转舵机功能号 (k_tiltMotorRight)
  // @Range: 0 159
  // @User: Standard
--]]
local TRI_SERVO_FR_PARAM = bind_add_param('SERVO_FR', 12, 76)

--[[
  // @Param: TRI_SERVO_FL
  // @DisplayName: 前左倾转舵机
  // @Description: 前左倾转舵机功能号 (k_tiltMotorLeft)
  // @Range: 0 159
  // @User: Standard
--]]
local TRI_SERVO_FL_PARAM = bind_add_param('SERVO_FL', 13, 75)

--[[
  // @Param: TRI_SERVO_REAR
  // @DisplayName: 后置倾转舵机
  // @Description: 后置倾转舵机功能号 (k_tiltMotorRear)
  // @Range: 0 159
  // @User: Standard
--]]
local TRI_SERVO_REAR_PARAM = bind_add_param('SERVO_REAR', 14, 45)

-- ============================================================================
-- 倾转功能配置（从参数读取）
-- ============================================================================
-- 从参数读取配置值
local ENABLE_TILT = TRI_ENABLE_PARAM:get() > 0.5 -- 启用倾转功能（false=不倾转，使用静态混控）

-- 倾转参数（仅在 ENABLE_TILT=true 时有效）
local TILT_ANGLE_MAX = TRI_ANGLE_MAX_PARAM:get()         -- 最大倾转角度（度）
local TILT_PITCH_GAIN = TRI_PITCH_GAIN_PARAM:get()       -- Pitch控制增益
local TILT_UPDATE_THRESHOLD = TRI_UPDATE_THR_PARAM:get() -- 倾转角度变化阈值（度），小于此值不更新

-- 控制分配参数（根据F_alloc矩阵）
-- lx: X轴方向距离（前电机到后电机的距离，归一化值）
-- ly: Y轴方向距离（左电机到右电机的距离，归一化值）
local LX = TRI_LX_PARAM:get() -- X轴方向距离
local LY = TRI_LY_PARAM:get() -- Y轴方向距离
local LY_MIN = 0.01           -- ly的最小值，避免除零
local LX_MIN = 0.01           -- lx的最小值，避免除零

-- 控制输入到力和力矩的缩放系数（归一化）
-- ArduPilot的控制输入是-1到1的归一化值，需要转换为力和力矩
local FORCE_SCALE = TRI_FORCE_SCALE_PARAM:get()   -- 力缩放系数
local MOMENT_SCALE = TRI_MOMENT_SCALE_PARAM:get() -- 力矩缩放系数

-- 倾转方向参数（1.0=正向，-1.0=反向）
-- PWM > 1500时：右前超前(+1)，后方朝前(+1)，左前朝后(-1)
local TILT_DIR_FRONT_RIGHT = TRI_DIR_FR_PARAM:get() -- 前右倾转方向（正角度时向前倾转）
local TILT_DIR_FRONT_LEFT = TRI_DIR_FL_PARAM:get()  -- 前左倾转方向（正角度时向后倾转）
local TILT_DIR_REAR = TRI_DIR_REAR_PARAM:get()      -- 后置倾转方向（正角度时向前倾转）

-- 舵机配置（使用ArduPilot标准倾转舵机功能号）
local TILT_SERVO_FRONT_RIGHT = math.floor(TRI_SERVO_FR_PARAM:get()) -- 前右倾转舵机功能号 (k_tiltMotorRight)
local TILT_SERVO_FRONT_LEFT = math.floor(TRI_SERVO_FL_PARAM:get())  -- 前左倾转舵机功能号 (k_tiltMotorLeft)
local TILT_SERVO_REAR = math.floor(TRI_SERVO_REAR_PARAM:get())      -- 后置倾转舵机功能号 (k_tiltMotorRear)

-- 舵机PWM范围
local SERVO_PWM_MIN = 1000
local SERVO_PWM_MAX = 2000
local SERVO_PWM_CENTER = 1500
local SERVO_PWM_RANGE = SERVO_PWM_MAX - SERVO_PWM_MIN

-- 控制输出常量
local CONTROL_OUTPUT_PITCH = 2

-- 更新频率（毫秒，100Hz）
local UPDATE_RATE_MS = 10

-- 电机索引（从0开始）
local MOTOR_FRONT_RIGHT_UPPER = 0 -- 右上CW
local MOTOR_FRONT_RIGHT_LOWER = 1 -- 右下CCW
local MOTOR_REAR_UPPER = 2        -- 后上CW
local MOTOR_REAR_LOWER = 3        -- 后下CCW
local MOTOR_FRONT_LEFT_UPPER = 4  -- 左上CW
local MOTOR_FRONT_LEFT_LOWER = 5  -- 左下CCW

-- 共轴对的Yaw因子（旋转方向相反抵消yaw，通常为0）
local YAW_FACTOR_COAXIAL_CW = -1 -- 共轴对不产生净yaw力矩
local YAW_FACTOR_COAXIAL_CCW = 1 -- 共轴对不产生净yaw力矩

-- ============================================================================
-- 控制分配模块（基于F_alloc矩阵的伪逆）
-- ============================================================================
local AllocationControl = {}

-- 使用伪逆矩阵计算中间变量
-- 输入：Fx, Fz, Mx, My, Mz (力和力矩需求)
-- 输出：f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3 (中间变量)
-- 其中：f1_s1 = F1*sin(a1), f1_c1 = F1*cos(a1) 等
function AllocationControl.solve_intermediate_variables(Fx, Fz, Mx, My, Mz)
    -- 保护除零
    local ly = math.max(LY, LY_MIN)
    local lx = math.max(LX, LX_MIN)

    -- 根据伪逆矩阵公式计算中间变量
    -- pinv(F_alloc) = [
    --   -1/3,    0,     0,        0,  1/(2*ly)  ;  % f1_s1
    --   -1/3,    0,     0,        0,         0  ;  % f2_s2
    --   -1/3,    0,     0,        0, -1/(2*ly)  ;  % f3_s3
    --      0, -1/2,     0, 1/(2*lx),         0  ;  % f1_c1
    --      0,    0, -1/ly,    -1/lx,         0  ;  % f2_c2
    --      0, -1/2,  1/ly, 1/(2*lx),         0  ;  % f3_c3
    -- ]

    local f1_s1 = -Fx / 3.0 + Mz / (2.0 * ly)
    local f2_s2 = -Fx / 3.0
    local f3_s3 = -Fx / 3.0 - Mz / (2.0 * ly)
    local f1_c1 = -Fz / 2.0 + My / (2.0 * lx)
    local f2_c2 = -Mx / ly - My / lx
    local f3_c3 = -Fz / 2.0 + Mx / ly + My / (2.0 * lx)

    return f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3
end

-- ============================================================================
-- 从中间变量计算推力和角度
-- 输入：中间变量 f_sin, f_cos
-- 输出：推力 F 和角度 a (弧度)
-- ============================================================================
function AllocationControl.compute_thrust_and_angle(f_sin, f_cos)
    -- F = sqrt((F*sin(a))^2 + (F*cos(a))^2)
    local F = math.sqrt(f_sin * f_sin + f_cos * f_cos)

    -- a = atan2(F*sin(a), F*cos(a))
    local alpha = math.atan2(f_sin, f_cos)

    return F, alpha
end

-- ============================================================================
-- 倾转控制模块
-- ============================================================================
local TiltControl = {}

-- 计算倾转角度和推力（基于F_alloc矩阵反解）
-- 返回：tilt_right, tilt_left, tilt_rear (弧度), F1, F2, F3 (推力)
function TiltControl.calculate_tilt_angles()
    if not ENABLE_TILT then
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 -- 不倾转时返回0
    end

    -- 1. 获取控制输入（归一化值，-1到1）
    local roll_demand = motors:get_roll() or 0.0
    local pitch_demand = motors:get_pitch() or 0.0
    local yaw_demand = motors:get_yaw() or 0.0
    local throttle_demand = motors:get_throttle() or 0.0
    local forward_demand = motors:get_forward() or 0.0

    -- 检查输入有效性（NaN保护）
    if roll_demand ~= roll_demand then roll_demand = 0.0 end
    if pitch_demand ~= pitch_demand then pitch_demand = 0.0 end
    if yaw_demand ~= yaw_demand then yaw_demand = 0.0 end
    if throttle_demand ~= throttle_demand then throttle_demand = 0.0 end
    if forward_demand ~= forward_demand then forward_demand = 0.0 end

    -- 2. 转换为力和力矩需求
    -- 注意：ArduPilot的控制输入已经是归一化的，这里直接使用
    -- 如果需要物理单位，可以乘以缩放系数
    local Fx = forward_demand * FORCE_SCALE  -- X方向力（前向为正）
    local Fz = throttle_demand * FORCE_SCALE -- Z方向力（向上为正，但ArduPilot中throttle为正时向下，需要取反）
    local Mx = roll_demand * MOMENT_SCALE    -- Roll力矩
    local My = pitch_demand * MOMENT_SCALE   -- Pitch力矩
    local Mz = yaw_demand * MOMENT_SCALE     -- Yaw力矩

    -- 注意：根据ArduPilot约定，Fz可能需要取反（throttle为正时产生向下力）
    -- 但这里先保持原样，根据实际测试调整

    -- 3. 使用伪逆矩阵计算中间变量
    local f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3 =
        AllocationControl.solve_intermediate_variables(Fx, Fz, Mx, My, Mz)

    -- 检查中间变量有效性
    if f1_s1 ~= f1_s1 or f1_c1 ~= f1_c1 then
        f1_s1, f1_c1 = 0.0, 0.0
    end
    if f2_s2 ~= f2_s2 or f2_c2 ~= f2_c2 then
        f2_s2, f2_c2 = 0.0, 0.0
    end
    if f3_s3 ~= f3_s3 or f3_c3 ~= f3_c3 then
        f3_s3, f3_c3 = 0.0, 0.0
    end

    -- 4. 计算推力和角度
    local F1, a1 = AllocationControl.compute_thrust_and_angle(f1_s1, f1_c1)
    local F2, a2 = AllocationControl.compute_thrust_and_angle(f2_s2, f2_c2)
    local F3, a3 = AllocationControl.compute_thrust_and_angle(f3_s3, f3_c3)

    -- 5. 限制角度在最大倾转范围内
    local max_angle_rad = math.rad(TILT_ANGLE_MAX)
    a1 = math.max(-max_angle_rad, math.min(max_angle_rad, a1))
    a2 = math.max(-max_angle_rad, math.min(max_angle_rad, a2))
    a3 = math.max(-max_angle_rad, math.min(max_angle_rad, a3))

    -- 6. 限制推力在合理范围内（0到最大推力，这里假设最大推力为1.0）
    local max_thrust = 1.0
    F1 = math.max(0.0, math.min(max_thrust, F1))
    F2 = math.max(0.0, math.min(max_thrust, F2))
    F3 = math.max(0.0, math.min(max_thrust, F3))

    -- 返回角度（弧度）和推力
    -- a1对应前右，a2对应后置，a3对应前左（根据MATLAB代码中的定义）
    return a1, a3, a2, F1, F2, F3 -- tilt_right, tilt_left, tilt_rear, F1, F2, F3
end

-- 角度转PWM（返回整数）
-- angle_rad: 倾转角度（弧度）
-- direction: 方向系数（1.0=正向，-1.0=反向）
function TiltControl.angle_to_pwm(angle_rad, direction)
    direction = direction or 1.0                                                   -- 默认方向为正
    local angle_deg = math.deg(angle_rad) * direction                              -- 应用方向系数
    local normalized = math.max(-1.0, math.min(1.0, angle_deg / TILT_ANGLE_MAX))
    return math.floor(SERVO_PWM_CENTER + normalized * (SERVO_PWM_RANGE / 2) + 0.5) -- 四舍五入到最近的整数
end

-- 设置舵机输出
function TiltControl.set_servo_outputs(tilt_right, tilt_left, tilt_rear)
    if not ENABLE_TILT then
        return
    end

    local ch_right = SRV_Channels:find_channel(TILT_SERVO_FRONT_RIGHT)
    local ch_left = SRV_Channels:find_channel(TILT_SERVO_FRONT_LEFT)
    local ch_rear = SRV_Channels:find_channel(TILT_SERVO_REAR)

    if ch_right then
        -- 前右：正角度时向前倾转（PWM > 1500）
        SRV_Channels:set_output_pwm_chan_timeout(ch_right,
            TiltControl.angle_to_pwm(tilt_right, TILT_DIR_FRONT_RIGHT), 100)
    end
    if ch_left then
        -- 前左：正角度时向后倾转（PWM < 1500）
        SRV_Channels:set_output_pwm_chan_timeout(ch_left,
            TiltControl.angle_to_pwm(tilt_left, TILT_DIR_FRONT_LEFT), 100)
    end
    if ch_rear then
        -- 后置：正角度时向前倾转（PWM > 1500）
        SRV_Channels:set_output_pwm_chan_timeout(ch_rear,
            TiltControl.angle_to_pwm(tilt_rear, TILT_DIR_REAR), 100)
    end
end

-- ============================================================================
-- 混控因子计算模块
-- ============================================================================
local MixingFactors = {}

-- 计算静态混控因子（不倾转模式）
function MixingFactors.calculate_static_factors(factors)
    -- Roll因子：前左正，前右负，后为零
    factors:roll(MOTOR_FRONT_RIGHT_UPPER, -1.0)
    factors:roll(MOTOR_FRONT_RIGHT_LOWER, -1.0)
    factors:roll(MOTOR_REAR_UPPER, 0)
    factors:roll(MOTOR_REAR_LOWER, 0)
    factors:roll(MOTOR_FRONT_LEFT_UPPER, 1.0)
    factors:roll(MOTOR_FRONT_LEFT_LOWER, 1.0)

    -- Pitch因子：前电机0.5，后电机-1.0
    factors:pitch(MOTOR_FRONT_RIGHT_UPPER, 0.5)
    factors:pitch(MOTOR_FRONT_RIGHT_LOWER, 0.5)
    factors:pitch(MOTOR_REAR_UPPER, -1.0)
    factors:pitch(MOTOR_REAR_LOWER, -1.0)
    factors:pitch(MOTOR_FRONT_LEFT_UPPER, 0.5)
    factors:pitch(MOTOR_FRONT_LEFT_LOWER, 0.5)

    -- Yaw因子：共轴对相互抵消（无净yaw）
    factors:yaw(MOTOR_FRONT_RIGHT_UPPER, YAW_FACTOR_COAXIAL_CW)
    factors:yaw(MOTOR_FRONT_RIGHT_LOWER, YAW_FACTOR_COAXIAL_CCW)
    factors:yaw(MOTOR_REAR_UPPER, YAW_FACTOR_COAXIAL_CW)
    factors:yaw(MOTOR_REAR_LOWER, YAW_FACTOR_COAXIAL_CCW)
    factors:yaw(MOTOR_FRONT_LEFT_UPPER, YAW_FACTOR_COAXIAL_CW)
    factors:yaw(MOTOR_FRONT_LEFT_LOWER, YAW_FACTOR_COAXIAL_CCW)

    -- Throttle因子：所有电机等量贡献
    factors:throttle(MOTOR_FRONT_LEFT_UPPER, 1.0)
    factors:throttle(MOTOR_FRONT_LEFT_LOWER, 1.0)
    factors:throttle(MOTOR_FRONT_RIGHT_UPPER, 1.0)
    factors:throttle(MOTOR_FRONT_RIGHT_LOWER, 1.0)
    factors:throttle(MOTOR_REAR_UPPER, 1.0)
    factors:throttle(MOTOR_REAR_LOWER, 1.0)
end

-- 计算动态混控因子（倾转模式，根据计算出的推力和角度）
-- 输入：tilt_right, tilt_left, tilt_rear (弧度), F1, F2, F3 (推力)
function MixingFactors.calculate_tilt_factors(factors, tilt_right, tilt_left, tilt_rear, F1, F2, F3)
    -- 计算三角函数值
    local cos_right = math.cos(tilt_right)
    local cos_left = math.cos(tilt_left)
    local cos_rear = math.cos(tilt_rear)
    local sin_right = math.sin(tilt_right)
    local sin_left = math.sin(tilt_left)
    local sin_rear = math.sin(tilt_rear)

    -- 保护除零和无效值
    if cos_right ~= cos_right then cos_right = 1.0 end
    if cos_left ~= cos_left then cos_left = 1.0 end
    if cos_rear ~= cos_rear then cos_rear = 1.0 end
    if sin_right ~= sin_right then sin_right = 0.0 end
    if sin_left ~= sin_left then sin_left = 0.0 end
    if sin_rear ~= sin_rear then sin_rear = 0.0 end

    -- 归一化推力（用于因子计算）
    -- 注意：这里使用推力来调整因子，但ArduPilot的混控系统主要使用角度相关的因子
    -- 推力主要通过throttle输入控制，这里我们主要使用角度因子

    -- 保护除零
    local ly = math.max(LY, LY_MIN)
    local lx = math.max(LX, LX_MIN)

    -- Roll因子：根据F_alloc矩阵 Mx = -ly*F1*cos(a1) + ly*F3*cos(a3)
    -- 归一化后：前右 -ly*cos(a1)，前左 +ly*cos(a3)，后置0（ly=0）
    -- 注意：这里需要根据实际布局调整符号
    local roll_factor_right = -ly * cos_right -- 前右
    local roll_factor_left = ly * cos_left    -- 前左
    local roll_factor_rear = 0.0              -- 后置ly=0，不贡献

    -- 归一化roll因子（使其在合理范围内）
    local roll_scale = 1.0 / math.max(math.abs(roll_factor_right), math.abs(roll_factor_left), 0.01)
    factors:roll(MOTOR_FRONT_RIGHT_UPPER, roll_factor_right * roll_scale)
    factors:roll(MOTOR_FRONT_RIGHT_LOWER, roll_factor_right * roll_scale)
    factors:roll(MOTOR_REAR_UPPER, roll_factor_rear)
    factors:roll(MOTOR_REAR_LOWER, roll_factor_rear)
    factors:roll(MOTOR_FRONT_LEFT_UPPER, roll_factor_left * roll_scale)
    factors:roll(MOTOR_FRONT_LEFT_LOWER, roll_factor_left * roll_scale)

    -- Pitch因子：根据F_alloc矩阵 My = lx*F1*cos(a1) - lx*F2*cos(a2) - lx*F3*cos(a3)
    -- 归一化后：前右 +lx*cos(a1)，后置 -lx*cos(a2)，前左 -lx*cos(a3)
    local pitch_factor_right = lx * cos_right -- 前右
    local pitch_factor_rear = -lx * cos_rear  -- 后置
    local pitch_factor_left = -lx * cos_left  -- 前左

    -- 归一化pitch因子
    local pitch_scale = 1.0 /
    math.max(math.abs(pitch_factor_right), math.abs(pitch_factor_rear), math.abs(pitch_factor_left), 0.01)
    factors:pitch(MOTOR_FRONT_RIGHT_UPPER, pitch_factor_right * pitch_scale)
    factors:pitch(MOTOR_FRONT_RIGHT_LOWER, pitch_factor_right * pitch_scale)
    factors:pitch(MOTOR_REAR_UPPER, pitch_factor_rear * pitch_scale)
    factors:pitch(MOTOR_REAR_LOWER, pitch_factor_rear * pitch_scale)
    factors:pitch(MOTOR_FRONT_LEFT_UPPER, pitch_factor_left * pitch_scale)
    factors:pitch(MOTOR_FRONT_LEFT_LOWER, pitch_factor_left * pitch_scale)

    -- Yaw因子：根据F_alloc矩阵 Mz = ly*F1*sin(a1) - ly*F3*sin(a3)
    -- 归一化后：前右 +ly*sin(a1)，前左 -ly*sin(a3)，后置0
    -- 注意：共轴对的yaw因子需要与旋转方向结合
    local yaw_factor_right = ly * sin_right -- 前右
    local yaw_factor_left = -ly * sin_left  -- 前左
    local yaw_factor_rear = 0.0             -- 后置ly=0，不贡献

    -- 归一化yaw因子
    local yaw_scale = 1.0 / math.max(math.abs(yaw_factor_right), math.abs(yaw_factor_left), 0.01)
    factors:yaw(MOTOR_FRONT_RIGHT_UPPER, YAW_FACTOR_COAXIAL_CW + yaw_factor_right * yaw_scale)
    factors:yaw(MOTOR_FRONT_RIGHT_LOWER, YAW_FACTOR_COAXIAL_CCW + yaw_factor_right * yaw_scale)
    factors:yaw(MOTOR_REAR_UPPER, YAW_FACTOR_COAXIAL_CW + yaw_factor_rear)
    factors:yaw(MOTOR_REAR_LOWER, YAW_FACTOR_COAXIAL_CCW + yaw_factor_rear)
    factors:yaw(MOTOR_FRONT_LEFT_UPPER, YAW_FACTOR_COAXIAL_CW + yaw_factor_left * yaw_scale)
    factors:yaw(MOTOR_FRONT_LEFT_LOWER, YAW_FACTOR_COAXIAL_CCW + yaw_factor_left * yaw_scale)

    -- Throttle因子：根据F_alloc矩阵 Fz = -(F1*cos(a1) + F2*cos(a2) + F3*cos(a3))
    -- 倾转导致升力损失，通过cos项补偿
    -- 注意：这里使用推力F1, F2, F3来调整，但主要使用cos项
    -- 共轴对的上下电机使用相同的推力
    local throttle_base_right = F1 * cos_right -- 前右共轴对
    local throttle_base_rear = F2 * cos_rear   -- 后置共轴对
    local throttle_base_left = F3 * cos_left   -- 前左共轴对

    -- 归一化throttle因子（使其总和为1.0，用于保持总升力）
    local throttle_sum = throttle_base_right + throttle_base_rear + throttle_base_left
    local throttle_scale = 1.0
    if throttle_sum > 0.01 then
        throttle_scale = 1.0 / throttle_sum
    end

    factors:throttle(MOTOR_FRONT_RIGHT_UPPER, throttle_base_right * throttle_scale)
    factors:throttle(MOTOR_FRONT_RIGHT_LOWER, throttle_base_right * throttle_scale)
    factors:throttle(MOTOR_REAR_UPPER, throttle_base_rear * throttle_scale)
    factors:throttle(MOTOR_REAR_LOWER, throttle_base_rear * throttle_scale)
    factors:throttle(MOTOR_FRONT_LEFT_UPPER, throttle_base_left * throttle_scale)
    factors:throttle(MOTOR_FRONT_LEFT_LOWER, throttle_base_left * throttle_scale)
end

-- 初始化电机及测试顺序（按照电机索引0-5顺序）
-- 注意：Motors_6DoF_dynamic需要3个参数：motor_num, testing_order, reversible
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_RIGHT_UPPER, 1, false) -- 电机0：右上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_RIGHT_LOWER, 2, false) -- 电机1：右下，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_REAR_UPPER, 3, false)        -- 电机2：后上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_REAR_LOWER, 4, false)        -- 电机3：后下，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_LEFT_UPPER, 5, false)  -- 电机4：左上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_LEFT_LOWER, 6, false)  -- 电机5：左下，不可逆

-- 创建因子表（6DOF版本，包含forward和right因子）
local factors = motor_factor_table_6dof()

-- 根据ENABLE_TILT选择初始化方式
if ENABLE_TILT then
    -- 倾转模式：初始化为0度倾转状态，推力为1.0
    MixingFactors.calculate_tilt_factors(factors, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
else
    -- 不倾转模式：使用静态因子
    MixingFactors.calculate_static_factors(factors)
end

-- 设置forward和right因子（所有模式下都为0，垂直推力）
for i = 0, 5 do
    factors:forward(i, 0.0)
    factors:right(i, 0.0)
end

-- 加载因子并初始化
Motors_6DoF_dynamic:load_factors(factors)
assert(Motors_6DoF_dynamic:init(6), "初始化动态混控失败")

-- 设置框架字符串
if ENABLE_TILT then
    motors:set_frame_string("共轴三旋翼Y6B-倾转")
else
    motors:set_frame_string("共轴三旋翼Y6B")
end

-- 更新函数
local first_update = true -- 首次更新标志

function update()
    if ENABLE_TILT then
        -- 倾转模式：动态计算混控因子
        local tilt_right, tilt_left, tilt_rear, F1, F2, F3 = TiltControl.calculate_tilt_angles()

        -- 首次更新时，如果计算结果异常，强制设置为0（中位）
        if first_update then
            if tilt_right ~= tilt_right or tilt_left ~= tilt_left or tilt_rear ~= tilt_rear then
                -- 强制设置为中位（0度倾转，推力为1.0）
                tilt_right = 0.0
                tilt_left = 0.0
                tilt_rear = 0.0
                F1 = 1.0
                F2 = 1.0
                F3 = 1.0
            end
            first_update = false
        end

        -- 设置舵机输出
        TiltControl.set_servo_outputs(tilt_right, tilt_left, tilt_rear)

        -- 重新计算混控因子（使用计算出的推力和角度）
        local factors = motor_factor_table_6dof()
        MixingFactors.calculate_tilt_factors(factors, tilt_right, tilt_left, tilt_rear, F1, F2, F3)

        -- 设置forward和right因子为0（垂直推力，通过倾转实现前向推力）
        for i = 0, 5 do
            factors:forward(i, 0.0)
            factors:right(i, 0.0)
        end

        -- 加载新因子
        Motors_6DoF_dynamic:load_factors(factors)
    else
        -- 不倾转模式：静态混控，无需更新
        -- 因子已在初始化时设置
    end

    return update, UPDATE_RATE_MS
end

-- 发送加载消息
if ENABLE_TILT then
    gcs:send_text(6, "共轴三旋翼Y6B-倾转混控器已加载")
else
    gcs:send_text(6, "共轴三旋翼Y6B混控器已加载")
end
return update()
