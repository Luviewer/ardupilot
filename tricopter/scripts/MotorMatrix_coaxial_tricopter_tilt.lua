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
-- 倾转功能配置
-- ============================================================================
-- local ENABLE_TILT = false  -- 启用倾转功能（false=不倾转，使用静态混控）
local ENABLE_TILT = true  -- 启用倾转功能（false=不倾转，使用静态混控）

-- 倾转参数（仅在 ENABLE_TILT=true 时有效）
local TILT_ANGLE_MAX = 45.0  -- 最大倾转角度（度）
local TILT_PITCH_GAIN = 1.0  -- Pitch控制增益
local TILT_UPDATE_THRESHOLD = 0.5  -- 倾转角度变化阈值（度），小于此值不更新

-- 倾转方向参数（1.0=正向，-1.0=反向）
-- PWM > 1500时：右前超前(+1)，后方朝前(+1)，左前朝后(-1)
local TILT_DIR_FRONT_RIGHT = 1.0   -- 前右倾转方向（正角度时向前倾转）
local TILT_DIR_FRONT_LEFT = -1.0   -- 前左倾转方向（正角度时向后倾转）
local TILT_DIR_REAR = 1.0          -- 后置倾转方向（正角度时向前倾转）

-- 舵机配置（使用ArduPilot标准倾转舵机功能号）
local TILT_SERVO_FRONT_RIGHT = 76   -- 前右倾转舵机功能号 (k_tiltMotorRight)
local TILT_SERVO_FRONT_LEFT = 75   -- 前左倾转舵机功能号 (k_tiltMotorLeft)
local TILT_SERVO_REAR = 45         -- 后置倾转舵机功能号 (k_tiltMotorRear)

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
local MOTOR_FRONT_RIGHT_UPPER = 0  -- 右上CW
local MOTOR_FRONT_RIGHT_LOWER = 1  -- 右下CCW
local MOTOR_REAR_UPPER = 2         -- 后上CW
local MOTOR_REAR_LOWER = 3         -- 后下CCW
local MOTOR_FRONT_LEFT_UPPER = 4   -- 左上CW
local MOTOR_FRONT_LEFT_LOWER = 5   -- 左下CCW

-- 共轴对的Yaw因子（旋转方向相反抵消yaw，通常为0）
local YAW_FACTOR_COAXIAL_CW = -1  -- 共轴对不产生净yaw力矩
local YAW_FACTOR_COAXIAL_CCW = 1  -- 共轴对不产生净yaw力矩

-- ============================================================================
-- 倾转控制模块
-- ============================================================================
local TiltControl = {}

-- 计算倾转角度（始终使用5DoF模式，从forward_thrust计算）
function TiltControl.calculate_tilt_angles()
    if not ENABLE_TILT then
        return 0.0, 0.0, 0.0  -- 不倾转时返回0
    end
    
    -- 始终从motors获取forward_thrust（5DoF模式）
    -- forward_thrust范围：-1.0 到 +1.0
    -- 根据控制分配矩阵：Fx = -Fi*(sin(a1) + sin(a2) + sin(a3))
    -- 当所有电机倾转角度相同（a1 = a2 = a3 = a）时：Fx = -Fi * 3 * sin(a)
    -- forward_thrust是归一化的值，直接映射到倾转角度范围
    local forward_thrust = motors:get_forward()
    
    -- 检查forward_thrust是否有效（如果为nil或NaN，使用0）
    if forward_thrust == nil or forward_thrust ~= forward_thrust then
        forward_thrust = 0.0
    end
    
    local forward_normalized = math.max(-1.0, math.min(1.0, forward_thrust))
    local tilt_base = forward_normalized * TILT_ANGLE_MAX * TILT_PITCH_GAIN
    
    -- 限制在最大倾转角度内
    tilt_base = math.max(-TILT_ANGLE_MAX, math.min(TILT_ANGLE_MAX, tilt_base))
    
    -- 前右倾转角度（a1）：正forward_thrust时向前倾转（正角度）
    local tilt_right = math.rad(tilt_base)
    
    -- 前左倾转角度（a2）：正forward_thrust时向前倾转（正角度）
    local tilt_left = math.rad(tilt_base)
    
    -- 后置倾转角度（a3）：与前电机相同
    local tilt_rear = math.rad(tilt_base)
    
    return tilt_right, tilt_left, tilt_rear
end

-- 角度转PWM（返回整数）
-- angle_rad: 倾转角度（弧度）
-- direction: 方向系数（1.0=正向，-1.0=反向）
function TiltControl.angle_to_pwm(angle_rad, direction)
    direction = direction or 1.0  -- 默认方向为正
    local angle_deg = math.deg(angle_rad) * direction  -- 应用方向系数
    local normalized = math.max(-1.0, math.min(1.0, angle_deg / TILT_ANGLE_MAX))
    return math.floor(SERVO_PWM_CENTER + normalized * (SERVO_PWM_RANGE / 2) + 0.5)  -- 四舍五入到最近的整数
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

-- 计算动态混控因子（倾转模式，根据MATLAB公式）
function MixingFactors.calculate_tilt_factors(factors, tilt_right, tilt_left, tilt_rear)
    local cos_right = math.cos(tilt_right)
    local cos_left = math.cos(tilt_left)
    local cos_rear = math.cos(tilt_rear)
    local sin_right = math.sin(tilt_right)
    local sin_left = math.sin(tilt_left)
    
    -- Roll因子：根据MATLAB公式 -Fi*ly*(cos(a1) - cos(a2))
    -- 归一化后：前右 cos(a1) 为正，前左 cos(a2) 为负
    factors:roll(MOTOR_FRONT_RIGHT_UPPER, -0.5 * cos_right)
    factors:roll(MOTOR_FRONT_RIGHT_LOWER, -0.5 * cos_right)
    factors:roll(MOTOR_REAR_UPPER, 0)  -- 后置ly=0，不贡献
    factors:roll(MOTOR_REAR_LOWER, 0)
    factors:roll(MOTOR_FRONT_LEFT_UPPER, 0.5 * cos_left)
    factors:roll(MOTOR_FRONT_LEFT_LOWER, 0.5 * cos_left)
    
    -- Pitch因子：根据MATLAB公式 -Fi*lx*(cos(a2) - cos(a1) + cos(a3))
    -- 归一化后：前左 -cos(a2)，前右 +cos(a1)，后置 +cos(a3)
    -- 注意：这里需要根据实际布局调整符号和比例
    local pitch_factor_right = 0.5 * cos_right  -- 前右贡献
    local pitch_factor_left = -0.5 * cos_left   -- 前左贡献（负号）
    local pitch_factor_rear = -1.0 * cos_rear   -- 后置贡献
    
    factors:pitch(MOTOR_FRONT_RIGHT_UPPER, pitch_factor_right)
    factors:pitch(MOTOR_FRONT_RIGHT_LOWER, pitch_factor_right)
    factors:pitch(MOTOR_FRONT_LEFT_UPPER, pitch_factor_left)
    factors:pitch(MOTOR_FRONT_LEFT_LOWER, pitch_factor_left)
    factors:pitch(MOTOR_REAR_UPPER, pitch_factor_rear)
    factors:pitch(MOTOR_REAR_LOWER, pitch_factor_rear)
    
    -- Yaw因子：根据MATLAB公式 Fi*ly*(sin(a1) - sin(a2))
    -- 归一化后：前右 sin(a1) 为正，前左 -sin(a2) 为负
    -- 注意：共轴对的yaw因子需要与旋转方向结合
    local yaw_factor_right = 0.5 * sin_right
    local yaw_factor_left = -0.5 * sin_left
    
    factors:yaw(MOTOR_FRONT_RIGHT_UPPER, YAW_FACTOR_COAXIAL_CW + yaw_factor_right)
    factors:yaw(MOTOR_FRONT_RIGHT_LOWER, YAW_FACTOR_COAXIAL_CCW + yaw_factor_right)
    factors:yaw(MOTOR_FRONT_LEFT_UPPER, YAW_FACTOR_COAXIAL_CW + yaw_factor_left)
    factors:yaw(MOTOR_FRONT_LEFT_LOWER, YAW_FACTOR_COAXIAL_CCW + yaw_factor_left)
    factors:yaw(MOTOR_REAR_UPPER, YAW_FACTOR_COAXIAL_CW)  -- 后置ly=0，不贡献
    factors:yaw(MOTOR_REAR_LOWER, YAW_FACTOR_COAXIAL_CCW)
    
    -- Throttle因子：根据MATLAB公式 -Fi*(cos(a1) + cos(a2) + cos(a3))
    -- 倾转导致升力损失，通过cos项补偿
    factors:throttle(MOTOR_FRONT_RIGHT_UPPER, cos_right)
    factors:throttle(MOTOR_FRONT_RIGHT_LOWER, cos_right)
    factors:throttle(MOTOR_FRONT_LEFT_UPPER, cos_left)
    factors:throttle(MOTOR_FRONT_LEFT_LOWER, cos_left)
    factors:throttle(MOTOR_REAR_UPPER, cos_rear)
    factors:throttle(MOTOR_REAR_LOWER, cos_rear)
end

-- 初始化电机及测试顺序（按照电机索引0-5顺序）
-- 注意：Motors_6DoF_dynamic需要3个参数：motor_num, testing_order, reversible
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_RIGHT_UPPER, 1, false)  -- 电机0：右上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_RIGHT_LOWER, 2, false)  -- 电机1：右下，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_REAR_UPPER, 3, false)         -- 电机2：后上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_REAR_LOWER, 4, false)         -- 电机3：后下，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_LEFT_UPPER, 5, false)    -- 电机4：左上，不可逆
Motors_6DoF_dynamic:add_motor(MOTOR_FRONT_LEFT_LOWER, 6, false)    -- 电机5：左下，不可逆

-- 创建因子表（6DOF版本，包含forward和right因子）
local factors = motor_factor_table_6dof()

-- 根据ENABLE_TILT选择初始化方式
if ENABLE_TILT then
    -- 倾转模式：初始化为0度倾转状态
    MixingFactors.calculate_tilt_factors(factors, 0.0, 0.0, 0.0)
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
local last_tilt_right, last_tilt_left, last_tilt_rear = nil, nil, nil  -- 初始化为nil，确保首次更新
local first_update = true  -- 首次更新标志

function update()
    if ENABLE_TILT then
        -- 倾转模式：动态计算混控因子
        local tilt_right, tilt_left, tilt_rear = TiltControl.calculate_tilt_angles()
        
        -- 首次更新时，如果forward_thrust异常（绝对值很大），强制设置为0（中位）
        if first_update then
            local forward_thrust = motors:get_forward()
            if forward_thrust == nil or forward_thrust ~= forward_thrust or math.abs(forward_thrust) > 0.5 then
                -- 强制设置为中位（0度倾转）
                tilt_right = 0.0
                tilt_left = 0.0
                tilt_rear = 0.0
            end
            first_update = false
        end
        
        -- 检查是否需要更新（避免频繁更新）
        -- 首次运行时（last_tilt_right为nil）强制更新
        local needs_update = false
        if last_tilt_right == nil or
           math.abs(tilt_right - last_tilt_right) > math.rad(TILT_UPDATE_THRESHOLD) or
           math.abs(tilt_left - last_tilt_left) > math.rad(TILT_UPDATE_THRESHOLD) or
           math.abs(tilt_rear - last_tilt_rear) > math.rad(TILT_UPDATE_THRESHOLD) then
            needs_update = true
            last_tilt_right = tilt_right
            last_tilt_left = tilt_left
            last_tilt_rear = tilt_rear
        end
        
        if needs_update then
            -- 设置舵机输出
            TiltControl.set_servo_outputs(tilt_right, tilt_left, tilt_rear)
            
            -- 重新计算混控因子
            local factors = motor_factor_table_6dof()
            MixingFactors.calculate_tilt_factors(factors, tilt_right, tilt_left, tilt_rear)
            
            -- 设置forward和right因子为0（垂直推力）
            for i = 0, 5 do
                factors:forward(i, 0.0)
                factors:right(i, 0.0)
            end
            
            -- 加载新因子
            Motors_6DoF_dynamic:load_factors(factors)
        end
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
