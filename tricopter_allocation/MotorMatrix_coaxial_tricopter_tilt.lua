-- 共轴三旋翼倾转电机配置
-- 此脚本配置共轴三旋翼（共6个电机：3个共轴对）
-- 三个电机均可通过倾转实现X轴（pitch）控制
--
-- 电机布局：
--   电机0-1：前左共轴（上桨CW，下桨CCW，或相反）
--   电机2-3：前右共轴（上桨CCW，下桨CW，或相反）
--   电机4-5：后共轴（上桨CW，下桨CCW，或相反）
--
-- 倾转控制：
--   三个电机通过舵机倾转提供pitch控制
--   倾转角度根据pitch控制输出动态计算
--   倾转角度动态影响roll、pitch、throttle因子

-- 配置参数
local TILT_ANGLE_MAX = 45  -- 最大倾转角度（度，默认45）
local TILT_SERVO_LEFT = 9   -- 前左倾转舵机功能号（默认9，根据需要调整）
local TILT_SERVO_RIGHT = 10 -- 前右倾转舵机功能号（默认10，根据需要调整）
local TILT_SERVO_REAR = 11  -- 后置倾转舵机功能号（默认11，根据需要调整）
local TILT_PITCH_GAIN = 1.0 -- Pitch控制增益（默认1.0，用于调参）
local REAR_TILT_RATIO = 0.0 -- 后置倾转比例（0.0=不倾转，1.0=与前电机相同，默认0.0）
local UPDATE_RATE_MS = 10   -- 更新频率（毫秒，100Hz）

-- 控制输出常量（用于获取pitch控制需求）
local CONTROL_OUTPUT_ROLL = 1
local CONTROL_OUTPUT_PITCH = 2
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4

-- 舵机PWM范围（典型值1000-2000，中心1500）
local SERVO_PWM_MIN = 1000
local SERVO_PWM_MAX = 2000
local SERVO_PWM_CENTER = 1500
local SERVO_PWM_RANGE = SERVO_PWM_MAX - SERVO_PWM_MIN

-- 电机索引（从0开始）
local MOTOR_FRONT_LEFT_UPPER = 0
local MOTOR_FRONT_LEFT_LOWER = 1
local MOTOR_FRONT_RIGHT_UPPER = 2
local MOTOR_FRONT_RIGHT_LOWER = 3
local MOTOR_REAR_UPPER = 4
local MOTOR_REAR_LOWER = 5

-- 共轴对的Yaw因子（旋转方向相反抵消yaw，通常为0）
local YAW_FACTOR_COAXIAL = 0  -- 共轴对不产生净yaw力矩

-- 初始化电机及测试顺序
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT_UPPER, 1)
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT_LOWER, 2)
Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT_UPPER, 3)
Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT_LOWER, 4)
Motors_dynamic:add_motor(MOTOR_REAR_UPPER, 5)
Motors_dynamic:add_motor(MOTOR_REAR_LOWER, 6)

-- 创建因子表
local factors = motor_factor_table()

-- 基础配置（pitch因子将动态更新）
-- Roll因子：前左负值，前右正值，后为零
factors:roll(MOTOR_FRONT_LEFT_UPPER, -0.5)
factors:roll(MOTOR_FRONT_LEFT_LOWER, -0.5)
factors:roll(MOTOR_FRONT_RIGHT_UPPER, 0.5)
factors:roll(MOTOR_FRONT_RIGHT_LOWER, 0.5)
factors:roll(MOTOR_REAR_UPPER, 0)
factors:roll(MOTOR_REAR_LOWER, 0)

-- Pitch因子：将根据倾转角度动态更新
-- 初始值（无倾转）
factors:pitch(MOTOR_FRONT_LEFT_UPPER, 0)
factors:pitch(MOTOR_FRONT_LEFT_LOWER, 0)
factors:pitch(MOTOR_FRONT_RIGHT_UPPER, 0)
factors:pitch(MOTOR_FRONT_RIGHT_LOWER, 0)
factors:pitch(MOTOR_REAR_UPPER, 0.5)  -- 后电机提供基础pitch控制
factors:pitch(MOTOR_REAR_LOWER, 0.5)

-- Yaw因子：共轴对相互抵消（无净yaw）
factors:yaw(MOTOR_FRONT_LEFT_UPPER, YAW_FACTOR_COAXIAL)
factors:yaw(MOTOR_FRONT_LEFT_LOWER, YAW_FACTOR_COAXIAL)
factors:yaw(MOTOR_FRONT_RIGHT_UPPER, YAW_FACTOR_COAXIAL)
factors:yaw(MOTOR_FRONT_RIGHT_LOWER, YAW_FACTOR_COAXIAL)
factors:yaw(MOTOR_REAR_UPPER, YAW_FACTOR_COAXIAL)
factors:yaw(MOTOR_REAR_LOWER, YAW_FACTOR_COAXIAL)

-- Throttle因子：所有电机等量贡献
factors:throttle(MOTOR_FRONT_LEFT_UPPER, 1.0)
factors:throttle(MOTOR_FRONT_LEFT_LOWER, 1.0)
factors:throttle(MOTOR_FRONT_RIGHT_UPPER, 1.0)
factors:throttle(MOTOR_FRONT_RIGHT_LOWER, 1.0)
factors:throttle(MOTOR_REAR_UPPER, 1.0)
factors:throttle(MOTOR_REAR_LOWER, 1.0)

-- 加载初始因子并初始化
Motors_dynamic:load_factors(factors)
assert(Motors_dynamic:init(6), "初始化动态混控失败")

motors:set_frame_string("共轴三旋翼倾转")

-- 辅助函数：根据pitch控制输出计算目标倾转角度（弧度）
local function calculate_tilt_angle_from_pitch(pitch_demand)
    -- pitch_demand范围：-1.0 到 1.0
    -- 转换为倾转角度：-TILT_ANGLE_MAX 到 +TILT_ANGLE_MAX（度）
    local tilt_angle_deg = pitch_demand * TILT_ANGLE_MAX
    -- 转换为弧度
    return math.rad(tilt_angle_deg)
end

-- 辅助函数：将倾转角度转换为PWM值
local function tilt_angle_to_pwm(angle_rad)
    -- 将角度（弧度）转换为度
    local angle_deg = math.deg(angle_rad)
    -- 归一化到-1到1范围
    local normalized = angle_deg / TILT_ANGLE_MAX
    -- 转换为PWM（1000-2000，中心1500）
    local pwm = SERVO_PWM_CENTER + normalized * (SERVO_PWM_RANGE / 2)
    -- 限制在有效范围内
    return math.max(SERVO_PWM_MIN, math.min(SERVO_PWM_MAX, pwm))
end

-- 更新函数：根据pitch控制需求计算倾转角度并更新混控因子
local last_tilt_left = nil
local last_tilt_right = nil
local last_tilt_rear = nil
local last_pitch_demand = nil

function update()
    -- 获取pitch控制输出（范围：-1.0 到 1.0）
    local pitch_demand = vehicle:get_control_output(CONTROL_OUTPUT_PITCH)
    
    -- 根据pitch需求计算目标倾转角度
    -- 前左和前右：对称倾转（相反方向）
    local tilt_angle_left = calculate_tilt_angle_from_pitch(pitch_demand)
    local tilt_angle_right = calculate_tilt_angle_from_pitch(-pitch_demand)  -- 相反方向
    -- 后置：根据REAR_TILT_RATIO决定是否倾转
    local tilt_angle_rear = calculate_tilt_angle_from_pitch(pitch_demand * REAR_TILT_RATIO)
    
    -- 检测倾转角度变化
    local tilt_threshold = math.rad(0.5)  -- 0.5度阈值
    local needs_update = false
    
    if last_tilt_left == nil or math.abs(tilt_angle_left - last_tilt_left) > tilt_threshold then
        needs_update = true
        last_tilt_left = tilt_angle_left
    end
    
    if last_tilt_right == nil or math.abs(tilt_angle_right - last_tilt_right) > tilt_threshold then
        needs_update = true
        last_tilt_right = tilt_angle_right
    end
    
    if last_tilt_rear == nil or math.abs(tilt_angle_rear - last_tilt_rear) > tilt_threshold then
        needs_update = true
        last_tilt_rear = tilt_angle_rear
    end
    
    -- 如果pitch需求变化，也需要更新
    if last_pitch_demand == nil or math.abs(pitch_demand - last_pitch_demand) > 0.01 then
        needs_update = true
        last_pitch_demand = pitch_demand
    end
    
    if needs_update then
        -- 设置舵机PWM输出
        local tilt_servo_left_ch = SRV_Channels:find_channel(TILT_SERVO_LEFT)
        local tilt_servo_right_ch = SRV_Channels:find_channel(TILT_SERVO_RIGHT)
        local tilt_servo_rear_ch = SRV_Channels:find_channel(TILT_SERVO_REAR)
        
        if tilt_servo_left_ch then
            local pwm_left = tilt_angle_to_pwm(tilt_angle_left)
            SRV_Channels:set_output_pwm_chan_timeout(tilt_servo_left_ch, pwm_left, 100)
        end
        
        if tilt_servo_right_ch then
            local pwm_right = tilt_angle_to_pwm(tilt_angle_right)
            SRV_Channels:set_output_pwm_chan_timeout(tilt_servo_right_ch, pwm_right, 100)
        end
        
        if tilt_servo_rear_ch and REAR_TILT_RATIO ~= 0.0 then
            local pwm_rear = tilt_angle_to_pwm(tilt_angle_rear)
            SRV_Channels:set_output_pwm_chan_timeout(tilt_servo_rear_ch, pwm_rear, 100)
        end
        
        -- 计算所有混控因子（根据MATLAB推导）
        -- Roll因子：受cos(θ)影响
        factors:roll(MOTOR_FRONT_LEFT_UPPER, -0.5 * math.cos(tilt_angle_left))
        factors:roll(MOTOR_FRONT_LEFT_LOWER, -0.5 * math.cos(tilt_angle_left))
        factors:roll(MOTOR_FRONT_RIGHT_UPPER, 0.5 * math.cos(tilt_angle_right))
        factors:roll(MOTOR_FRONT_RIGHT_LOWER, 0.5 * math.cos(tilt_angle_right))
        factors:roll(MOTOR_REAR_UPPER, 0)  -- 后置ly=0，不贡献roll
        factors:roll(MOTOR_REAR_LOWER, 0)
        
        -- Pitch因子：根据MATLAB推导
        -- 前左：sin项为正（直接推力），cos项为负（位置力矩，简化忽略）
        local pitch_factor_left = math.sin(tilt_angle_left) * TILT_PITCH_GAIN * 0.5
        -- 前右：sin项为负，cos项为正（简化忽略）
        local pitch_factor_right = -math.sin(tilt_angle_right) * TILT_PITCH_GAIN * 0.5
        -- 后置：主要用cos项（位置力矩）
        local pitch_factor_rear = math.cos(tilt_angle_rear) * 0.5
        
        factors:pitch(MOTOR_FRONT_LEFT_UPPER, pitch_factor_left)
        factors:pitch(MOTOR_FRONT_LEFT_LOWER, pitch_factor_left)
        factors:pitch(MOTOR_FRONT_RIGHT_UPPER, pitch_factor_right)
        factors:pitch(MOTOR_FRONT_RIGHT_LOWER, pitch_factor_right)
        factors:pitch(MOTOR_REAR_UPPER, pitch_factor_rear)
        factors:pitch(MOTOR_REAR_LOWER, pitch_factor_rear)
        
        -- Throttle因子：受cos(θ)影响（升力损失）
        factors:throttle(MOTOR_FRONT_LEFT_UPPER, math.cos(tilt_angle_left))
        factors:throttle(MOTOR_FRONT_LEFT_LOWER, math.cos(tilt_angle_left))
        factors:throttle(MOTOR_FRONT_RIGHT_UPPER, math.cos(tilt_angle_right))
        factors:throttle(MOTOR_FRONT_RIGHT_LOWER, math.cos(tilt_angle_right))
        factors:throttle(MOTOR_REAR_UPPER, math.cos(tilt_angle_rear))
        factors:throttle(MOTOR_REAR_LOWER, math.cos(tilt_angle_rear))
        
        -- Yaw因子保持为0（共轴对，已在初始化时设置）
        
        -- 重新加载因子到动态混控器
        Motors_dynamic:load_factors(factors)
    end
    
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "共轴三旋翼倾转混控器已加载")
return update()
