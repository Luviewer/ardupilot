-- 共轴三旋翼倾转电机配置
-- 此脚本配置共轴三旋翼（共6个电机：3个共轴对）
-- 前左和前右电机可通过倾转实现X轴（pitch）控制
--
-- 电机布局：
--   电机0-1：前左共轴（上桨CW，下桨CCW，或相反）
--   电机2-3：前右共轴（上桨CCW，下桨CW，或相反）
--   电机4-5：后共轴（上桨CW，下桨CCW，或相反）
--
-- 倾转控制：
--   前左和前右电机通过舵机倾转提供pitch控制
--   倾转角度动态影响pitch因子

-- 配置参数
local TILT_ANGLE_MAX = 45  -- 最大倾转角度（度，默认45）
local TILT_SERVO_LEFT = 9   -- 前左倾转舵机功能号（默认9，根据需要调整）
local TILT_SERVO_RIGHT = 10 -- 前右倾转舵机功能号（默认10，根据需要调整）
local TILT_PITCH_GAIN = 1.0 -- Pitch控制增益（默认1.0，用于调参）
local UPDATE_RATE_MS = 10   -- 更新频率（毫秒，100Hz）

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

-- 查找倾转舵机通道
local tilt_servo_left_ch = SRV_Channels:find_channel(TILT_SERVO_LEFT)
local tilt_servo_right_ch = SRV_Channels:find_channel(TILT_SERVO_RIGHT)

-- 辅助函数：将舵机PWM转换为倾转角度（弧度）
local function pwm_to_tilt_angle(pwm)
    -- 将PWM（1000-2000）转换为归一化值（-1到1）
    local normalized = (pwm - SERVO_PWM_CENTER) / (SERVO_PWM_RANGE / 2)
    -- 转换为倾转角度（弧度）
    local angle_rad = math.rad(normalized * TILT_ANGLE_MAX)
    return angle_rad
end

-- 辅助函数：从舵机输出获取倾转角度
local function get_tilt_angle(servo_func_num)
    local servo_ch = SRV_Channels:find_channel(servo_func_num)
    if not servo_ch then
        return 0  -- 如果未找到舵机则无倾转
    end
    local pwm = SRV_Channels:get_output_pwm(servo_func_num)
    if not pwm then
        return 0  -- 如果PWM不可用则无倾转
    end
    return pwm_to_tilt_angle(pwm)
end

-- 更新函数：根据倾转角度动态调整pitch因子
local last_tilt_left = nil
local last_tilt_right = nil

function update()
    -- 获取当前倾转角度
    local tilt_angle_left = get_tilt_angle(TILT_SERVO_LEFT)
    local tilt_angle_right = get_tilt_angle(TILT_SERVO_RIGHT)
    
    -- 仅在倾转角度显著变化时更新（避免不必要的更新）
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
    
    if needs_update then
        -- 根据倾转角度计算pitch因子
        -- 前左：正倾转角度产生正pitch力矩
        local pitch_factor_left = math.sin(tilt_angle_left) * TILT_PITCH_GAIN * 0.5
        -- 前右：正倾转角度产生负pitch力矩（相反方向）
        local pitch_factor_right = -math.sin(tilt_angle_right) * TILT_PITCH_GAIN * 0.5
        
        -- 更新前电机的pitch因子
        factors:pitch(MOTOR_FRONT_LEFT_UPPER, pitch_factor_left)
        factors:pitch(MOTOR_FRONT_LEFT_LOWER, pitch_factor_left)
        factors:pitch(MOTOR_FRONT_RIGHT_UPPER, pitch_factor_right)
        factors:pitch(MOTOR_FRONT_RIGHT_LOWER, pitch_factor_right)
        
        -- 后电机保持基础pitch控制（如需要可调整）
        -- 后pitch因子用于平衡前倾转电机
        local rear_pitch_base = 0.5
        factors:pitch(MOTOR_REAR_UPPER, rear_pitch_base)
        factors:pitch(MOTOR_REAR_LOWER, rear_pitch_base)
        
        -- 重新加载因子到动态混控器
        Motors_dynamic:load_factors(factors)
    end
    
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "共轴三旋翼倾转混控器已加载")
return update()
