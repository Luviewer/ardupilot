-- 共轴三旋翼Y6B电机配置
-- 此脚本配置共轴三旋翼（共6个电机：3个共轴对）
-- 使用标准Y6B混控配置，无倾转控制
--
-- 电机布局：
--   电机0-1：前右共轴（上桨CW，下桨CCW）
--   电机2-3：后共轴（上桨CW，下桨CCW）
--   电机4-5：前左共轴（上桨CW，下桨CCW）

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

-- 初始化电机及测试顺序（按照电机索引0-5顺序）
Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT_UPPER, 1)  -- 电机0：右上
Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT_LOWER, 2)  -- 电机1：右下
Motors_dynamic:add_motor(MOTOR_REAR_UPPER, 3)         -- 电机2：后上
Motors_dynamic:add_motor(MOTOR_REAR_LOWER, 4)         -- 电机3：后下
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT_UPPER, 5)    -- 电机4：左上
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT_LOWER, 6)    -- 电机5：左下

-- 创建因子表
local factors = motor_factor_table()

-- Y6B混控因子配置
-- Roll因子：前左负值，前右正值，后为零
factors:roll(MOTOR_FRONT_RIGHT_UPPER, -1.0)
factors:roll(MOTOR_FRONT_RIGHT_LOWER, -1.0)
factors:roll(MOTOR_FRONT_LEFT_UPPER, 1.0)
factors:roll(MOTOR_FRONT_LEFT_LOWER, 1.0)
factors:roll(MOTOR_REAR_UPPER, 0)
factors:roll(MOTOR_REAR_LOWER, 0)

-- Pitch因子：前电机0.5，后电机-1.0
factors:pitch(MOTOR_FRONT_RIGHT_UPPER, 0.5)
factors:pitch(MOTOR_FRONT_RIGHT_LOWER, 0.5)
factors:pitch(MOTOR_REAR_UPPER, -1.0)  -- 后电机提供pitch控制
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

-- 加载初始因子并初始化
Motors_dynamic:load_factors(factors)
assert(Motors_dynamic:init(6), "初始化动态混控失败")

motors:set_frame_string("共轴三旋翼Y6B")

-- 更新函数：静态混控，无需动态更新
function update()
    -- 静态混控，因子已在初始化时设置，无需更新
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "共轴三旋翼Y6B混控器已加载")
return update()
