--[[
   共轴三旋翼Y6B倾转控制脚本
   
   版本: 1.0.0
   日期: 2026-01-14
   
   描述:
   基于MATLAB控制分配矩阵伪逆法推导的共轴三旋翼(Y6B)动态混控脚本，支持倾转旋翼功能。
   
   电机布局:
   - 电机 0-1: 前右共轴对 (上桨CW顺时针, 下桨CCW逆时针)
   - 电机 2-3: 后置共轴对 (上桨CW顺时针, 下桨CCW逆时针)
   - 电机 4-5: 前左共轴对 (上桨CW顺时针, 下桨CCW逆时针)
   
   系统要求:
   - ArduPilot 4.5+
   - FRAME_CLASS = 16 (6自由度)
   - MOT_PWM_TYPE 兼容动态混控
   
   参数说明:
   - TRI_ENABLE: 启用/禁用倾转控制
   - TRI_ANGLE_MAX: 最大倾转角度（度）
   - TRI_LX, TRI_LY, TRI_LREAR: 几何参数（归一化值）
   - TRI_SERVO_FR/FL/REAR: 倾转舵机功能号
   
   ArduPilot混控约定:
   - Roll横滚: 正值 = 左滚转（左侧向下）
   - Pitch俯仰: 正值 = 机头上仰
   - Yaw偏航: 正值 = 顺时针旋转（俯视）
   - Throttle油门: 正值 = 向上推力
--]]

-- ============================================================================
-- 配置常量定义
-- ============================================================================
local Config = {
    VERSION = "1.0.0",
    SCRIPT_NAME = "共轴Y6B倾转",
    
    -- 更新频率
    UPDATE_RATE_MS = 10,  -- 100Hz
    
    -- 舵机PWM范围
    SERVO_PWM_MIN = 1000,
    SERVO_PWM_MAX = 2000,
    SERVO_PWM_CENTER = 1500,
    
    -- 安全限制
    MIN_VALUE = 0.01,      -- 除零保护
    MAX_THRUST = 1.0,      -- 最大归一化推力
    
    -- 共轴电机偏航因子
    YAW_COAXIAL_CW = -1,   -- 顺时针电机
    YAW_COAXIAL_CCW = 1,   -- 逆时针电机
    
    -- 电机索引
    MOTOR = {
        FRONT_RIGHT_UPPER = 0,  -- 前右上 CW
        FRONT_RIGHT_LOWER = 1,  -- 前右下 CCW
        REAR_UPPER = 2,         -- 后上 CW
        REAR_LOWER = 3,         -- 后下 CCW
        FRONT_LEFT_UPPER = 4,   -- 前左上 CW
        FRONT_LEFT_LOWER = 5,   -- 前左下 CCW
    },
}

-- 计算得出的常量
Config.SERVO_PWM_RANGE = Config.SERVO_PWM_MAX - Config.SERVO_PWM_MIN

-- ============================================================================
-- 工具函数模块
-- ============================================================================
local Utils = {}

-- 检查数值是否有效（非NaN或无穷）
function Utils.is_valid_number(value)
    return value == value and value ~= math.huge and value ~= -math.huge
end

-- 验证并清理数值（无效值替换为默认值）
function Utils.sanitize_number(value, default)
    if Utils.is_valid_number(value) then
        return value
    end
    return default
end

-- 限制数值在最小值和最大值之间
function Utils.constrain(value, min_val, max_val)
    return math.max(min_val, math.min(max_val, value))
end

-- 限制角度在 +/- 最大角度之间
function Utils.constrain_angle(angle, max_angle)
    return Utils.constrain(angle, -max_angle, max_angle)
end

-- 安全求最大值（除零保护）
function Utils.safe_max(value, min_val)
    return math.max(math.abs(value), min_val)
end

-- ============================================================================
-- 参数表配置
-- ============================================================================
local PARAM_TABLE_KEY = 100
local PARAM_TABLE_PREFIX = "TRI_"

-- 添加并绑定参数的辅助函数
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
        string.format('添加参数失败: %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 创建参数表（15个参数）
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15),
    '创建参数表失败')

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
  // @Description: 前电机X轴方向距离（归一化值）
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_LX_PARAM = bind_add_param('LX', 5, 1.0)

--[[
  // @Param: TRI_LY
  // @DisplayName: 控制分配LY
  // @Description: Y轴方向距离（左右电机间距的一半，归一化值）
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

--[[
  // @Param: TRI_LREAR
  // @DisplayName: 控制分配LREAR
  // @Description: 后置电机X轴方向距离（归一化值）
  // @Range: 0.1 10
  // @User: Advanced
--]]
local TRI_LREAR_PARAM = bind_add_param('LREAR', 15, 1.0)

-- ============================================================================
-- 读取并验证参数
-- ============================================================================
local Params = {
    enable_tilt = TRI_ENABLE_PARAM:get() > 0.5,
    angle_max = TRI_ANGLE_MAX_PARAM:get(),
    pitch_gain = TRI_PITCH_GAIN_PARAM:get(),
    update_threshold = TRI_UPDATE_THR_PARAM:get(),
    
    -- 几何参数
    lx = TRI_LX_PARAM:get(),
    ly = TRI_LY_PARAM:get(),
    lrear = TRI_LREAR_PARAM:get(),
    
    -- 缩放系数
    force_scale = TRI_FORCE_SCALE_PARAM:get(),
    moment_scale = TRI_MOMENT_SCALE_PARAM:get(),
    
    -- 舵机方向
    dir_front_right = TRI_DIR_FR_PARAM:get(),
    dir_front_left = TRI_DIR_FL_PARAM:get(),
    dir_rear = TRI_DIR_REAR_PARAM:get(),
    
    -- 舵机通道
    servo_front_right = math.floor(TRI_SERVO_FR_PARAM:get()),
    servo_front_left = math.floor(TRI_SERVO_FL_PARAM:get()),
    servo_rear = math.floor(TRI_SERVO_REAR_PARAM:get()),
}

-- 参数验证函数
local function validate_params()
    -- 验证几何参数
    if Params.lx < Config.MIN_VALUE or Params.ly < Config.MIN_VALUE or Params.lrear < Config.MIN_VALUE then
        gcs:send_text(0, string.format("%s: 错误 - 几何参数无效", Config.SCRIPT_NAME))
        return false
    end
    
    -- 验证角度范围
    if Params.angle_max < 0 or Params.angle_max > 90 then
        gcs:send_text(0, string.format("%s: 错误 - ANGLE_MAX超出范围 [0, 90]", Config.SCRIPT_NAME))
        return false
    end
    
    -- 如果启用倾转，验证舵机通道
    if Params.enable_tilt then
        if Params.servo_front_right < 0 or Params.servo_front_left < 0 or Params.servo_rear < 0 then
            gcs:send_text(0, string.format("%s: 错误 - 舵机通道无效", Config.SCRIPT_NAME))
            return false
        end
    end
    
    return true
end

-- 启动时验证参数
if not validate_params() then
    gcs:send_text(0, string.format("%s: 加载失败 - 参数无效", Config.SCRIPT_NAME))
    return
end

-- ============================================================================
-- 控制分配模块（基于F_alloc矩阵伪逆）
-- ============================================================================
local AllocationControl = {}

--[[
   使用F_alloc矩阵的伪逆求解中间变量
   
   输入: Fx, Fz, Mx, My, Mz (力和力矩需求)
   输出: f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3 (中间变量)
   其中: f_s = F*sin(角度), f_c = F*cos(角度)
   
   伪逆矩阵（来自MATLAB）:
                      Fx       Fz                              Mx              My              Mz
     f1_s1 [          -1/3        0                               0               0    1/(2*lfront_y) ]
     f1_c1 [             0     -1/2  (lfront_x - lrear)/(2*lfront_y*lrear)  1/(2*lrear)               0 ]
     f2_s2 [          -1/3        0                               0               0               0 ]
     f2_c2 [             0        0         -lfront_x/(lfront_y*lrear)       -1/lrear               0 ]
     f3_s3 [          -1/3        0                               0               0   -1/(2*lfront_y) ]
     f3_c3 [             0     -1/2  (lfront_x + lrear)/(2*lfront_y*lrear)  1/(2*lrear)               0 ]
--]]
function AllocationControl.solve_intermediate_variables(Fx, Fz, Mx, My, Mz)
    -- 保护的几何值
    local ly = Utils.safe_max(Params.ly, Config.MIN_VALUE)
    local lx = Utils.safe_max(Params.lx, Config.MIN_VALUE)
    local lr = Utils.safe_max(Params.lrear, Config.MIN_VALUE)
    
    -- 使用伪逆矩阵计算中间变量
    local f1_s1 = -Fx / 3.0 + Mz / (2.0 * ly)
    local f1_c1 = -Fz / 2.0 + Mx * (lx - lr) / (2.0 * ly * lr) + My / (2.0 * lr)
    local f2_s2 = -Fx / 3.0
    local f2_c2 = -Mx * lx / (ly * lr) - My / lr
    local f3_s3 = -Fx / 3.0 - Mz / (2.0 * ly)
    local f3_c3 = -Fz / 2.0 + Mx * (lx + lr) / (2.0 * ly * lr) + My / (2.0 * lr)
    
    return f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3
end

-- 从中间变量计算推力和角度
-- 输入: f_sin = F*sin(a), f_cos = F*cos(a)
-- 输出: F (推力), a (角度，弧度)
function AllocationControl.compute_thrust_and_angle(f_sin, f_cos)
    local F = math.sqrt(f_sin * f_sin + f_cos * f_cos)
    local angle = math.atan2(f_sin, f_cos)
    return F, angle
end

-- ============================================================================
-- 倾转控制模块
-- ============================================================================
local TiltControl = {}

-- 计算倾转角度和推力
-- 返回: tilt_right, tilt_left, tilt_rear (弧度), F1, F2, F3 (推力)
function TiltControl.calculate_tilt_angles()
    if not Params.enable_tilt then
        return 0.0, 0.0, 0.0, 1.0, 1.0, 1.0
    end
    
    -- 获取控制输入（归一化值 -1 到 1）
    local roll = Utils.sanitize_number(motors:get_roll(), 0.0)
    local pitch = Utils.sanitize_number(motors:get_pitch(), 0.0)
    local yaw = Utils.sanitize_number(motors:get_yaw(), 0.0)
    local throttle = Utils.sanitize_number(motors:get_throttle(), 0.0)
    local forward = Utils.sanitize_number(motors:get_forward(), 0.0)
    
    -- 转换为力和力矩
    local Fx = forward * Params.force_scale
    local Fz = throttle * Params.force_scale
    local Mx = roll * Params.moment_scale
    local My = pitch * Params.moment_scale
    local Mz = yaw * Params.moment_scale
    
    -- 求解中间变量
    local f1_s1, f2_s2, f3_s3, f1_c1, f2_c2, f3_c3 =
        AllocationControl.solve_intermediate_variables(Fx, Fz, Mx, My, Mz)
    
    -- 清理中间变量
    f1_s1 = Utils.sanitize_number(f1_s1, 0.0)
    f1_c1 = Utils.sanitize_number(f1_c1, 0.0)
    f2_s2 = Utils.sanitize_number(f2_s2, 0.0)
    f2_c2 = Utils.sanitize_number(f2_c2, 0.0)
    f3_s3 = Utils.sanitize_number(f3_s3, 0.0)
    f3_c3 = Utils.sanitize_number(f3_c3, 0.0)
    
    -- 计算推力和角度
    local F1, a1 = AllocationControl.compute_thrust_and_angle(f1_s1, f1_c1)
    local F2, a2 = AllocationControl.compute_thrust_and_angle(f2_s2, f2_c2)
    local F3, a3 = AllocationControl.compute_thrust_and_angle(f3_s3, f3_c3)
    
    -- 限制角度在最大倾转范围内
    local max_angle_rad = math.rad(Params.angle_max)
    a1 = Utils.constrain_angle(a1, max_angle_rad)
    a2 = Utils.constrain_angle(a2, max_angle_rad)
    a3 = Utils.constrain_angle(a3, max_angle_rad)
    
    -- 限制推力在合理范围内
    F1 = Utils.constrain(F1, 0.0, Config.MAX_THRUST)
    F2 = Utils.constrain(F2, 0.0, Config.MAX_THRUST)
    F3 = Utils.constrain(F3, 0.0, Config.MAX_THRUST)
    
    -- 返回: tilt_right, tilt_left, tilt_rear, F1, F2, F3
    return a1, a3, a2, F1, F2, F3
end

-- 将角度转换为PWM值
function TiltControl.angle_to_pwm(angle_rad, direction)
    local angle_deg = math.deg(angle_rad) * direction
    local normalized = Utils.constrain(angle_deg / Params.angle_max, -1.0, 1.0)
    return math.floor(Config.SERVO_PWM_CENTER + normalized * (Config.SERVO_PWM_RANGE / 2) + 0.5)
end

-- 设置倾转舵机输出
function TiltControl.set_servo_outputs(tilt_right, tilt_left, tilt_rear)
    if not Params.enable_tilt then
        return
    end
    
    local ch_right = SRV_Channels:find_channel(Params.servo_front_right)
    local ch_left = SRV_Channels:find_channel(Params.servo_front_left)
    local ch_rear = SRV_Channels:find_channel(Params.servo_rear)
    
    if ch_right then
        SRV_Channels:set_output_pwm_chan_timeout(ch_right,
            TiltControl.angle_to_pwm(tilt_right, Params.dir_front_right), 100)
    end
    if ch_left then
        SRV_Channels:set_output_pwm_chan_timeout(ch_left,
            TiltControl.angle_to_pwm(tilt_left, Params.dir_front_left), 100)
    end
    if ch_rear then
        SRV_Channels:set_output_pwm_chan_timeout(ch_rear,
            TiltControl.angle_to_pwm(tilt_rear, Params.dir_rear), 100)
    end
end

-- ============================================================================
-- 混控因子计算模块
-- ============================================================================
local MixingFactors = {}

-- 计算静态混控因子（非倾转模式）
function MixingFactors.calculate_static(factors)
    local M = Config.MOTOR
    
    -- Roll因子: 左正，右负，后零
    factors:roll(M.FRONT_RIGHT_UPPER, -1.0)
    factors:roll(M.FRONT_RIGHT_LOWER, -1.0)
    factors:roll(M.REAR_UPPER, 0.0)
    factors:roll(M.REAR_LOWER, 0.0)
    factors:roll(M.FRONT_LEFT_UPPER, 1.0)
    factors:roll(M.FRONT_LEFT_LOWER, 1.0)
    
    -- Pitch因子: 前 0.5，后 -1.0
    factors:pitch(M.FRONT_RIGHT_UPPER, 0.5)
    factors:pitch(M.FRONT_RIGHT_LOWER, 0.5)
    factors:pitch(M.REAR_UPPER, -1.0)
    factors:pitch(M.REAR_LOWER, -1.0)
    factors:pitch(M.FRONT_LEFT_UPPER, 0.5)
    factors:pitch(M.FRONT_LEFT_LOWER, 0.5)
    
    -- Yaw因子: 共轴对相互抵消
    factors:yaw(M.FRONT_RIGHT_UPPER, Config.YAW_COAXIAL_CW)
    factors:yaw(M.FRONT_RIGHT_LOWER, Config.YAW_COAXIAL_CCW)
    factors:yaw(M.REAR_UPPER, Config.YAW_COAXIAL_CW)
    factors:yaw(M.REAR_LOWER, Config.YAW_COAXIAL_CCW)
    factors:yaw(M.FRONT_LEFT_UPPER, Config.YAW_COAXIAL_CW)
    factors:yaw(M.FRONT_LEFT_LOWER, Config.YAW_COAXIAL_CCW)
    
    -- Throttle因子: 等量贡献
    for i = 0, 5 do
        factors:throttle(i, 1.0)
    end
end

-- 计算动态混控因子（倾转模式）
function MixingFactors.calculate_tilt(factors, tilt_right, tilt_left, tilt_rear)
    local M = Config.MOTOR
    
    -- 计算三角函数值
    local cos_right = Utils.sanitize_number(math.cos(tilt_right), 1.0)
    local cos_left = Utils.sanitize_number(math.cos(tilt_left), 1.0)
    local cos_rear = Utils.sanitize_number(math.cos(tilt_rear), 1.0)
    local sin_right = Utils.sanitize_number(math.sin(tilt_right), 0.0)
    local sin_left = Utils.sanitize_number(math.sin(tilt_left), 0.0)
    local sin_rear = Utils.sanitize_number(math.sin(tilt_rear), 0.0)
    
    -- 保护的几何值
    local ly = Utils.safe_max(Params.ly, Config.MIN_VALUE)
    local lx = Utils.safe_max(Params.lx, Config.MIN_VALUE)
    local lr = Utils.safe_max(Params.lrear, Config.MIN_VALUE)
    
    -- Roll因子: Mx = -ly*f1_c1 + ly*f3_c3
    local roll_right = -ly * cos_right
    local roll_left = ly * cos_left
    local roll_rear = 0.0
    
    local roll_scale = 1.0 / Utils.safe_max(math.max(math.abs(roll_right), math.abs(roll_left)), 0.01)
    factors:roll(M.FRONT_RIGHT_UPPER, roll_right * roll_scale)
    factors:roll(M.FRONT_RIGHT_LOWER, roll_right * roll_scale)
    factors:roll(M.REAR_UPPER, roll_rear)
    factors:roll(M.REAR_LOWER, roll_rear)
    factors:roll(M.FRONT_LEFT_UPPER, roll_left * roll_scale)
    factors:roll(M.FRONT_LEFT_LOWER, roll_left * roll_scale)
    
    -- Pitch因子: My = lx*f1_c1 - lr*f2_c2 - lx*f3_c3
    local pitch_right = lx * cos_right
    local pitch_rear = -lr * cos_rear
    local pitch_left = -lx * cos_left
    
    local pitch_scale = 1.0 / Utils.safe_max(math.max(math.abs(pitch_right), math.abs(pitch_rear), math.abs(pitch_left)), 0.01)
    factors:pitch(M.FRONT_RIGHT_UPPER, pitch_right * pitch_scale)
    factors:pitch(M.FRONT_RIGHT_LOWER, pitch_right * pitch_scale)
    factors:pitch(M.REAR_UPPER, pitch_rear * pitch_scale)
    factors:pitch(M.REAR_LOWER, pitch_rear * pitch_scale)
    factors:pitch(M.FRONT_LEFT_UPPER, pitch_left * pitch_scale)
    factors:pitch(M.FRONT_LEFT_LOWER, pitch_left * pitch_scale)
    
    -- Yaw因子: Mz = ly*f1_s1 - ly*f3_s3
    local yaw_right = ly * sin_right
    local yaw_left = -ly * sin_left
    local yaw_rear = 0.0
    
    local yaw_scale = 1.0 / Utils.safe_max(math.max(math.abs(yaw_right), math.abs(yaw_left)), 0.01)
    factors:yaw(M.FRONT_RIGHT_UPPER, Config.YAW_COAXIAL_CW + yaw_right * yaw_scale)
    factors:yaw(M.FRONT_RIGHT_LOWER, Config.YAW_COAXIAL_CCW + yaw_right * yaw_scale)
    factors:yaw(M.REAR_UPPER, Config.YAW_COAXIAL_CW + yaw_rear)
    factors:yaw(M.REAR_LOWER, Config.YAW_COAXIAL_CCW + yaw_rear)
    factors:yaw(M.FRONT_LEFT_UPPER, Config.YAW_COAXIAL_CW + yaw_left * yaw_scale)
    factors:yaw(M.FRONT_LEFT_LOWER, Config.YAW_COAXIAL_CCW + yaw_left * yaw_scale)
    
    -- Throttle因子: Fz = -f1_c1 - f2_c2 - f3_c3 (补偿倾转导致的升力损失)
    local throttle_sum = cos_right + cos_rear + cos_left
    local throttle_scale = 3.0
    if throttle_sum > 0.01 then
        throttle_scale = 3.0 / throttle_sum
    end
    
    factors:throttle(M.FRONT_RIGHT_UPPER, cos_right * throttle_scale)
    factors:throttle(M.FRONT_RIGHT_LOWER, cos_right * throttle_scale)
    factors:throttle(M.REAR_UPPER, cos_rear * throttle_scale)
    factors:throttle(M.REAR_LOWER, cos_rear * throttle_scale)
    factors:throttle(M.FRONT_LEFT_UPPER, cos_left * throttle_scale)
    factors:throttle(M.FRONT_LEFT_LOWER, cos_left * throttle_scale)
end

-- ============================================================================
-- 初始化
-- ============================================================================
local function init_motors()
    local M = Config.MOTOR
    
    -- 添加电机 (电机号, 测试顺序, 可逆性)
    Motors_6DoF_dynamic:add_motor(M.FRONT_RIGHT_UPPER, 1, false)
    Motors_6DoF_dynamic:add_motor(M.FRONT_RIGHT_LOWER, 2, false)
    Motors_6DoF_dynamic:add_motor(M.REAR_UPPER, 3, false)
    Motors_6DoF_dynamic:add_motor(M.REAR_LOWER, 4, false)
    Motors_6DoF_dynamic:add_motor(M.FRONT_LEFT_UPPER, 5, false)
    Motors_6DoF_dynamic:add_motor(M.FRONT_LEFT_LOWER, 6, false)
    
    -- 创建并初始化因子表
    local factors = motor_factor_table_6dof()
    
    if Params.enable_tilt then
        -- 倾转模式: 初始化为零倾转
        MixingFactors.calculate_tilt(factors, 0.0, 0.0, 0.0)
    else
        -- 静态模式: 使用固定因子
        MixingFactors.calculate_static(factors)
    end
    
    -- 设置前向和横向因子为零（仅垂直推力）
    for i = 0, 5 do
        factors:forward(i, 0.0)
        factors:right(i, 0.0)
    end
    
    -- 加载因子并初始化
    Motors_6DoF_dynamic:load_factors(factors)
    assert(Motors_6DoF_dynamic:init(6), "初始化Motors_6DoF_dynamic失败")
    
    -- 设置机架名称
    local frame_name = Params.enable_tilt and "共轴Y6B倾转" or "共轴Y6B"
    motors:set_frame_string(frame_name)
    
    return factors
end

-- 初始化电机
local cached_factors = init_motors()

-- ============================================================================
-- 更新循环
-- ============================================================================
local update_state = {
    first_run = true,
    last_angles = {right = 0.0, left = 0.0, rear = 0.0},
}

function update()
    if not Params.enable_tilt then
        -- 静态模式: 无需更新
        return update, Config.UPDATE_RATE_MS
    end
    
    -- 计算倾转角度和推力
    local tilt_right, tilt_left, tilt_rear, F1, F2, F3 = TiltControl.calculate_tilt_angles()
    
    -- 首次运行保护: 如果值无效则强制为零倾转
    if update_state.first_run then
        if not (Utils.is_valid_number(tilt_right) and Utils.is_valid_number(tilt_left) and Utils.is_valid_number(tilt_rear)) then
            tilt_right, tilt_left, tilt_rear = 0.0, 0.0, 0.0
        end
        update_state.first_run = false
    end
    
    -- 更新舵机输出
    TiltControl.set_servo_outputs(tilt_right, tilt_left, tilt_rear)
    
    -- 更新混控因子
    MixingFactors.calculate_tilt(cached_factors, tilt_right, tilt_left, tilt_rear)
    
    -- 设置前向和横向因子为零
    for i = 0, 5 do
        cached_factors:forward(i, 0.0)
        cached_factors:right(i, 0.0)
    end
    
    -- 加载更新后的因子
    Motors_6DoF_dynamic:load_factors(cached_factors)
    
    -- 存储当前角度
    update_state.last_angles.right = tilt_right
    update_state.last_angles.left = tilt_left
    update_state.last_angles.rear = tilt_rear
    
    return update, Config.UPDATE_RATE_MS
end

-- ============================================================================
-- 启动
-- ============================================================================
gcs:send_text(6, string.format("%s v%s 已加载", Config.SCRIPT_NAME, Config.VERSION))

return update()
