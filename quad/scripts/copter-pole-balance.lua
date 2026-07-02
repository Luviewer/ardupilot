--[[
    copter-pole-balance.lua

    目标：
      在 Copter GUIDED 模式下，接收 Gazebo 圆杆通过 MAVLink ODOMETRY 发来的位置/速度，
      用 LQR/full-state feedback 计算期望水平加速度，再转换为 roll/pitch 姿态角。

    这个脚本目前只做最简单的竖直圆杆假设：
      1. 不考虑圆杆倾斜；
      2. 用圆杆中心位置 + 杆长/2 估算圆杆底端高度；
      3. 用 XY 距离 + 高度差判断“圆杆是否落到飞机上”；
      4. 控制飞机水平 XY，油门杆在限幅内给 GUIDED climb_rate。

    运行方式：
      1. 把本文件复制到 ArduPilot SITL 的 scripts/ 目录；
      2. 把 libraries/AP_Scripting/modules/MAVLink/mavlink_msgs.lua 复制到 scripts/modules/MAVLink/；
      3. 把 libraries/AP_Scripting/modules/MAVLink/mavlink_msg_ODOMETRY.lua 复制到 scripts/modules/MAVLink/；
      4. 设置 SCR_ENABLE=1；
      5. Gazebo 的 PoleMavlinkPosePlugin 发送 ODOMETRY 到 ArduPilot 能收到的串口端口；
      6. 飞机进入 GUIDED 模式并起飞后，本脚本自动运行控制逻辑。
--]]

local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- ===== 常量区 =====

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

local UPDATE_INTERVAL_MS = 10
local COPTER_MODE_GUIDED = 4
local ODOMETRY_ID = 331
local MAX_MAVLINK_MSGS_PER_UPDATE = 1
local POLE_UPDATE_TIMEOUT_MS = 500
local CONTROL_TIMEOUT_MS = 1000
local TEXT_PREFIX_STR = "copter-pole-balance:"

-- ===== 参数区 =====
--
-- 所有参数都会出现在地面站里，前缀为 PLBL_。
-- 例如：PLBL_K_R、PLBL_K_RD、PLBL_POLE_LEN。

local PARAM_TABLE_KEY = 140
local PARAM_TABLE_PREFIX = "PLBL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 21), "could not add param table")

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("could not add param %s", PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 0: 禁用脚本输出；1: 启用脚本输出
local PLBL_ENABLE = bind_add_param("ENABLE", 1, 1)

-- Gazebo 圆杆插件发 MAVLink 时使用的 sysid。设为 0 表示接受任意 sysid。
local PLBL_SYSID = bind_add_param("SYSID", 2, 42)

-- LQR/full-state feedback gains.  State per horizontal axis:
--   [vehicle_hold_error, vehicle_velocity, pole_vehicle_error, pole_vehicle_relative_velocity]
-- Control law:
--   accel = -K_X * vehicle_hold_error
--           -K_V * vehicle_velocity
--           +K_R * pole_vehicle_error
--           +K_RD * pole_vehicle_relative_velocity
local PLBL_K_X = bind_add_param("K_X", 3, -0.316)
local PLBL_K_V = bind_add_param("K_V", 4, -0.946)
local PLBL_K_R = bind_add_param("K_R", 5, 65.452)
local PLBL_K_RD = bind_add_param("K_RD", 6, 12.375)

-- 输出限幅和保护
-- 最大水平加速度。姿态角由 atan(accel/g) 得到，例如 2m/s^2 约等于 11.5deg。
local PLBL_ACC_MAX = bind_add_param("ACC_MAX", 7, 2.0)
local PLBL_DIST_MAX = bind_add_param("DIST_MAX", 8, 10.0)
local PLBL_DEBUG = bind_add_param("DEBUG", 9, 0)
-- ON_VEH 后，RC roll/pitch 推动 hold 参考点的最大水平速度。
local PLBL_REF_VEL_MAX = bind_add_param("REF_VEL_MAX", 10, 0.2)

-- 简单“圆杆落到飞机上”判断参数。
-- 当前假设圆杆竖直，不考虑倾斜。
local PLBL_POLE_LEN = bind_add_param("POLE_LEN", 11, 0.7)
local PLBL_TOP_OFS = bind_add_param("TOP_OFS", 12, 0.08)
local PLBL_LAND_XY = bind_add_param("LAND_XY", 13, 0.12)
local PLBL_LAND_Z = bind_add_param("LAND_Z", 14, 0.08)
-- 控制激活高度门槛：圆杆底端和飞机顶部高度差小于该值时才输出姿态控制。
local PLBL_ACT_Z = bind_add_param("ACT_Z", 15, 1.0)
-- Throttle stick commanded vertical speed limit while the script is controlling.
local PLBL_Z_VEL_MAX = bind_add_param("Z_VEL_MAX", 16, 1.0)
-- One-shot pole toss sequence, triggered by THROW_CH going high.
local PLBL_THROW_CH = bind_add_param("THROW_CH", 17, 8)
local PLBL_THROW_UP_MS = bind_add_param("THROW_UP_MS", 18, 300)
local PLBL_THROW_DN_MS = bind_add_param("THROW_DN_MS", 19, 300)
local PLBL_THROW_UP_V = bind_add_param("THROW_UP_V", 20, 2.0)
local PLBL_THROW_DN_V = bind_add_param("THROW_DN_V", 21, -2.0)

-- ===== MAVLink 接收配置 =====

local msg_map = {}
msg_map[ODOMETRY_ID] = "ODOMETRY"

mavlink:init(10, 1)
mavlink:register_rx_msgid(ODOMETRY_ID)

-- ===== 状态变量 =====

local pole_sysid = nil

-- Gazebo 插件发送的圆杆中心位置/速度，坐标为 NED，单位 m / m/s。
local pole_pos_NED = Vector3f()
local pole_vel_NED = Vector3f()

local pole_update_ms = uint32_t(0)
local pole_timeout_prev = true
local last_control_ms = uint32_t(0)
local warn_ms = uint32_t(0)
local pole_xyz_print_ms = uint32_t(0)
local odom_missing_warn_ms = uint32_t(0)
local debug_print_ms = uint32_t(0)

-- 简单状态标志：圆杆是否已经被判定落到飞机上。
local pole_on_vehicle = false
local control_active_prev = false
local control_state_reason_prev = ""
local hold_yaw_deg = nil
local pole_ref_pos_NED = nil
local pilot_roll = assert(rc:get_channel(assert(param:get("RCMAP_ROLL"))))
local pilot_pitch = assert(rc:get_channel(assert(param:get("RCMAP_PITCH"))))
local pilot_throttle = assert(rc:get_channel(assert(param:get("RCMAP_THROTTLE"))))
local throw_active = false
local throw_start_ms = uint32_t(0)
local throw_switch_was_high = false

-- ===== 小工具函数 =====

function constrain(value, min_value, max_value)
    if value < min_value then
        return min_value
    end
    if value > max_value then
        return max_value
    end
    return value
end

function print_warning(text)
    local now_ms = millis()
    if now_ms - warn_ms > 1000 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s %s", TEXT_PREFIX_STR, text))
        warn_ms = now_ms
    end
end

function print_pole_xyz()
    local now_ms = millis()
    if now_ms - pole_xyz_print_ms > 1000 then
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s pole xyz NED x:%4.2f y:%4.2f z:%4.2f",
            TEXT_PREFIX_STR, pole_pos_NED:x(), pole_pos_NED:y(), pole_pos_NED:z()))
        pole_xyz_print_ms = now_ms
    end
end

function print_odometry_missing_warning()
    local now_ms = millis()
    if now_ms - odom_missing_warn_ms > 1000 then
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("%s no ODOMETRY(331), please send mavlink_msg_ODOMETRY from pole plugin",
            TEXT_PREFIX_STR))
        odom_missing_warn_ms = now_ms
    end
end

function update_control_state_notice(can_control, reason)
    if can_control and not control_active_prev then
        hold_yaw_deg = math.deg(ahrs:get_yaw_rad())
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s control active (%s), hold yaw:%4.1fdeg", TEXT_PREFIX_STR, reason, hold_yaw_deg))
    elseif (not can_control) and control_active_prev then
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("%s control inactive (%s)", TEXT_PREFIX_STR, reason))
        hold_yaw_deg = nil
    elseif (not can_control) and reason ~= control_state_reason_prev then
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s waiting (%s)", TEXT_PREFIX_STR, reason))
    end

    control_active_prev = can_control
    control_state_reason_prev = reason
end

function clear_offsets()
    -- 清理历史 poscontrol offset，避免旧版本脚本或异常退出留下控制量。
    -- 当前脚本实际使用 GUIDED 姿态目标，不再依赖 poscontrol offset。
    poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
end

function reset_controller_state()
    last_control_ms = uint32_t(0)
    hold_yaw_deg = nil
    pole_ref_pos_NED = nil
    throw_active = false
    throw_start_ms = uint32_t(0)
    throw_switch_was_high = get_throw_switch_high()

    clear_offsets()
end

function update_pole_reference(now_ms)
    if not pole_on_vehicle or pole_ref_pos_NED == nil then
        return 0.0, 0.0
    end

    local dt = (now_ms - last_control_ms):tofloat() * 0.001
    if dt <= 0 or dt > 1.0 then
        return 0.0, 0.0
    end

    local pitch_input = pilot_pitch:norm_input_dz()
    local roll_input = pilot_roll:norm_input_dz()
    local ref_vel_max = PLBL_REF_VEL_MAX:get()
    local ref_vel_forward = pitch_input * ref_vel_max
    local ref_vel_right = roll_input * ref_vel_max

    local yaw_rad = ahrs:get_yaw_rad()
    local cos_yaw = math.cos(yaw_rad)
    local sin_yaw = math.sin(yaw_rad)
    local ref_vel_N = cos_yaw * ref_vel_forward - sin_yaw * ref_vel_right
    local ref_vel_E = sin_yaw * ref_vel_forward + cos_yaw * ref_vel_right

    pole_ref_pos_NED:x(pole_ref_pos_NED:x() + ref_vel_N * dt)
    pole_ref_pos_NED:y(pole_ref_pos_NED:y() + ref_vel_E * dt)

    return ref_vel_N, ref_vel_E
end

function get_pilot_climb_rate_ms()
    return pilot_throttle:norm_input_dz() * PLBL_Z_VEL_MAX:get()
end

function get_throw_switch_high()
    local throw_ch = math.floor(PLBL_THROW_CH:get())
    if throw_ch < 1 then
        return false
    end

    local pwm = rc:get_pwm(throw_ch)
    return pwm ~= nil and pwm > 1700
end

function get_commanded_climb_rate_ms(now_ms)
    local switch_high = get_throw_switch_high()
    if pole_on_vehicle and switch_high and not throw_switch_was_high and not throw_active then
        throw_active = true
        throw_start_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s throw sequence start", TEXT_PREFIX_STR))
    end
    throw_switch_was_high = switch_high

    if throw_active then
        local elapsed_ms = (now_ms - throw_start_ms):tofloat()
        local up_ms = math.max(0, PLBL_THROW_UP_MS:get())
        local dn_ms = math.max(0, PLBL_THROW_DN_MS:get())
        if elapsed_ms < up_ms then
            return PLBL_THROW_UP_V:get()
        elseif elapsed_ms < up_ms + dn_ms then
            return PLBL_THROW_DN_V:get()
        end

        throw_active = false
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s throw sequence complete", TEXT_PREFIX_STR))
    end

    return get_pilot_climb_rate_ms()
end

-- ===== MAVLink ODOMETRY 接收 =====

function handle_odometry(msg)
    -- 只接受指定 sysid 的圆杆消息，避免把飞控/地面站其他 ODOMETRY 当成杆子。
    if PLBL_SYSID:get() > 0 and msg.sysid ~= PLBL_SYSID:get() then
        return false
    end

    -- 第一次收到匹配 sysid 后锁定它。
    if pole_sysid == nil then
        pole_sysid = msg.sysid
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s found pole sysid:%d", TEXT_PREFIX_STR, msg.sysid))
    elseif pole_sysid ~= msg.sysid then
        return false
    end

    pole_pos_NED:x(msg.x)
    pole_pos_NED:y(msg.y)
    pole_pos_NED:z(msg.z)
    pole_vel_NED:x(msg.vx)
    pole_vel_NED:y(msg.vy)
    pole_vel_NED:z(msg.vz)

    pole_update_ms = millis()
    print_pole_xyz()
    return true
end

function receive_pole_odometry()
    local received = false
    local msg
    local processed = 0

    repeat
        msg, _ = mavlink:receive_chan()
        if msg ~= nil then
            processed = processed + 1
            local parsed_msg = mavlink_msgs.decode(msg, msg_map)
            if parsed_msg ~= nil and parsed_msg.msgid == ODOMETRY_ID then
                if handle_odometry(parsed_msg) then
                    received = true
                end
            end
        end
    until msg == nil or processed >= MAX_MAVLINK_MSGS_PER_UPDATE

    return received
end

-- ===== 飞机状态读取 =====

function get_vehicle_pos_NED()
    -- 优先用 EKF origin 下的位置；没有时退回 home 下的位置。
    local pos = ahrs:get_relative_position_NED_origin()
    if pos ~= nil then
        return pos
    end

    return ahrs:get_relative_position_NED_home()
end

function calc_relative_position_error(vehicle_pos_NED)
    local err = Vector3f()

    -- 杆子和飞机已在同一个 NED 世界坐标系下，直接用二者位置差作为相对误差。
    err:x(pole_pos_NED:x() - vehicle_pos_NED:x())
    err:y(pole_pos_NED:y() - vehicle_pos_NED:y())
    err:z(pole_pos_NED:z() - vehicle_pos_NED:z())

    return err
end

-- ===== 判断圆杆是否落到飞机上 =====

function pole_vehicle_vertical_error(vehicle_pos_NED)
    -- 当前假设圆杆竖直：
    --   圆杆底端 z = 圆杆中心 z + 杆长 / 2
    --   飞机顶部 z = 飞机位置 z - 机体顶部高度
    local pole_bottom_z = pole_pos_NED:z() + PLBL_POLE_LEN:get() * 0.5
    local vehicle_top_z = vehicle_pos_NED:z() - PLBL_TOP_OFS:get()
    return math.abs(pole_bottom_z - vehicle_top_z)
end

function pole_is_on_vehicle(vehicle_pos_NED)
    -- 这里使用用户指定的最简单假设：圆杆始终近似竖直。
    --
    -- NED 坐标 z 轴向下为正：
    --   圆杆底端 z = 圆杆中心 z + 杆长 / 2
    --   飞机顶部 z = 飞机位置 z - 机体顶部高度
    --
    -- 同时要求圆杆中心 XY 和飞机 XY 足够接近。
    local dx = pole_pos_NED:x() - vehicle_pos_NED:x()
    local dy = pole_pos_NED:y() - vehicle_pos_NED:y()
    local xy_err = math.sqrt(dx * dx + dy * dy)

    local z_err = pole_vehicle_vertical_error(vehicle_pos_NED)

    return xy_err < PLBL_LAND_XY:get() and z_err < PLBL_LAND_Z:get(), xy_err, z_err
end

function update_pole_landed_state(vehicle_pos_NED)
    if pole_on_vehicle then
        return
    end

    local landed, xy_err, z_err = pole_is_on_vehicle(vehicle_pos_NED)
    if landed then
        pole_on_vehicle = true
        pole_ref_pos_NED = pole_pos_NED:copy()
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s pole on vehicle xy:%4.2f z:%4.2f", TEXT_PREFIX_STR, xy_err, z_err))
    end
end

-- ===== 控制器 =====

function accel_to_lean_angle_deg(accel_mss)
    -- 小角度时 accel ~= g * angle_rad；这里用 atan 在角度较大时更合理。
    return math.deg(math.atan(accel_mss / 9.80665))
end

function send_guided_attitude(accel_cmd_N, accel_cmd_E, climb_rate_ms)
    -- LQR 输出的是 N/E 水平加速度指令，先转换到机体系前/右方向，
    -- 再转换成 roll/pitch 姿态角。
    -- 注意：
    --   accel_cmd_N/E 是世界系 NED 下的水平加速度；
    --   roll/pitch 是机体系姿态角；
    --   因此必须用“当前实际 yaw”做坐标变换。
    --   yaw 目标本身则使用进入控制时锁定的 hold_yaw_deg。
    --
    -- yaw=0 时：
    --   North 加速度对应机头前向，需要负 pitch（机头下俯）；
    --   East  加速度对应机体右向，需要正 roll。
    local yaw_rad = ahrs:get_yaw_rad()
    local cos_yaw = math.cos(yaw_rad)
    local sin_yaw = math.sin(yaw_rad)

    local accel_forward = cos_yaw * accel_cmd_N + sin_yaw * accel_cmd_E
    local accel_right = -sin_yaw * accel_cmd_N + cos_yaw * accel_cmd_E

    local pitch_deg = -accel_to_lean_angle_deg(accel_forward)
    local roll_deg = accel_to_lean_angle_deg(accel_right)
    if hold_yaw_deg == nil then
        hold_yaw_deg = math.deg(yaw_rad)
    end

    -- climb_rate 由油门杆在限幅内给出，高度通道仍由 ArduPilot 控制。
    if not vehicle:set_target_angle_and_climbrate(roll_deg, pitch_deg, hold_yaw_deg, climb_rate_ms, false, 0.0) then
        print_warning("failed to set guided attitude target")
    end

    return roll_deg, pitch_deg
end

function run_lqr_controller(vehicle_pos_NED, vehicle_vel_NED)
    -- rel_pos_NED 的含义：
    --   同一个 NED 世界坐标系下，圆杆中心位置 - 飞机位置
    -- 只使用 x/y，z 不参与控制。
    local rel_pos_NED = calc_relative_position_error(vehicle_pos_NED)

    if PLBL_DIST_MAX:get() > 0 and rel_pos_NED:xy():length() > PLBL_DIST_MAX:get() then
        print_warning(string.format("pole too far %4.1fm", rel_pos_NED:xy():length()))
        reset_controller_state()
        pole_on_vehicle = false
        return
    end

    local now_ms = millis()
    local ref_vel_N, ref_vel_E = update_pole_reference(now_ms)
    local rel_vel_N = pole_vel_NED:x() - vehicle_vel_NED:x()
    local rel_vel_E = pole_vel_NED:y() - vehicle_vel_NED:y()
    local pole_ref_err_N = 0.0
    local pole_ref_err_E = 0.0
    local pole_vel_err_N = 0.0
    local pole_vel_err_E = 0.0

    if pole_on_vehicle and pole_ref_pos_NED ~= nil then
        pole_ref_err_N = pole_pos_NED:x() - pole_ref_pos_NED:x()
        pole_ref_err_E = pole_pos_NED:y() - pole_ref_pos_NED:y()
        pole_vel_err_N = pole_vel_NED:x() - ref_vel_N
        pole_vel_err_E = pole_vel_NED:y() - ref_vel_E
    end

    local accel_cmd_N = -PLBL_K_X:get() * pole_ref_err_N
                        -PLBL_K_V:get() * pole_vel_err_N
                        +PLBL_K_R:get() * rel_pos_NED:x()
                        +PLBL_K_RD:get() * rel_vel_N
    local accel_cmd_E = -PLBL_K_X:get() * pole_ref_err_E
                        -PLBL_K_V:get() * pole_vel_err_E
                        +PLBL_K_R:get() * rel_pos_NED:y()
                        +PLBL_K_RD:get() * rel_vel_E

    accel_cmd_N = constrain(accel_cmd_N, -PLBL_ACC_MAX:get(), PLBL_ACC_MAX:get())
    accel_cmd_E = constrain(accel_cmd_E, -PLBL_ACC_MAX:get(), PLBL_ACC_MAX:get())
    local climb_rate_ms = get_commanded_climb_rate_ms(now_ms)

    -- 输出给 GUIDED 姿态控制：
    --   LQR/full-state feedback -> 期望水平加速度
    --   水平加速度 -> roll/pitch 角度
    local roll_deg, pitch_deg = send_guided_attitude(accel_cmd_N, accel_cmd_E, climb_rate_ms)
    last_control_ms = now_ms

    if PLBL_DEBUG:get() > 0 and now_ms - debug_print_ms > 200 then
        debug_print_ms = now_ms
        local state = "TRACK"
        if pole_on_vehicle then
            state = "ON_VEH"
        end
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s %s rel N:%4.2f E:%4.2f pref N:%4.2f E:%4.2f refv N:%4.2f E:%4.2f rvel N:%4.2f E:%4.2f acc N:%4.2f E:%4.2f cz:%4.2f r:%4.1f p:%4.1f",
            TEXT_PREFIX_STR, state, rel_pos_NED:x(), rel_pos_NED:y(),
            pole_ref_err_N, pole_ref_err_E, ref_vel_N, ref_vel_E, rel_vel_N, rel_vel_E,
            accel_cmd_N, accel_cmd_E, climb_rate_ms, roll_deg, pitch_deg))
    end
end

-- ===== 主循环 =====

gcs:send_text(MAV_SEVERITY.INFO, "copter-pole-balance script loaded")

function update()
    -- 1. 脚本禁用时清空 offset，防止残留控制量。
    if PLBL_ENABLE:get() <= 0 then
        reset_controller_state()
        pole_on_vehicle = false
        return update, 1000
    end

    -- 2. 每一拍都先读取圆杆 ODOMETRY。
    receive_pole_odometry()

    -- 3. ODOMETRY 超时保护。
    local pole_timeout = millis() - pole_update_ms > POLE_UPDATE_TIMEOUT_MS
    if pole_timeout ~= pole_timeout_prev then
        if pole_timeout then
            print_odometry_missing_warning()
            reset_controller_state()
            pole_on_vehicle = false
        else
            gcs:send_text(MAV_SEVERITY.INFO, string.format("%s pole odometry received", TEXT_PREFIX_STR))
        end
    end
    pole_timeout_prev = pole_timeout

    if pole_timeout then
        print_odometry_missing_warning()
    end

    -- 4. 读取飞机位置和速度。
    local vehicle_pos_NED = get_vehicle_pos_NED()
    local vehicle_vel_NED = ahrs:get_velocity_NED()
    if vehicle_pos_NED == nil or vehicle_vel_NED == nil then
        return update, UPDATE_INTERVAL_MS
    end

    -- 5. 只有 GUIDED + 已解锁起飞 + ODOMETRY 正常时才控制。
    local mode = vehicle:get_mode()
    local control_mode = (mode == COPTER_MODE_GUIDED)
    local pole_z_err = pole_vehicle_vertical_error(vehicle_pos_NED)
    local height_close = pole_z_err < PLBL_ACT_Z:get()
    local can_control = arming:is_armed()
                        and vehicle:get_likely_flying()
                        and control_mode
                        and height_close
                        and not pole_timeout
    local control_reason = "ready"
    if pole_timeout then
        control_reason = "waiting for ODOMETRY"
    elseif not control_mode then
        control_reason = "mode is not GUIDED"
    elseif not arming:is_armed() then
        control_reason = "vehicle not armed"
    elseif not vehicle:get_likely_flying() then
        control_reason = "vehicle not flying"
    elseif not height_close then
        control_reason = string.format("pole height error %4.2fm > ACT_Z", pole_z_err)
    end
    update_control_state_notice(can_control, control_reason)

    if can_control then
        -- 控制器始终提前运行，不等人工切换。
        -- 圆杆未落到飞机上前：主要是追踪 XY；
        -- 判断落到飞机上后：同一套 LQR 反馈会加入 pole reference 状态。
        update_pole_landed_state(vehicle_pos_NED)
        run_lqr_controller(vehicle_pos_NED, vehicle_vel_NED)
    else
        if millis() - last_control_ms > CONTROL_TIMEOUT_MS then
            reset_controller_state()
            pole_on_vehicle = false
        end
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
