--[[
    copter-pole-balance.lua

    目标：
      在 Copter GUIDED 模式下，接收 Gazebo 圆杆通过 MAVLink ODOMETRY 发来的位置/速度，
      用“位置 P 外环 + 速度 PID 内环”计算期望水平加速度，再转换为 roll/pitch 姿态角。

    这个脚本目前只做最简单的竖直圆杆假设：
      1. 不考虑圆杆倾斜；
      2. 用圆杆中心位置 + 杆长/2 估算圆杆底端高度；
      3. 用 XY 距离 + 高度差判断“圆杆是否落到飞机上”；
      4. 只控制飞机水平 XY，高度通过 GUIDED climb_rate=0 定高。

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
-- 例如：PLBL_POS_P、PLBL_VEL_P、PLBL_POLE_LEN。

local PARAM_TABLE_KEY = 83
local PARAM_TABLE_PREFIX = "PLBL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), "could not add param table")

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("could not add param %s", PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 0: 禁用脚本输出；1: 启用脚本输出
local PLBL_ENABLE = bind_add_param("ENABLE", 1, 1)

-- Gazebo 圆杆插件发 MAVLink 时使用的 sysid。设为 0 表示接受任意 sysid。
local PLBL_SYSID = bind_add_param("SYSID", 2, 42)

-- 位置 P 外环：位置误差 [m] -> 目标水平速度 [m/s]
local PLBL_POS_P = bind_add_param("POS_P", 3, 0.6)

-- 速度 PID 内环：速度误差 [m/s] -> 期望水平加速度 [m/s^2]
local PLBL_VEL_P = bind_add_param("VEL_P", 4, 0.8)
local PLBL_VEL_I = bind_add_param("VEL_I", 5, 0.0)
local PLBL_VEL_D = bind_add_param("VEL_D", 6, 0.0)

-- 输出限幅和保护
local PLBL_VEL_MAX = bind_add_param("VEL_MAX", 7, 1.5)
-- 最大水平加速度。姿态角由 atan(accel/g) 得到，例如 2m/s^2 约等于 11.5deg。
local PLBL_ACC_MAX = bind_add_param("ACC_MAX", 8, 2.0)
local PLBL_I_MAX = bind_add_param("I_MAX", 9, 0.5)
local PLBL_DIST_MAX = bind_add_param("DIST_MAX", 10, 10.0)
local PLBL_DEBUG = bind_add_param("DEBUG", 11, 0)

-- 简单“圆杆落到飞机上”判断参数。
-- 当前假设圆杆竖直，不考虑倾斜。
local PLBL_POLE_LEN = bind_add_param("POLE_LEN", 12, 0.7)
local PLBL_TOP_OFS = bind_add_param("TOP_OFS", 13, 0.08)
local PLBL_LAND_XY = bind_add_param("LAND_XY", 14, 0.12)
local PLBL_LAND_Z = bind_add_param("LAND_Z", 15, 0.08)
-- 控制激活高度门槛：圆杆底端和飞机顶部高度差小于该值时才输出姿态控制。
local PLBL_ACT_Z = bind_add_param("ACT_Z", 16, 1.0)

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

-- 用于对齐 Gazebo 局部坐标和 ArduPilot 本地坐标的相对原点。
-- 初始化后，控制使用：
--   pole_rel    = pole_pos    - pole_origin
--   vehicle_rel = vehicle_pos - vehicle_origin
--   pos_error   = pole_rel - vehicle_rel
local pole_origin_NED = nil
local vehicle_origin_NED = nil

local pole_update_ms = uint32_t(0)
local pole_timeout_prev = true
local last_control_ms = uint32_t(0)
local warn_ms = uint32_t(0)
local pole_xyz_print_ms = uint32_t(0)
local odom_missing_warn_ms = uint32_t(0)

-- 速度 PID 内环的积分和上一拍误差。
local vel_i_N = 0.0
local vel_i_E = 0.0
local vel_err_N_prev = 0.0
local vel_err_E_prev = 0.0
local vel_pid_ms = uint32_t(0)

-- 简单状态标志：圆杆是否已经被判定落到飞机上。
local pole_on_vehicle = false
local control_active_prev = false
local control_state_reason_prev = ""
local hold_yaw_deg = nil

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
    vel_i_N = 0.0
    vel_i_E = 0.0
    vel_err_N_prev = 0.0
    vel_err_E_prev = 0.0
    hold_yaw_deg = nil
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

function init_relative_origins(vehicle_pos_NED)
    -- 第一次可以控制时，记录圆杆和飞机各自的“零点”。
    -- 后续只用二者相对自身零点的变化量做控制，减少坐标原点不一致造成的 XY 偏移。
    if pole_origin_NED == nil then
        pole_origin_NED = pole_pos_NED:copy()
    end
    if vehicle_origin_NED == nil then
        vehicle_origin_NED = vehicle_pos_NED:copy()
    end
end

function calc_relative_position_error(vehicle_pos_NED)
    local err = Vector3f()

    err:x((pole_pos_NED:x() - pole_origin_NED:x()) - (vehicle_pos_NED:x() - vehicle_origin_NED:x()))
    err:y((pole_pos_NED:y() - pole_origin_NED:y()) - (vehicle_pos_NED:y() - vehicle_origin_NED:y()))
    err:z((pole_pos_NED:z() - pole_origin_NED:z()) - (vehicle_pos_NED:z() - vehicle_origin_NED:z()))

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
        clear_offsets()
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s pole on vehicle xy:%4.2f z:%4.2f", TEXT_PREFIX_STR, xy_err, z_err))
    end
end

-- ===== 控制器 =====

function calc_axis_pid(vel_err, prev_err, integral, dt)
    integral = integral + vel_err * dt * PLBL_VEL_I:get()
    integral = constrain(integral, -PLBL_I_MAX:get(), PLBL_I_MAX:get())

    local derivative = 0.0
    if dt > 0 then
        derivative = (vel_err - prev_err) / dt
    end

    local output = PLBL_VEL_P:get() * vel_err + integral + PLBL_VEL_D:get() * derivative
    output = constrain(output, -PLBL_ACC_MAX:get(), PLBL_ACC_MAX:get())
    return output, integral
end

function accel_to_lean_angle_deg(accel_mss)
    -- 小角度时 accel ~= g * angle_rad；这里用 atan 在角度较大时更合理。
    return math.deg(math.atan(accel_mss / 9.80665))
end

function send_guided_attitude(accel_cmd_N, accel_cmd_E)
    -- 速度 PID 输出的是 N/E 水平加速度指令，先转换到机体系前/右方向，
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

    -- climb_rate=0：高度交给 ArduPilot 高度控制器保持。
    if not vehicle:set_target_angle_and_climbrate(roll_deg, pitch_deg, hold_yaw_deg, 0.0, false, 0.0) then
        print_warning("failed to set guided attitude target")
    end

    return roll_deg, pitch_deg
end

function run_position_velocity_controller(vehicle_pos_NED, vehicle_vel_NED)
    init_relative_origins(vehicle_pos_NED)

    -- pos_err_NED 的含义：
    --   圆杆相对初始位置移动了多少 - 飞机相对初始位置移动了多少
    -- 只使用 x/y，z 不参与控制。
    local pos_err_NED = calc_relative_position_error(vehicle_pos_NED)

    if PLBL_DIST_MAX:get() > 0 and pos_err_NED:xy():length() > PLBL_DIST_MAX:get() then
        print_warning(string.format("pole too far %4.1fm", pos_err_NED:xy():length()))
        clear_offsets()
        return
    end

    local now_ms = millis()
    local dt = (now_ms - vel_pid_ms):tofloat() * 0.001
    vel_pid_ms = now_ms

    if dt <= 0 or dt > 1 then
        clear_offsets()
        return
    end

    -- 外环：位置 P。
    -- 位置误差越大，目标水平速度越大。
    local desired_vel_N = constrain(PLBL_POS_P:get() * pos_err_NED:x(),
                                    -PLBL_VEL_MAX:get(), PLBL_VEL_MAX:get())
    local desired_vel_E = constrain(PLBL_POS_P:get() * pos_err_NED:y(),
                                    -PLBL_VEL_MAX:get(), PLBL_VEL_MAX:get())

    -- 内环：速度 PID。
    -- 目标是让飞机相对圆杆的速度跟随 desired_vel。
    local vel_err_N = desired_vel_N - (vehicle_vel_NED:x() - pole_vel_NED:x())
    local vel_err_E = desired_vel_E - (vehicle_vel_NED:y() - pole_vel_NED:y())

    local accel_cmd_N
    local accel_cmd_E
    accel_cmd_N, vel_i_N = calc_axis_pid(vel_err_N, vel_err_N_prev, vel_i_N, dt)
    accel_cmd_E, vel_i_E = calc_axis_pid(vel_err_E, vel_err_E_prev, vel_i_E, dt)
    vel_err_N_prev = vel_err_N
    vel_err_E_prev = vel_err_E

    -- 输出给 GUIDED 姿态控制：
    --   位置 P 外环：位置误差 -> 期望水平速度
    --   速度 PID 内环：速度误差 -> 期望水平加速度
    --   水平加速度 -> roll/pitch 角度
    local roll_deg, pitch_deg = send_guided_attitude(accel_cmd_N, accel_cmd_E)
    last_control_ms = now_ms

    if PLBL_DEBUG:get() > 0 then
        local state = "TRACK"
        if pole_on_vehicle then
            state = "ON_VEH"
        end
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s %s err N:%4.2f E:%4.2f vel N:%4.2f E:%4.2f acc N:%4.2f E:%4.2f r:%4.1f p:%4.1f",
            TEXT_PREFIX_STR, state, pos_err_NED:x(), pos_err_NED:y(),
            desired_vel_N, desired_vel_E, accel_cmd_N, accel_cmd_E, roll_deg, pitch_deg))
    end
end

-- ===== 主循环 =====

gcs:send_text(MAV_SEVERITY.INFO, "copter-pole-balance script loaded")

function update()
    -- 1. 脚本禁用时清空 offset，防止残留控制量。
    if PLBL_ENABLE:get() <= 0 then
        clear_offsets()
        return update, 1000
    end

    -- 2. 每一拍都先读取圆杆 ODOMETRY。
    receive_pole_odometry()

    -- 3. ODOMETRY 超时保护。
    local pole_timeout = millis() - pole_update_ms > POLE_UPDATE_TIMEOUT_MS
    if pole_timeout ~= pole_timeout_prev then
        if pole_timeout then
            print_odometry_missing_warning()
            clear_offsets()
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
        -- 判断落到飞机上后：继续使用同一套位置 P + 速度 PID 输出。
        update_pole_landed_state(vehicle_pos_NED)
        run_position_velocity_controller(vehicle_pos_NED, vehicle_vel_NED)
    else
        if millis() - last_control_ms > CONTROL_TIMEOUT_MS then
            clear_offsets()
        end
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
