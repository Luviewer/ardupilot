#include "Copter.h"

#if MODE_PENDULUM_ENABLED

#define PENDULUM_TEXT_PREFIX "Pendulum:"
#define PENDULUM_CONTROL_INTERVAL_MS 5

const AP_Param::GroupInfo ModePendulum::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Pendulum control enable
    // @Description: Enables roll and pitch control from the pendulum controller while in Pendulum mode. If disabled the mode behaves like AltHold.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, ModePendulum, _enable, 1),

    // @Param: SYSID
    // @DisplayName: Pendulum odometry MAVLink system id
    // @Description: MAVLink system id used by the pole odometry sender. Set to 0 to accept any system id.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("SYSID", 2, ModePendulum, _sysid, 42),

    // @Param: K_X
    // @DisplayName: Pendulum vehicle position feedback gain
    // @Description: Vehicle position feedback gain used by the full-state pendulum controller.
    // @Range: 0 10
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("K_X", 3, ModePendulum, _k_x, 0.2),

    // @Param: K_V
    // @DisplayName: Pendulum vehicle velocity feedback gain
    // @Description: Vehicle velocity feedback gain used by the full-state pendulum controller.
    // @Range: 0 10
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("K_V", 4, ModePendulum, _k_v, 0.4),

    // @Param: K_R
    // @DisplayName: Pendulum relative position feedback gain
    // @Description: Pole relative position feedback gain used by the full-state pendulum controller.
    // @Range: 0 20
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("K_R", 5, ModePendulum, _k_r, 1.0),

    // @Param: K_RD
    // @DisplayName: Pendulum relative velocity feedback gain
    // @Description: Pole relative velocity feedback gain used by the full-state pendulum controller.
    // @Range: 0 20
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("K_RD", 6, ModePendulum, _k_rd, 0.5),

    // @Param: ACT_Z
    // @DisplayName: Pendulum activation height distance
    // @Description: Maximum vertical distance between the pole odometry position and the vehicle position before pendulum control may become active.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ACT_Z", 7, ModePendulum, _act_z_m, 1.0),

    // @Param: TIMEOUT_MS
    // @DisplayName: Pendulum odometry timeout
    // @Description: Time without pole odometry before pendulum control is disabled.
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT_MS", 8, ModePendulum, _timeout_ms, 500),

    // @Param: DEBUG
    // @DisplayName: Pendulum debug output
    // @Description: Enables periodic GCS debug messages from Pendulum mode.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 9, ModePendulum, _debug, 0),

    AP_GROUPEND
};

ModePendulum::ModePendulum() :
    ModeAltHold()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModePendulum::init(bool ignore_checks)
{
    if (!ModeAltHold::init(ignore_checks)) {
        return false;
    }

    pos_control->NE_init_controller();
    reset_controller();
    return true;
}

void ModePendulum::reset_controller()
{
    _vel_pid_ms = 0;
    _last_control_ms = 0;
    _control_accel_ne_mss.zero();
    _have_hold_pos = false;
    _control_active_prev = false;
    _control_reason_prev = "";
}

bool ModePendulum::accepts_odometry(const mavlink_message_t &msg) const
{
    if (_enable <= 0) {
        return false;
    }

    return _sysid <= 0 || msg.sysid == uint8_t(_sysid.get());
}

void ModePendulum::handle_odometry(const mavlink_message_t &msg)
{
    if (!accepts_odometry(msg)) {
        return;
    }

    mavlink_odometry_t odom {};
    mavlink_msg_odometry_decode(&msg, &odom);

    if (odom.frame_id != MAV_FRAME_LOCAL_NED) {
        return;
    }

    _pole_pos_ned_m = Vector3f{odom.x, odom.y, odom.z};
    _pole_vel_ned_ms = Vector3f{odom.vx, odom.vy, odom.vz};
    _pole_update_ms = AP_HAL::millis();

    if (!_have_pole) {
        _have_pole = true;
        gcs().send_text(MAV_SEVERITY_INFO, "%s pole odometry received sysid:%u", PENDULUM_TEXT_PREFIX, unsigned(msg.sysid));
    }
}

void ModePendulum::update_control_state_notice(bool can_control, const char *reason)
{
    if (can_control && ! _control_active_prev) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s control active", PENDULUM_TEXT_PREFIX);
    } else if (!can_control && _control_active_prev) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s control inactive: %s", PENDULUM_TEXT_PREFIX, reason);
    } else if (!can_control && reason != _control_reason_prev && AP_HAL::millis() - _last_warn_ms > 1000) {
        _last_warn_ms = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_INFO, "%s waiting: %s", PENDULUM_TEXT_PREFIX, reason);
    }

    _control_active_prev = can_control;
    _control_reason_prev = reason;
}

#if HAL_LOGGING_ENABLED
void ModePendulum::write_log(const Vector2f &vehicle_pos_err_ne, const Vector2f &vehicle_vel_ne, const Vector2f &pole_pos_err_ne,
                             const Vector2f &pole_rel_vel_ne, const Vector2f &accel_ne_mss, const Vector3f &att_target_rad, float dt)
{
    // Mode run normally happens at the main loop rate, so log at about 25Hz.
    if (_log_counter++ % 16 != 0) {
        return;
    }

    AP::logger().WriteStreaming("PEND",
                                "TimeUS,XN,XE,VN,VE,RN,RE,RVN,RVE,AN,AE,Roll,Pitch,DT",
                                "Qfffffffffffff",
                                AP_HAL::micros64(),
                                double(vehicle_pos_err_ne.x),
                                double(vehicle_pos_err_ne.y),
                                double(vehicle_vel_ne.x),
                                double(vehicle_vel_ne.y),
                                double(pole_pos_err_ne.x),
                                double(pole_pos_err_ne.y),
                                double(pole_rel_vel_ne.x),
                                double(pole_rel_vel_ne.y),
                                double(accel_ne_mss.x),
                                double(accel_ne_mss.y),
                                double(degrees(att_target_rad.x)),
                                double(degrees(att_target_rad.y)),
                                double(dt));
}
#endif

bool ModePendulum::get_vehicle_state(Vector3f &pos_ned_m, Vector3f &vel_ned_ms) const
{
    Vector3p pos_ned_p;
    if (!ahrs.get_relative_position_NED_origin(pos_ned_p)) {
        return false;
    }
    pos_ned_m = pos_ned_p.tofloat();

    return ahrs.get_velocity_NED(vel_ned_ms);
}

bool ModePendulum::run_pendulum_controller(Vector3f &pos_ned_m, Vector3f &vel_ned_ms, Vector2f &target_accel_ne_mss)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (_vel_pid_ms == 0) {
        _vel_pid_ms = now_ms;
        return false;
    }

    const float dt = (now_ms - _vel_pid_ms) * 0.001f;
    _vel_pid_ms = now_ms;
    if (!is_positive(dt) || dt > 1.0f) {
        return false;
    }

    if (!_have_hold_pos) {
        _hold_pos_ne_m = pos_ned_m.xy();
        _have_hold_pos = true;
    }

    const Vector2f vehicle_pos_err_ne = pos_ned_m.xy() - _hold_pos_ne_m;
    const Vector2f vehicle_vel_ne = vel_ned_ms.xy();
    const Vector2f pole_pos_err_ne = (_pole_pos_ned_m - pos_ned_m).xy();
    const Vector2f pole_rel_vel_ne = _pole_vel_ned_ms.xy() - vel_ned_ms.xy();
    target_accel_ne_mss = -vehicle_pos_err_ne * _k_x
                          -vehicle_vel_ne * _k_v
                          +pole_pos_err_ne * _k_r
                          +pole_rel_vel_ne * _k_rd;
    _last_control_ms = now_ms;

#if HAL_LOGGING_ENABLED
    write_log(vehicle_pos_err_ne, vehicle_vel_ne, pole_pos_err_ne, pole_rel_vel_ne, target_accel_ne_mss,
              attitude_control->get_att_target_euler_rad(), dt);
#endif

    if (_debug > 0 && now_ms - _last_debug_ms > 1000) {
        _last_debug_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "%s x N:%.2f E:%.2f r N:%.2f E:%.2f acc N:%.2f E:%.2f",
                        PENDULUM_TEXT_PREFIX,
                        double(vehicle_pos_err_ne.x),
                        double(vehicle_pos_err_ne.y),
                        double(pole_pos_err_ne.x),
                        double(pole_pos_err_ne.y),
                        double(target_accel_ne_mss.x),
                        double(target_accel_ne_mss.y));
    }

    return true;
}

void ModePendulum::run()
{
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    update_simple_mode();

    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    const float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();

    const AltHoldModeState althold_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    switch (althold_state) {
    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->D_relax_controller(0.0f);
        reset_controller();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->D_relax_controller(0.0f);
        reset_controller();
        break;

    case AltHoldModeState::Takeoff:
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0, 10.0));
        }
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        break;

    case AltHoldModeState::Flying: {
#if AP_AVOIDANCE_ALTHOLD_ENABLED
        copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

        Vector3f vehicle_pos_ned_m;
        Vector3f vehicle_vel_ned_ms;
        const uint32_t now_ms = AP_HAL::millis();
        const bool odom_timeout = !_have_pole || (now_ms - _pole_update_ms > uint32_t(MAX(_timeout_ms.get(), 100)));
        const bool have_vehicle_state = get_vehicle_state(vehicle_pos_ned_m, vehicle_vel_ned_ms);
        const float height_error_m = have_vehicle_state ? fabsf(_pole_pos_ned_m.z - vehicle_pos_ned_m.z) : 0.0f;
        const bool height_close = have_vehicle_state && (_act_z_m.get() <= 0 || height_error_m < _act_z_m.get());

        const char *reason = "ready";
        bool can_control = _enable > 0 && !odom_timeout && have_vehicle_state && height_close;
        if (_enable <= 0) {
            reason = "disabled";
        } else if (odom_timeout) {
            reason = "waiting for pole odometry";
        } else if (!have_vehicle_state) {
            reason = "waiting for vehicle position";
        } else if (!height_close) {
            reason = "waiting for pole height";
        }

        if (can_control) {
            if (_last_control_ms == 0 || now_ms - _last_control_ms >= PENDULUM_CONTROL_INTERVAL_MS) {
                can_control = run_pendulum_controller(vehicle_pos_ned_m, vehicle_vel_ned_ms, _control_accel_ne_mss);
                if (!can_control) {
                    reason = "controller not ready";
                }
            }
            if (can_control) {
                const float yaw_rad = ahrs.get_yaw_rad();
                const float cos_yaw = cosf(yaw_rad);
                const float sin_yaw = sinf(yaw_rad);
                const float accel_forward = cos_yaw * _control_accel_ne_mss.x + sin_yaw * _control_accel_ne_mss.y;
                const float accel_right = -sin_yaw * _control_accel_ne_mss.x + cos_yaw * _control_accel_ne_mss.y;
                float pendulum_pitch_rad = -atanf(accel_forward / GRAVITY_MSS);
                float pendulum_roll_rad = atanf(accel_right / GRAVITY_MSS);
                const float lean_max = attitude_control->lean_angle_max_rad();
                target_roll_rad = constrain_float(pendulum_roll_rad, -lean_max, lean_max);
                target_pitch_rad = constrain_float(pendulum_pitch_rad, -lean_max, lean_max);
            }
        } else {
            _vel_pid_ms = 0;
            _last_control_ms = 0;
            _control_accel_ne_mss.zero();
            _have_hold_pos = false;
        }

        update_control_state_notice(can_control, reason);
        break;
    }
    }

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);
    pos_control->D_update_controller();
}

#endif // MODE_PENDULUM_ENABLED
