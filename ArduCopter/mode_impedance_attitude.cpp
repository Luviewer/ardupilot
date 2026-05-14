#include "Copter.h"

#if MODE_IMPEDANCE_ATTITUDE_ENABLED

static constexpr uint32_t IMPEDANCE_ATT_TARE_SETTLE_MS = 600;
static constexpr uint32_t IMPEDANCE_ATT_SENSOR_FAILSAFE_MS = 300;

/*
 * Impedance attitude flight mode - Loiter-based lateral hold with admittance
 * force control on body-X mapped directly to pitch attitude. Roll, yaw and
 * throttle follow Loiter/AltHold behavior; body-X is not injected as a Loiter
 * velocity offset.
 */

const AP_Param::GroupInfo ModeImpedanceAttitude::var_info[] = {

    // @Param: ADM_GAIN
    // @DisplayName: Admittance force-to-pitch gain
    // @Description: Scale from normalized force error to pitch attitude angle.
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ADM_GAIN", 1, ModeImpedanceAttitude, _adm_gain, 0.3f),

    // @Param: ADM_TAU
    // @DisplayName: Admittance time constant
    // @Description: Unused: admittance is proportional (no first-order dynamics). Kept for existing parameter storage.
    // @Range: 0.05 5.0
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("ADM_TAU", 2, ModeImpedanceAttitude, _adm_tau, 0.4f),

    // @Param: ADM_DZ
    // @DisplayName: Admittance stick deadzone
    // @Description: Pitch stick deadzone used when changing the normalized force reference.
    // @Range: 0.0 0.3
    // @User: Advanced
    AP_GROUPINFO("ADM_DZ", 3, ModeImpedanceAttitude, _adm_dz, 0.05f),

    // @Param: ADM_FMAX
    // @DisplayName: Admittance force reference limit
    // @Description: Maximum absolute normalized force reference commanded by the pitch stick.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("ADM_FMAX", 4, ModeImpedanceAttitude, _adm_fmax, 0.5f),

    // @Param: ADM_FRATE
    // @DisplayName: Admittance force reference rate
    // @Description: Full-stick rate for changing the normalized force reference. Centered pitch stick holds the current force reference.
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("ADM_FRATE", 5, ModeImpedanceAttitude, _adm_force_ref_rate, 0.01f),

    // @Param: ADM_PMAX
    // @DisplayName: Admittance max pitch angle
    // @Description: Maximum pitch angle commanded by the body-X admittance attitude controller.
    // @Range: 1 15
    // @Increment: 0.5
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("ADM_PMAX", 6, ModeImpedanceAttitude, _adm_pitch_max_deg, 5.0f),

    // @Param: ADM_FLPF
    // @DisplayName: Force feedback low-pass frequency
    // @Description: Cutoff frequency for low-pass filtering the normalized CMCU-06A force feedback. Lower values are smoother but add lag; set 0 to disable.
    // @Range: 0.0 10.0
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("ADM_FLPF", 7, ModeImpedanceAttitude, _adm_fest_lpf_hz, 3.0f),

    // @Param: ADM_FGMAX
    // @DisplayName: Admittance force full-scale grams
    // @Description: CMCU-06A body-X force in grams that maps to normalized force feedback 1.0.
    // @Range: 1 100000
    // @Units: g
    // @User: Advanced
    AP_GROUPINFO("ADM_FGMAX", 8, ModeImpedanceAttitude, _adm_force_g_max, 1000.0f),

    // @Param: ADM_FREV
    // @DisplayName: Admittance force reverse
    // @Description: Reverse the sign of the CMCU-06A body-X force feedback when sensor installation direction is opposite.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    AP_GROUPINFO("ADM_FREV", 9, ModeImpedanceAttitude, _adm_force_reverse, 0),

    // @Param: ADM_FDZ
    // @DisplayName: Admittance force feedback deadzone
    // @Description: Normalized force feedback deadzone applied after filtering. Feedback smaller than this value is treated as zero to avoid control from small sensor noise.
    // @Range: 0.0 0.3
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ADM_FDZ", 10, ModeImpedanceAttitude, _adm_force_dz, 0.02f),

    AP_GROUPEND
};

ModeImpedanceAttitude::ModeImpedanceAttitude() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeImpedanceAttitude::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;
    update_simple_mode();

    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, 0.0f);
    loiter_nav->init_target();

    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    admittance_reset();
    _adm_force_ref = 0.0f;
    _tare_requested = false;
    _tare_reported = false;
    _tare_start_ms = 0;
    _sensor_failsafe_ms = 0;
#if AP_CMCU06A_ENABLED
    if (!copter.cmcu06a.healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ImpAtt: CMCU06A not healthy");
        return false;
    }

    if (!copter.cmcu06a.tare()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ImpAtt: CMCU06A tare busy");
        return false;
    }
    _tare_requested = true;
    _tare_start_ms = AP_HAL::millis();
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ImpAtt: PMAX %.1f Gain %.2f FMAX %.2f",
                  double(_adm_pitch_max_deg.get()),
                  double(_adm_gain.get()),
                  double(_adm_fmax.get()));

    return true;
}

void ModeImpedanceAttitude::exit()
{
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ImpAtt: exit FRef %.2f PMAX %.1f Gain %.2f FMAX %.2f",
                  double(_adm_force_ref),
                  double(_adm_pitch_max_deg.get()),
                  double(_adm_gain.get()),
                  double(_adm_fmax.get()));
    admittance_reset();
    loiter_nav->clear_vel_offset_NE_ms();
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();
}

void ModeImpedanceAttitude::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;
    const uint32_t now_ms = AP_HAL::millis();

    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    update_simple_mode();

    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    float pitch_stick_norm = -channel_pitch->norm_input_dz();
    if (fabsf(pitch_stick_norm) < _adm_dz.get()) {
        pitch_stick_norm = 0.0f;
    }
    const float fmax = constrain_float(_adm_fmax.get(), 0.01f, 1.0f);
    const float dt_s = pos_control->get_dt_s();
    if (dt_s > 0.0f && dt_s <= 0.1f) {
        const float force_ref_rate = constrain_float(_adm_force_ref_rate.get(), 0.01f, 2.0f);
        _adm_force_ref += pitch_stick_norm * force_ref_rate * dt_s;
    }
    _adm_force_ref = constrain_float(_adm_force_ref, -fmax, fmax);

    bool force_ready = false;
    bool tare_settled = false;
    float raw_fest = 0.0f;
#if AP_CMCU06A_ENABLED
    tare_settled = _tare_requested &&
                   (now_ms - _tare_start_ms >= IMPEDANCE_ATT_TARE_SETTLE_MS) &&
                   (copter.cmcu06a.last_tare_ms() >= _tare_start_ms);
    if (tare_settled && !_tare_reported) {
        _tare_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ImpAtt: CMCU06A tare OK");
    }
    force_ready = tare_settled && copter.cmcu06a.healthy();
    if (force_ready) {
        const float force_g_max = MAX(_adm_force_g_max.get(), 1.0f);
        raw_fest = constrain_float(float(copter.cmcu06a.get_value()) / force_g_max, -1.0f, 1.0f);
        if (_adm_force_reverse.get() != 0) {
            raw_fest = -raw_fest;
        }
    }
#else
    force_ready = true;
#endif
    if (!force_ready) {
        admittance_reset();
        if (!tare_settled) {
            _sensor_failsafe_ms = 0;
        }
    }

    const float lpf_hz = _adm_fest_lpf_hz.get();
    if (lpf_hz > 0.0f) {
        const float dt = pos_control->get_dt_s();
        const float alpha = constrain_float(dt * lpf_hz * M_2PI, 0.0f, 1.0f);
        _adm_force_est_filt += (raw_fest - _adm_force_est_filt) * alpha;
    } else {
        _adm_force_est_filt = raw_fest;
    }
    const float force_dz = constrain_float(_adm_force_dz.get(), 0.0f, 0.3f);
    _adm_force_est = (fabsf(_adm_force_est_filt) < force_dz) ? 0.0f : _adm_force_est_filt;

#if AP_CMCU06A_ENABLED
    if (!tare_settled) {
        _sensor_failsafe_ms = 0;
    } else if (force_ready) {
        _sensor_failsafe_ms = 0;
    } else if (_sensor_failsafe_ms == 0) {
        _sensor_failsafe_ms = now_ms;
    } else if (now_ms - _sensor_failsafe_ms >= IMPEDANCE_ATT_SENSOR_FAILSAFE_MS) {
        admittance_reset();
        copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::UNAVAILABLE);
        return;
    }
#endif

    _virtual_pitch_rad = -_adm_v_body_x;
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, 0.0f);

    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    AltHoldModeState imp_state = get_alt_hold_state_D_ms(target_climb_rate_ms);
    bool use_attitude_output = false;

    switch (imp_state) {

    case AltHoldModeState::MotorStopped:
        admittance_reset();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        admittance_reset();
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        admittance_reset();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        pos_control->D_relax_controller(0.0f);
        break;

    case AltHoldModeState::Takeoff:
        admittance_reset();
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0, 10.0));
        }
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        loiter_nav->update();
        break;

    case AltHoldModeState::Flying:
        if (force_ready) {
            admittance_update(pos_control->get_dt_s());
        }
        _virtual_pitch_rad = -_adm_v_body_x;
        loiter_nav->update();
        use_attitude_output = true;

#if HAL_LOGGING_ENABLED
        copter.Log_Write_Impedance(
            _adm_force_ref, _adm_force_est,
            _adm_force_ref - _adm_force_est,
            _adm_v_body_x,
            0.0f, 0.0f,
            _virtual_pitch_rad);
#endif

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif

        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    if (use_attitude_output) {
        const Vector3f& accel_target_ned = pos_control->get_accel_target_NED_mss();
        const float accel_right_mss = -accel_target_ned.x * ahrs.sin_yaw() + accel_target_ned.y * ahrs.cos_yaw();
        const float roll_lat_rad = accel_mss_to_angle_rad(accel_right_mss * cosf(_virtual_pitch_rad));

        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
            roll_lat_rad,
            _virtual_pitch_rad,
            target_yaw_rate_rads);
    } else {
        attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    }
    pos_control->D_update_controller();
}

float ModeImpedanceAttitude::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeImpedanceAttitude::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}

void ModeImpedanceAttitude::admittance_reset()
{
    _adm_v_body_x = 0.0f;
    _adm_force_est = 0.0f;
    _adm_force_est_filt = 0.0f;
    _virtual_pitch_rad = 0.0f;
}

void ModeImpedanceAttitude::admittance_update(float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    const float force_err = _adm_force_ref - _adm_force_est;
    const float gain = _adm_gain.get();
    const float pitch_max_rad = radians(MAX(_adm_pitch_max_deg.get(), 0.0f));

    _adm_v_body_x = constrain_float(gain * force_err, -pitch_max_rad, pitch_max_rad);
}

#endif // MODE_IMPEDANCE_ATTITUDE_ENABLED
