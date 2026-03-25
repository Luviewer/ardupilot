#include "Copter.h"

/*
 * Impedance flight mode — Loiter-based mode with admittance control on the
 * forward (pitch) axis.  Roll, yaw and throttle behave identically to Loiter.
 *
 * Pitch stick input is interpreted as a normalised force reference (-1..+1).
 * The estimated body-X thrust ratio from the motor mixer is used as force
 * feedback.  An admittance (force → velocity) filter produces a body-X
 * velocity command that is injected into the Loiter trajectory generator as a
 * NE velocity offset.
 */

const AP_Param::GroupInfo ModeImpedance::var_info[] = {

    // @Param: ADM_GAIN
    // @DisplayName: Admittance force-to-velocity gain
    // @Description: Scale from force error to admittance velocity output (m/s per unit force). Set to 0 to disable admittance and behave like Loiter.
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ADM_GAIN", 1, ModeImpedance, _adm_gain, 0.1f),

    // @Param: ADM_TAU
    // @DisplayName: Admittance time constant
    // @Description: First-order time constant for admittance filter. Larger = softer response
    // @Range: 0.05 5.0
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("ADM_TAU", 2, ModeImpedance, _adm_tau, 0.5f),

    // @Param: ADM_VMAX
    // @DisplayName: Admittance max velocity
    // @Description: Maximum velocity output from admittance controller
    // @Range: 0.5 20.0
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("ADM_VMAX", 3, ModeImpedance, _adm_vmax, 5.0f),

    // @Param: ADM_DZ
    // @DisplayName: Admittance stick deadzone
    // @Description: Pitch stick deadzone for force reference input (normalized 0-1)
    // @Range: 0.0 0.3
    // @User: Advanced
    AP_GROUPINFO("ADM_DZ", 4, ModeImpedance, _adm_dz, 0.05f),

    // @Param: ADM_FMAX
    // @DisplayName: Admittance force reference limit
    // @Description: Maximum absolute value of force reference from pitch stick (0-1). Limits how much force the pilot can command.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("ADM_FMAX", 5, ModeImpedance, _adm_fmax, 0.5f),

    // @Param: ADM_FLPF
    // @DisplayName: Force estimate low-pass filter frequency
    // @Description: Cutoff frequency for low-pass filter on force estimate. Lower = smoother but more lag. Set 0 to disable.
    // @Range: 0.0 10.0
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("ADM_FLPF", 6, ModeImpedance, _adm_fest_lpf_hz, 2.0f),

    // @Param: ADM_DAMP
    // @DisplayName: Admittance force-error derivative damping
    // @Description: Damping on rate of change of force error. Suppresses oscillation when contacting obstacles. Set 0 to disable.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ADM_DAMP", 7, ModeImpedance, _adm_damp, 0.0f),

    AP_GROUPEND
};

ModeImpedance::ModeImpedance() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeImpedance::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;
    update_simple_mode();

    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);
    loiter_nav->init_target();

    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    admittance_reset();

    return true;
}

void ModeImpedance::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;

    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    update_simple_mode();

    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    // Pitch stick → force reference (sign: push-forward = negative norm_input,
    // but we want push-forward = positive force_ref so negate)
    float pitch_stick_norm = -channel_pitch->norm_input_dz();
    if (fabsf(pitch_stick_norm) < _adm_dz.get()) {
        pitch_stick_norm = 0.0f;
    }
    const float fmax = constrain_float(_adm_fmax.get(), 0.01f, 1.0f);
    _adm_force_ref = constrain_float(pitch_stick_norm, -fmax, fmax);

    const float raw_fest = copter.motors->get_est_body_x_thrust_ratio();
    const float lpf_hz = _adm_fest_lpf_hz.get();
    if (lpf_hz > 0.0f) {
        const float dt = pos_control->get_dt_s();
        const float alpha = constrain_float(dt * lpf_hz * M_2PI, 0.0f, 1.0f);
        _adm_force_est_filt += (raw_fest - _adm_force_est_filt) * alpha;
    } else {
        _adm_force_est_filt = raw_fest;
    }
    _adm_force_est = _adm_force_est_filt;

    // Feed a virtual pitch angle proportional to admittance velocity so that
    // Loiter's internal brake/drag logic sees "pilot input" on the forward
    // axis and does not fight the admittance velocity offset.
    _virtual_pitch_rad = 0.0f;
    if (!is_zero(_adm_v_body_x)) {
        const float speed_max = MAX(loiter_nav->get_speed_max_NE_ms(), 0.1f);
        const float angle_max = loiter_nav->get_angle_max_rad();
        _virtual_pitch_rad = -(_adm_v_body_x / speed_max) * angle_max;
        _virtual_pitch_rad = constrain_float(_virtual_pitch_rad, -angle_max, angle_max);
    }
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, _virtual_pitch_rad);

    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    AltHoldModeState imp_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

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
        admittance_update(pos_control->get_dt_s());
        loiter_nav->update();

#if HAL_LOGGING_ENABLED
        copter.Log_Write_Impedance(
            _adm_force_ref, _adm_force_est,
            _adm_force_ref - _adm_force_est,
            _adm_v_body_x,
            _vel_offset_ne.x, _vel_offset_ne.y,
            _virtual_pitch_rad);
#endif

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif

        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    pos_control->D_update_controller();
}

float ModeImpedance::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeImpedance::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}

void ModeImpedance::admittance_reset()
{
    _adm_v_body_x = 0.0f;
    _adm_force_ref = 0.0f;
    _adm_force_est = 0.0f;
    _adm_force_est_filt = 0.0f;
    _prev_force_err = 0.0f;
    _virtual_pitch_rad = 0.0f;
    _vel_offset_ne.zero();
    loiter_nav->clear_vel_offset_NE_ms();
}

void ModeImpedance::admittance_update(float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    const float force_err = _adm_force_ref - _adm_force_est;
    const float d_force_err = (force_err - _prev_force_err) / dt;
    _prev_force_err = force_err;

    const float gain = _adm_gain.get();
    const float tau  = MAX(_adm_tau.get(), 0.01f);
    const float vmax = _adm_vmax.get();
    const float damp = _adm_damp.get();

    _adm_v_body_x += (gain * force_err - damp * d_force_err - _adm_v_body_x) * (dt / tau);
    _adm_v_body_x = constrain_float(_adm_v_body_x, -vmax, vmax);

    _vel_offset_ne.x = _adm_v_body_x * ahrs.cos_yaw();
    _vel_offset_ne.y = _adm_v_body_x * ahrs.sin_yaw();

    loiter_nav->set_vel_offset_NE_ms(_vel_offset_ne);
}
