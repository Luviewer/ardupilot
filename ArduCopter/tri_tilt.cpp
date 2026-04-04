#include "Copter.h"

#if AP_SCRIPTING_ENABLED && AP_MOTORS_TRI_TILT_ENABLED

// Send a GCS warning message at most once per second.
static void throttled_warn(uint32_t &last_ms, const char *msg)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_ms >= 1000U) {
        last_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_WARNING, "%s", msg);
    }
}

void Copter::tritilt_update()
{
    using State = TriTiltState::State;

    // Early exit: only run for TriTilt frames with tilt enabled
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() != AP_Motors::MOTOR_FRAME_TRI ||
        (AP_Motors::motor_frame_type)g.frame_type.get()    != AP_Motors::MOTOR_FRAME_TYPE_TRI_TILT ||
        !motors->get_tilt_enable()) {
        return;
    }

    // Read RC inputs
    RC_Channel *pitch_ctrl_ch  = rc().find_channel_for_option(RC_Channel::AUX_FUNC::TRITILT_PITCH_CTRL);
    RC_Channel *return_zero_ch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::TRITILT_RETURN_TO_ZERO);
    const uint16_t pwm_return_zero = (return_zero_ch != nullptr) ? return_zero_ch->get_radio_in() : 0U;

    const uint32_t now_ms = AP_HAL::millis();
    const float    dt     = G_Dt;

    // Normalised pitch stick: 0.0 in deadzone, ±1.0 at full deflection.
    // norm_input_dz() respects RC calibration, trim and deadzone parameters.
    const float pitch_norm = (pitch_ctrl_ch != nullptr) ? pitch_ctrl_ch->norm_input_dz() : 0.0f;

    // Derived booleans
    const bool pitch_ctrl_valid    = (pitch_ctrl_ch != nullptr) && (uint16_t)pitch_ctrl_ch->get_radio_in() >= 900U;
    const bool pitch_in_mid_dz     = pitch_ctrl_valid && is_zero(pitch_norm);
    const bool pitch_manual_active = pitch_ctrl_valid && !is_zero(pitch_norm);
    const bool return_zero_valid   = (pwm_return_zero >= 900U && pwm_return_zero <= 2100U);
    const bool return_zero_is_low  = return_zero_valid && (pwm_return_zero < 1500U);
    const bool rtz_trigger         = return_zero_valid && (pwm_return_zero > 1800U && pwm_return_zero < 2100U);
    // Invalid signal or switch low both reset RTZ
    const bool rtz_reset           = !return_zero_valid || (pwm_return_zero < 1500U);

    // Clear RTZ-needs-reset latch as soon as the switch goes back low
    if (rtz_reset) {
        _tritilt.rtz_needs_reset = false;
    }

    // Angle limits
    const float max_pitch_off = MIN(MAX(0.0f, motors->get_tilt_max_deg()), 90.0f);
    const float min_pitch_off = -max_pitch_off;

    float& pitch_off_deg = _tritilt.pitch_off_deg;
    float  pitch_rate    = 0.0f;

    // -----------------------------------------------------------------------
    // MAVLink command pre-processing (overrides FSM state when present)
    // -----------------------------------------------------------------------
    if (_tritilt.cmd_pending) {
        _tritilt.cmd_pending = false;
        // Both modes move smoothly to target; cmd_rate==0 means use TTLT_RATE_MAX
        _tritilt.state = State::GOTO_TARGET;
    }

    // -----------------------------------------------------------------------
    // FSM: state transitions + rate output
    // -----------------------------------------------------------------------
    switch (_tritilt.state) {

    case State::IDLE:
        // Arm the stick when it is first seen centered
        if (pitch_ctrl_valid) {
            if (pitch_in_mid_dz) {
                _tritilt.pitch_stick_armed = true;
            }
            _tritilt.state = State::HOLD;
        }
        break;

    case State::HOLD:
        if (!pitch_ctrl_valid) {
            _tritilt.state = State::IDLE;
        } else if (rtz_trigger && !_tritilt.rtz_needs_reset) {
            _tritilt.state = State::RTZ_WAIT;
        } else if (pitch_manual_active) {
            if (!_tritilt.pitch_stick_armed) {
                // Stick was not at center at power-on; wait for it to be centered first
                throttled_warn(_tritilt.warn_stick_not_armed_ms,
                               "Tilt servo center pitch stick to enable");
            } else if (return_zero_is_low) {
                _tritilt.state = State::MANUAL;
            } else {
                throttled_warn(_tritilt.warn_switch_low_ms,
                               "Tilt servo lower return switch for manual pitch");
            }
        } else if (pitch_in_mid_dz) {
            // Record centering so future deflections are allowed
            _tritilt.pitch_stick_armed = true;
        }
        // pitch_rate stays 0 — hold current angle
        break;

    case State::MANUAL:
        if (!pitch_ctrl_valid) {
            _tritilt.state = State::IDLE;
        } else if (!pitch_manual_active) {
            _tritilt.state = State::HOLD;
        } else if (!return_zero_is_low) {
            throttled_warn(_tritilt.warn_switch_low_ms,
                           "Tilt servo lower return switch for manual pitch");
            // pitch_rate stays 0 — refuse to move
        } else {
            pitch_rate = pitch_norm * _tritilt.rate_max.get();
        }
        break;

    case State::RTZ_WAIT:
        if (rtz_reset) {
            _tritilt.state = State::HOLD;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo return to zero cancelled");
            _tritilt.warn_center_stick_ms = 0;
        } else if (pitch_in_mid_dz) {
            _tritilt.state = State::RTZ_ACTIVE;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo returning to zero");
            _tritilt.warn_center_stick_ms = 0;
        } else {
            throttled_warn(_tritilt.warn_center_stick_ms,
                           "Tilt servo center pitch stick first");
        }
        break;

    case State::RTZ_ACTIVE:
        if (rtz_reset) {
            _tritilt.state = State::HOLD;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo return to zero cancelled");
        } else if (is_zero(pitch_off_deg)) {
            _tritilt.state                  = State::HOLD;
            _tritilt.rtz_needs_reset        = true;
            _tritilt.zero_position_reported = true;  // suppress duplicate from boundary check
            gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo pitch at zero");
        } else {
            pitch_rate = (pitch_off_deg > 0.0f) ? -_tritilt.rtz_rate.get() : _tritilt.rtz_rate.get();
        }
        break;

    case State::GOTO_TARGET: {
        if (!pitch_ctrl_valid) {
            // RC lost: return to zero immediately (no stick to confirm RTZ_WAIT)
            _tritilt.state = State::RTZ_ACTIVE;
        } else if (rtz_trigger && !_tritilt.rtz_needs_reset) {
            // RTZ switch: interrupt goto, require stick-centre confirmation
            _tritilt.state = State::RTZ_WAIT;
        } else if (pitch_manual_active && _tritilt.pitch_stick_armed && return_zero_is_low) {
            // Pilot takes over manually
            _tritilt.state = State::MANUAL;
        } else {
            const float target = constrain_float(_tritilt.cmd_value, min_pitch_off, max_pitch_off);
            // cmd_rate == 0: use TTLT_RATE_MAX default; otherwise clamp to [0.1, rate_max]
            const float goto_rate = is_positive(_tritilt.cmd_rate)
                                    ? constrain_float(_tritilt.cmd_rate, 0.1f, _tritilt.rate_max.get())
                                    : _tritilt.rate_max.get();
            const float err = target - pitch_off_deg;
            if (fabsf(err) <= goto_rate * dt) {
                // Close enough: snap to target and hold
                pitch_off_deg  = target;
                _tritilt.state = State::HOLD;
                _tritilt.zero_position_reported = is_zero(target);
            } else {
                pitch_rate = (err > 0.0f) ? goto_rate : -goto_rate;
            }
        }
        break;
    }
    }

    // -----------------------------------------------------------------------
    // Integrate rate → angle, with clamping and RTZ snap-to-zero
    // -----------------------------------------------------------------------
    if ((pitch_off_deg >= max_pitch_off && pitch_rate > 0.0f) ||
        (pitch_off_deg <= min_pitch_off && pitch_rate < 0.0f)) {
        pitch_rate = 0.0f;
    } else if (_tritilt.state == State::RTZ_ACTIVE && !is_zero(pitch_rate)) {
        const float next = pitch_off_deg + pitch_rate * dt;
        if ((pitch_off_deg > 0.0f && next <= 0.0f) ||
            (pitch_off_deg < 0.0f && next >= 0.0f)) {
            // Snap to zero on crossover
            pitch_off_deg = 0.0f;
            pitch_rate    = 0.0f;
        } else {
            pitch_off_deg = constrain_float(next, min_pitch_off, max_pitch_off);
        }
    } else {
        pitch_off_deg += pitch_rate * dt;
        pitch_off_deg  = constrain_float(pitch_off_deg, min_pitch_off, max_pitch_off);
    }

    // -----------------------------------------------------------------------
    // Write to INS and compass
    // -----------------------------------------------------------------------
    AP::ins().set_imu_pitch_rot_rate_deg_per_sec(pitch_rate);
    AP::ins().set_imu_pitch_rot_deg(pitch_off_deg);
    AP::compass().set_imu_pitch_rot_deg(pitch_off_deg);

    // -----------------------------------------------------------------------
    // Logging (TTLT)
    // -----------------------------------------------------------------------
#if HAL_LOGGING_ENABLED
    {
        const float pit_virt = AP::ins().get_imu_pitch_rot_deg();
        const float pit_ahrs = degrees(ahrs.get_pitch());
        Log_Write_TriTilt(pitch_off_deg, pit_virt, pit_ahrs,
                          pit_ahrs + pit_virt,
                          pitch_rate,
                          degrees(ahrs.get_gyro().y));
    }
#endif

    // -----------------------------------------------------------------------
    // MAVLink telemetry: PitOff / PitTrue / RTZCh at 5 Hz
    // -----------------------------------------------------------------------
    if (now_ms - _tritilt.named_float_send_ms >= 200U) {
        _tritilt.named_float_send_ms = now_ms;
        const float pit_true = degrees(ahrs.get_pitch()) + AP::ins().get_imu_pitch_rot_deg();
        gcs().send_named_float("PitOff",  pitch_off_deg);
        gcs().send_named_float("PitTrue", pit_true);
        gcs().send_named_float("RTZCh",   return_zero_is_low ? 0.0f : 1.0f);
    }

    // -----------------------------------------------------------------------
    // Boundary notifications (one-shot per crossing; flag cleared when condition lifts)
    // -----------------------------------------------------------------------
    const float epsilon        = 0.01f;
    const bool  at_upper_limit = (pitch_off_deg >= max_pitch_off - epsilon);
    const bool  at_lower_limit = (pitch_off_deg <= min_pitch_off + epsilon);
    const bool  at_zero        = is_zero(pitch_off_deg);

    if (at_upper_limit && !_tritilt.upper_limit_reported) {
        _tritilt.upper_limit_reported = true;
        gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo pitch upper limit reached");
    } else if (!at_upper_limit) {
        _tritilt.upper_limit_reported = false;
    }

    if (at_lower_limit && !_tritilt.lower_limit_reported) {
        _tritilt.lower_limit_reported = true;
        gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo pitch lower limit reached");
    } else if (!at_lower_limit) {
        _tritilt.lower_limit_reported = false;
    }

    if (at_zero && !_tritilt.zero_position_reported) {
        _tritilt.zero_position_reported = true;
        gcs().send_text(MAV_SEVERITY_NOTICE, "Tilt servo pitch at zero");
    } else if (!at_zero) {
        _tritilt.zero_position_reported = false;
    }
}

#endif  // AP_SCRIPTING_ENABLED && AP_MOTORS_TRI_TILT_ENABLED
