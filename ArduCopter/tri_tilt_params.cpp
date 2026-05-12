#include "Copter.h"

#if AP_SCRIPTING_ENABLED && (AP_MOTORS_TRI_TILT_ENABLED || AP_MOTORS_QUAD_TILT_ENABLED)

const AP_Param::GroupInfo TriTiltState::var_info[] = {

    // @Param: RATE_MAX
    // @DisplayName: Tilt pitch max rate
    // @Description: Maximum tilt servo pitch rate for manual RC control. RC stick full deflection commands this rate.
    // @Units: deg/s
    // @Range: 0.5 90
    // @User: Standard
    AP_GROUPINFO("RATE_MAX", 1, TriTiltState, rate_max, 10.0f),

    // @Param: RTZ_RATE
    // @DisplayName: Tilt return to zero rate
    // @Description: Tilt servo pitch rate used when returning to the zero position.
    // @Units: deg/s
    // @Range: 0.5 30
    // @User: Standard
    AP_GROUPINFO("RTZ_RATE", 2, TriTiltState, rtz_rate, 5.0f),

    AP_GROUPEND
};

TriTiltState::TriTiltState()
    : state(State::IDLE),
      pitch_off_deg(0.0f),
      cmd_pending(false),
      cmd_value(0.0f),
      cmd_rate(0.0f),
      pitch_stick_armed(false),
      rtz_needs_reset(false),
      warn_stick_not_armed_ms(0),
      warn_center_stick_ms(0),
      warn_switch_low_ms(0),
      named_float_send_ms(0),
      contact_float_send_ms(0),
      upper_limit_reported(false),
      lower_limit_reported(false),
      zero_position_reported(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_SCRIPTING_ENABLED && (AP_MOTORS_TRI_TILT_ENABLED || AP_MOTORS_QUAD_TILT_ENABLED)
