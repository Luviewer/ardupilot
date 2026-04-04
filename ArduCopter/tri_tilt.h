#pragma once

#include <AP_Param/AP_Param.h>

// TriTilt tilt-servo pitch control state and tunable parameters.
// Lifetime is owned by Copter::_tritilt; registered in ParametersG2 as TTLT_.
struct TriTiltState {
    static const AP_Param::GroupInfo var_info[];

    // Tunable parameters (prefix TTLT_ in GCS)
    AP_Float rate_max;      // TTLT_RATE_MAX: max manual pitch rate (deg/s)
    AP_Float rtz_rate;      // TTLT_RTZ_RATE: return-to-zero pitch rate (deg/s)

    enum class State : uint8_t {
        IDLE = 0,
        HOLD,
        MANUAL,
        RTZ_WAIT,
        RTZ_ACTIVE,
        GOTO_TARGET,   // move toward cmd_value at cmd_rate, then hold
    } state;

    float    pitch_off_deg;

    // MAVLink command fields
    bool     cmd_pending;
    float    cmd_value;  // target angle (deg)
    float    cmd_rate;   // motion rate (deg/s); 0 = use TTLT_RATE_MAX default

    // Set true once the pitch stick has been centered after power-on.
    // Manual rate commands are blocked until this is true.
    bool pitch_stick_armed;

    // Set true when RTZ completes (pitch at zero). Cleared when the RTZ
    // switch goes low. Prevents RTZ from re-triggering while switch stays high.
    bool rtz_needs_reset;

    // Throttle-limited warning timestamps
    uint32_t warn_stick_not_armed_ms;
    uint32_t warn_center_stick_ms;
    uint32_t warn_switch_low_ms;
    uint32_t named_float_send_ms;

    // Boundary notification flags (fire once on entry)
    bool upper_limit_reported;
    bool lower_limit_reported;
    bool zero_position_reported;

    TriTiltState();
};
