#include "Copter.h"

namespace {
// First coarse tuning pass: use a larger geometric arc so the suspended
// tool has enough horizontal travel to rise clear of the ground.
constexpr float TSDT4_ARC_RADIUS_CM = 200.0f;
// The suspended-tool vehicle needs a slower reference than a bare frame;
// sixteen seconds limits horizontal acceleration and endpoint overshoot.
constexpr uint32_t TSDT4_ARC_TIME_MS = 16000U;
// Give the suspended tool more time to settle during the reverse landing arc.
constexpr uint32_t TSDT4_LAND_ARC_TIME_MS = 30000U;
// The suspended saw reaches the ground while the airframe is still about
// 1.3 m above the home reference.  End the reverse arc at that support height
// instead of driving the airframe through the ground.
constexpr float TSDT4_LAND_ENDPOINT_Z_CM = 130.0f;
// The hanging tool is mounted about 1.6 m forward of the airframe reference;
// put that support point over the takeoff footprint at the landing endpoint.
constexpr float TSDT4_LAND_ENDPOINT_X_CM = -160.0f;
// Never let the commanded vertical path get far below the airframe.  This
// prevents a suspended-tool contact from accumulating a large downward
// altitude error and then releasing the vehicle into a drop.
constexpr float TSDT4_LAND_MAX_Z_LAG_CM = 10.0f;
// The 1.3 m tool-support height is not an aircraft landing.  Continue a slow,
// altitude-controlled final descent until ArduCopter's real land detector
// confirms that the airframe is down.
constexpr float TSDT4_FINAL_DESCENT_CMS = 5.0f;
constexpr uint32_t TSDT4_FINAL_DESCENT_TIMEOUT_MS = 40000U;
constexpr uint16_t TSDT4_TRIGGER_PWM = 1800U;
constexpr uint32_t TSDT4_SWITCH_LOW_CONFIRM_MS = 500U;
enum class TsdT4TakeoffState : uint8_t { IDLE, ARC, HOLD, LAND_ARC, LAND_HOLD, ABORT };
TsdT4TakeoffState tsdt4_state = TsdT4TakeoffState::IDLE;
Vector3f tsdt4_start_neu_cm;
float tsdt4_start_yaw_rad = 0.0f;
uint32_t tsdt4_start_ms = 0;
uint32_t tsdt4_land_start_ms = 0;
uint32_t tsdt4_land_hold_start_ms = 0;
bool tsdt4_trigger_high = false;
bool tsdt4_land_alt_hold = false;
float tsdt4_land_target_z_cm = 0.0f;
float tsdt4_land_target_velocity_z_cms = 0.0f;
// Do not treat a high RC13 value present at boot as an operator command.
bool tsdt4_trigger_seen_low = false;
uint32_t tsdt4_switch_low_since_ms = 0;

float smoothstep5(const float s) { return 10.0f*s*s*s - 15.0f*s*s*s*s + 6.0f*s*s*s*s*s; }

void tsdt4_update_arc(Copter &copter, const uint32_t now_ms)
{
    const float duration_s = TSDT4_ARC_TIME_MS * 0.001f;
    const float t = constrain_float((now_ms - tsdt4_start_ms) * 0.001f, 0.0f, duration_s);
    const float s = t / duration_s;
    const float lambda = smoothstep5(s);
    const float theta = 0.5f * float(M_PI) * lambda;
    const float x = TSDT4_ARC_RADIUS_CM * (1.0f - cosf(theta));
    const float z = TSDT4_ARC_RADIUS_CM * sinf(theta);
    const float cy = cosf(tsdt4_start_yaw_rad);
    const float sy = sinf(tsdt4_start_yaw_rad);
    const Vector3f destination = tsdt4_start_neu_cm + Vector3f(cy*x, sy*x, z);
    if (!copter.tsdt4_set_guided_position(destination,
                                          tsdt4_start_yaw_rad * 18000.0f / float(M_PI))) {
        tsdt4_state = TsdT4TakeoffState::ABORT;
    } else if (s >= 1.0f) {
        tsdt4_state = TsdT4TakeoffState::HOLD;
    }
}

void tsdt4_update_land_arc(Copter &copter, const uint32_t now_ms)
{
    const float duration_s = TSDT4_LAND_ARC_TIME_MS * 0.001f;
    const float t = constrain_float((now_ms - tsdt4_land_start_ms) * 0.001f, 0.0f, duration_s);
    const float s = t / duration_s;
    const float lambda = smoothstep5(s);
    const float theta = 0.5f * float(M_PI) * (1.0f - lambda);
    const float x = TSDT4_LAND_ENDPOINT_X_CM +
        (TSDT4_ARC_RADIUS_CM - TSDT4_LAND_ENDPOINT_X_CM) * sinf(theta);
    // Keep the 2 m horizontal quarter-arc, but reserve the measured support
    // height for the hanging tool at the landing endpoint.
    const float z = TSDT4_LAND_ENDPOINT_Z_CM +
        (TSDT4_ARC_RADIUS_CM - TSDT4_LAND_ENDPOINT_Z_CM) * sinf(theta);
    const float cy = cosf(tsdt4_start_yaw_rad);
    const float sy = sinf(tsdt4_start_yaw_rad);
    if (s < 0.5f) {
        const Vector3f destination = tsdt4_start_neu_cm + Vector3f(cy*x, sy*x, z);
        if (!copter.tsdt4_set_guided_position(destination,
                                              tsdt4_start_yaw_rad * 18000.0f / float(M_PI))) {
            tsdt4_state = TsdT4TakeoffState::ABORT;
        }
    } else {
        // Preserve the vertical part of the reverse arc after horizontal
        // position control is released.  ModeAltHold consumes this absolute
        // height and velocity target while keeping roll/pitch pilot-controlled.
        const float dlambda_ds = 30.0f * s * s * (1.0f - s) * (1.0f - s);
        const float dtheta_dt = -0.5f * float(M_PI) * dlambda_ds / duration_s;
        tsdt4_land_target_z_cm = tsdt4_start_neu_cm.z + z;
        tsdt4_land_target_velocity_z_cms =
            (TSDT4_ARC_RADIUS_CM - TSDT4_LAND_ENDPOINT_Z_CM) * cosf(theta) * dtheta_dt;
        if (!tsdt4_land_alt_hold) {
            if (!copter.tsdt4_set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND)) {
                tsdt4_state = TsdT4TakeoffState::ABORT;
                return;
            }
            tsdt4_land_alt_hold = true;
            gcs().send_text(MAV_SEVERITY_INFO, "TSDT4: landing midpoint, ALT_HOLD attitude control");
        }
        // Neutral throttle prevents the normal AltHold stick path from moving
        // its Z target.  The explicit, lag-limited Z target is applied in
        // ModeAltHold::run; no horizontal position target is sent here.
        RC_Channels::set_override(CH_3, 1500U, now_ms);
        if (s >= 1.0f) {
            tsdt4_state = TsdT4TakeoffState::LAND_HOLD;
            tsdt4_land_hold_start_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "TSDT4: landing arc endpoint reached");
        }
    }
}
} // namespace

bool Copter::tsdt4_set_guided_target(const Vector3f& destination, const Vector3f& velocity,
                                     const Vector3f& acceleration, const float yaw_cd)
{
    return mode_guided.set_destination_posvelaccel(destination, velocity, acceleration, true, yaw_cd);
}

bool Copter::tsdt4_set_guided_position(const Vector3f& destination, const float yaw_cd)
{
    return mode_guided.set_destination(destination, true, yaw_cd, false, 0.0f, false, false);
}

bool Copter::tsdt4_set_mode(const Mode::Number mode, const ModeReason reason)
{
    return set_mode(mode, reason);
}

bool Copter::tsdt4_get_alt_hold_z_target(float& position_z_cm, float& velocity_z_cms) const
{
    if ((tsdt4_state != TsdT4TakeoffState::LAND_ARC &&
         tsdt4_state != TsdT4TakeoffState::LAND_HOLD) ||
        !tsdt4_land_alt_hold) {
        return false;
    }

    const float current_z_cm = inertial_nav.get_position_neu_cm().z;
    position_z_cm = MAX(tsdt4_land_target_z_cm, current_z_cm - TSDT4_LAND_MAX_Z_LAG_CM);
    velocity_z_cms = (position_z_cm > tsdt4_land_target_z_cm) ?
        0.0f : tsdt4_land_target_velocity_z_cms;
    return true;
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool trigger_high = RC_Channels::get_radio_in(CH_13) >= TSDT4_TRIGGER_PWM;
    if (!trigger_high) {
        if (tsdt4_switch_low_since_ms == 0) {
            tsdt4_switch_low_since_ms = now_ms;
        } else if (now_ms - tsdt4_switch_low_since_ms >= TSDT4_SWITCH_LOW_CONFIRM_MS) {
            tsdt4_trigger_seen_low = true;
        }
    } else {
        tsdt4_switch_low_since_ms = 0;
    }
    const bool rising_edge = trigger_high && tsdt4_trigger_seen_low && !tsdt4_trigger_high;
    const bool falling_edge = !trigger_high && tsdt4_trigger_high;
    tsdt4_trigger_high = trigger_high;

    // A failed attempt is re-armed only after the operator returns CH13 low.
    if (!trigger_high && tsdt4_state == TsdT4TakeoffState::ABORT) {
        tsdt4_state = TsdT4TakeoffState::IDLE;
    }

    if (rising_edge && tsdt4_state == TsdT4TakeoffState::IDLE) {
        // Consume the edge even when a safety gate rejects it.  A held-high
        // switch must never retry autonomously after EKF/GPS becomes ready.
        tsdt4_trigger_seen_low = false;
        if (!position_ok() || !ap.land_complete || any_failsafe_triggered()) {
            tsdt4_state = TsdT4TakeoffState::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING, "TSDT4: takeoff gate failed");
        } else if (!set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            tsdt4_state = TsdT4TakeoffState::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING, "TSDT4: GUIDED rejected");
        } else if (!motors->armed() && !arming.arm(AP_Arming::Method::MAVLINK)) {
            tsdt4_state = TsdT4TakeoffState::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING, "TSDT4: arming rejected");
        } else {
            tsdt4_start_neu_cm = inertial_nav.get_position_neu_cm();
            tsdt4_start_yaw_rad = ahrs.get_yaw();
            tsdt4_start_ms = now_ms;
            // The arc controller is the takeoff mode for the suspended tool;
            // prevent the ground disarm path from treating its initial pose as idle.
            set_land_complete(false);
            set_land_complete_maybe(false);
            ap.auto_armed = true;
            tsdt4_state = TsdT4TakeoffState::ARC;
            gcs().send_text(MAV_SEVERITY_INFO, "TSDT4: arc takeoff started");
        }
    }

    if (tsdt4_state == TsdT4TakeoffState::ARC) {
        tsdt4_update_arc(*this, now_ms);
    } else if (falling_edge && tsdt4_state == TsdT4TakeoffState::HOLD) {
        tsdt4_land_start_ms = now_ms;
        tsdt4_land_alt_hold = false;
        tsdt4_state = TsdT4TakeoffState::LAND_ARC;
        gcs().send_text(MAV_SEVERITY_INFO, "TSDT4: reverse landing arc started");
    } else if (tsdt4_state == TsdT4TakeoffState::LAND_ARC) {
        tsdt4_update_land_arc(*this, now_ms);
    } else if (tsdt4_state == TsdT4TakeoffState::HOLD) {
        const float cy = cosf(tsdt4_start_yaw_rad);
        const float sy = sinf(tsdt4_start_yaw_rad);
        const Vector3f destination = tsdt4_start_neu_cm +
            Vector3f(cy*TSDT4_ARC_RADIUS_CM, sy*TSDT4_ARC_RADIUS_CM, TSDT4_ARC_RADIUS_CM);
        if (!tsdt4_set_guided_position(destination,
                                       tsdt4_start_yaw_rad * 18000.0f / float(M_PI))) {
            tsdt4_state = TsdT4TakeoffState::ABORT;
        }
    } else if (tsdt4_state == TsdT4TakeoffState::LAND_HOLD) {
        if (ap.land_complete) {
            arming.disarm(AP_Arming::Method::LANDED);
            tsdt4_state = TsdT4TakeoffState::IDLE;
            gcs().send_text(MAV_SEVERITY_INFO, "TSDT4: landing complete");
        } else {
            // Tool support is only the start of the final landing phase.  Keep
            // horizontal control released, but continue an absolute Z target
            // at a slow rate until the airframe itself is detected as landed.
            const float final_elapsed_s = (now_ms - tsdt4_land_hold_start_ms) * 0.001f;
            const float final_floor_z_cm = tsdt4_start_neu_cm.z;
            tsdt4_land_target_z_cm = MAX(final_floor_z_cm,
                tsdt4_start_neu_cm.z + TSDT4_LAND_ENDPOINT_Z_CM -
                TSDT4_FINAL_DESCENT_CMS * final_elapsed_s);
            tsdt4_land_target_velocity_z_cms =
                (tsdt4_land_target_z_cm > final_floor_z_cm) ? -TSDT4_FINAL_DESCENT_CMS : 0.0f;
            RC_Channels::set_override(CH_3, 1500U, now_ms);

            if (now_ms - tsdt4_land_hold_start_ms > TSDT4_FINAL_DESCENT_TIMEOUT_MS) {
                tsdt4_set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
                tsdt4_state = TsdT4TakeoffState::ABORT;
                gcs().send_text(MAV_SEVERITY_WARNING, "TSDT4: final descent timeout, LAND fallback");
            }
        }
    }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
