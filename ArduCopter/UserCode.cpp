#include "Copter.h"

namespace {

constexpr uint16_t TSDT4_TRIGGER_PWM = 1800U;
constexpr uint32_t TSDT4_SWITCH_LOW_CONFIRM_MS = 500U;

constexpr float TSDT4_TAKEOFF_HEIGHT_CM = 200.0f;
// Keep the Round5 takeoff command unchanged while the landing phase boundary
// is tested as the only control-logic change in this run.
constexpr float TSDT4_TAKEOFF_PITCH_RAD = radians(-0.5f);
constexpr float TSDT4_TAKEOFF_CLIMB_RATE_CMS = 15.0f;
constexpr uint32_t TSDT4_TAKEOFF_BLEND_MS = 1000U;
constexpr uint32_t TSDT4_TAKEOFF_LEVEL_MS = 1000U;
constexpr uint32_t TSDT4_TAKEOFF_TIMEOUT_MS = 30000U;

// The fixed post-contact command is intentionally independent of XY error and
// velocity.  Attitude magnitude is not itself a pass/fail criterion; command
// continuity, touchdown, and natural final rest are.
constexpr float TSDT4_LAND_PITCH_RAD = radians(4.0f);
constexpr float TSDT4_LAND_CLIMB_RATE_CMS = -5.0f;
constexpr uint32_t TSDT4_LAND_BLEND_MS = 1000U;
constexpr uint32_t TSDT4_CONTACT_ARM_DELAY_MS = 1500U;
constexpr float TSDT4_CONTACT_MIN_DESCENT_CM = 20.0f;
constexpr float TSDT4_CONTACT_ACCEL_Z_MSS = 0.18f;
constexpr uint8_t TSDT4_CONTACT_CONFIRM_SAMPLES = 2U;
constexpr float TSDT4_AIRFRAME_LAND_GATE_Z_CM = 60.0f;
constexpr uint32_t TSDT4_LAND_TIMEOUT_MS = 90000U;

enum class TsdT4State : uint8_t {
    IDLE,
    TAKEOFF_ANGLE,
    TAKEOFF_LEVEL,
    HOLD,
    LAND_DESCEND,
    LAND_ANGLE,
    ABORT,
};

TsdT4State tsdt4_state = TsdT4State::IDLE;
Vector3f tsdt4_start_neu_cm;
Vector3f tsdt4_land_hold_neu_cm;
float tsdt4_start_yaw_rad;
float tsdt4_takeoff_initial_roll_rad;
float tsdt4_takeoff_initial_pitch_rad;
float tsdt4_land_initial_roll_rad;
float tsdt4_land_initial_pitch_rad;
uint32_t tsdt4_takeoff_start_ms;
uint32_t tsdt4_level_start_ms;
uint32_t tsdt4_land_start_ms;
uint32_t tsdt4_land_angle_start_ms;
float tsdt4_land_start_z_cm;
uint8_t tsdt4_contact_accel_count;
bool tsdt4_trigger_high;
bool tsdt4_trigger_seen_low;
uint32_t tsdt4_switch_low_since_ms;
bool tsdt4_force_flying_owned;

float smoothstep5(const float value)
{
    const float s = constrain_float(value, 0.0f, 1.0f);
    return 10.0f * s * s * s - 15.0f * s * s * s * s +
           6.0f * s * s * s * s * s;
}

float smoothstep5_integral(const float value)
{
    const float s = constrain_float(value, 0.0f, 1.0f);
    const float s2 = s * s;
    const float s4 = s2 * s2;
    return 2.5f * s4 - 3.0f * s4 * s + s4 * s2;
}

Quaternion attitude_target(const float roll_rad, const float pitch_rad,
                           const float yaw_rad)
{
    Quaternion attitude_quat;
    attitude_quat.from_euler(roll_rad, pitch_rad, yaw_rad);
    return attitude_quat;
}

} // namespace

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
    const bool trigger_high =
        RC_Channels::get_radio_in(CH_13) >= TSDT4_TRIGGER_PWM;

    // A boot-time high value is never accepted as an operator edge.  CH13
    // must first remain low for a full debounce interval.
    if (!trigger_high) {
        if (tsdt4_switch_low_since_ms == 0U) {
            tsdt4_switch_low_since_ms = now_ms;
        } else if (now_ms - tsdt4_switch_low_since_ms >=
                   TSDT4_SWITCH_LOW_CONFIRM_MS) {
            tsdt4_trigger_seen_low = true;
        }
    } else {
        tsdt4_switch_low_since_ms = 0U;
    }

    const bool rising_edge =
        trigger_high && tsdt4_trigger_seen_low && !tsdt4_trigger_high;
    const bool falling_edge = !trigger_high && tsdt4_trigger_high;
    tsdt4_trigger_high = trigger_high;

    if (!trigger_high && tsdt4_state == TsdT4State::ABORT) {
        tsdt4_state = TsdT4State::IDLE;
    }

    if (rising_edge && tsdt4_state == TsdT4State::IDLE) {
        // Consume the qualified edge even if a safety gate rejects the run.
        tsdt4_trigger_seen_low = false;
        if (!position_ok() || !ap.land_complete || any_failsafe_triggered()) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: takeoff gate failed");
        } else if (!set_mode(Mode::Number::GUIDED,
                             ModeReason::GCS_COMMAND)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: GUIDED rejected");
        } else if (!motors->armed() &&
                   !arming.arm(AP_Arming::Method::MAVLINK)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: arming rejected");
        } else {
            tsdt4_start_neu_cm = inertial_nav.get_position_neu_cm();
            tsdt4_start_yaw_rad = ahrs.get_yaw();
            tsdt4_takeoff_initial_roll_rad = ahrs.get_roll();
            tsdt4_takeoff_initial_pitch_rad = ahrs.get_pitch();
            tsdt4_takeoff_start_ms = now_ms;

            // [Cybernetics Ch.4] Close the ground-to-flight transition once;
            // subsequent landing state is owned by the native detector.
            set_land_complete(false);
            set_land_complete_maybe(false);
            ap.auto_armed = true;

            tsdt4_state = TsdT4State::TAKEOFF_ANGLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: takeoff angle started");
        }
    }

    const bool active =
        tsdt4_state != TsdT4State::IDLE &&
        tsdt4_state != TsdT4State::ABORT;
    if (active &&
        (any_failsafe_triggered() ||
         flightmode->mode_number() != Mode::Number::GUIDED)) {
        if (tsdt4_force_flying_owned) {
            force_flying = false;
            tsdt4_force_flying_owned = false;
        }
        tsdt4_state = TsdT4State::ABORT;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4: external mode or failsafe abort");
        return;
    }

    const Vector3f no_angular_velocity{};

    if (tsdt4_state == TsdT4State::TAKEOFF_ANGLE) {
        const uint32_t elapsed_ms = now_ms - tsdt4_takeoff_start_ms;
        const float blend = smoothstep5(
            elapsed_ms / float(TSDT4_TAKEOFF_BLEND_MS));
        const float roll_rad =
            tsdt4_takeoff_initial_roll_rad * (1.0f - blend);
        const float pitch_rad =
            tsdt4_takeoff_initial_pitch_rad +
            (TSDT4_TAKEOFF_PITCH_RAD - tsdt4_takeoff_initial_pitch_rad) *
                blend;
        const float climb_rate_cms = TSDT4_TAKEOFF_CLIMB_RATE_CMS * blend;

        // [Cybernetics Ch.4] Guided Angle is refreshed at 10 Hz; no XY
        // position or velocity enters this fixed attitude command.
        mode_guided.set_angle(
            attitude_target(roll_rad, pitch_rad, tsdt4_start_yaw_rad),
            no_angular_velocity, climb_rate_cms, false);

        if (inertial_nav.get_position_neu_cm().z >=
            tsdt4_start_neu_cm.z + TSDT4_TAKEOFF_HEIGHT_CM) {
            tsdt4_level_start_ms = now_ms;
            tsdt4_state = TsdT4State::TAKEOFF_LEVEL;
        } else if (elapsed_ms >= TSDT4_TAKEOFF_TIMEOUT_MS) {
            const Vector3f hold_neu_cm = inertial_nav.get_position_neu_cm();
            const bool hold_set = mode_guided.set_destination(
                hold_neu_cm, true, degrees(tsdt4_start_yaw_rad) * 100.0f,
                false, 0.0f, false, false);
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            hold_set ?
                            "TSDT4: takeoff timeout, position abort" :
                            "TSDT4: takeoff timeout, hold rejected");
        }
        return;
    }

    if (tsdt4_state == TsdT4State::TAKEOFF_LEVEL) {
        const uint32_t elapsed_ms = now_ms - tsdt4_level_start_ms;
        const float blend = smoothstep5(
            elapsed_ms / float(TSDT4_TAKEOFF_LEVEL_MS));
        const float pitch_rad = TSDT4_TAKEOFF_PITCH_RAD * (1.0f - blend);
        const float climb_rate_cms =
            TSDT4_TAKEOFF_CLIMB_RATE_CMS * (1.0f - blend);

        mode_guided.set_angle(
            attitude_target(0.0f, pitch_rad, tsdt4_start_yaw_rad),
            no_angular_velocity, climb_rate_cms, false);

        if (elapsed_ms >= TSDT4_TAKEOFF_LEVEL_MS) {
            const Vector3f hold_neu_cm = inertial_nav.get_position_neu_cm();
            if (mode_guided.set_destination(
                    hold_neu_cm, true,
                    degrees(tsdt4_start_yaw_rad) * 100.0f,
                    false, 0.0f, false, false)) {
                tsdt4_state = TsdT4State::HOLD;
                gcs().send_text(MAV_SEVERITY_INFO,
                                "TSDT4: takeoff position hold");
            } else {
                tsdt4_state = TsdT4State::ABORT;
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "TSDT4: position hold rejected");
            }
        }
        return;
    }

    if (falling_edge && tsdt4_state == TsdT4State::HOLD) {
        tsdt4_land_hold_neu_cm = inertial_nav.get_position_neu_cm();
        tsdt4_land_start_ms = now_ms;
        tsdt4_land_start_z_cm = tsdt4_land_hold_neu_cm.z;
        tsdt4_contact_accel_count = 0U;
        tsdt4_state = TsdT4State::LAND_DESCEND;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "TSDT4: landing descent started");
    }

    const bool landing_active =
        tsdt4_state == TsdT4State::LAND_DESCEND ||
        tsdt4_state == TsdT4State::LAND_ANGLE;
    if (landing_active) {
        const bool airframe_above_land_gate =
            inertial_nav.get_position_neu_cm().z >
            tsdt4_start_neu_cm.z + TSDT4_AIRFRAME_LAND_GATE_Z_CM;
        if (airframe_above_land_gate) {
            force_flying = true;
            tsdt4_force_flying_owned = true;
        } else if (tsdt4_force_flying_owned) {
            // This gate only filters premature tool-contact detection.  It
            // never changes the attitude or vertical command.
            force_flying = false;
            tsdt4_force_flying_owned = false;
        }

        if (ap.land_complete) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            arming.disarm(AP_Arming::Method::LANDED);
            tsdt4_state = TsdT4State::IDLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: landing complete");
            return;
        }
    }

    if (tsdt4_state == TsdT4State::LAND_DESCEND) {
        const uint32_t elapsed_ms = now_ms - tsdt4_land_start_ms;
        const float blend_time_s = TSDT4_LAND_BLEND_MS * 0.001f;
        const float elapsed_s = elapsed_ms * 0.001f;
        const float normalized_time = elapsed_s / blend_time_s;
        const float ramp_distance_s =
            blend_time_s * smoothstep5_integral(normalized_time);
        const float constant_distance_s =
            MAX(elapsed_s - blend_time_s, 0.0f);
        Vector3f descent_target_neu_cm = tsdt4_land_hold_neu_cm;
        descent_target_neu_cm.z += TSDT4_LAND_CLIMB_RATE_CMS *
                                   (ramp_distance_s + constant_distance_s);

        // [Cybernetics Ch.4] Keep native Guided Position ownership of XY
        // during the slow pre-contact descent.  Only the Z destination moves;
        // no XY error or velocity is converted into a custom pitch command.
        if (!mode_guided.set_destination(
                descent_target_neu_cm, true,
                degrees(tsdt4_start_yaw_rad) * 100.0f,
                false, 0.0f, false, false)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: descent position rejected");
            return;
        }

        const bool contact_detection_armed =
            elapsed_ms >= TSDT4_CONTACT_ARM_DELAY_MS &&
            inertial_nav.get_position_neu_cm().z <=
                tsdt4_land_start_z_cm - TSDT4_CONTACT_MIN_DESCENT_CM;
        const float accel_z_mss = ahrs.get_accel_ef().z + GRAVITY_MSS;
        if (contact_detection_armed &&
            fabsf(accel_z_mss) >= TSDT4_CONTACT_ACCEL_Z_MSS) {
            tsdt4_contact_accel_count++;
        } else {
            tsdt4_contact_accel_count = 0U;
        }

        if (tsdt4_contact_accel_count >= TSDT4_CONTACT_CONFIRM_SAMPLES) {
            // [Cybernetics Ch.4] Preserve command continuity across the phase
            // boundary by starting Angle from the native Position controller's
            // outgoing attitude target, not instantaneous tracking error.
            const Vector3f position_attitude_target_rad =
                attitude_control->get_att_target_euler_rad();
            tsdt4_land_initial_roll_rad = position_attitude_target_rad.x;
            tsdt4_land_initial_pitch_rad = position_attitude_target_rad.y;
            tsdt4_land_angle_start_ms = now_ms;
            tsdt4_state = TsdT4State::LAND_ANGLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: landing contact detected");
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: landing angle started");
        } else if (elapsed_ms >= TSDT4_LAND_TIMEOUT_MS) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: contact timeout, LAND fallback");
        }
        return;
    }

    if (tsdt4_state == TsdT4State::LAND_ANGLE) {
        const uint32_t elapsed_ms = now_ms - tsdt4_land_angle_start_ms;
        const uint32_t total_landing_ms = now_ms - tsdt4_land_start_ms;

        const float blend = smoothstep5(
            elapsed_ms / float(TSDT4_LAND_BLEND_MS));
        const float roll_rad =
            tsdt4_land_initial_roll_rad * (1.0f - blend);
        const float pitch_rad =
            tsdt4_land_initial_pitch_rad +
            (TSDT4_LAND_PITCH_RAD - tsdt4_land_initial_pitch_rad) * blend;

        mode_guided.set_angle(
            attitude_target(roll_rad, pitch_rad, tsdt4_start_yaw_rad),
            no_angular_velocity, TSDT4_LAND_CLIMB_RATE_CMS, false);

        if (total_landing_ms >= TSDT4_LAND_TIMEOUT_MS) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            // [Cybernetics Ch.17] This is a failed-run safety fallback only;
            // a passing run never leaves GUIDED before disarm.
            set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: landing timeout, LAND fallback");
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
