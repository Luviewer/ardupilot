#include "Copter.h"

namespace {

// TSDT4 one-key flight controls.  Handle rising edges so a held switch
// cannot repeatedly re-enter a mode or trigger a second takeoff after landing.
constexpr uint16_t TSDT4_SWITCH_HIGH = 1800;
bool tsdt4_takeoff_switch_was_high;
bool tsdt4_land_switch_was_high;

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
    const bool takeoff_switch_high = rc().get_radio_in(CH_13) > TSDT4_SWITCH_HIGH;
    const bool land_switch_high = rc().get_radio_in(CH_14) > TSDT4_SWITCH_HIGH;
    const bool takeoff_requested = takeoff_switch_high && !tsdt4_takeoff_switch_was_high;
    const bool land_requested = land_switch_high && !tsdt4_land_switch_was_high;

    tsdt4_takeoff_switch_was_high = takeoff_switch_high;
    tsdt4_land_switch_was_high = land_switch_high;

    // Landing has priority if both switches are raised together.
    if (land_requested) {
        if (!motors->armed() || ap.land_complete) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4 LAND ignored: vehicle is not airborne");
        } else if (!set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "TSDT4 LAND rejected");
        }
        return;
    }

    if (!takeoff_requested) {
        return;
    }
    if (!position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4 TAKEOFF rejected: position unavailable");
        return;
    }
    if (!ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4 TAKEOFF ignored: vehicle is airborne");
        return;
    }
    if (!set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4 TAKEOFF rejected: Loiter unavailable");
        return;
    }
    if (!motors->armed() && !arming.arm(AP_Arming::Method::MAVLINK)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4 TAKEOFF rejected: arming failed");
        return;
    }
    if (!flightmode->do_user_takeoff(g.pilot_takeoff_alt, true)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4 TAKEOFF rejected by flight controller");
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
