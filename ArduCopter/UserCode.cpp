#include "Copter.h"

float aim_pitch_deg;
float delta_aim_pitch_deg;
float aim_pitch_deg_last;

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
    if ((!copter.failsafe.radio) && rc().has_had_rc_receiver()) {
        uint16_t chin = hal.rcin->read(CH_7);
        if (chin > 1450 && chin < 1550) chin = 1500;
        aim_pitch_deg = ((float)chin - 1500) / 500.0f * g2.user_parameters.get_MaxDegParam();

        delta_aim_pitch_deg = (aim_pitch_deg - aim_pitch_deg_last);
        aim_pitch_deg_last  = aim_pitch_deg;

        hiwonder_l.set_position(SERVO_4, int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);
        hiwonder_r.set_position(SERVO_2, -int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);
    } else {
        delta_aim_pitch_deg = 0;
        aim_pitch_deg_last  = 0;
        aim_pitch_deg       = 0;
    }
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
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
