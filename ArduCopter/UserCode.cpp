#include "Copter.h"

float aim_pitch_deg_before;
float aim_pitch_deg;
float delta_aim_pitch_deg;
float aim_pitch_deg_last;
float pitch_b;

float tilt_cdeg_R, tilt_cdeg_L;
float tilt2_cdeg_R, tilt2_cdeg_L;

#define MANUAL_TILT

void Copter::trans_speed(uint16_t rc_ch)
{
    static uint8_t state = 0;
    switch (state) {
        case 0:
            if (rc_ch > 1700) {
                state = 1;
            }
            aim_pitch_deg       = 0;
            delta_aim_pitch_deg = 0;
            aim_pitch_deg_last  = 0;
            break;

        case 1:
            aim_pitch_deg += g2.user_parameters.get_TiltSPParam();
            if (aim_pitch_deg > g2.user_parameters.get_MaxDegParam())
                aim_pitch_deg = g2.user_parameters.get_MaxDegParam();
            if (rc_ch < 1600) {
                state = 2;
            }

            break;

        case 2:
            aim_pitch_deg -= g2.user_parameters.get_TiltSPParam();
            if (aim_pitch_deg < 0.5)
                state = 0;

            break;

        default:
            break;
    }

    delta_aim_pitch_deg = (aim_pitch_deg - aim_pitch_deg_last);
    aim_pitch_deg_last  = aim_pitch_deg;
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    tilt_cdeg_R  = g2.user_parameters.get_tiltR_Param() * 100.0f;
    tilt_cdeg_L  = g2.user_parameters.get_tiltL_Param() * 100.0f;
    tilt2_cdeg_R = g2.user_parameters.get_tilt2R_Param() * 100.0f;
    tilt2_cdeg_L = g2.user_parameters.get_tilt2L_Param() * 100.0f;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
const float alpha = 0.02f;
void        Copter::userhook_50Hz()
{
    // put your 50Hz code here
    if ((!copter.failsafe.radio) && rc().has_had_rc_receiver()) {
        uint16_t chin = hal.rcin->read(CH_7);

# ifdef MANUAL_TILT
        if (chin > 1480 && chin < 1520) chin = 1500;
        aim_pitch_deg_before = ((float)chin - 1500) / 500.0f * 90;

        aim_pitch_deg = aim_pitch_deg_before * alpha + (1.0f - alpha) * aim_pitch_deg;

        delta_aim_pitch_deg = (aim_pitch_deg - aim_pitch_deg_last);
        aim_pitch_deg_last  = aim_pitch_deg;
# else
        trans_speed(chin);
# endif

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
    copter.Log_Write_Virtual_Pitch(degrees(ahrs.get_pitch()), pitch_b, aim_pitch_deg);
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

    pitch_b = degrees(ahrs.get_pitch()) + aim_pitch_deg;
    // gcs().send_text(MAV_SEVERITY_NOTICE, "ab=%f, ch=[%d]", pitch_b, hal.rcin->read(CH_8));
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
