#include "Copter.h"

float aim_pitch_deg;
float delta_aim_pitch_deg;
float aim_pitch_deg_last;
float pitch_b;

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

        SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorLeft, aim_pitch_deg / 150.0f * 4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorRight, -aim_pitch_deg / 150.0f * 4500);

        // hiwonder_l.set_position(SERVO_4, int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);
        // hiwonder_r.set_position(SERVO_2, -int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);

    } else {
        delta_aim_pitch_deg = 0;
        aim_pitch_deg_last  = 0;
        aim_pitch_deg       = 0;
    }

    // const float speed_angle = 1.5f;
    // float       max_Deg     = g2.user_parameters.get_MaxDegParam();

    // if ((!copter.failsafe.radio) && rc().has_had_rc_receiver()) {
    //     uint16_t chin = hal.rcin->read(CH_8);
    //     if (chin > 1400 && chin < 1600) {
    //         if (aim_pitch_deg < -1) {
    //             aim_pitch_deg = aim_pitch_deg + speed_angle / 50.0f; //
    //         } else if (aim_pitch_deg > 1) {
    //             aim_pitch_deg = aim_pitch_deg - speed_angle / 50.0f; //
    //         }
    //     } else if (chin > 1800 && chin < 2000) {
    //         if (aim_pitch_deg < max_Deg) {
    //             aim_pitch_deg = aim_pitch_deg + speed_angle / 50.0f;
    //         } else {
    //             aim_pitch_deg = max_Deg;
    //         }
    //     } else if (chin > 1000 && chin < 1200) {
    //         if (aim_pitch_deg > -max_Deg)
    //             aim_pitch_deg = aim_pitch_deg - speed_angle / 50.0f; //
    //         else
    //             aim_pitch_deg = -max_Deg;
    //     } else {
    //     }

    //     delta_aim_pitch_deg = (aim_pitch_deg - aim_pitch_deg_last);
    //     aim_pitch_deg_last  = aim_pitch_deg;

    //     hiwonder_l.set_position(SERVO_4, int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);
    //     hiwonder_r.set_position(SERVO_2, -int(aim_pitch_deg / 120.0f * 500.0f) + 1500, 0);

    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorLeft, aim_pitch_deg);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorRight, aim_pitch_deg);
    // } else {
    //     delta_aim_pitch_deg = 0;
    //     aim_pitch_deg_last  = 0;
    //     aim_pitch_deg       = 0;
    // }
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
