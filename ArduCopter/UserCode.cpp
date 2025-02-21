#include "Copter.h"

const float alpha = 0.02f;

bool fly_mode_rc;

float aim_pitch_deg_before;
float aim_pitch_deg;
float delta_aim_pitch_deg;
float aim_pitch_deg_last;
float pitch_b;

float tilt_cdeg_R, tilt_cdeg_L;
float tilt2_cdeg_R, tilt2_cdeg_L;

float yaw_factor_f;

float tilt_MaxDeg;

float pitch_offset;

#define MANUAL_TILT

uint16_t chin = 1500;

void Copter::trans_speed(uint16_t& rc_ch)
{
    static uint8_t state = 0;

    static uint32_t gettime;

    hal.console->printf("rc_ch=%d\r\n", rc_ch);

    switch (state) {
        case 0:
            rc_ch++;

            if (rc_ch >= 2000) {
                rc_ch   = 2000;
                state   = 1;
                gettime = millis();
            }
            break;

        case 1:
            if ((millis() - gettime) > 5000) {
                state = 2;
            }
            break;

        case 2:
            rc_ch--;
            if (rc_ch <= 1000) {
                rc_ch   = 1000;
                state   = 3;
                gettime = millis();
            }
            break;

        case 3:
            if ((millis() - gettime) > 5000) {
                state = 4;
            }
            break;

        case 4:
            rc_ch++;

            if (rc_ch >= 1500) {
                rc_ch = 1500;
            }
            break;

        default:
            break;
    }
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
    if ((!copter.failsafe.radio) && rc().has_had_rc_receiver()) {
        aim_pitch_deg_before = ((float)chin - 1500) / 500.0f * tilt_MaxDeg;

        aim_pitch_deg = aim_pitch_deg_before * alpha + (1.0f - alpha) * aim_pitch_deg;

        delta_aim_pitch_deg = (aim_pitch_deg - aim_pitch_deg_last);
        aim_pitch_deg_last  = aim_pitch_deg;

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

# ifdef MANUAL_TILT
    if (hal.rcin->read(CH_7) > 1480 && hal.rcin->read(CH_7) < 1520)
        chin = 1500;
    else if (hal.rcin->read(CH_7) < 2000 && hal.rcin->read(CH_7) > 1000) {
        chin = hal.rcin->read(CH_7);
    }

# else
    if (hal.rcin->read(CH_7) > 1600)
        trans_speed(chin);
# endif
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

    if (hal.rcin->read(CH_8) > 1700)
        fly_mode_rc = true;
    else
        fly_mode_rc = false;

    tilt_cdeg_R  = myusr.get_tiltR_Param() * 100.0f;
    tilt_cdeg_L  = myusr.get_tiltL_Param() * 100.0f;
    tilt2_cdeg_R = myusr.get_tilt2R_Param() * 100.0f;
    tilt2_cdeg_L = myusr.get_tilt2L_Param() * 100.0f;

    yaw_factor_f = myusr.get_yaw_fact_Param();

    tilt_MaxDeg = myusr.get_MaxDegParam();

    pitch_offset = myusr.get_pitch_offset_Param();

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
