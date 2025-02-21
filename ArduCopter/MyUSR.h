#pragma once

#include <AP_Param/AP_Param.h>

class MYUSR {

public:
    MYUSR();
    static const struct AP_Param::GroupInfo var_info[];

    AP_Float get_MaxDegParam() const { return _maxDeg; }
    AP_Float get_TiltSPParam() const { return _TILSP; }

    AP_Float get_tilt2R_Param() const { return _tilt2R; }
    AP_Float get_tilt2L_Param() const { return _tilt2L; }

    AP_Float get_tiltR_Param() const { return _tiltR; }
    AP_Float get_tiltL_Param() const { return _tiltL; }

    AP_Float get_yaw_fact_Param() const { return _yaw_fact; }

    AP_Float get_pitch_offset_Param() const { return _pitch_offset; }

private:
    AP_Float _maxDeg;
    AP_Float _TILSP;

    AP_Float _tilt2R;
    AP_Float _tilt2L;
    AP_Float _tiltR;
    AP_Float _tiltL;

    AP_Float _yaw_fact;

    AP_Float _pitch_offset;
};