#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8  get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }

    AP_Float get_MaxDegParam() const { return _maxDeg; }
    AP_Float get_TiltSPParam() const { return _TILSP; }

    AP_Float get_tilt2R_Param() const { return _tilt2R; }
    AP_Float get_tilt2L_Param() const { return _tilt2L; }

    AP_Float get_tiltR_Param() const { return _tiltR; }
    AP_Float get_tiltL_Param() const { return _tiltL; }

private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;

    AP_Float _maxDeg;
    AP_Float _TILSP;

    AP_Float _tilt2R;
    AP_Float _tilt2L;
    AP_Float _tiltR;
    AP_Float _tiltL;
};
