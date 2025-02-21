#include "MyUSR.h"
#include "config.h"

MYUSR::MYUSR()
{
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::GroupInfo MYUSR::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_MaxDeg", 0, MYUSR, _maxDeg, 0),
    AP_GROUPINFO("_TILSP", 1, MYUSR, _TILSP, 0),

    AP_GROUPINFO("_tilt2R", 2, MYUSR, _tilt2R, 0),
    AP_GROUPINFO("_tilt2L", 3, MYUSR, _tilt2L, 0),
    AP_GROUPINFO("_tiltR", 4, MYUSR, _tiltR, 0),
    AP_GROUPINFO("_tiltL", 5, MYUSR, _tiltL, 0),

    AP_GROUPINFO("_yawft", 6, MYUSR, _yaw_fact, 0.25f),

    AP_GROUPINFO("_pitoff", 7, MYUSR, _pitch_offset, 0),

    AP_GROUPEND
};
