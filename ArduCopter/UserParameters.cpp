#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    AP_GROUPINFO("_MaxDeg", 3, UserParameters, _maxDeg, 0),
    AP_GROUPINFO("_TILSP", 4, UserParameters, _TILSP, 0),

    AP_GROUPINFO("_tilt2R", 5, UserParameters, _tilt2R, 0),
    AP_GROUPINFO("_tilt2L", 6, UserParameters, _tilt2L, 0),
    AP_GROUPINFO("_tiltR", 7, UserParameters, _tiltR, 0),
    AP_GROUPINFO("_tiltL", 8, UserParameters, _tiltL, 0),

    AP_GROUPINFO("_yawft", 9, UserParameters, _yaw_fact, 0.25f),

    AP_GROUPINFO("_pitoff", 10, UserParameters, _pitch_offset, 0),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
