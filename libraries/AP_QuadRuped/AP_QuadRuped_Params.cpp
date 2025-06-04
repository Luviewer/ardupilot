#include "AP_QuadRuped_Params.h"

const AP_Param::GroupInfo AP_QuadRuped_Params::var_info[] = {
    AP_GROUPINFO("C_DIR", 1, AP_QuadRuped_Params, _COXA_DIR, 0),
    AP_GROUPINFO("F_DIR", 2, AP_QuadRuped_Params, _FEMU_DIR, 0),
    AP_GROUPINFO("T_DIR", 3, AP_QuadRuped_Params, _TIBI_DIR, 0),

    AP_GROUPINFO("C_OFS", 4, AP_QuadRuped_Params, _COXA_OFS, 0),
    AP_GROUPINFO("F_OFS", 5, AP_QuadRuped_Params, _FEMU_OFS, 0),
    AP_GROUPINFO("T_OFS", 6, AP_QuadRuped_Params, _TIBI_OFS, 0),

    AP_GROUPEND
};

AP_QuadRuped_Params::AP_QuadRuped_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::GroupInfo AP_QuadRuped_CHANNEL_Params::var_info[] = {
    AP_GROUPINFO("THR", 1, AP_QuadRuped_CHANNEL_Params, throttle_channel, -1),
    AP_GROUPINFO("HGH", 2, AP_QuadRuped_CHANNEL_Params, height_channel, -1),
    AP_GROUPINFO("GAT", 3, AP_QuadRuped_CHANNEL_Params, gait_channel, -1),

    AP_GROUPINFO("ROL", 4, AP_QuadRuped_CHANNEL_Params, roll_channel, -1),
    AP_GROUPINFO("PIT", 5, AP_QuadRuped_CHANNEL_Params, pitch_channel, -1),
    AP_GROUPINFO("YAW", 6, AP_QuadRuped_CHANNEL_Params, yaw_channel, -1),

    AP_GROUPEND
};

AP_QuadRuped_CHANNEL_Params::AP_QuadRuped_CHANNEL_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
