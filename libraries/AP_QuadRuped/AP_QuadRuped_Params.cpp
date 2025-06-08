#include "AP_QuadRuped_Params.h"

#define COXA_LEN_DEFAULT    47.1f
#define FEMUR_LEN_DEFAULT   133.0f
#define TIBIA_LEN_DEFAULT   144.1f
#define FRAME_LEN_DEFAULT   185.0f
#define FRAME_WIDTH_DEFAULT 185.0f

const AP_Param::GroupInfo AP_QuadRuped_Params::var_info[] = {
    AP_GROUPINFO("C_DIR", 1, AP_QuadRuped_Params, COXA_DIR, 1),
    AP_GROUPINFO("F_DIR", 2, AP_QuadRuped_Params, FEMU_DIR, 1),
    AP_GROUPINFO("T_DIR", 3, AP_QuadRuped_Params, TIBI_DIR, 1),

    AP_GROUPINFO("C_OFS", 4, AP_QuadRuped_Params, COXA_OFS, 0),
    AP_GROUPINFO("F_OFS", 5, AP_QuadRuped_Params, FEMU_OFS, 0),
    AP_GROUPINFO("T_OFS", 6, AP_QuadRuped_Params, TIBI_OFS, 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_QuadRuped_CHANNEL_Params::var_info[] = {
    AP_GROUPINFO("THR", 1, AP_QuadRuped_CHANNEL_Params, throttle_channel, -1),
    AP_GROUPINFO("HGH", 2, AP_QuadRuped_CHANNEL_Params, height_channel, -1),
    AP_GROUPINFO("GAT", 3, AP_QuadRuped_CHANNEL_Params, gait_channel, -1),

    AP_GROUPINFO("ROL", 4, AP_QuadRuped_CHANNEL_Params, roll_channel, -1),
    AP_GROUPINFO("PIT", 5, AP_QuadRuped_CHANNEL_Params, pitch_channel, -1),
    AP_GROUPINFO("YAW", 6, AP_QuadRuped_CHANNEL_Params, yaw_channel, -1),
    AP_GROUPINFO("CEN_X", 7, AP_QuadRuped_CHANNEL_Params, centre_offset_x_channel, -1),
    AP_GROUPINFO("CEN_Y", 8, AP_QuadRuped_CHANNEL_Params, centre_offset_y_channel, -1),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_QuadRuped_SYS_Params::var_info[] = {
    AP_GROUPINFO("_COXA", 1, AP_QuadRuped_SYS_Params, COXA_LEN, COXA_LEN_DEFAULT),
    AP_GROUPINFO("_FEMUR", 2, AP_QuadRuped_SYS_Params, FEMUR_LEN, FEMUR_LEN_DEFAULT),
    AP_GROUPINFO("_TIBIA", 3, AP_QuadRuped_SYS_Params, TIBIA_LEN, TIBIA_LEN_DEFAULT),
    AP_GROUPINFO("_FLEN", 4, AP_QuadRuped_SYS_Params, FRAME_LEN, FRAME_LEN_DEFAULT),
    AP_GROUPINFO("_FWID", 5, AP_QuadRuped_SYS_Params, FRAME_WIDTH, FRAME_WIDTH_DEFAULT),

    AP_GROUPEND
};

AP_QuadRuped_Params::AP_QuadRuped_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_QuadRuped_CHANNEL_Params::AP_QuadRuped_CHANNEL_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_QuadRuped_SYS_Params::AP_QuadRuped_SYS_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
