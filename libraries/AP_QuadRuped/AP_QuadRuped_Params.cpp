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
    AP_GROUPINFO("THR_X", 1, AP_QuadRuped_CHANNEL_Params, throttle_x_channel, -1),
    AP_GROUPINFO("THR_Y", 2, AP_QuadRuped_CHANNEL_Params, throttle_y_channel, -1),
    AP_GROUPINFO("HGH", 3, AP_QuadRuped_CHANNEL_Params, body_height, -1),
    AP_GROUPINFO("LIFT", 4, AP_QuadRuped_CHANNEL_Params, left_lift, -1),

    AP_GROUPINFO("ROL", 5, AP_QuadRuped_CHANNEL_Params, roll_channel, -1),
    AP_GROUPINFO("PIT", 6, AP_QuadRuped_CHANNEL_Params, pitch_channel, -1),
    AP_GROUPINFO("YAW", 7, AP_QuadRuped_CHANNEL_Params, yaw_channel, -1),

    AP_GROUPINFO("CEN_X", 8, AP_QuadRuped_CHANNEL_Params, centre_offset_x_channel, -1),
    AP_GROUPINFO("CEN_Y", 9, AP_QuadRuped_CHANNEL_Params, centre_offset_y_channel, -1),

    AP_GROUPINFO("THR_XMX", 10, AP_QuadRuped_CHANNEL_Params, throttle_x_max, 100),
    AP_GROUPINFO("THR_YMX", 11, AP_QuadRuped_CHANNEL_Params, throttle_y_max, 100),

    AP_GROUPINFO("MODE", 12, AP_QuadRuped_CHANNEL_Params, mode_channel, -1),
    AP_GROUPINFO("FLY", 13, AP_QuadRuped_CHANNEL_Params, fly_mode_channel, -1),
    AP_GROUPINFO("WAK", 14, AP_QuadRuped_CHANNEL_Params, walk_mode_channel, -1),
    AP_GROUPINFO("CLAW", 15, AP_QuadRuped_CHANNEL_Params, claw_channel, -1),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_QuadRuped_SYS_Params::var_info[] = {
    AP_GROUPINFO("COXA", 1, AP_QuadRuped_SYS_Params, COXA_LEN, COXA_LEN_DEFAULT),
    AP_GROUPINFO("FEMUR", 2, AP_QuadRuped_SYS_Params, FEMUR_LEN, FEMUR_LEN_DEFAULT),
    AP_GROUPINFO("TIBIA", 3, AP_QuadRuped_SYS_Params, TIBIA_LEN, TIBIA_LEN_DEFAULT),
    AP_GROUPINFO("FLEN", 4, AP_QuadRuped_SYS_Params, FRAME_LEN, FRAME_LEN_DEFAULT),
    AP_GROUPINFO("FWID", 5, AP_QuadRuped_SYS_Params, FRAME_WIDTH, FRAME_WIDTH_DEFAULT),

    AP_GROUPINFO("ALPH_A", 6, AP_QuadRuped_SYS_Params, Alpha_A, 55),
    AP_GROUPINFO("ALPH_B", 7, AP_QuadRuped_SYS_Params, Alpha_B, 50),

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
