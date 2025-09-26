#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_QuadRuped_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_QuadRuped_Params(void);

    AP_Float COXA_OFS;
    AP_Float FEMU_OFS;
    AP_Float TIBI_OFS;

    AP_Int8 COXA_DIR;
    AP_Int8 FEMU_DIR;
    AP_Int8 TIBI_DIR;
};

class AP_QuadRuped_CHANNEL_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_QuadRuped_CHANNEL_Params(void);

    AP_Int8 throttle_x_channel;
    AP_Int8 throttle_y_channel;
    AP_Int8 roll_channel;
    AP_Int8 pitch_channel;
    AP_Int8 yaw_channel;
    AP_Int8 centre_offset_x_channel;
    AP_Int8 centre_offset_y_channel;

    AP_Float throttle_x_max;
    AP_Float throttle_y_max;

    AP_Float body_height;
    AP_Float left_lift;

    AP_Int8 mode_channel;
    AP_Int8 walk_mode_channel; // 陆地模式通道
    AP_Int8 fly_mode_channel;  // 飞行模式通道
    AP_Int8 claw_channel;
};

class AP_QuadRuped_SYS_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_QuadRuped_SYS_Params(void);

    /* COXA的长度 */
    AP_Float COXA_LEN;

    /* FEMUR的长度 */
    AP_Float FEMUR_LEN;

    /* TIBIA的长度 */
    AP_Float TIBIA_LEN;

    /* 机身的X长度 */
    AP_Float FRAME_LEN;

    /* 机身的Y长度 */
    AP_Float FRAME_WIDTH;

    /* 附加参数 */
    AP_Float Alpha_A;
    AP_Float Alpha_B;
};
