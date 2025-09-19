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

    AP_Int8  throttle_x_channel;
    AP_Int8  throttle_y_channel;
    AP_Int16 height_channel;
    AP_Int16 lift_channel;
    AP_Int8  roll_channel;
    AP_Int8  pitch_channel;
    AP_Int8  yaw_channel;
    AP_Int8  centre_offset_x_channel;
    AP_Int8  centre_offset_y_channel;
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
};
