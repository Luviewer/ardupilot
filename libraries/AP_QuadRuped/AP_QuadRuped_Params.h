#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_QuadRuped_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_QuadRuped_Params(void);

    AP_Float _COXA_OFS;
    AP_Float _FEMU_OFS;
    AP_Float _TIBI_OFS;

    AP_Int8 _COXA_DIR;
    AP_Int8 _FEMU_DIR;
    AP_Int8 _TIBI_DIR;
};

class AP_QuadRuped_CHANNEL_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_QuadRuped_CHANNEL_Params(void);

    AP_Int8 throttle_channel;
    AP_Int8 height_channel;
    AP_Int8 gait_channel;
    AP_Int8 roll_channel;
    AP_Int8 pitch_channel;
    AP_Int8 yaw_channel;
    AP_Int8 com_offset_x_channel;
    AP_Int8 com_offset_y_channel;
};
