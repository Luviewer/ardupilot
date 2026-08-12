#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_HexRuped_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_HexRuped_Params(void);

    AP_Float COXA_OFS;
    AP_Float FEMU_OFS;
    AP_Float TIBI_OFS;

    AP_Int8 COXA_DIR;
    AP_Int8 FEMU_DIR;
    AP_Int8 TIBI_DIR;
};

class AP_HexRuped_CHANNEL_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_HexRuped_CHANNEL_Params(void);

    AP_Int8 throttle_x_channel;
    AP_Int8 throttle_y_channel;
    AP_Int8 roll_channel;
    AP_Int8 pitch_channel;
    AP_Int8 yaw_channel;
    AP_Int8 centre_offset_x_channel;
    AP_Int8 centre_offset_y_channel;

    AP_Float throttle_x_max;
    AP_Float throttle_y_max;

    AP_Float throttle_roll_max;
    AP_Float throttle_pitch_max;
    AP_Float throttle_yaw_max;

    AP_Float body_height;
    AP_Float left_lift;

    AP_Int8 mode_channel;
    AP_Int8 walk_mode_channel; // 陆地模式通道
    AP_Int8 fly_mode_channel;  // 飞行模式通道
    AP_Int8 claw_channel;
};

class AP_HexRuped_SYS_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_HexRuped_SYS_Params(void);

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

    // 右侧前、中、后腿的安装偏航角；左侧由镜像关系计算。
    AP_Float FRONT_YAW;
    AP_Float MIDDLE_YAW;
    AP_Float REAR_YAW;

    // 中腿髋关节相对机体中心的X坐标，左右中腿共用。
    AP_Float MIDDLE_X;
};

class AP_HexRuped_CTRL_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AC_PID roll_pid {
        AC_PID::Defaults {
            .p         = 1.5f,
            .i         = 0.5f,
            .d         = 0.000f,
            .imax      = 10,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID pitch_pid {
        AC_PID::Defaults {
            .p         = 1.5f,
            .i         = 0.5f,
            .d         = 0.000f,
            .imax      = 100,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID yaw_pid {
        AC_PID::Defaults {
            .p         = 0.5f,
            .i         = 0.01f,
            .d         = 0.05f,
            .imax      = 1,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID pos_y_pid {
        AC_PID::Defaults {
            .p         = 1.0f,
            .i         = 0.1f,
            .d         = 0.0f,
            .imax      = 50.0f,
            .filt_T_hz = 2.0f,
            .filt_E_hz = 5.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };
};
