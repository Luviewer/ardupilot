#pragma once

#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Param/AP_Param.h>
#include <stdio.h>

enum {
    Leg_RF = 0,
    Leg_RB,
    Leg_LB,
    Leg_LF,

    LEG_ALL,
};

class AP_QuadRuped_Base {
public:
    AP_QuadRuped_Base(AP_AHRS_View& ahrs, AP_Motors& motors);

    // Empty destructor to suppress compiler warning
    virtual ~AP_QuadRuped_Base() { }

    enum GaitType {
        GAIT_DIAGONAL = 0,
        GAIT_WAVE     = 1
    };

    AP_AHRS_View& _ahrs;

    AP_Motors& _motors;

    static const struct AP_Param::GroupInfo var_info[];

    virtual void gait_init() = 0;
    virtual void calc_gait_sequence();

    virtual void trajectory_generation(uint8_t leg_index) { };
    virtual void yaw_trajectory_generation(uint8_t leg_index) { };

    virtual void     main_inverse_kinematics();
    virtual Vector3f body_forward_kinematics(uint8_t leg_index);
    virtual Vector3f leg_inverse_kinematics(Vector3f posxyz);

    virtual void set_centre_offset(float x, float y, float z);

    virtual void reset_leg();
    virtual void right_sleep_leg();
    virtual void update_leg() { };

    virtual void controller();
    virtual void balance_controller();

    virtual void update() { };

    void init();

    void output_leg_angle();

    bool hw_set_servo_cmd();

    float getFreq() { return gait_hz; }

protected:
    bool move_requested;

    uint8_t gait_step_leg_start[LEG_ALL];
    uint8_t gait_lift_divisor;
    uint8_t gait_travel_divisor;

    Vector3ui servo_output_cmd[LEG_ALL];

    /* 抬腿的高度 */
    AP_Float leg_lift_height;

    /* 步态计算频率 */
    AP_Float gait_hz;

    /* 步态计算数 */
    AP_Int16 gait_step_total;

    /* 当前步态计数 */
    uint16_t gait_step_now;

    /* 四足腿末端位置 */
    Vector3f endpoint_leg_pos[LEG_ALL];

    /* 四足腿对应机身点的位置 */
    Vector3f endpoint_leg_frame[LEG_ALL];

    /* 四足腿的角度 */
    Vector3f endpoint_leg_angle[LEG_ALL];
    Vector3f endpoint_leg_angle_last[LEG_ALL];

    /* 四足末端需要走的位置 */
    Vector3f gait_pos_xyz[LEG_ALL];

    /* 机体旋转角度 */
    Vector3f body_rot_xyz_deg;

    float gait_rot_z[LEG_ALL];

    float target_yaw;
    float throttle_travel;
    float z_travel;
    float yaw_travel;
    float roll_travel;
    float pitch_travel;

    float max_yaw_rate; // 最大允许角速度（rad/s）
    bool  first_run = true;

    Vector3f centre_offset; // 主动控制的重心偏移量

    Vector2f offset_xy; // 重心平移控制

    /* 油门最大值 */
    AP_Float throttle_max;

    AP_QuadRuped_SYS_Params Sys_Param;

    AP_QuadRuped_Params leg_param[LEG_ALL];

    AP_QuadRuped_CHANNEL_Params channel;

    AC_PID roll_pid {
        AC_PID::Defaults {
            .p         = 1.0f,
            .i         = 0.02f,
            .d         = 0.05f,
            .imax      = 1,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID pitch_pid {
        AC_PID::Defaults {
            .p         = 1.0f,
            .i         = 0.02f,
            .d         = 0.1f,
            .imax      = 1,
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
};
