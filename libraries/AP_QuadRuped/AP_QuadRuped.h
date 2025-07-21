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

enum GaitType {
    GAIT_DIAGONAL = 0,
    GAIT_WAVE     = 1
};

class AP_QuadRuped {

protected:
    bool move_requested;

    uint8_t gait_type;

    uint8_t gait_step_leg_start[LEG_ALL];
    uint8_t gait_lift_divisor;      //划分抬腿阶段所占的比例    
    uint8_t gait_travel_divisor;    //对角步态划分抬腿阶段所占的比例

    float gait_rot_z[LEG_ALL];

    /* 抬腿的高度 */
    AP_Float leg_lift_height;

    /* 步态计算频率 */
    AP_Float gait_hz;

    /* 步态计算数 */
    AP_Int16 gait_step_total;

    /* 当前步态计数 */
    uint16_t gait_step_now;

    /* 油门最大值 */
    AP_Float throttle_max;

    AP_Float S;

    /* 四足腿末端位置 */
    Vector3f endpoint_leg_pos[LEG_ALL];

    /* 四足腿对应机身点的位置 */
    Vector3f endpoint_leg_frame[LEG_ALL];

    /* 四足腿的角度 */
    Vector3f endpoint_leg_angle[LEG_ALL];
    Vector3f endpoint_leg_angle_last[LEG_ALL];

    /* 机体旋转角度 */
    Vector3f body_rot_xyz_deg;

    /* 四足末端需要走的位置 */
    Vector3f gait_pos_xyz[LEG_ALL];

    AP_QuadRuped_SYS_Params Sys_Param;

    AP_QuadRuped_Params leg_param[LEG_ALL];

    AP_QuadRuped_CHANNEL_Params channel;

    AP_Float zfactor;

    float target_yaw = 0;
    float throttle_travel;
    float z_travel;
    float yaw_travel;
    float roll_travel;
    float pitch_travel;

    float max_yaw_rate; // 最大允许角速度（rad/s）
    bool  first_run = true;

    Vector3f centre_offset; // 主动控制的重心偏移量

    Vector2f offset_xy; // 重心平移控制

    uint32_t start_time;

    AP_AHRS_View*&         _ahrs;
    AP_MotorsMulticopter*& _motors;

    AC_PID diag_yaw_pid {
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

    AC_PID wave_yaw_pid {
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

    float   aim_yaw;
    float   wave_balance_factor = 0.4f; // 波浪步态补偿系数
    uint8_t last_lifted_leg     = 0;    // 记录上一步抬起的腿

    Vector3ui servo_output_cmd[LEG_ALL];

    // 添加重心控制方法
    Vector3f centre_offset_current = Vector3f(0, 0, 0); // 当前平滑偏移值
    Vector3f centre_offset_target  = Vector3f(0, 0, 0); // 目标偏移值
    float    body_offset_x         = 0;
    Vector3f body_centre { 0, 0, 0 };
    Vector3f foot_lock_pos[LEG_ALL];
    void     set_centre_offset(float x, float y, float z);
    Vector2f wave_balance_offset; // 波浪步态专用平衡补偿
    Vector2f support_triangle_center;
    Vector3f support_center { 0, 0, 0 };
    uint8_t  support_count = 0;
    void     update_centre_offset(uint8_t lifting_leg);
    void     set_standby();
    bool     is_transitioning = false;
    float    transition_progress;

    // 线性插值辅助函数
    Vector3f lerp(const Vector3f& a, const Vector3f& b, float t) const
    {
        t = constrain_value(t, 0.0f, 1.0f);
        return a + (b - a) * t;
    }

public:
    AP_QuadRuped(AP_AHRS_View*& ahrs, AP_MotorsMulticopter*& motors);
    ~AP_QuadRuped() { };

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    void gait_select();
    void calc_gait_sequence(void);
    void main_inverse_kinematics();

    void balance_controller();
    void update_centre_offset_wave(uint8_t leg_index);

    Vector3f body_forward_kinematics(uint8_t leg_index);
    Vector3f leg_inverse_kinematics(Vector3f posxyz);

    void set_throttle_travel(int16_t _throttle) { throttle_travel = _throttle; }
    void set_yaw_travel(int16_t _yaw_travel) { yaw_travel = _yaw_travel; }
    // Vector3f get_endpoint_leg(uint8_t leg) { return endpoint_leg[leg]; }

    void output_leg_angle();
    bool servo_estimate();

    void reset_leg(void);
    void right_sleep_leg(void);
    void update_leg();

    void controller(void);

    void trajectory_generation(uint8_t leg_index);
    void yaw_trajectory_generation(uint8_t leg_index);

    void roll_test();
    void pitch_test();

    float getFreq() { return gait_hz; }
    bool  hw_set_servo_cmd();
};
