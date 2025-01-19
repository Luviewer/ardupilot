#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Param/AP_Param.h>

enum {
    Leg_RF = 0,
    Leg_RB,
    Leg_LB,
    Leg_LF,

    LEG_ALL,
};

class AP_QuadRuped {

protected:
    bool move_requested;

    uint8_t gait_type;

    uint8_t gait_step_leg_start[LEG_ALL];
    uint8_t gait_lifted_steps;
    uint8_t gait_down_steps;
    uint8_t gait_lift_divisor;
    uint8_t gait_half_lift_height;
    uint8_t gait_travel_divisor;

    float gait_rot_z[LEG_ALL];

    /* 抬腿的高度 */
    AP_Float leg_lift_height;

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

    /* 步态计算频率 */
    AP_Float gait_hz;

    /* 步态计算数 */
    AP_Int16 gait_step_total;

    /* 当前步态计数 */
    uint16_t gait_step_now;

    /* 油门最大值 */
    AP_Float throttle_max;

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

    AP_Int8 leg_coxa_direction[LEG_ALL];
    AP_Int8 leg_femur_direction[LEG_ALL];
    AP_Int8 leg_tibia_direction[LEG_ALL];

    float throttle_travel;
    float z_travel;
    float roll_travel;
    float pitch_travel;
    float yaw_travel;

    uint32_t start_time;

    AP_AHRS_View*&         _ahrs;
    AP_MotorsMulticopter*& _motors;

    AC_PID yaw_pid {
        AC_PID::Defaults {
            .p         = 0.35f,
            .i         = 0.35f,
            .d         = 0.001f,
            .ff        = 0.0f,
            .imax      = 1,
            .filt_T_hz = 5.0f,

            .filt_E_hz = 5.0f,
            .filt_D_hz = 5.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    float aim_yaw;

    // Parameter block

public:
    AP_QuadRuped(AP_AHRS_View*& ahrs, AP_MotorsMulticopter*& motors);
    ~AP_QuadRuped() { };

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    void     gait_select();
    void     calc_gait_sequence(void);
    void     main_inverse_kinematics();
    Vector3f body_forward_kinematics(uint8_t leg_index);
    Vector3f leg_inverse_kinematics(Vector3f posxyz);

    void set_throttle_travel(int16_t _throttle) { throttle_travel = _throttle; }
    void set_yaw_travel(int16_t _yaw_travel) { yaw_travel = _yaw_travel; }
    // Vector3f get_endpoint_leg(uint8_t leg) { return endpoint_leg[leg]; }

    void output_leg_angle();
    bool servo_estimate();

    void reset_leg(void);
    void left_sleep_leg(void);
    void update_leg();

    void contoller(void);

    Vector3f trajectory_generation(uint8_t leg_index);
    void     yaw_trajectory_generation(uint8_t leg_index);
};
