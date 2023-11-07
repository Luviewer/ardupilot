#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include <AC_PID/AC_PI.h>
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <AP_BalanceCAN/AP_BalanceCAN.h>

// default rate controller PID gains
#define AC_BALANCE_MOTOR_P              0.5
#define AC_BALANCE_MOTOR_I              0.0f
#define AC_BALANCE_MOTOR_D              0.1f
#define AC_BALANCE_MOTOR_IMAX           1.0f
#define AC_BALANCE_MOTOR_TARGET_FILT_HZ 0.135f
#define AC_BALANCE_MOTOR_ERROR_FILT_HZ  2.5f

#define AC_BALANCE_ANGLE_P              0.75
#define AC_BALANCE_ANGLE_I              0.0f
#define AC_BALANCE_ANGLE_D              0.15f
#define AC_BALANCE_ANGLE_IMAX           1.0f
#define AC_BALANCE_ANGLE_TARGET_FILT_HZ 0.135f
#define AC_BALANCE_ANGLE_ERROR_FILT_HZ  2.5f

#define AC_BALANCE_SPEED_P              -0.25f
#define AC_BALANCE_SPEED_I              -0.03f
#define AC_BALANCE_SPEED_D              0.0f
#define AC_BALANCE_SPEED_IMAX           10.0f
#define AC_BALANCE_SPEED_TARGET_FILT_HZ 0.135f
#define AC_BALANCE_SPEED_ERROR_FILT_HZ  2.5f

#define AC_BALANCE_TURN_P               0.3f
#define AC_BALANCE_TURN_I               0.0f
#define AC_BALANCE_TURN_D               0.1f
#define AC_BALANCE_TURN_IMAX            1.0f
#define AC_BALANCE_TURN_TARGET_FILT_HZ  0.135f
#define AC_BALANCE_TURN_ERROR_FILT_HZ   2.5f

#define AC_BALANCE_ZERO_ANGLE           0.0f

#define AC_BALANCE_MAX_SPEED            10000.0f

#define AC_BALANCE_TARGET_X_SPEED       1.0f
#define AC_BALANCE_TARGET_Z_SPEED       1.0f

#define AC_BALANCE_ROLL_P               0.5f
#define AC_BALANCE_ROLL_I               0.2
#define AC_BALANCE_ROLL_D               0.01f
#define AC_BALANCE_ROLL_IMAX            1.0f
#define AC_BALANCE_ROLL_TARGET_FILT_HZ  0.135f
#define AC_BALANCE_ROLL_ERROR_FILT_HZ   2.5f

class AC_BalanceControl {
public:
    AC_BalanceControl(AP_Motors* motors, AP_AHRS_View* ahrs);

    // empty destructor to suppress compiler warning
    virtual ~AC_BalanceControl() { }

    float Balance(float Angle, float Gyro);
    float Velocity(float encoder_left, float encoder_right);
    float Turn(float yaw, float gyro);

    void RollControl(float roll);

    void update(void);

    void setAltOK(bool sta) {alt_ok = sta;}

    void setAltData(float data) {alt_cm = data; }

    uint8_t get_Balance_Mode() { return balanceMode; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Motors*          _motors;
    const AP_AHRS_View* _ahrs;

    enum moveFlag {
        none      = 0,
        moveFront = 1,
        moveBack  = 2,
        moveRight = 3,
        moveLeft  = 4,
    };

    enum BalanceMode {
        ground                 = 0,
        flying_with_balance    = 1,
        flying_without_balance = 2,
        landing_check          = 3,
    };


protected:

    ///////////////////////////////////////////////////////
    // PID参数
    AC_PID _pid_angle;

    AC_PID _pid_speed;

    AC_PID _pid_turn;

    AC_PID _pid_roll;

    ///////////////////////////////////////////////////////

    AP_Float _zero_angle; // 机械零值

    AP_Float _max_speed;

    AP_Float Target_Velocity_X;
    AP_Float Target_Velocity_Z;

    ///////////////////////////////////////////////////////
    // 速度环参数

    LowPassFilterFloat speed_low_pass_filter; // 一阶低通滤波器

    ///////////////////////////////////////////////////////
    // 转向环参数
    float Turn_Target;
    float Turn_Kp;
    float Turn_Kd;

    uint8_t _moveflag_x;
    uint8_t _moveflag_z;

    float control_balance, control_velocity, control_turn;

    float _dt;

    enum BalanceMode balanceMode;

    bool alt_ok;
    float alt_cm;
};
