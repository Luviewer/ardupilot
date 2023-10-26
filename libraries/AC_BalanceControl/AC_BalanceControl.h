#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include <AC_PID/AC_PI.h>
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <AP_RMUART/AP_RMUART.h>

// default rate controller PID gains

#define AC_BALANCE_BALANCE_P           0.5
#define AC_BALANCE_BALANCE_D           0.1f

#define AC_BALANCE_VELOCITY_P          0.5f
#define AC_BALANCE_VELOCITY_I          0.05f
#define AC_BALANCE_VELOCITY_IMAX       1.0f
#define AC_BALANCE_VELOCITY_D          0.0f

#define AC_BALANCE_TURN_P              0.3f
#define AC_BALANCE_TURN_D              0.1f

#define AC_BALANCE_ZERO_ANGLE          0.0f

#define AC_BALANCE_MAX_SPEED           10000.0f

#define AC_BALANCE_TARGET_X_SPEED      1.0f
#define AC_BALANCE_TARGET_Z_SPEED      1.0f

#define AC_BALANCE_ROLL_P              0.5f
#define AC_BALANCE_ROLL_I              0.2
#define AC_BALANCE_ROLL_D              0.01f
#define AC_BALANCE_ROLL_IMAX           1.0f
#define AC_BALANCE_ROLL_TARGET_FILT_HZ 0.135f
#define AC_BALANCE_ROLL_ERROR_FILT_HZ  2.5f

class AC_BalanceControl {
public:
    AC_BalanceControl(AP_Motors& motors, AP_AHRS_View& ahrs, AP_RMUART& _rmuart);

    // empty destructor to suppress compiler warning
    virtual ~AC_BalanceControl() { }

    float Balance(float Angle, float Gyro);
    float Velocity(float encoder_left, float encoder_right);
    float Turn(float gyro);

    void RollControl(float roll);

    void balance_all_control(void);

    void set_control_zeros(void);

    uint8_t get_Balance_Mode() { return balanceMode; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Motors&          _motors;
    const AP_AHRS_View& _ahrs;
    AP_RMUART&          _rmuart;

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
    AP_Float _balance_bal_p;
    AP_Float _balance_bal_d;

    AP_Float _balance_velocity_p;
    AP_Float _balance_velocity_i;
    AP_Float _balance_velocity_d;
    AP_Float _balance_velocity_imax;

    AP_Float _balance_turn_p;
    AP_Float _balance_turn_d;

    AP_Float _zero_angle; // 机械零值

    AP_Float _max_speed;

    AP_Float Target_Velocity_X;
    AP_Float Target_Velocity_Z;

    AC_PID _pid_roll;

    ///////////////////////////////////////////////////////
    // 平衡环参数
    float Balance_Angle_bias;
    float Balance_Gyro_bias;

    ///////////////////////////////////////////////////////
    // 速度环参数
    float Encoder_bias_filter; // 一阶低通滤波器
    float Encoder_Integral;    // 速度积分
    float Encoder_Least;       // 速度误差
    float Encoder_bias;        // 速度偏置
    float Encoder_Movement;    // 速度运动

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
};
