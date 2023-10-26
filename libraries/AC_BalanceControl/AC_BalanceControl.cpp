#include "AC_BalanceControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "stdio.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_BalanceControl::var_info[] = {
    AP_GROUPINFO("BAL_P", 0, AC_BalanceControl, _balance_bal_p, AC_BALANCE_BALANCE_P),

    AP_GROUPINFO("BAL_D", 1, AC_BalanceControl, _balance_bal_d, AC_BALANCE_BALANCE_D),

    AP_GROUPINFO("VEL_P", 2, AC_BalanceControl, _balance_velocity_p, AC_BALANCE_VELOCITY_P),

    AP_GROUPINFO("VEL_I", 3, AC_BalanceControl, _balance_velocity_i, AC_BALANCE_VELOCITY_I),

    AP_GROUPINFO("VEL_IMAX", 4, AC_BalanceControl, _balance_velocity_imax, AC_BALANCE_VELOCITY_IMAX),

    AP_GROUPINFO("TURN_P", 5, AC_BalanceControl, _balance_turn_p, AC_BALANCE_TURN_P),

    AP_GROUPINFO("TURN_D", 6, AC_BalanceControl, _balance_turn_d, AC_BALANCE_TURN_D),

    AP_GROUPINFO("ZERO", 7, AC_BalanceControl, _zero_angle, AC_BALANCE_ZERO_ANGLE),

    AP_GROUPINFO("MAX_SPEED", 8, AC_BalanceControl, _max_speed, AC_BALANCE_MAX_SPEED),

    AP_GROUPINFO("TAR_SPEED_X", 9, AC_BalanceControl, Target_Velocity_X, AC_BALANCE_TARGET_X_SPEED),

    AP_GROUPINFO("TAR_SPEED_Z", 10, AC_BalanceControl, Target_Velocity_Z, AC_BALANCE_TARGET_Z_SPEED),

    AP_SUBGROUPINFO(_pid_roll, "ROL_", 11, AC_BalanceControl, AC_PID),

    AP_GROUPINFO("VEL_D", 12, AC_BalanceControl, _balance_velocity_d, AC_BALANCE_VELOCITY_D),

    AP_GROUPEND
};

AC_BalanceControl::AC_BalanceControl(AP_Motors& motors, AP_AHRS_View& ahrs, AP_RMUART& rmuart)
    : _motors(motors)
    , _ahrs(ahrs)
    , _rmuart(rmuart)
    , _pid_roll(AC_BALANCE_ROLL_P,
                AC_BALANCE_ROLL_I,
                AC_BALANCE_ROLL_D,
                0.0f,
                AC_BALANCE_ROLL_IMAX,
                AC_BALANCE_ROLL_TARGET_FILT_HZ,
                AC_BALANCE_ROLL_ERROR_FILT_HZ,
                0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    Encoder_bias_filter = 0.2f;

    _dt = 1.0f / 400.0f;

    _moveflag_x = moveFlag::none;
    _moveflag_z = moveFlag::none;

    balanceMode = BalanceMode::ground;
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/
float AC_BalanceControl::Balance(float Angle, float Gyro)
{
    float balance;
    Balance_Angle_bias = _zero_angle - Angle;                                                      // 求出平衡的角度中值 和机械相关
    Balance_Gyro_bias  = 0 - Gyro;
    balance            = _balance_bal_p * Balance_Angle_bias + Balance_Gyro_bias * _balance_bal_d; // 计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
// 修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
float AC_BalanceControl::Velocity(float encoder_left, float encoder_right)
{
    float velocity;

    //================遥控前进后退部分====================//
    // if (_moveflag_x == moveFlag::moveFront) {
    //     Encoder_Movement = Target_Velocity_X;  // 收到前进信号
    // } else if (_moveflag_x == moveFlag::moveBack) {
    //     Encoder_Movement = -Target_Velocity_X; // 收到后退信号
    // } else {
    //     Encoder_Movement = 0;
    // }
        Encoder_Movement = 0;

    //================速度PI控制器=====================//
    Encoder_Least = Encoder_Movement - (encoder_left + encoder_right); // 获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）

    Encoder_bias *= Encoder_bias_filter;                               // 一阶低通滤波器
    Encoder_bias += Encoder_Least * (1 - Encoder_bias_filter);         // 一阶低通滤波器，减缓速度变化

    Encoder_Integral += Encoder_bias;                                  // 积分出位移 积分时间：10ms
    Encoder_Integral = Encoder_Integral;                               // 接收遥控器数据，控制前进后退

    if (Encoder_Integral > AC_BALANCE_VELOCITY_IMAX)
        Encoder_Integral = AC_BALANCE_VELOCITY_IMAX;                                        // 积分限幅
    if (Encoder_Integral < -AC_BALANCE_VELOCITY_IMAX)
        Encoder_Integral = -AC_BALANCE_VELOCITY_IMAX;                                       // 积分限幅

    static float last_Encoder_bias;

    velocity = Encoder_bias * _balance_velocity_p + Encoder_Integral * _balance_velocity_i + (Encoder_bias-last_Encoder_bias) * _balance_velocity_d; // 速度控制
    // if (Turn_Off(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
    //     Encoder_Integral = 0;                                                           // 电机关闭后清除积分

    last_Encoder_bias = Encoder_bias;

    return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
float AC_BalanceControl::Turn(float gyro)
{
    float turn;

    Turn_Kp = _balance_turn_p;
    //===================遥控左右旋转部分=================//
    if (_moveflag_z == moveFlag::moveLeft)
        Turn_Target = -Target_Velocity_Z;
    else if (_moveflag_z == moveFlag::moveRight)
        Turn_Target = Target_Velocity_Z;
    else
        Turn_Target = 0;

    // if (_moveflag == moveFlag::moveFront || _moveflag == moveFlag::moveBack)
    //     Kd = _balance_turn_d;
    // else
    //     Kd = 0; // 转向的时候取消陀螺仪的纠正 有点模糊PID的思想

    Turn_Kd = _balance_turn_d;

    //===================转向PD控制器=================//
    turn = Turn_Target * Turn_Kp + gyro * Turn_Kd; // 结合Z轴陀螺仪进行PD控制

    return turn;                                   // 转向环PWM右转为正，左转为负
}

void AC_BalanceControl::RollControl(float roll)
{
    float roll_out;

    roll_out = _pid_roll.update_all(0.0f, roll, _dt);

    _motors.set_roll_out(roll_out); // -1 ~ 1
}

void AC_BalanceControl::balance_all_control(void)
{
    static float   _wheel_left_f, _wheel_right_f;
    static int16_t _wheel_left_int, _wheel_right_int;
    static float   motor_target_left_f, motor_target_right_f;
    static int16_t motor_target_left_int, motor_target_right_int;

    static float angle_y, gyro_y, gyro_z;

    _rmuart.getWheelSpeed(_wheel_left_int, _wheel_right_int);
    _wheel_left_f  = (float)_wheel_left_int *0.01f /  (float)_max_speed;
    _wheel_right_f = (float)_wheel_right_int *0.01f/  (float)_max_speed;

    static uint8_t cnnttt=0;
    cnnttt++;
    if(cnnttt>30){
        gcs().send_text(MAV_SEVERITY_INFO,"left_s=%.3f, right_s=%.3f\n", _wheel_left_f, _wheel_right_f);
        // gcs().send_text(MAV_SEVERITY_INFO,"%x %x %x %x %x %x %x \r\n",receive_buff[0],receive_buff[1],receive_buff[2],receive_buff[3],receive_buff[4],receive_buff[5],receive_buff[6]);
        cnnttt=0;
    }

    angle_y = _ahrs.pitch;
    gyro_y  = _ahrs.get_gyro_latest()[1];
    gyro_z  = _ahrs.get_gyro_latest()[2];

    control_balance  = Balance(angle_y, gyro_y);                // 平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
    control_velocity = Velocity(_wheel_left_f, _wheel_right_f); // 速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
    control_turn     = Turn(gyro_z);                            // 转向环PID控制

    RollControl(_ahrs.roll);                                    // 腿部舵机控制

    // motor值正数使小车前进，负数使小车后退, 范围【-1，1】
    motor_target_left_f  = control_balance + control_velocity + control_turn; // 计算左轮电机最终PWM
    motor_target_right_f = control_balance + control_velocity - control_turn; // 计算右轮电机最终PWM

    motor_target_left_int  = (int16_t)(motor_target_left_f * _max_speed);
    motor_target_right_int = (int16_t)(motor_target_right_f * _max_speed);

    /////////////////////////////////////////////////////////////////
    Vector3f acc { 0, 0, 0 };
    switch (balanceMode) {
    case BalanceMode::ground:
        if ((hal.rcin->read(CH_3) > 1400) && (_motors.armed())) {
            printf("flying_with_balance\r\n");
            balanceMode = BalanceMode::flying_with_balance;
        }
        break;

    case BalanceMode::flying_with_balance:
        if ((hal.rcin->read(CH_8) > 1600)) {
            printf("flying_without_balance\r\n");

            balanceMode = BalanceMode::flying_without_balance;
        }
        break;

    case BalanceMode::flying_without_balance:
        set_control_zeros();

        motor_target_left_int  = 0.0f;
        motor_target_right_int = 0.0f;

        acc = _ahrs.get_accel_ef();
        if ((acc.length() > 15.0f) && (hal.rcin->read(CH_3) < 1450)) {
            printf("acc:%f\r\n", acc.length());
            printf("landing_check\r\n");

            balanceMode = BalanceMode::landing_check;
        }
        break;

    case BalanceMode::landing_check:
        _motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        balanceMode = BalanceMode::ground;
        printf("ground\r\n");
        break;

    default:
        break;
    }

    _rmuart.setWheelSpeed(motor_target_left_int, motor_target_right_int);

    uint16_t pwm_x = hal.rcin->read(CH_7);
    uint16_t pwm_z = hal.rcin->read(CH_6);

    if (pwm_x < 1300) {
        _moveflag_x = moveFlag::moveBack;
    } else if (pwm_x > 1700) {
        _moveflag_x = moveFlag::moveFront;
    } else {
        _moveflag_x = moveFlag::none;
    }

    if (pwm_z < 1300) {
        _moveflag_z = moveFlag::moveLeft;
    } else if (pwm_z > 1700) {
        _moveflag_z = moveFlag::moveRight;
    } else {
        _moveflag_z = moveFlag::none;
    }
}

void AC_BalanceControl::set_control_zeros(void)
{

    Balance_Angle_bias = 0;
    Balance_Gyro_bias  = 0;

    Encoder_Integral = 0;
    Encoder_Least    = 0;
    Encoder_bias     = 0;
    Encoder_Movement = 0;

    Turn_Target = 0;
    Turn_Kp     = 0;
    Turn_Kd     = 0;
}
