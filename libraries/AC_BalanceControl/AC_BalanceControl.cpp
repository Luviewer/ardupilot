#include "AC_BalanceControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "stdio.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_BalanceControl::var_info[] = {

    AP_SUBGROUPINFO(_pid_angle, "ANG_", 1, AC_BalanceControl, AC_PID),

    AP_SUBGROUPINFO(_pid_speed, "SPD_", 2, AC_BalanceControl, AC_PID),

    AP_SUBGROUPINFO(_pid_turn, "TRN_", 3, AC_BalanceControl, AC_PID),

    AP_SUBGROUPINFO(_pid_roll, "ROL_", 4, AC_BalanceControl, AC_PID),

    AP_GROUPINFO("ZERO", 5, AC_BalanceControl, _zero_angle, AC_BALANCE_ZERO_ANGLE),

    AP_GROUPINFO("MAX_SPEED", 6, AC_BalanceControl, _max_speed, AC_BALANCE_MAX_SPEED),

    AP_GROUPINFO("T_SPD_MAX_X", 7, AC_BalanceControl, Target_MAX_Velocity_X, AC_BALANCE_TARGET_X_SPEED),

    AP_GROUPINFO("T_SPD_MAX_Z", 8, AC_BalanceControl, Target_MAX_Velocity_Z, AC_BALANCE_TARGET_Z_SPEED),

    AP_GROUPEND
};

AC_BalanceControl::AC_BalanceControl(AP_Motors* motors, AP_AHRS_View* ahrs)
    : _pid_angle(AC_BALANCE_ANGLE_P, 0, AC_BALANCE_ANGLE_D, 0, 0, 0, 0, 0),
      _pid_speed(AC_BALANCE_SPEED_P, AC_BALANCE_SPEED_I, 0, 0, AC_BALANCE_SPEED_IMAX, 0, 0, 0),
      _pid_turn(AC_BALANCE_TURN_P, 0, AC_BALANCE_TURN_D, 0, 0, 0, 0, 0),
      _pid_roll(AC_BALANCE_ROLL_P, AC_BALANCE_ROLL_I, AC_BALANCE_ROLL_D, 0, AC_BALANCE_ROLL_IMAX, 0, 0, 0),
      _motors(motors),
      _ahrs(ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);

    _dt = 1.0f / 200.0f;

    speed_low_pass_filter.set_cutoff_frequency(30.0f);
    speed_low_pass_filter.reset(0);

    _movement_x = moveFlag::none;
    _movement_z = moveFlag::none;

    balanceMode = BalanceMode::ground;

    stop_balance_control = false;

    force_stop_balance_control = false;
}

void AC_BalanceControl::init()
{
    balanceCAN = AP_BalanceCAN::get_singleton();
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/
float AC_BalanceControl::angle_controller(float Angle, float Gyro)
{
    // 求出平衡的角度中值 和机械相关
    angle_bias = _zero_angle - Angle;

    // 计算角速度误差
    gyro_bias = 0.0f - Gyro;

    // 计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    angle_out = _pid_angle.kP() * angle_bias + gyro_bias * _pid_angle.kD();

    if (stop_balance_control || Flag_Stop || force_stop_balance_control) {
        angle_out  = 0;
        angle_bias = 0;
        gyro_bias  = 0;
    }

    return angle_out;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
float AC_BalanceControl::velocity_controller(float encoder_left, float encoder_right)
{
    //================遥控前进后退部分====================//
    encoder_movement = _movement_x / 500 * Target_MAX_Velocity_X;

    //================速度PI控制器=====================//

    // 获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）
    encoder_error = (encoder_left + encoder_right) - encoder_movement;

    // ��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ�
    //更新速度输出

    if (stop_balance_control || Flag_Stop || force_stop_balance_control) {
        _pid_speed.reset_I();
        encoder_error        = 0;
        encoder_error_filter = 0;
        encoder_movement     = 0;
        velocity_out         = 0;
    }

    return velocity_out;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
float AC_BalanceControl::turn_controller(float yaw, float gyro)
{
    //===================遥控左右旋转部分=================//
    turn_target = _movement_z / 500 * Target_MAX_Velocity_Z;

    //===================转向PD控制器=================//
    turn_out = turn_target * _pid_turn.kP() + gyro * _pid_turn.kD(); // 结合Z轴陀螺仪进行PD控制

    if (stop_balance_control || Flag_Stop || force_stop_balance_control) {
        turn_out    = 0;
        turn_target = 0;
    }

    return turn_out;
}

void AC_BalanceControl::roll_controller(float roll)
{
    if (_motors == nullptr) return;

    float roll_out, roll_target;

    if(fabsf((hal.rcin->read(CH_1)-1500)) < 20){
        roll_target = 0.0f;
    }else{
        roll_target = (float)_movement_y / 500.0f * radians(60.0f);
    }
    roll_out = _pid_roll.update_all(roll_target, roll, _dt);

    _motors->set_roll_out(JT * roll_out); // -1 ~ 1
}

void AC_BalanceControl::hight_controller()
{
    if (_motors == nullptr) return;

    float high_out;

    high_out = (float)_movement_h / 500.0f * radians(30.0f);

    _motors->set_high_out(JT * high_out); // -1 ~ 1
}

void AC_BalanceControl::update(void)
{
    if (_motors == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_motors = nullptr");
        return;
    }

    // if (balanceCAN == nullptr) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "balanceCAN = nullptr");
    //     return;
    // }
    if (_ahrs == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_ahrs = nullptr");
        return;
    }

    static float angle_y, gyro_y, gyro_z;
    static float wheel_left_f, wheel_right_f;
    static float motor_target_left_f, motor_target_right_f;
    const float  max_scale_value = 10000.0f;

    angle_y = _ahrs->pitch;
    gyro_y  = _ahrs->get_gyro_latest()[1];
    gyro_z  = _ahrs->get_gyro_latest()[2];

    // 转速缩小1000倍
    wheel_left_f  = (float)balanceCAN->getSpeed(0) / max_scale_value;
    wheel_right_f = -(float)balanceCAN->getSpeed(1) / max_scale_value;

    // 平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
    control_balance = angle_controller(angle_y, gyro_y);

    // 速度环PID控制,记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
    control_velocity = velocity_controller(wheel_left_f, wheel_right_f);

    // 转向环PID控制
    control_turn = turn_controller(_ahrs->yaw, gyro_z);

    // motor值正数使小车前进，负数使小车后退, 范围【-1，1】
    motor_target_left_f  = control_balance + control_velocity + control_turn; // 计算左轮电机最终PWM
    motor_target_right_f = control_balance + control_velocity - control_turn; // 计算右轮电机最终PWM

    motor_target_left_int  = (int16_t)(motor_target_left_f * max_scale_value);
    motor_target_right_int = -(int16_t)(motor_target_right_f * max_scale_value);
    if(motor_target_left_int > 0){motor_target_left_int += 100;}
   else {motor_target_left_int -=100;}
    if(motor_target_right_int > 0){motor_target_right_int += 60;}
   else {motor_target_right_int -= 60;}

    // 最终的电机输入量
    balanceCAN->setCurrent(0, (int16_t)motor_target_left_int);
    balanceCAN->setCurrent(1, (int16_t)motor_target_right_int);

    // 腿部舵机控制
    roll_controller(_ahrs->roll);

    // 设置模式
    set_control_mode();

    // 遥控输入
    pilot_control();

    // 检查是否失控
    if (Pick_Up(_ahrs->get_accel_ef().z, angle_y, balanceCAN->getSpeed(0), balanceCAN->getSpeed(1))) {
        Flag_Stop = true;
    }

    if (Put_Down(angle_y, balanceCAN->getSpeed(0), balanceCAN->getSpeed(1))) {
        Flag_Stop = false;
    }

    debug_info();

    if (hal.rcin->read(CH_8) > 1700) {
        force_stop_balance_control = true;
    } else {
        force_stop_balance_control = false;
    }
}

// void AC_BalanceControl::function_s()
// {
//     if (_motors == nullptr) return;

//     switch (balanceMode) {
//         case BalanceMode::ground:{
//             S_GF = 0.0f;
//             S_FG = 1.0f;
//             _motors->set_fac_out(S_GF);

//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "ground");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");

//             if ((hal.rcin->read(CH_7) > 1300) && (hal.rcin->read(CH_7) < 1700)) { // 通道7切换到二档，进入过渡模式
//                 balanceMode = BalanceMode::transition;
//             }
//             break;}

//         case BalanceMode::transition:{
//             int16_t T = hal.rcin->read(CH_3);
//             S_GF      = 1 / (1 + expf(-((T - Target_Offset_SGF_B) / Target_Slope_SGF_R)));     // 0 ~ 1
//             S_FG      = 1 - 1 / (1 + expf(-((T - Target_Offset_SFG_B) / Target_Slope_SFG_R))); // 0 ~ 1
//             _motors->set_fac_out(S_GF);

//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "transition");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");

//             if (hal.rcin->read(CH_7) > 1700) { // 通道7切换到3档，进入空中模式
//                 balanceMode = BalanceMode::aerial;
//             }

//             if ((hal.rcin->read(CH_7) < 1300) && (hal.rcin->read(CH_3) < 1200)) { // 防止无人机在空中的时候误切到地面模式直接掉落
//                 balanceMode = BalanceMode::ground;
//             }
//             break;}

//         case BalanceMode::aerial:{
//             S_GF = 1.0f;
//             S_FG = 0.0f;
//             _motors->set_fac_out(S_GF);

//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "aerial");
//             // gcs().send_text(MAV_SEVERITY_NOTICE, "*****************");

//             if ((hal.rcin->read(CH_7) > 1300) && (hal.rcin->read(CH_7) < 1700)) { // 通道7切换到2档，进入过渡模式，准备降落
//                 balanceMode = BalanceMode::transition;
//             }
//             break;}

//         default:
//             break;
//     }
// }

// void AC_BalanceControl::checkAcc_func(){
// uint32_t timems_start = AP_HAL::millis64();
// while((AP_HAL::millis64() - timems_start) > 2){
    
// }

// }

void AC_BalanceControl::check_Acceleration(){
    accelData =  _ahrs->get_accel_ef().z + 9.8f; //获取当前加速度值
    int16_t T = hal.rcin->read(CH_3); //获取当前油门输入值
    switch(balanceMode){
        case BalanceMode::ground:{
            S_GF = 1.0f; //飞行部分影响因子置1，正常
            S_FG = 1.0f; //轮足电机影响因子置1，开启
            JT = 1;      //关节舵机影响因子置1，开启
            _motors->set_fac_out(JT);   //输出飞行部分影响因子，方便调用

            if((fabsf(accelData) > _take_off_acc) && (hal.rcin->read(CH_3) > _take_off_thr) && (hal.rcin->read(CH_7) < 1500)){ //当反馈的加速度值大于设定的起飞加速度，油门输入大于设定的起飞油门并且当前处于地空过渡时，则进入过渡
            gcs().send_text(MAV_SEVERITY_NOTICE, "*************************************");
            gcs().send_text(MAV_SEVERITY_NOTICE, "Balance_Copter is taking off, accel = %f", accelData);
            gcs().send_text(MAV_SEVERITY_NOTICE, "*************************************");

            S_GF = 1.0f;    //起飞，飞行部分影响因子置1
            S_FG = 0.0f;    //轮足电机影响因子置0，关闭
            JT = 0.0f;      //关节舵机影响因子置0，关闭
            _motors->set_fac_out(JT);

            balanceMode = BalanceMode::aerial; //进入飞行模式
            }
            break;

        case BalanceMode::balance_car:
            if ((_motors->armed()) && (hal.rcin->read(CH_3) < 1200)) {
                balanceMode = BalanceMode::flying_with_balance;
                gcs().send_text(MAV_SEVERITY_NOTICE, "flying_with_balance");
            }
            break;

        case BalanceMode::flying_with_balance:
            if ((alt_cm >= 8) && (hal.rcin->read(CH_3) > 1200) && (hal.rcin->read(CH_8)) > 1600) {
                stop_balance_control = true;
                balanceMode          = BalanceMode::flying_without_balance;
                gcs().send_text(MAV_SEVERITY_NOTICE, "flying_without_balance");
            }
            break;

        case BalanceMode::flying_without_balance:
            // set_control_zeros();
            // stop_balance_control = true;
            if ((alt_cm < 10) && (hal.rcin->read(CH_3) < 1200)) {
                stop_balance_control = true;
                balanceMode          = BalanceMode::landing_ground_idle;
                gcs().send_text(MAV_SEVERITY_NOTICE, "landing_ground_idle");
            }
            break;

        case BalanceMode::landing_ground_idle:
            if ((alt_cm < 10) && (hal.rcin->read(CH_3) < 1200) && (hal.rcin->read(CH_8)) < 1600) {
                stop_balance_control = false;
                balanceMode          = BalanceMode::landing_finish;
                gcs().send_text(MAV_SEVERITY_NOTICE, "landing_finish");
            }
            break;

        case BalanceMode::landing_finish:
            if ((alt_cm < 8)) {
                balanceMode = BalanceMode::ground;
                gcs().send_text(MAV_SEVERITY_NOTICE, "ground");
            }
            break;
        }

        default:
            break;
    }
}

void AC_BalanceControl::pilot_control()
{
    int16_t pwm_x = hal.rcin->read(CH_1) - 1500;
    int16_t pwm_z = hal.rcin->read(CH_4) - 1500;

    if (pwm_x < 50 && pwm_x > -50) {
        _movement_x = 0;
    } else if (abs(pwm_z) > 500) {
        _movement_z = 0;
    } else {
        _movement_x = pwm_x;
    }

    if (pwm_z < 50 && pwm_z > -50) {
        _movement_z = 0;
    } else if (abs(pwm_z) > 500) {
        _movement_z = 0;
    } else {
        _movement_z = pwm_z;
    }

    if (pwm_h < 20 && pwm_h > -20) {
        _movement_h = 0;
    } else if (abs(pwm_h) > 500) {
        _movement_h = 0;
    } else {
        _movement_h = pwm_h;
    }
}

    // 调试用
    static uint16_t cnt = 0;
    cnt++;
    if (cnt > 400) {
        cnt = 0;
        gcs().send_text(MAV_SEVERITY_NOTICE, "--------------------");
        gcs().send_text(MAV_SEVERITY_NOTICE, "left_real_speed=%d", balanceCAN->getSpeed(0));
        gcs().send_text(MAV_SEVERITY_NOTICE, "right_real_speed=%d", balanceCAN->getSpeed(1));
        gcs().send_text(MAV_SEVERITY_NOTICE, "left_target_current=%d", balanceCAN->getCurrent(0));
        gcs().send_text(MAV_SEVERITY_NOTICE, "right_target_current=%d", balanceCAN->getCurrent(1));
        gcs().send_text(MAV_SEVERITY_NOTICE, "altok=%d, alt_cm=%f", alt_ok, alt_cm);
        gcs().send_text(MAV_SEVERITY_NOTICE, "--------------------");
    }
}

/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
bool AC_BalanceControl::Pick_Up(float Acceleration, float Angle, int16_t encoder_left, int16_t encoder_right)
{
    static uint16_t flag, count0, count1, count2;
    if (flag == 0) // 第一步
    {
        if ((abs(encoder_left) + abs(encoder_right)) < 100) // 条件1，小车接近静止
            count0++;
        else
            count0 = 0;
        if (count0 > 10) flag = 1, count0 = 0;
    }
    if (flag == 1) // 进入第二步
    {
        if (++count1 > (2 * 200)) count1 = 0, flag = 0;         // 超时不再等待2000ms，返回第一步
        if ((Acceleration > 0.75) && ((fabsf(Angle) - 10) < 0)) // 条件2，小车是在0度附近被拿起
            flag = 2;
    }
    if (flag == 2) // 第三步
    {
        if (++count2 > (1 * 200)) count2 = 0, flag = 0; // 超时不再等待1000ms
        if (abs(encoder_left + encoder_right) > 15000)  // 条件3，小车的轮胎因为正反馈达到最大的转速
        {
            flag = 0;
            return true; // 检测到小车被拿起
        }
    }
    return false;
}

/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
bool AC_BalanceControl::Put_Down(float Angle, int encoder_left, int encoder_right)
{
    static uint16_t flag, count;
    if (Flag_Stop == false) // 防止误检
        return 0;
    if (flag == 0) {
        if ((fabsf(Angle) - 10) < 0 && abs(encoder_left) < 20 && abs(encoder_right) < 20) // 条件1，小车是在0度附近的
            flag = 1;
    }
    if (flag == 1) {
        if (++count > 50) // 超时不再等待 500ms
        {
            count = 0;
            flag  = 0;
        }
        if (abs(encoder_left) > 50 && abs(encoder_right) > 50) // 条件2，小车的轮胎在未上电的时候被人为转动
        {
            flag = 0;
            return true; // 检测到小车被放下
        }
    }
    return false;
}
