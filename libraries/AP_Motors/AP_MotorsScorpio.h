/// @file	AP_MotorsScorpio.h
/// @brief	天蝎座三倾转旋翼的电机控制类,骨架完全沿用 AP_MotorsTri
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

/// @class      AP_MotorsScorpio
class AP_MotorsScorpio : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsScorpio(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends minimum values out to the motors
    void                output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    static const AP_Param::GroupInfo var_info[];

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "SCORPIO"; }
    const char*  get_type_string() const override { return "tilt-tri"; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

private:
    // 声明三个倾转舵机的角度行程,并由安装角刷新 forward/lateral 分配因子
    void setup_tilt_servos();

    // motor test 时向倾转舵机输出 PWM,反相时绕舵机 trim 镜像
    void output_test_tilt(SRV_Channel::Aux_servo_function_t function, bool reversed, int16_t pwm);

    // parameters

    // 前倾转轴安装角:前左取前右的镜像;后旋翼在中轴线上,固定纯横向倾转
    AP_Float        _front_axis_angle_deg;

    // 三个倾转舵机的最大行程(度),满偏对应此角,不再使用 Tri 的 MOT_YAW_SV_ANGLE
    AP_Float        _servo_angle_max_deg;

    // 后/前悬停推力比:等腰三角形中后旋翼离重心近,需承担更大推力份额。
    // 从悬停日志实测:后电机推力 / 单个前电机推力
    AP_Float        _rear_thrust_ratio;

    // 三个倾转舵机的反相设置:0 不变,1 反向(输出与 motor test 都生效)
    AP_Int8         _fr_tilt_reverse;
    AP_Int8         _rr_tilt_reverse;
    AP_Int8         _fl_tilt_reverse;

    // 三个倾转舵机的归一化输入(±1)与输出角(弧度),顺序:前右、后、前左
    float           _tilt_in[3];
    float           _servo_angle[3];
    float           _thrust_right;
    float           _thrust_rear;
    float           _thrust_left;

    // forward/lateral 投影到各倾转方向的因子,顺序:前右、后、前左
    float           _forward_factor[3];
    float           _lateral_factor[3];

    // 集体油门分配因子,由 _rear_thrust_ratio 归一化得到(最大者为 1)
    float           _throttle_factor_front;
    float           _throttle_factor_rear;
};
