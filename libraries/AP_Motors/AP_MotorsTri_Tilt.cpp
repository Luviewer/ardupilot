/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   共轴 Y6B 倾转三旋翼（5DOF 控制）

   电机布局：
   - 电机 0-1：前右共轴一对（上 CW、下 CCW）
   - 电机 2-3：后置共轴一对（上 CW、下 CCW）
   - 电机 4-5：前左共轴一对（上 CW、下 CCW）

   控制分配基于 MATLAB 推导：
   静态矩阵 F_alloc 将 5DOF [Fx, Fz, Mx, My, Mz] 映射到 6 个中间量
   [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
*/

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_MotorsTri_Tilt.h"

extern const AP_HAL::HAL& hal;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 参数
////////////////////////////////////////////////////////////////////////////////////////////////////////////
const AP_Param::GroupInfo AP_MotorsTri_Tilt::var_info[] = {
    // @Param: TRI_TILT_LX
    // @DisplayName: 前臂 X 方向距离（归一化）
    // @Description: 前部转子臂到重心的 X 轴距离（归一化）
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LX", 1, AP_MotorsTri_Tilt, _lfront_x, 0.5f),

    // @Param: TRI_TILT_LY
    // @DisplayName: 前臂 Y 方向距离（归一化）
    // @Description: 前部转子臂到重心的 Y 轴距离（归一化）
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LY", 2, AP_MotorsTri_Tilt, _lfront_y, 0.5f),

    // @Param: TRI_TILT_LREAR
    // @DisplayName: 后臂长度（归一化）
    // @Description: 后部转子臂到重心的长度（归一化）
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LREAR", 3, AP_MotorsTri_Tilt, _lrear, 1.0f),

    // @Param: TRI_TILT_ANG_MAX
    // @DisplayName: 最大倾转角
    // @Description: 转子允许的最大倾转角（度）
    // @Range: 0 135
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("TILT_ANG_MAX", 4, AP_MotorsTri_Tilt, _servo_angle_max, 135.0f),

    // @Param: TRI_TILT_YAW_FAC
    // @DisplayName: 偏航力矩因子
    // @Description: 共轴对差分推力产生偏航力矩的比例因子
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_FAC", 8, AP_MotorsTri_Tilt, _yaw_torque_factor, 0.15f),

    // @Param: TRI_TILT_YAW_DIR
    // @DisplayName: 偏航方向
    // @Description: 若偏航响应反向（机体偏航方向与指令相反），设置为 -1
    // @Values: -1:Reversed, 1:Normal
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_DIR", 9, AP_MotorsTri_Tilt, _yaw_dir, 1),

    // @Param: TRI_TILT_SVO_FR_REV
    // @DisplayName: 前右倾转舵机反向
    // @Description: 设为 1 以反向前右倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FR_REV", 11, AP_MotorsTri_Tilt, _tilt_servo_fr_rev, 0),

    // @Param: TRI_TILT_SVO_REAR_REV
    // @DisplayName: 后置倾转舵机反向
    // @Description: 设为 1 以反向后置倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_REAR_REV", 12, AP_MotorsTri_Tilt, _tilt_servo_rear_rev, 0),

    // @Param: TRI_TILT_SVO_FL_REV
    // @DisplayName: 前左倾转舵机反向
    // @Description: 设为 1 以反向前左倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FL_REV", 13, AP_MotorsTri_Tilt, _tilt_servo_fl_rev, 0),

    // @Param: TRI_TILT_PIT_OFF_MAX
    // @DisplayName: 最大俯仰姿态偏置（RC7）
    // @Description: RC7 指令的最大俯仰姿态偏置（度）（1500->0，1000->-max，2000->+max）
    // @Range: 0 45
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PIT_OFF_MAX", 14, AP_MotorsTri_Tilt, _tilt_pitch_off_max_deg, 20.0f),

    AP_GROUPEND
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 启用 6 电机的共轴 Y6B 配置
    add_motor_num(AP_MOTORS_MOT_1);  // 前右上
    add_motor_num(AP_MOTORS_MOT_2);  // 前右下
    add_motor_num(AP_MOTORS_MOT_3);  // 后置上
    add_motor_num(AP_MOTORS_MOT_4);  // 后置下
    add_motor_num(AP_MOTORS_MOT_5);  // 前左上
    add_motor_num(AP_MOTORS_MOT_6);  // 前左下

    // 设置电机更新频率
    set_update_rate(_speed_hz);

    // 标记电机启用（用于校准）
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // 设置默认电机与舵机映射（可通过 SERVOx_FUNCTION 覆盖）
    // 映射：a1->前右，a2->后置，a3->前左
    const bool ok_fr = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, AP_MOTORS_TRI_TILT_SERVO_FR);
    const bool ok_rear = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRear, AP_MOTORS_TRI_TILT_SERVO_REAR);
    const bool ok_fl = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, AP_MOTORS_TRI_TILT_SERVO_FL);

    // 设置倾转舵机默认 PWM 范围（硬件：500~2500us）
    // 仅修改默认值；用户设置的 SERVOx_MIN/MAX 不会被覆盖。
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRight, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRear, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorLeft, 500, 2500);

    // 设置倾转舵机角度范围（厘度）
    // 若 _servo_angle_max 为 0，则视为“无软件限幅”，这里设置为较宽范围。
    const float ang_max_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
    const int16_t servo_range_cd = int16_t(constrain_float(ang_max_deg, 0.0f, float(AP_MOTORS_TRI_TILT_ANGLE_MAX)) * 100.0f);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, servo_range_cd);

    // 检查舵机是否已分配（默认或用户映射）
    _servos_assigned =
        (ok_fr || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)) &&
        (ok_rear || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear)) &&
        (ok_fl || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));

    // 配置电机与分配矩阵
    setup_motors(frame_class, frame_type);

    _mav_type = MAV_TYPE_TRICOPTER;

    // 记录初始化成功
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI && _servos_assigned);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置机架类别与类型
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 机架变化时重新初始化
    if (frame_class != _active_frame_class || frame_type != _active_frame_type) {
        _active_frame_class = frame_class;
        _active_frame_type = frame_type;
        setup_motors(frame_class, frame_type);
    }

    set_initialised_ok((frame_class == MOTOR_FRAME_TRI) && _servos_assigned);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置电机更新频率
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::set_update_rate(uint16_t speed_hz)
{
    // 记录请求的频率
    _speed_hz = speed_hz;

    // 为全部 6 个电机设置更新频率
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6;
    rc_set_freq(mask, _speed_hz);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 配置电机 - 设置静态分配矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 注意：
    // 不要在此处调用 AP_MotorsMatrix::remove_motor()。
    // 该后端通过 rc_write() 直接驱动电机输出，并使用 motor_enabled[]
    // 实现起转状态逻辑。remove_motor() 会清空 motor_enabled[]，导致输出停止
    //（用户反馈电机输出卡在 0）。
    //
    // 电机通道默认值在 init() 里通过 add_motor_num() 设置。
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // 矩阵清零
    memset(_alloc_matrix, 0, sizeof(_alloc_matrix));
    memset(_alloc_matrix_pinv, 0, sizeof(_alloc_matrix_pinv));
    memset(_thrust, 0, sizeof(_thrust));
    memset(_tilt_angle, 0, sizeof(_tilt_angle));
    memset(_intermediate, 0, sizeof(_intermediate));

    // 计算分配矩阵及其伪逆
    calculate_allocation_matrix();
    calculate_allocation_matrix_pinv();

    _frame_class_string = "TRI_TILT";
    _frame_type_string = "Coaxial-Y6B";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取电机掩码
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t AP_MotorsTri_Tilt::get_motor_mask()
{
    uint32_t mask = AP_MotorsMatrix::get_motor_mask();

    // 加入倾转舵机输出
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRight, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRear, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorLeft, chan)) {
        mask |= 1U << chan;
    }

    return mask;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 执行解锁检查
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AP_MotorsTri_Tilt::arming_checks(size_t buflen, char *buffer) const
{
    // 检查舵机是否已分配
    if (!_servos_assigned) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Servos not assigned");
        return false;
    }

    // 检查几何参数是否有效
    if (_lfront_x <= 0.0f || _lfront_y <= 0.0f || _lrear <= 0.0f) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Invalid geometry params");
        return false;
    }

    // 检查角度限制：
    // _servo_angle_max == 0 表示“无软件限幅”（仍受 SERVOx_MIN/MAX 约束）。
    // 否则强制一个最小值，避免范围过小导致过度缩放/饱和。
    const float ang_max = _servo_angle_max;
    if ((!is_zero(ang_max) && ang_max < AP_MOTORS_TRI_TILT_ANGLE_MIN) ||
        ang_max < 0.0f ||
        ang_max > AP_MOTORS_TRI_TILT_ANGLE_MAX) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Invalid angle limits");
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 推力补偿
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::thrust_compensation(void)
{
    // 调用父类推力补偿
    AP_MotorsMatrix::thrust_compensation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_armed_stabilizing - 主要控制分配
// 实现流程：5DOF 输入 -> 静态矩阵 -> 6 个中间量 -> 推力 + 倾转角
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::output_armed_stabilizing()
{
    // 获取电压与高度补偿增益
    const float compensation_gain = thr_lin.get_compensation_gain();

    // 准备 5DOF 控制输入
    float desired[5];

    // 注意：
    // - Fx（前向力）必须来自 get_forward()（由 AP_Motors::set_forward 写入），不是 _pitch_in。
    // - _pitch_in 是俯仰力矩请求（My）。
    float throttle_thrust = get_throttle() * compensation_gain;
    // throttle 在电机库内限定为 [0, 1]
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= 1.0f) {
        throttle_thrust = 1.0f;
        limit.throttle_upper = true;
    }

    // 前向力随油门缩放（与 6DoF 脚本混控行为一致）
    const float forward_thrust = get_forward() * throttle_thrust;

    // 重要符号约定（与 cal_alloc_tri.m 一致）：
    // 推导使用机体系 Z 轴向下为正（NED）。
    // 转子推力指向“上”，因此机体系的 Fz 为负。
    // 所以对常规多旋翼向上油门，期望 Fz 必须为负。
    //
    // 同时 Fx 行定义为 Fx = -(F1*sin(a1)+F2*sin(a2)+F3*sin(a3))，
    // 因此前向正向推力指令需要在这里取负号。
    desired[0] = -forward_thrust;     // Fx（前向力，+x 向前）
    desired[1] = -throttle_thrust;    // Fz（向下为正，所以上推力为负）
    // Mx（滚转力矩）
    desired[2] = (_roll_in + _roll_in_ff) * compensation_gain;
    // My（俯仰力矩）
    desired[3] = (_pitch_in + _pitch_in_ff) * compensation_gain;
    // Mz（偏航力矩）：由 output_to_motors() 中共轴差分推力处理
    desired[4] = 0.0f;

    // 使用伪逆矩阵计算 6 个中间变量
    // _intermediate = [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
    for (int i = 0; i < 6; i++) {
        _intermediate[i] = 0.0f;
        for (int j = 0; j < 5; j++) {
            _intermediate[i] += _alloc_matrix_pinv[i][j] * desired[j];
        }
    }

    // 根据中间变量求解推力与倾转角
    // 电机 1：前右
    float f1_sin = _intermediate[0];
    float f1_cos = _intermediate[1];
    _thrust[0] = sqrtf(f1_sin * f1_sin + f1_cos * f1_cos);
    _tilt_angle[0] = atan2f(f1_sin, f1_cos);

    // 电机 2：后置
    float f2_sin = _intermediate[2];
    float f2_cos = _intermediate[3];
    _thrust[1] = sqrtf(f2_sin * f2_sin + f2_cos * f2_cos);
    _tilt_angle[1] = atan2f(f2_sin, f2_cos);

    // 电机 3：前左
    float f3_sin = _intermediate[4];
    float f3_cos = _intermediate[5];
    _thrust[2] = sqrtf(f3_sin * f3_sin + f3_cos * f3_cos);
    _tilt_angle[2] = atan2f(f3_sin, f3_cos);

    // 施加约束
    // 若 _servo_angle_max == 0，则倾转角不做软件限幅
    const bool clamp_tilt = (_servo_angle_max > 0.0f);
    const float max_angle_rad = clamp_tilt ? radians(_servo_angle_max) : radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX));

    for (int i = 0; i < 3; i++) {
        if (clamp_tilt) {
            _tilt_angle[i] = constrain_float(_tilt_angle[i], -max_angle_rad, max_angle_rad);
        } else {
            // 仍需保持合理范围，避免下游 NaN/溢出
            _tilt_angle[i] = constrain_float(_tilt_angle[i], -radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX)), radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX)));
        }

        // 约束推力 [0, 1]
        _thrust[i] = constrain_float(_thrust[i], 0.0f, 1.0f);

        // 限幅标记
        if (_thrust[i] >= 1.0f) {
            limit.throttle_upper = true;
        }
        if (_thrust[i] <= 0.0f) {
            limit.throttle_lower = true;
        }
    }

    // 若存在电机饱和则做推力缩放
    float max_thrust = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (_thrust[i] > max_thrust) {
            max_thrust = _thrust[i];
        }
    }

    // 超限则缩放
    if (max_thrust > 1.0f) {
        float scale = 1.0f / max_thrust;
        for (int i = 0; i < 3; i++) {
            _thrust[i] *= scale;
        }
        limit.throttle_upper = true;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_to_motors - 发送电机与舵机指令
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // 电机输出最小值
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                    rc_write(i, get_pwm_output_min());
                }
            }
            // 舵机回中
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            break;
        }

        case SpoolState::GROUND_IDLE: {
            // 解锁但未起飞时给电机输出
            float spin_up = actuator_spin_up_to_ground_idle();
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], spin_up);
                    rc_write(i, output_to_pwm(_actuator[i]));
                }
            }
            // 舵机回中
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            break;
        }

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN: {
            // 根据推力请求设置电机输出
            // 对共轴对施加偏航差分
            
            // 计算偏航差分推力
            float yaw_thrust = (_yaw_in + _yaw_in_ff) * _yaw_torque_factor * float(_yaw_dir.get());
            yaw_thrust = constrain_float(yaw_thrust, -0.5f, 0.5f);

            // 前右共轴对（电机 0-1）
            apply_coaxial_yaw(AP_MOTORS_MOT_1, AP_MOTORS_MOT_2, _thrust[0], yaw_thrust);

            // 后置共轴对（电机 2-3）
            apply_coaxial_yaw(AP_MOTORS_MOT_3, AP_MOTORS_MOT_4, _thrust[1], yaw_thrust);

            // 前左共轴对（电机 4-5）
            apply_coaxial_yaw(AP_MOTORS_MOT_5, AP_MOTORS_MOT_6, _thrust[2], yaw_thrust);

            // 输出倾转舵机角度（厘度）
            // 若 _servo_angle_max == 0，则无软件限幅（仍受 SERVOx_MIN/MAX 限制）。
            const float lim_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
            const int16_t fr_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[0]), -lim_deg, lim_deg) * 100);
            const int16_t fl_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[2]), -lim_deg, lim_deg) * 100);
            const int16_t rear_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[1]), -lim_deg, lim_deg) * 100);

            // 若已配置 SRV_Channels::set_angle()，scaled 输出使用厘度
            const bool fr_rev = (_tilt_servo_fr_rev.get() != 0);
            const bool rear_rev = (_tilt_servo_rear_rev.get() != 0);
            const bool fl_rev = (_tilt_servo_fl_rev.get() != 0);

            const int16_t fr_out_cd = fr_rev ? -fr_angle_cd : fr_angle_cd;
            const int16_t rear_out_cd = rear_rev ? -rear_angle_cd : rear_angle_cd;
            const int16_t fl_out_cd = fl_rev ? -fl_angle_cd : fl_angle_cd;

            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, fr_out_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, rear_out_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, fl_out_cd);
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 输出测试序列
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // 将测试序号映射到实际电机号
    uint8_t motor_num;
    switch (motor_seq) {
        case 1:
            motor_num = AP_MOTORS_MOT_1;
            break;
        case 2:
            motor_num = AP_MOTORS_MOT_2;
            break;
        case 3:
            motor_num = AP_MOTORS_MOT_3;
            break;
        case 4:
            motor_num = AP_MOTORS_MOT_4;
            break;
        case 5:
            motor_num = AP_MOTORS_MOT_5;
            break;
        case 6:
            motor_num = AP_MOTORS_MOT_6;
            break;
        default:
            return;
    }

    // 输出 PWM 到电机
    if (motor_enabled[motor_num]) {
        rc_write(motor_num, pwm);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 通过共轴差分推力施加偏航力矩
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::apply_coaxial_yaw(uint8_t motor_upper, uint8_t motor_lower, 
                                           float base_thrust, float yaw_thrust)
{
    // 上电机为 CW、下电机为 CCW
    // 正偏航（机体顺时针）时，增加 CCW 电机、减少 CW 电机
    float thrust_upper = base_thrust - yaw_thrust;
    float thrust_lower = base_thrust + yaw_thrust;

    // 约束到有效范围 [0, 1]
    thrust_upper = constrain_float(thrust_upper, 0.0f, 1.0f);
    thrust_lower = constrain_float(thrust_lower, 0.0f, 1.0f);

    // 推力转换为执行器量
    float actuator_upper = thr_lin.thrust_to_actuator(thrust_upper);
    float actuator_lower = thr_lin.thrust_to_actuator(thrust_lower);

    // 输出到电机
    rc_write(motor_upper, output_to_pwm(actuator_upper));
    rc_write(motor_lower, output_to_pwm(actuator_lower));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 计算静态分配矩阵 F_alloc[5][6]
// 基于 tricopter_allocation/cal_alloc_tri.m 的 MATLAB 推导
// 将 [Fx, Fz, Mx, My, Mz] 映射为 [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::calculate_allocation_matrix()
{
    float lx = _lfront_x;
    float ly = _lfront_y;
    float lr = _lrear;

    // 第 0 行：Fx（前向力）
    // Fx = -(F1*sin(a1) + F2*sin(a2) + F3*sin(a3))
    _alloc_matrix[0][0] = -1.0f;  // F1*sin(a1)
    _alloc_matrix[0][1] = 0.0f;   // F1*cos(a1)
    _alloc_matrix[0][2] = -1.0f;  // F2*sin(a2)
    _alloc_matrix[0][3] = 0.0f;   // F2*cos(a2)
    _alloc_matrix[0][4] = -1.0f;  // F3*sin(a3)
    _alloc_matrix[0][5] = 0.0f;   // F3*cos(a3)

    // 第 1 行：Fz（垂向力/油门）
    // Fz = -(F1*cos(a1) + F2*cos(a2) + F3*cos(a3))
    _alloc_matrix[1][0] = 0.0f;   // F1*sin(a1)
    _alloc_matrix[1][1] = -1.0f;  // F1*cos(a1)
    _alloc_matrix[1][2] = 0.0f;   // F2*sin(a2)
    _alloc_matrix[1][3] = -1.0f;  // F2*cos(a2)
    _alloc_matrix[1][4] = 0.0f;   // F3*sin(a3)
    _alloc_matrix[1][5] = -1.0f;  // F3*cos(a3)

    // 第 2 行：Mx（滚转力矩）
    // Mx = -ly*F1*cos(a1) + ly*F3*cos(a3)
    _alloc_matrix[2][0] = 0.0f;       // F1*sin(a1)
    _alloc_matrix[2][1] = -ly;        // F1*cos(a1)
    _alloc_matrix[2][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[2][3] = 0.0f;       // F2*cos(a2)
    _alloc_matrix[2][4] = 0.0f;       // F3*sin(a3)
    _alloc_matrix[2][5] = ly;         // F3*cos(a3)

    // 第 3 行：My（俯仰力矩）
    // My = lx*F1*cos(a1) - lr*F2*cos(a2) - lx*F3*cos(a3)
    _alloc_matrix[3][0] = 0.0f;       // F1*sin(a1)
    _alloc_matrix[3][1] = lx;         // F1*cos(a1)
    _alloc_matrix[3][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[3][3] = -lr;        // F2*cos(a2)
    _alloc_matrix[3][4] = 0.0f;       // F3*sin(a3)
    _alloc_matrix[3][5] = -lx;        // F3*cos(a3)

    // 第 4 行：Mz（偏航力矩）
    // Mz = ly*F1*sin(a1) - ly*F3*sin(a3)
    _alloc_matrix[4][0] = ly;         // F1*sin(a1)
    _alloc_matrix[4][1] = 0.0f;       // F1*cos(a1)
    _alloc_matrix[4][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[4][3] = 0.0f;       // F2*cos(a2)
    _alloc_matrix[4][4] = -ly;        // F3*sin(a3)
    _alloc_matrix[4][5] = 0.0f;       // F3*cos(a3)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 计算分配矩阵的伪逆
// 基于 MATLAB：simplify(pinv(F_alloc))
// 将 [Fx, Fz, Mx, My, Mz] 映射为 [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::calculate_allocation_matrix_pinv()
{
    float lx = _lfront_x;
    float ly = _lfront_y;
    float lr = _lrear;

    // 防止除零
    if (fabsf(ly) < 0.01f || fabsf(lr) < 0.01f) {
        // 参数无效时使用安全默认值
        ly = 0.5f;
        lr = 1.0f;
    }

    // 第 0 行：F1*sin(a1)
    _alloc_matrix_pinv[0][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[0][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[0][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[0][3] = 0.0f;                    // My
    _alloc_matrix_pinv[0][4] = 1.0f/(2.0f*ly);          // Mz

    // 第 1 行：F1*cos(a1)
    _alloc_matrix_pinv[1][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[1][1] = -1.0f/2.0f;              // Fz
    _alloc_matrix_pinv[1][2] = (lx - lr)/(2.0f*ly*lr);  // Mx
    _alloc_matrix_pinv[1][3] = 1.0f/(2.0f*lr);          // My
    _alloc_matrix_pinv[1][4] = 0.0f;                    // Mz

    // 第 2 行：F2*sin(a2)
    _alloc_matrix_pinv[2][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[2][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[2][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[2][3] = 0.0f;                    // My
    _alloc_matrix_pinv[2][4] = 0.0f;                    // Mz

    // 第 3 行：F2*cos(a2)
    _alloc_matrix_pinv[3][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[3][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[3][2] = -lx/(ly*lr);             // Mx
    _alloc_matrix_pinv[3][3] = -1.0f/lr;                // My
    _alloc_matrix_pinv[3][4] = 0.0f;                    // Mz

    // 第 4 行：F3*sin(a3)
    _alloc_matrix_pinv[4][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[4][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[4][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[4][3] = 0.0f;                    // My
    _alloc_matrix_pinv[4][4] = -1.0f/(2.0f*ly);         // Mz

    // 第 5 行：F3*cos(a3)
    _alloc_matrix_pinv[5][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[5][1] = -1.0f/2.0f;              // Fz
    _alloc_matrix_pinv[5][2] = (lx + lr)/(2.0f*ly*lr);  // Mx
    _alloc_matrix_pinv[5][3] = 1.0f/(2.0f*lr);          // My
    _alloc_matrix_pinv[5][4] = 0.0f;                    // Mz
}

#endif  // AP_MOTORS_TRI_TILT_ENABLED
