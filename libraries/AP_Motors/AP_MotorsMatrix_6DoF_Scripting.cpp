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

#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_MotorsMatrix_6DoF_Scripting::output_to_motors()
{
    // 根据当前的马达状态执行不同的输出逻辑
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: // 状态：关闭
        case SpoolState::GROUND_IDLE: // 状态：地面待机
        {
            // 在关闭或待机状态下，不输出任何信号，因为我们不知道马达应该如何旋转
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) { // 检查当前马达是否启用
                    _actuator[i] = 0.0f; // 设置马达输出为0
                }
            }
            break; // 退出当前状态处理
        }
        case SpoolState::SPOOLING_UP: // 状态：加速
        case SpoolState::THROTTLE_UNLIMITED: // 状态：油门无限制
        case SpoolState::SPOOLING_DOWN: // 状态：减速
            // 根据推力请求设置马达输出
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) { // 检查当前马达是否启用
                    if (_reversible[i]) { // 检查马达是否可逆
                        // 可逆马达可以提供正负推力，最大旋转和最小旋转不适用
                        if (is_positive(_thrust_rpyt_out[i])) { 
                            // 如果推力为正，应用推力曲线和电压缩放，并乘以最大旋转值
                            _actuator[i] = thr_lin.apply_thrust_curve_and_volt_scaling(_thrust_rpyt_out[i]) * thr_lin.get_spin_max();
                        } else if (is_negative(_thrust_rpyt_out[i])) {
                            // 如果推力为负，应用推力曲线和电压缩放，并乘以最大旋转值（取反）
                            _actuator[i] = -thr_lin.apply_thrust_curve_and_volt_scaling(-_thrust_rpyt_out[i]) * thr_lin.get_spin_max();
                        } else {
                            // 如果推力为零，设置输出为0
                            _actuator[i] = 0.0f;
                        }
                    } else {
                        // 非可逆马达只能提供单向推力，正常飞行器的旋转范围
                        _actuator[i] = thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]);
                    }
                }
            }
            break; // 退出当前状态处理
    }

    // 将计算的输出发送到每个马达
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) { // 检查当前马达是否启用
            // 将马达的输出值乘以4500并发送到相应的马达输出通道
            SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _actuator[i] * 4500);
        }
    }
}

// output_armed - sends commands to the motors
void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()
{
    uint8_t i;                          // 通用计数器
    float   roll_thrust;                // 滚转推力输入值，范围 ±1.0
    float   pitch_thrust;               // 俯仰推力输入值，范围 ±1.0
    float   yaw_thrust;                 // 偏航推力输入值，范围 ±1.0
    float   throttle_thrust;            // 油门推力输入值，范围 0.0 - 1.0
    float   forward_thrust;             // 前进推力输入值，范围 ±1.0
    float   right_thrust;               // 右侧推力输入值，范围 ±1.0

    // 注意，油门、前进和右侧输入不是在体坐标系中，而是在我们假装的“正常”4自由度多旋翼的坐标系中

    // 应用电压和气压补偿
    const float compensation_gain = thr_lin.get_compensation_gain(); // 电池电压和海拔高度的补偿
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;      // 计算滚转推力
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;   // 计算俯仰推力
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;         // 计算偏航推力
    throttle_thrust = get_throttle() * compensation_gain;            // 计算油门推力

    // 用油门缩放水平推力，这模仿了一个正常的多旋翼
    // 这样我们就不会破坏位置控制器假设的倾斜角度比例加速度
    forward_thrust = get_forward() * throttle_thrust;                // 计算前进推力
    right_thrust = get_lateral() * throttle_thrust;                  // 计算右侧推力

    // 设置油门限制标志
    if (throttle_thrust <= 0) {
        throttle_thrust = 0;
        // 我们不能向下推力，尽管车辆可以这样做，但这会破坏控制堆栈中更高层次的许多假设
        // 1G下降可能已经足够了......
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= 1) {
        throttle_thrust = 1;
        limit.throttle_upper = true;
    }

    // 将推力旋转到体坐标系
    Matrix3f rot;  // 定义一个3x3旋转矩阵
    Vector3f thrust_vec;  // 定义一个3维推力向量

    // 使用欧拉角（roll, pitch, yaw）创建旋转矩阵
    rot.from_euler312(_roll_offset, _pitch_offset, 0.0f);  // 使用滚转角和俯仰角创建旋转矩阵，偏航角为0

    /*
        向上的推力，独立于方向
    */
    thrust_vec.x = 0.0f;  // 推力向量的x分量为0
    thrust_vec.y = 0.0f;  // 推力向量的y分量为0
    thrust_vec.z = throttle_thrust;  // 推力向量的z分量为油门推力

    // 将推力向量从世界坐标系旋转到体坐标系
    thrust_vec = rot * thrust_vec;

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // 计算每个电机的推力
            _thrust_rpyt_out[i] =  thrust_vec.x * _forward_factor[i];  // 前进分量
            _thrust_rpyt_out[i] += thrust_vec.y * _right_factor[i];    // 右侧分量
            _thrust_rpyt_out[i] += thrust_vec.z * _throttle_factor[i]; // 油门分量

            if (fabsf(_thrust_rpyt_out[i]) >= 1) {
                // 如果推力超过1，可能是混合器缩放不正确
                limit.throttle_upper = true;
            }
            // 约束推力在-1.0到1.0之间
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], -1.0f, 1.0f);
        }
    }

    /*
        旋转：滚转、俯仰和偏航
    */
    float rpy_ratio = 1.0f;  // 缩放因子，输出将按此比例缩放，以便所有值都能均匀分布
    float thrust[AP_MOTORS_MAX_NUM_MOTORS];  // 定义一个数组来存储每个电机的推力

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // 计算每个电机的推力
            thrust[i] =  roll_thrust * _roll_factor[i];  // 滚转分量
            thrust[i] += pitch_thrust * _pitch_factor[i];  // 俯仰分量
            thrust[i] += yaw_thrust * _yaw_factor[i];  // 偏航分量

            float total_thrust = _thrust_rpyt_out[i] + thrust[i];  // 计算总推力

            // 控制输入将受到电机范围的限制
            if (total_thrust > 1.0f) {
                rpy_ratio = MIN(rpy_ratio, (1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                rpy_ratio = MIN(rpy_ratio, (-1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    // 如果输出被缩放，设置限制标志
    if (rpy_ratio < 1) {
        limit.roll = true;  // 设置滚转限制标志
        limit.pitch = true; // 设置俯仰限制标志
        limit.yaw = true;   // 设置偏航限制标志
    }

    // 均匀缩放旋转，使其适合
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // 计算并约束每个电机的推力
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * rpy_ratio, -1.0f, 1.0f);
        }
    }

    /*
        前进和横向推力，独立于方向
    */
    thrust_vec.x = forward_thrust;  // 推力向量的x分量为前进推力
    thrust_vec.y = right_thrust;    // 推力向量的y分量为右侧推力
    thrust_vec.z = 0.0f;            // 推力向量的z分量为0

    // 将推力向量从世界坐标系旋转到体坐标系
    thrust_vec = rot * thrust_vec;

    float horz_ratio = 1.0f;  // 水平推力的缩放因子，输出将按此比例缩放，以便所有值都能均匀分布

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // 计算每个电机的推力
            thrust[i] =  thrust_vec.x * _forward_factor[i];  // 前进分量
            thrust[i] += thrust_vec.y * _right_factor[i];    // 右侧分量
            thrust[i] += thrust_vec.z * _throttle_factor[i]; // 油门分量（这里为0）

            float total_thrust = _thrust_rpyt_out[i] + thrust[i];  // 计算总推力

            // 控制输入将受到电机范围的限制
            if (total_thrust > 1.0f) {
                horz_ratio = MIN(horz_ratio, (1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                horz_ratio = MIN(horz_ratio, (-1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    // 均匀缩放，使其适合
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {  // 遍历所有电机
        if (motor_enabled[i]) {  // 检查当前电机是否启用
            // 计算并约束每个电机的推力
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * horz_ratio, -1.0f, 1.0f);
        }
    }

    /*
        对可逆电机应用死区，这可以防止电机频繁改变方向
        重用偏航余量参数作为死区，限制最大为25%
    */
    const float deadzone = constrain_float(_yaw_headroom.get() * 0.001f, 0.0f, 0.25f);  // 计算死区值，范围在0.0到0.25之间

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {  // 遍历所有电机
        if (motor_enabled[i] && _reversible[i]) {  // 检查当前电机是否启用且是可逆电机
            if (is_negative(_thrust_rpyt_out[i])) {  // 检查当前电机的推力是否为负
                if ((_thrust_rpyt_out[i] > -deadzone) && is_positive(_last_thrust_out[i])) {  // 如果推力值大于负死区且上一次推力值为正
                    _thrust_rpyt_out[i] = 0.0f;  // 将推力值设为0
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];  // 更新上一次推力值
                }
            } else if (is_positive(_thrust_rpyt_out[i])) {  // 检查当前电机的推力是否为正
                if ((_thrust_rpyt_out[i] < deadzone) && is_negative(_last_thrust_out[i])) {  // 如果推力值小于死区且上一次推力值为负
                    _thrust_rpyt_out[i] = 0.0f;  // 将推力值设为0
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];  // 更新上一次推力值
                }
            }
        }
    }

}

// 设置滚转和俯仰偏移，这将在体坐标系中旋转推力向量
// 这些偏移通常设置为使油门推力向量指向地球坐标系的上方
void AP_MotorsMatrix_6DoF_Scripting::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset = radians(roll_deg);  // 将滚转角度从度转换为弧度，并赋值给滚转偏移
    _pitch_offset = radians(pitch_deg);  // 将俯仰角度从度转换为弧度，并赋值给俯仰偏移
}

// add_motor, take roll, pitch, yaw, throttle(up), forward, right factors along with a bool if the motor is reversible and the testing order, called from scripting
void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order)
{
    if (initialised_ok()) {
        // don't allow matrix to be changed after init
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        motor_enabled[motor_num] = true;

        _roll_factor[motor_num] = roll_factor;
        _pitch_factor[motor_num] = pitch_factor;
        _yaw_factor[motor_num] = yaw_factor;

        _throttle_factor[motor_num] = throttle_factor;
        _forward_factor[motor_num] = forward_factor;
        _right_factor[motor_num] = right_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // ensure valid motor number is provided
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);

        uint8_t chan;
        if (!SRV_Channels::find_channel(function, chan)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
            return;
        }

        _reversible[motor_num] = reversible;
        if (_reversible[motor_num]) {
            // reversible, set to angle type hard code trim to 1500
            SRV_Channels::set_angle(function, 4500);
            SRV_Channels::set_trim_to_pwm_for(function, 1500);
        } else {
            SRV_Channels::set_range(function, 4500);
        }
        SRV_Channels::set_output_min_max(function, get_pwm_output_min(), get_pwm_output_max());
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::init(uint8_t expected_num_motors) {
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    return true;
}

// singleton instance
AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::_singleton;

#endif // AP_SCRIPTING_ENABLED
