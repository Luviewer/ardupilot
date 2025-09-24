#include "AP_QuadRuped_Diag_New.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 腿部分组定义
const uint8_t AP_QuadRuped_Diag::DIAGONAL_GROUP_0[2] = { 0, 2 }; // RF + LB
const uint8_t AP_QuadRuped_Diag::DIAGONAL_GROUP_1[2] = { 1, 3 }; // LF + RB

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_Diag::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("STEP_H", 1, AP_QuadRuped_Diag, _step_height, 30.0f),      // 抬腿高度 30mm
    AP_GROUPINFO("STEP_L", 2, AP_QuadRuped_Diag, _step_length, 100.0f),     // 步长 100mm
    AP_GROUPINFO("STEP_F", 3, AP_QuadRuped_Diag, _step_frequency, 2.0f),    // 步频 2Hz
    AP_GROUPINFO("DUTY", 4, AP_QuadRuped_Diag, _duty_factor, 0.6f),         // 占空比 0.6
    AP_GROUPINFO("STAB_M", 5, AP_QuadRuped_Diag, _stability_margin, 50.0f), // 稳定性边距 50mm

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Diag::AP_QuadRuped_Diag(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, ahrs, motors)
    , _diag_phase(0)
    , _gait_cycle_time(0.0f)
    , _last_update_time(0)
{
    // 初始化腿部状态
    for (uint8_t i = 0; i < 4; i++) {
        _legs_in_air[i] = false;
    }
}

// 初始化
bool AP_QuadRuped_Diag::init()
{
    // 初始化步态
    gait_init();

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);

    // 初始化计时
    _last_update_time = AP_HAL::millis();

    _initialized = true;
    return true;
}

// 步态初始化
void AP_QuadRuped_Diag::gait_init()
{
    // // 初始化腿部位置为默认站立姿态
    // for (uint8_t i = 0; i < 4; i++) {
    //     _leg_positions[i] = Vector3f(0, 0, -_frontend.get_body_height());
    // }

    // // 计算步态周期时间
    // update_gait_timing();

    // // 初始化对角相位
    // _diag_phase = 0;

    // 设置每条腿的起始步数
    // 对角步态：左前右后同时抬起，右前左后同时抬起
    gait_step_leg_start[Leg_RF] = 0;                   // 右前腿从第0步开始
    gait_step_leg_start[Leg_RB] = gait_step_total / 2; // 右后腿从中间步开始
    gait_step_leg_start[Leg_LB] = 0;                   // 左后腿从第0步开始
    gait_step_leg_start[Leg_LF] = gait_step_total / 2; // 左前腿从中间步开始

    // 设置步态参数
    gait_travel_divisor = gait_step_total / 2; // 行程除数
    gait_lift_divisor   = 2;                   // 抬腿除数
}

// 更新步态时间
void AP_QuadRuped_Diag::update_gait_timing()
{
    if (_step_frequency > 0.1f) {
        _gait_cycle_time = 1.0f / _step_frequency;
    } else {
        _gait_cycle_time = 1.0f;
    }
}

// 主更新循环
void AP_QuadRuped_Diag::update()
{
    if (!_initialized) {
        return;
    }

    // 更新控制输入
    update_control_inputs();

    // 更新步态计时
    uint32_t current_time = AP_HAL::millis();
    float    dt           = (current_time - _last_update_time) * 0.001f; // 转换为秒
    _last_update_time     = current_time;

    // 更新对角相位
    update_diagonal_phase();

    // 计算步态序列
    calc_gait_sequence();

    // 更新每条腿的位置
    for (uint8_t i = 0; i < 4; i++) {
        trajectory_generation(i);
    }

    // 应用稳定性控制
    apply_stability_control();

    // 计算逆运动学并输出
    for (uint8_t i = 0; i < 4; i++) {
        _leg_angles[i] = leg_inverse_kinematics(_leg_positions[i]);
    }

    // 输出到电机
    output_leg_angle();
}

// 更新对角相位
void AP_QuadRuped_Diag::update_diagonal_phase()
{
    // 基于时间更新相位
    float phase_increment = (_step_frequency * 4.0f) * 0.01f; // 假设100Hz更新率
    _diag_phase           = (_diag_phase + (uint8_t)phase_increment) % 4;
}

// 计算步态序列
void AP_QuadRuped_Diag::calc_gait_sequence()
{
    // 对角步态序列：
    // 相位0: 对角线0 (RF+LB) 抬腿，对角线1 (LF+RB) 支撑
    // 相位1: 过渡状态
    // 相位2: 对角线1 (LF+RB) 抬腿，对角线0 (RF+LB) 支撑
    // 相位3: 过渡状态

    for (uint8_t i = 0; i < 4; i++) {
        float phase = get_leg_phase(i);

        if (phase < _duty_factor) {
            _legs_in_air[i] = false; // 支撑相
        } else {
            _legs_in_air[i] = true; // 摆动相
        }
    }
}

// 获取腿部相位
float AP_QuadRuped_Diag::get_leg_phase(uint8_t leg_index) const
{
    // 根据对角线分组确定相位
    switch (leg_index) {
        case 0: // RF - 对角线0
        case 2: // LB - 对角线0
            return (_diag_phase % 2 == 0) ? 0.0f : 0.5f;
        case 1: // LF - 对角线1
        case 3: // RB - 对角线1
            return (_diag_phase % 2 == 0) ? 0.5f : 0.0f;
        default:
            return 0.0f;
    }
}

// 轨迹生成
void AP_QuadRuped_Diag::trajectory_generation(uint8_t leg_index)
{
    float phase = get_leg_phase(leg_index);

    if (_legs_in_air[leg_index]) {
        // 摆动相 - 计算抬腿轨迹
        _leg_positions[leg_index] = calculate_swing_trajectory(leg_index, phase);
    } else {
        // 支撑相 - 计算支撑轨迹
        _leg_positions[leg_index] = calculate_stance_trajectory(leg_index, phase);
    }
}

// 计算支撑轨迹
Vector3f AP_QuadRuped_Diag::calculate_stance_trajectory(uint8_t leg_index, float phase)
{
    Vector3f position;

    // 基础站立位置
    position.x = 0;
    position.y = 0;
    position.z = -_frontend.get_body_height();

    // 根据控制输入调整位置
    position.x += _throttle_x * _step_length * 0.5f;
    position.y += _throttle_y * _step_length * 0.5f;

    return position;
}

// 计算摆动轨迹
Vector3f AP_QuadRuped_Diag::calculate_swing_trajectory(uint8_t leg_index, float phase)
{
    Vector3f position;

    // 计算摆动相位 (归一化到0-1)
    float swing_phase = (phase - _duty_factor) / (1.0f - _duty_factor);
    swing_phase       = constrain_float(swing_phase, 0.0f, 1.0f);

    // 使用正弦函数生成平滑轨迹
    float height_factor = sinf(swing_phase * M_PI);

    // 基础位置
    position.x = _throttle_x * _step_length * (swing_phase - 0.5f);
    position.y = _throttle_y * _step_length * (swing_phase - 0.5f);
    position.z = -_frontend.get_body_height() + _step_height * height_factor;

    return position;
}

// 应用稳定性控制
void AP_QuadRuped_Diag::apply_stability_control()
{
    // 计算重心位置
    Vector3f center_of_mass(0, 0, 0);
    uint8_t  support_leg_count = 0;

    for (uint8_t i = 0; i < 4; i++) {
        if (!_legs_in_air[i]) {
            center_of_mass += _leg_positions[i];
            support_leg_count++;
        }
    }

    if (support_leg_count > 0) {
        center_of_mass /= support_leg_count;

        // 应用稳定性补偿
        for (uint8_t i = 0; i < 4; i++) {
            if (!_legs_in_air[i]) {
                Vector3f stability_offset = center_of_mass * 0.1f; // 10%补偿
                _leg_positions[i] -= stability_offset;
            }
        }
    }
}

// 健康状态检查
bool AP_QuadRuped_Diag::healthy() const
{
    // 检查参数是否在合理范围内
    if (_step_height <= 0 || _step_height > 100) return false;
    if (_step_length <= 0 || _step_length > 200) return false;
    if (_step_frequency <= 0 || _step_frequency > 10) return false;
    if (_duty_factor <= 0.3 || _duty_factor > 0.9) return false;

    return _initialized;
}