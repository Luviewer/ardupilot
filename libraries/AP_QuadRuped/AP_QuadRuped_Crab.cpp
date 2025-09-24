#include "AP_QuadRuped_Crab.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 腿部分组定义
const uint8_t AP_QuadRuped_Crab::LATERAL_GROUP_LEFT[2] = {1, 3};  // LF + LB
const uint8_t AP_QuadRuped_Crab::LATERAL_GROUP_RIGHT[2] = {0, 2}; // RF + RB

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_Crab::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("STEP_H", 1, AP_QuadRuped_Crab, _step_height, 20.0f),    // 抬腿高度 20mm
    AP_GROUPINFO("STEP_L", 2, AP_QuadRuped_Crab, _step_length, 60.0f),     // 步长 60mm
    AP_GROUPINFO("STEP_F", 3, AP_QuadRuped_Crab, _step_frequency, 1.0f),   // 步频 1.0Hz
    AP_GROUPINFO("CRAB_S", 4, AP_QuadRuped_Crab, _crab_stride, 40.0f),     // 工字步幅 40mm
    AP_GROUPINFO("LAT_S", 5, AP_QuadRuped_Crab, _lateral_spread, 30.0f),  // 横向展开 30mm
    AP_GROUPINFO("TURN_R", 6, AP_QuadRuped_Crab, _turning_radius, 100.0f), // 转弯半径 100mm

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Crab::AP_QuadRuped_Crab(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, ahrs, motors)
    , _crab_phase(0)
    , _turning_mode(false)
    , _turn_angle(0.0f)
    , _last_update_time(0)
{
}

// 初始化
bool AP_QuadRuped_Crab::init() {
    // 初始化步态
    gait_init();

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);

    // 初始化计时
    _last_update_time = AP_HAL::millis();

    // 设置初始状态
    _crab_phase = 0;
    _turning_mode = false;

    _initialized = true;
    return true;
}

// 步态初始化
void AP_QuadRuped_Crab::gait_init() {
    // 初始化腿部位置为工字步态站立姿态
    for (uint8_t i = 0; i < 4; i++) {
        Vector3f lateral_pos = calculate_lateral_displacement(i, 0);
        _leg_positions[i] = Vector3f(lateral_pos.x, lateral_pos.y, -_frontend.get_body_height());
    }

    // 重置工字相位
    _crab_phase = 0;
    _turn_angle = 0.0f;
}

// 更新工字计时
void AP_QuadRuped_Crab::update_crab_timing() {
    if (_step_frequency > 0.1f) {
        _gait_cycle_time = 1.0f / _step_frequency;
    } else {
        _gait_cycle_time = 1.0f;
    }
}

// 主更新循环
void AP_QuadRuped_Crab::update() {
    if (!_initialized) {
        return;
    }

    // 更新控制输入
    update_control_inputs();

    // 检测转弯模式
    _turning_mode = fabsf(_yaw_rate) > 0.1f;

    // 更新步态计时
    uint32_t current_time = AP_HAL::millis();
    float dt = (current_time - _last_update_time) * 0.001f; // 转换为秒
    _last_update_time = current_time;

    // 更新工字相位
    update_crab_phase();

    // 计算步态序列
    calc_gait_sequence();

    // 更新每条腿的位置
    for (uint8_t i = 0; i < 4; i++) {
        trajectory_generation(i);
    }

    // 应用转弯控制
    if (_turning_mode) {
        apply_turning_control();
    }

    // 应用横向运动
    calculate_lateral_movement();

    // 计算逆运动学并输出
    for (uint8_t i = 0; i < 4; i++) {
        _leg_angles[i] = leg_inverse_kinematics(_leg_positions[i]);
    }

    // 输出到电机
    output_leg_angle();
}

// 更新工字相位
void AP_QuadRuped_Crab::update_crab_phase() {
    // 工字步态：同侧腿同步，左右侧交替
    float phase_increment = (_step_frequency * 2.0f) * 0.01f; // 假设100Hz更新率
    _crab_phase = (_crab_phase + (uint8_t)phase_increment) % 4;
}

// 计算步态序列
void AP_QuadRuped_Crab::calc_gait_sequence() {
    // 工字步态序列：
    // 相位0: 右侧腿 (RF+RB) 抬腿，左侧腿 (LF+LB) 支撑
    // 相位1: 过渡状态
    // 相位2: 左侧腿 (LF+LB) 抬腿，右侧腿 (RF+RB) 支撑
    // 相位3: 过渡状态

    for (uint8_t i = 0; i < 4; i++) {
        float phase = get_leg_crab_phase(i);

        if (phase < 0.7f) { // 70%时间支撑
            _legs_in_air[i] = false;
        } else { // 30%时间摆动
            _legs_in_air[i] = true;
        }
    }
}

// 获取腿部工字相位
float AP_QuadRuped_Crab::get_leg_crab_phase(uint8_t leg_index) const {
    // 根据同侧分组确定相位
    switch (leg_index) {
        case 0: // RF - 右侧
        case 2: // RB - 右侧
            return (_crab_phase % 2 == 0) ? 0.0f : 0.5f;
        case 1: // LF - 左侧
        case 3: // LB - 左侧
            return (_crab_phase % 2 == 0) ? 0.5f : 0.0f;
        default:
            return 0.0f;
    }
}

// 判断腿部是否在支撑状态
bool AP_QuadRuped_Crab::is_leg_in_crab_support(uint8_t leg_index) const {
    return !_legs_in_air[leg_index];
}

// 轨迹生成
void AP_QuadRuped_Crab::trajectory_generation(uint8_t leg_index) {
    float phase = get_leg_crab_phase(leg_index);

    if (_turning_mode) {
        // 转弯模式
        _leg_positions[leg_index] = calculate_turning_trajectory(leg_index, phase);
    } else {
        // 正常工字步态
        _leg_positions[leg_index] = calculate_crab_trajectory(leg_index, phase);
    }
}

// 计算工字轨迹
Vector3f AP_QuadRuped_Crab::calculate_crab_trajectory(uint8_t leg_index, float phase) {
    Vector3f position;
    Vector3f lateral_displacement = calculate_lateral_displacement(leg_index, phase);

    if (is_leg_in_crab_support(leg_index)) {
        // 支撑相：保持稳定支撑
        position.x = lateral_displacement.x + _throttle_x * _step_length * 0.3f;
        position.y = lateral_displacement.y + _throttle_y * _step_length * 0.3f;
        position.z = -_frontend.get_body_height();
    } else {
        // 摆动相：按照工字模式移动
        float swing_phase = (phase - 0.7f) / 0.3f; // 归一化摆动相位
        swing_phase = constrain_float(swing_phase, 0.0f, 1.0f);

        // 使用正弦函数生成平滑轨迹
        float height_factor = sinf(swing_phase * M_PI);
        float forward_factor = swing_phase;

        // 工字运动：先横向移动，再纵向移动
        if (swing_phase < 0.5f) {
            // 前半段：横向移动
            position.x = lateral_displacement.x;
            position.y = lateral_displacement.y + _throttle_y * _crab_stride * (swing_phase * 2.0f - 0.5f);
        } else {
            // 后半段：纵向移动
            position.x = lateral_displacement.x + _throttle_x * _crab_stride * ((swing_phase - 0.5f) * 2.0f - 0.5f);
            position.y = lateral_displacement.y;
        }

        position.z = -_frontend.get_body_height() + _step_height * height_factor;
    }

    return position;
}

// 计算转弯轨迹
Vector3f AP_QuadRuped_Crab::calculate_turning_trajectory(uint8_t leg_index, float phase) {
    Vector3f position;
    Vector3f lateral_displacement = calculate_lateral_displacement(leg_index, phase);

    if (is_leg_in_crab_support(leg_index)) {
        // 支撑相：保持原地不动，为转弯提供支撑
        position.x = lateral_displacement.x;
        position.y = lateral_displacement.y;
        position.z = -_frontend.get_body_height();
    } else {
        // 摆动相：沿转弯轨迹移动
        float swing_phase = (phase - 0.7f) / 0.3f;
        swing_phase = constrain_float(swing_phase, 0.0f, 1.0f);

        // 计算转弯位移
        float turn_displacement = calculate_turn_displacement(leg_index, swing_phase);

        // 应用转弯轨迹
        position.x = lateral_displacement.x + turn_displacement * cosf(_turn_angle);
        position.y = lateral_displacement.y + turn_displacement * sinf(_turn_angle);
        position.z = -_frontend.get_body_height() + _step_height * sinf(swing_phase * M_PI);
    }

    return position;
}

// 计算横向位移
Vector2f AP_QuadRuped_Crab::calculate_lateral_displacement(uint8_t leg_index, float phase) {
    Vector2f displacement;

    // 根据腿的位置确定横向位移
    switch (leg_index) {
        case 0: // RF
            displacement.x = _lateral_spread * 0.5f;
            displacement.y = _lateral_spread * 0.5f;
            break;
        case 1: // LF
            displacement.x = -_lateral_spread * 0.5f;
            displacement.y = _lateral_spread * 0.5f;
            break;
        case 2: // RB
            displacement.x = _lateral_spread * 0.5f;
            displacement.y = -_lateral_spread * 0.5f;
            break;
        case 3: // LB
            displacement.x = -_lateral_spread * 0.5f;
            displacement.y = -_lateral_spread * 0.5f;
            break;
    }

    return displacement;
}

// 计算转弯位移
float AP_QuadRuped_Crab::calculate_turn_displacement(uint8_t leg_index, float phase) {
    // 根据转弯方向和腿部位置计算位移
    float turn_direction = (_yaw_rate > 0) ? 1.0f : -1.0f;
    float base_displacement = _turning_radius * 0.2f * turn_direction;

    // 不同腿在转弯时有不同的位移模式
    switch (leg_index) {
        case 0: // RF
        case 3: // LB
            return base_displacement * phase;
        case 1: // LF
        case 2: // RB
            return -base_displacement * phase;
        default:
            return 0.0f;
    }
}

// 偏航轨迹生成
void AP_QuadRuped_Crab::yaw_trajectory_generation(uint8_t leg_index) {
    // 工字步态的偏航控制通过特殊的腿部移动实现
    if (_turning_mode) {
        float turn_increment = _yaw_rate * 0.01f; // 假设100Hz更新率
        _turn_angle += turn_increment;
    }
}

// 应用转弯控制
void AP_QuadRuped_Crab::apply_turning_control() {
    // 在转弯模式下，调整所有腿部位置以保持平衡
    Vector3f center_of_mass(0, 0, 0);
    uint8_t support_leg_count = 0;

    for (uint8_t i = 0; i < 4; i++) {
        if (!_legs_in_air[i]) {
            center_of_mass += _leg_positions[i];
            support_leg_count++;
        }
    }

    if (support_leg_count > 0) {
        center_of_mass /= support_leg_count;

        // 应用转弯补偿
        for (uint8_t i = 0; i < 4; i++) {
            if (!_legs_in_air[i]) {
                Vector3f turn_compensation = Vector3f(-center_of_mass.y * 0.1f * _yaw_rate,
                                                     center_of_mass.x * 0.1f * _yaw_rate,
                                                     0);
                _leg_positions[i] += turn_compensation;
            }
        }
    }
}

// 应用横向运动
void AP_QuadRuped_Crab::calculate_lateral_movement() {
    // 工字步态特有的横向运动
    for (uint8_t i = 0; i < 4; i++) {
        Vector2f lateral_offset = calculate_lateral_displacement(i, 0);
        _leg_positions[i].x += lateral_offset.x * 0.1f * _throttle_y;
        _leg_positions[i].y += lateral_offset.y * 0.1f * _throttle_x;
    }
}

// 健康状态检查
bool AP_QuadRuped_Crab::healthy() const {
    // 检查参数是否在合理范围内
    if (_step_height <= 0 || _step_height > 50) return false;
    if (_step_length <= 0 || _step_length > 100) return false;
    if (_step_frequency <= 0 || _step_frequency > 3) return false;
    if (_crab_stride <= 0 || _crab_stride > 80) return false;
    if (_lateral_spread <= 0 || _lateral_spread > 100) return false;
    if (_turning_radius <= 0 || _turning_radius > 200) return false;

    return _initialized;
}