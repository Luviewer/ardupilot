#include "AP_QuadRuped_WAVE_New.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 波浪步态序列定义 (RF->LF->RB->LB)
const uint8_t AP_QuadRuped_WAVE::WAVE_SEQUENCE[4] = {0, 3, 1, 2}; // RF->LF->RB->LB

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_WAVE::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("STEP_H", 1, AP_QuadRuped_WAVE, _step_height, 25.0f),    // 抬腿高度 25mm
    AP_GROUPINFO("STEP_L", 2, AP_QuadRuped_WAVE, _step_length, 80.0f),     // 步长 80mm
    AP_GROUPINFO("STEP_F", 3, AP_QuadRuped_WAVE, _step_frequency, 1.5f),   // 步频 1.5Hz
    AP_GROUPINFO("WAVE_O", 4, AP_QuadRuped_WAVE, _wave_overlap, 0.2f),     // 波浪重叠 0.2
    AP_GROUPINFO("BODY_S", 5, AP_QuadRuped_WAVE, _body_sway, 10.0f),       // 机身摆动 10mm

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_WAVE::AP_QuadRuped_WAVE(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, ahrs, motors)
    , _current_lead_leg(0)
    , _centre_offset_x(0.0f)
    , _centre_offset_y(0.0f)
    , _centre_offset_z(0.0f)
    , _last_update_time(0)
{
    // 初始化波浪相位
    for (uint8_t i = 0; i < 4; i++) {
        _wave_phase[i] = i; // 初始相位错开
    }
}

// 初始化
bool AP_QuadRuped_WAVE::init() {
    // 初始化步态
    gait_init();

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);

    // 初始化计时
    _last_update_time = AP_HAL::millis();

    // 设置初始引导腿
    _current_lead_leg = 0;

    _initialized = true;
    return true;
}

// 步态初始化
void AP_QuadRuped_WAVE::gait_init() {
    // 初始化腿部位置为默认站立姿态
    for (uint8_t i = 0; i < 4; i++) {
        _leg_positions[i] = Vector3f(0, 0, -_frontend.get_body_height());
    }

    // 初始化波浪相位
    for (uint8_t i = 0; i < 4; i++) {
        _wave_phase[i] = (i * 1) % 4; // 每条腿相位错开1
    }

    // 重置重心偏移
    _centre_offset_x = 0.0f;
    _centre_offset_y = 0.0f;
    _centre_offset_z = 0.0f;
}

// 更新波浪计时
void AP_QuadRuped_WAVE::update_wave_timing() {
    // 基于频率计算时间间隔
    if (_step_frequency > 0.1f) {
        _gait_cycle_time = 1.0f / _step_frequency;
    } else {
        _gait_cycle_time = 1.0f;
    }
}

// 主更新循环
void AP_QuadRuped_WAVE::update() {
    if (!_initialized) {
        return;
    }

    // 更新控制输入
    update_control_inputs();

    // 更新步态计时
    uint32_t current_time = AP_HAL::millis();
    float dt = (current_time - _last_update_time) * 0.001f; // 转换为秒
    _last_update_time = current_time;

    // 更新波浪相位
    update_wave_phase();

    // 计算步态序列
    calc_gait_sequence();

    // 更新每条腿的位置
    for (uint8_t i = 0; i < 4; i++) {
        trajectory_generation(i);
    }

    // 应用重心偏移
    for (uint8_t i = 0; i < 4; i++) {
        handle_centre_offset_phase(i);
    }

    // 应用机身摆动
    apply_body_sway();

    // 计算逆运动学并输出
    for (uint8_t i = 0; i < 4; i++) {
        _leg_angles[i] = leg_inverse_kinematics(_leg_positions[i]);
    }

    // 输出到电机
    output_leg_angle();
}

// 更新波浪相位
void AP_QuadRuped_WAVE::update_wave_phase() {
    // 波浪步态：每条腿依次抬腿，相位相差90度
    float phase_increment = (_step_frequency * 4.0f) * 0.01f; // 假设100Hz更新率

    for (uint8_t i = 0; i < 4; i++) {
        _wave_phase[i] = (_wave_phase[i] + (uint8_t)phase_increment) % 4;
    }

    // 更新引导腿（相位为0的腿）
    for (uint8_t i = 0; i < 4; i++) {
        if (_wave_phase[i] == 0) {
            _current_lead_leg = i;
            break;
        }
    }
}

// 计算步态序列
void AP_QuadRuped_WAVE::calc_gait_sequence() {
    // 波浪步态序列分析
    // 总是有3条腿支撑，1条腿摆动
    // 摆动腿按RF->LF->RB->LB的顺序依次进行

    for (uint8_t i = 0; i < 4; i++) {
        float phase = get_leg_wave_phase(i);

        // 确定腿部状态
        if (phase < 0.75f) { // 75%时间支撑
            _legs_in_air[i] = false;
        } else { // 25%时间摆动
            _legs_in_air[i] = true;
        }
    }
}

// 获取腿部波浪相位
float AP_QuadRuped_WAVE::get_leg_wave_phase(uint8_t leg_index) const {
    return (float)_wave_phase[leg_index] / 4.0f;
}

// 判断腿部是否在支撑状态
bool AP_QuadRuped_WAVE::is_leg_in_wave_support(uint8_t leg_index) const {
    return !_legs_in_air[leg_index];
}

// 轨迹生成
void AP_QuadRuped_WAVE::trajectory_generation(uint8_t leg_index) {
    float phase = get_leg_wave_phase(leg_index);

    // 计算波浪轨迹
    _leg_positions[leg_index] = calculate_wave_trajectory(leg_index, phase);
}

// 计算波浪轨迹
Vector3f AP_QuadRuped_WAVE::calculate_wave_trajectory(uint8_t leg_index, float phase) {
    Vector3f position;

    // 基础站立位置
    position.x = 0;
    position.y = 0;
    position.z = -_frontend.get_body_height();

    if (is_leg_in_wave_support(leg_index)) {
        // 支撑相：腿部保持稳定支撑，轻微调整以保持平衡
        position.x += _throttle_x * _step_length * 0.25f;
        position.y += _throttle_y * _step_length * 0.25f;

        // 支撑相补偿其他腿的摆动
        if (_legs_in_air[0]) position.x -= _step_length * 0.1f;
        if (_legs_in_air[1]) position.y += _step_length * 0.1f;
        if (_legs_in_air[2]) position.x += _step_length * 0.1f;
        if (_legs_in_air[3]) position.y -= _step_length * 0.1f;
    } else {
        // 摆动相：腿部按照波浪序列抬腿和前进
        float swing_phase = (phase - 0.75f) / 0.25f; // 归一化摆动相位
        swing_phase = constrain_float(swing_phase, 0.0f, 1.0f);

        // 使用正弦函数生成平滑轨迹
        float height_factor = sinf(swing_phase * M_PI);
        float forward_factor = swing_phase;

        // 根据腿部位置确定前进方向
        switch (leg_index) {
            case 0: // RF
                position.x = _throttle_x * _step_length * (forward_factor - 0.5f);
                position.y = _throttle_y * _step_length * (forward_factor - 0.5f);
                break;
            case 1: // LF
                position.x = _throttle_x * _step_length * (forward_factor - 0.5f);
                position.y = _throttle_y * _step_length * (forward_factor - 0.5f);
                break;
            case 2: // RB
                position.x = _throttle_x * _step_length * (forward_factor - 0.5f);
                position.y = _throttle_y * _step_length * (forward_factor - 0.5f);
                break;
            case 3: // LB
                position.x = _throttle_x * _step_length * (forward_factor - 0.5f);
                position.y = _throttle_y * _step_length * (forward_factor - 0.5f);
                break;
        }

        position.z = -_frontend.get_body_height() + _step_height * height_factor;
    }

    return position;
}

// 设置重心偏移
void AP_QuadRuped_WAVE::set_centre_offset(float x, float y, float z) {
    _centre_offset_x = x;
    _centre_offset_y = y;
    _centre_offset_z = z;
}

// 处理重心偏移相位
void AP_QuadRuped_WAVE::handle_centre_offset_phase(uint8_t leg_index) {
    if (_legs_in_air[leg_index]) {
        // 摆动腿不参与重心调整
        return;
    }

    // 根据支撑腿数量调整重心
    uint8_t support_count = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (!_legs_in_air[i]) {
            support_count++;
        }
    }

    if (support_count > 0) {
        // 将重心偏移分配到各支撑腿
        float offset_per_leg_x = _centre_offset_x / support_count;
        float offset_per_leg_y = _centre_offset_y / support_count;

        _leg_positions[leg_index].x += offset_per_leg_x;
        _leg_positions[leg_index].y += offset_per_leg_y;
    }
}

// 处理抬腿阶段
void AP_QuadRuped_WAVE::handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps,
                                        uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target) {
    // 简化的抬腿处理
    if (delta_step < lift_steps) {
        float lift_ratio = (float)delta_step / lift_steps;
        leg_z_target = -_frontend.get_body_height() + _step_height * sinf(lift_ratio * M_PI_2);
    }
}

// 处理支撑阶段
void AP_QuadRuped_WAVE::handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target) {
    // 支撑阶段保持稳定
    leg_xy_target = Vector2f(_throttle_x * _step_length * 0.5f,
                           _throttle_y * _step_length * 0.5f);
    leg_z_target = -_frontend.get_body_height();
}

// 应用机身摆动
void AP_QuadRuped_WAVE::apply_body_sway() {
    // 波浪步态中，机身会随着腿部摆动产生轻微摆动
    float sway_factor = sinf(_wave_phase[0] * M_PI_2);

    Vector2f sway_offset = Vector2f(_body_sway * sway_factor * 0.1f,
                                   _body_sway * cosf(_wave_phase[0] * M_PI_2) * 0.1f);

    // 应用到所有腿部位置
    for (uint8_t i = 0; i < 4; i++) {
        _leg_positions[i].x += sway_offset.x;
        _leg_positions[i].y += sway_offset.y;
    }
}

// 健康状态检查
bool AP_QuadRuped_WAVE::healthy() const {
    // 检查参数是否在合理范围内
    if (_step_height <= 0 || _step_height > 80) return false;
    if (_step_length <= 0 || _step_length > 150) return false;
    if (_step_frequency <= 0 || _step_frequency > 5) return false;
    if (_wave_overlap < 0 || _wave_overlap > 0.5) return false;
    if (_body_sway < 0 || _body_sway > 50) return false;

    return _initialized;
}