#include "AP_QuadRuped_WAVE.h"

extern const AP_HAL::HAL& hal;

void AP_QuadRuped_WAVE::gait_init()
{
    // 波浪步态设置 - 四条腿依次移动，相位差为90度
    gait_step_leg_start[Leg_RF] = 0;                       // 右前腿 - 0度相位
    gait_step_leg_start[Leg_LF] = gait_step_total / 4;     // 左前腿 - 90度相位
    gait_step_leg_start[Leg_LB] = gait_step_total / 2;     // 左后腿 - 180度相位
    gait_step_leg_start[Leg_RB] = 3 * gait_step_total / 4; // 右后腿 - 270度相位

    // 调整步态参数
    gait_travel_divisor = gait_step_total / 2;
    gait_lift_divisor   = 2;
}

void AP_QuadRuped_WAVE::trajectory_generation(uint8_t leg_index)
{
    Vector2f leg_xy_target;
    float    leg_z_target = 0;

    // 计算当前腿相对于其起始步态的相位偏移
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    
    // 处理负相位偏移，确保在[0, gait_step_total)范围内
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }

    const uint16_t centre_offset_steps = gait_step_total / 12;
    const uint16_t lift_steps          = gait_step_total / 12;

    if (delta_step < centre_offset_steps) {
        // 中心偏移阶段
        handle_centre_offset_phase(leg_index);
    } else if (delta_step < (centre_offset_steps + lift_steps)) {
        // 抬腿阶段 - 使用正弦曲线实现平滑运动
        handle_lift_phase(delta_step, centre_offset_steps, lift_steps, 
                         leg_xy_target, leg_z_target);
    } else {
        // 支撑阶段 - 保持稳定姿态
        handle_support_phase(leg_xy_target, leg_z_target);
    }
    // 设置最终腿部位置
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

void AP_QuadRuped_WAVE::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }

    // 偏航控制分段处理
    if (delta_step <= 1) {
        // 初始阶段 - 保持零偏航
        gait_rot_z[leg_index] = 0;
    } else if (delta_step <= 3) {
        // 加速阶段 - 应用偏航增量
        gait_rot_z[leg_index] = yaw_travel / gait_lift_divisor;
    } else {
        // 减速阶段 - 平滑过渡到零
        gait_rot_z[leg_index] -= (yaw_travel / (gait_step_total - 4));
    }
}

// 处理中心偏移阶段
void AP_QuadRuped_WAVE::handle_centre_offset_phase(uint8_t leg_index)
{
    switch (leg_index) {
        case Leg_RF:
            set_centre_offset(0.0f, -50.0f);
            break;
        case Leg_LF:
            set_centre_offset(0.0f, 50.0f);
            break;
        case Leg_LB:
            set_centre_offset(throttle_travel, 50.0f);
            break;
        case Leg_RB:
            set_centre_offset(throttle_travel, -50.0f);
            break;
    }
}

// 处理抬腿阶段
void AP_QuadRuped_WAVE::handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps, 
                                         uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target)
{
    float delta = M_2PI * (delta_step - centre_offset_steps) / lift_steps;
    leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI;
    leg_xy_target[1] = 0;
    leg_z_target     = -leg_lift_height * (1.0f - cosf(delta));
}

// 处理支撑阶段
void AP_QuadRuped_WAVE::handle_support_phase(Vector2f& leg_xy_target, float& leg_z_target)
{
    leg_xy_target[0] = throttle_travel;
    leg_xy_target[1] = 0;
    leg_z_target     = 0;
}

void AP_QuadRuped_WAVE::set_centre_offset(float x, float y, float z)
{
    centre_offset = Vector3f(x, y, z);
}

void AP_QuadRuped_WAVE::update_leg()
{
    // 更新步态计数器，实现循环
    gait_step_now++;
    if (gait_step_now >= gait_step_total) {
        gait_step_now = 0;
    }

    // 为所有腿生成轨迹
    for (uint8_t leg = 0; leg < LEG_ALL; leg++) {
        trajectory_generation(leg);
        yaw_trajectory_generation(leg);
    }
}

void AP_QuadRuped_WAVE::update()
{
    controller();

    balance_controller();

    main_inverse_kinematics();

    output_leg_angle();
}
