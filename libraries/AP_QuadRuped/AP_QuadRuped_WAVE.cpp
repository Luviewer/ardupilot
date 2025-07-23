#include "AP_QuadRuped_WAVE.h"

extern const AP_HAL::HAL& hal;

void AP_QuadRuped_WAVE::gait_init()
{
    // 波浪步态设置 - 四条腿依次移动
    // 波浪步态设置 - 四条腿依次移动
    gait_step_leg_start[Leg_RF] = 0;                       // 右前腿
    gait_step_leg_start[Leg_RB] = 3 * gait_step_total / 4; // 左后腿
    gait_step_leg_start[Leg_LB] = gait_step_total / 2;     // 右后腿
    gait_step_leg_start[Leg_LF] = gait_step_total / 4;     // 左前腿

    // 调整步态参数
    gait_travel_divisor = gait_step_total / 2;
}

void AP_QuadRuped_WAVE::trajectory_generation(uint8_t leg_index)
{
    float    delta;
    Vector2f leg_xy_target;
    float    leg_z_target = 0;

    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index]; // 计算当前腿相对于其起始步态的相位偏移
    // gait_step_total一个完整步态周期的总步数，值越大，步态分解越精细，运动越平滑
    // gait_step_leg_start[leg_index]每条腿的步态起始相位，使不同腿的运动产生相位差，例如对角步态中，两条腿的起始相位相差 gait_step_total/2
    if (delta_step < 0) {
        delta_step = gait_step_total + delta_step; // 如果结果为负，通过加上总步态周期数来修正
    }

    uint16_t centre_offset_steps = gait_step_total / 12;
    uint16_t lift_steps          = gait_step_total / 12;

    if (delta_step < centre_offset_steps) {
        switch (leg_index) {
            case Leg_RF:
                set_centre_offset(0, -50, 0);
                break;
            case Leg_LF:
                set_centre_offset(0, 50, 0);
                break;
            case Leg_LB:
                set_centre_offset(throttle_travel, 50, 0);
                break;
            case Leg_RB:
                set_centre_offset(throttle_travel, -50, 0);
                break;
        }
    } else if (delta_step < (centre_offset_steps + lift_steps)) {
        // 抬腿阶段
        delta            = M_2PI * (delta_step - centre_offset_steps) / lift_steps;
        leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI;
        leg_xy_target[1] = 0;
        leg_z_target     = -leg_lift_height * (1.0f - cosf(delta));
    } else {
        // 支撑阶段
        leg_xy_target[0] = throttle_travel;
        leg_xy_target[1] = 0;
        leg_z_target     = 0;
    }
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target); // x：横向移动（如左右踏步）    y：前后移动（如前进/后退）
}

void AP_QuadRuped_WAVE::yaw_trajectory_generation(uint8_t leg_index)
{
}

void AP_QuadRuped_WAVE::set_centre_offset(float x, float y, float z = 0)
{
    centre_offset = Vector3f(x, y, z);
}

void AP_QuadRuped_WAVE::update_leg()
{
    gait_step_now++;
    if (gait_step_now > gait_step_total) gait_step_now = 0;

    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[moving_leg];

        if (delta_step < 0) delta_step += gait_step_total;

        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
    }
}

void AP_QuadRuped_WAVE::update()
{
    controller();

    main_inverse_kinematics();

    output_leg_angle();
}
