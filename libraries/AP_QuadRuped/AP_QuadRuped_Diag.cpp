#include "AP_QuadRuped_Diag.h"

extern const AP_HAL::HAL& hal;

void AP_QuadRuped_Diag::gait_init()
{
    gait_step_leg_start[Leg_RF] = 0;
    gait_step_leg_start[Leg_RB] = gait_step_total / 2;
    gait_step_leg_start[Leg_LB] = 0;
    gait_step_leg_start[Leg_LF] = gait_step_total / 2;

    gait_travel_divisor = gait_step_total / 2;
    gait_lift_divisor   = 2;
}

// gait_step本质上是一个离散化的时间变量，将连续的步态运动分解为多个离散的步骤
void AP_QuadRuped_Diag::trajectory_generation(uint8_t leg_index)
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

    if (delta_step <= (gait_step_total / 2)) {                                                       // 抬起移动阶段
        delta            = M_2PI * delta_step / gait_step_total * 2.0f;                              // 将当前步态相位映射到0-2π范围
        leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f - throttle_travel; // 最终将范围平移为[-throttle_travel → +throttle_travel]，将轨迹中心从throttle_travel移动到坐标系原点（0点）
        leg_xy_target[1] = 0;
        leg_z_target     = -leg_lift_height * (1.0f - cosf(delta)) * 1.0f; // 形成山峰形状
    } else {                                                               // 支撑返回阶段
        delta            = M_2PI * (delta_step - gait_step_total / 2) / gait_step_total * 2.0f;
        leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f + throttle_travel; //[+throttle_travel → -throttle_travel]，核心作用是通过坐标平移实现运动方向反转和相位同步
        leg_xy_target[1] = 0;
        leg_z_target     = 0;
    }

    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target); // x：横向移动（如左右踏步）    y：前后移动（如前进/后退）
}

void AP_QuadRuped_Diag::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }
    // float progress = (float)delta_step / gait_step_total;
    switch (delta_step) {
        case 0:
            gait_rot_z[leg_index] = 0;
            break;

        case 1:
            gait_rot_z[leg_index] = yaw_travel / gait_lift_divisor;
            break;

        default:
            gait_rot_z[leg_index] = gait_rot_z[leg_index] - (yaw_travel / gait_travel_divisor);
            break;
    }
}

void AP_QuadRuped_Diag::update_leg()
{
    gait_step_now++;
    if (gait_step_now > gait_step_total) gait_step_now = 0;

    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[moving_leg];

        if (delta_step < 0) delta_step += gait_step_total;
        // update_centre_offset(moving_leg);

        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
    }
}

void AP_QuadRuped_Diag::update()
{
    controller();

    main_inverse_kinematics();
}
