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
// 和逆运动学相互约束，逆运动学解算出相应的关节角，再通过轨迹生成生成轨迹
void AP_QuadRuped_Diag::trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total;

    const float p = (float)delta_step / (float)gait_step_total; // [0,1)
    float       delta;
    Vector2f    leg_xy_target;
    float       leg_z_target = 0.0f;

    if (p < 0.5f) { // 用 <，确保两半各占一半
        delta            = M_2PI * (p * 2.0f);
        leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f - throttle_travel;
        leg_xy_target[1] = 0.0f;
        leg_z_target     = -leg_lift_height * (1.0f - cosf(delta));
    } else {
        delta            = M_2PI * ((p - 0.5f) * 2.0f);
        leg_xy_target[0] = -throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f + throttle_travel;
        leg_xy_target[1] = 0.0f;
        leg_z_target     = 0.0f;
    }
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

void AP_QuadRuped_Diag::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total;

    const float p    = (float)delta_step / (float)gait_step_total; // progress ∈ [0,1)
    const float peak = yaw_travel / (float)gait_lift_divisor;

    if (p < (1.0f / 12.0f)) {
        gait_rot_z[leg_index] = 0.0f;
    } else if (p < (1.0f / 6.0f)) {
        gait_rot_z[leg_index] = peak;
    } else {
        // 线性从 peak 衰减到 0，区间长度 = 5/6
        const float t         = (p - (1.0f / 6.0f)) / (5.0f / 6.0f);    // t ∈ [0,1)
        gait_rot_z[leg_index] = peak * (1.0f - t);                      // 直接给定，不依赖上一帧
        if (gait_rot_z[leg_index] < 0.0f) gait_rot_z[leg_index] = 0.0f; // 数值保险
    }
}

void AP_QuadRuped_Diag::update_leg()
{
    gait_step_now++;
    if (gait_step_now >= gait_step_total) gait_step_now = 0;

    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[moving_leg];

        if (delta_step < 0) delta_step += gait_step_total;

        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
    }
}

void AP_QuadRuped_Diag::update()
{
    controller();

    balance_controller();

    main_inverse_kinematics();

    output_leg_angle();
}
