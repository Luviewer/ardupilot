#include "AP_QuadRuped_Diag.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_QuadRuped_Diag::var_info[] = {
    // 频率
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Base, gait_hz, SPEED_HZ_DEFAULT), // 步态频率

    // 步长
    AP_GROUPINFO("STEP", 5, AP_QuadRuped_Base, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数
};

// 初始化对角步态参数
void AP_QuadRuped_Diag::gait_init()
{
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

// gait_step本质上是一个离散化的时间变量，将连续的步态运动分解为多个离散的步骤
// 和逆运动学相互约束，逆运动学解算出相应的关节角，再通过轨迹生成生成轨迹
// 末段时间缩放函数：C2 连续，末端 v=a=0
inline float slow_phi(float s, float s0)
{
    if (s <= s0) return s;
    float sigma = (s - s0) / (1.0f - s0); // 0..1
    float w     = sigma
        + 4.0f * powf(sigma, 3.0f)
        - 7.0f * powf(sigma, 4.0f)
        + 3.0f * powf(sigma, 5.0f); // w(0)=0,w'(0)=1; w(1)=1,w'(1)=0
    return s0 + (1.0f - s0) * w;
}

void AP_QuadRuped_Diag::trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total;

    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 期望的平面行程向量（前后X, 左右Y）
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    Vector2f leg_xy_target;
    float    leg_z_target = 0.0f;

    if (p < 0.5f) {                                      // 摆动相
        const float phase      = p * 2.0f;               // 0..1
        const float phase_slow = slow_phi(phase, 0.80f); // 末段减速
        const float delta      = M_2PI * phase_slow;

        // 2D 摆线：对 v 的两个分量都按同一标量函数变换
        const float S = (delta - sinf(delta)) / M_2PI * 2.0f;  // 0..2
        leg_xy_target = throttle_travel * S - throttle_travel; // 原公式在 X 上的 1D 推广到 2D
        leg_z_target  = -leg_lift_height * (1.0f - cosf(delta));
    } else { // 支撑相
        const float phase = (p - 0.5f) * 2.0f;
        const float delta = M_2PI * phase;

        const float S = (delta - sinf(delta)) / M_2PI * 2.0f;
        leg_xy_target = -throttle_travel * S + throttle_travel; // 同样推广到 2D
        leg_z_target  = 0.0f;
    }

    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 生成偏航（旋转）轨迹
void AP_QuadRuped_Diag::yaw_trajectory_generation(uint8_t leg_index)
{
    // 计算当前腿的步数偏移
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

    const float p    = (float)delta_step / (float)gait_step_total; // 步态进度 ∈ [0,1)
    const float peak = yaw_travel / (float)gait_lift_divisor;      // 旋转峰值

    if (p < (1.0f / 12.0f)) { // 前1/12时间段：无旋转
        gait_rot_z[leg_index] = 0.0f;
    } else if (p < (1.0f / 6.0f)) { // 1/12到1/6时间段：达到峰值旋转
        gait_rot_z[leg_index] = peak;
    } else { // 剩余5/6时间段：线性衰减到0
        // 线性从 peak 衰减到 0，区间长度 = 5/6
        const float t         = (p - (1.0f / 6.0f)) / (5.0f / 6.0f); // t ∈ [0,1)
        gait_rot_z[leg_index] = peak * (1.0f - t);                   // 直接给定，不依赖上一帧
    }
}

// 更新腿部运动
void AP_QuadRuped_Diag::update_leg()
{
    // 更新步态计数器
    gait_step_now++;
    if (gait_step_now >= gait_step_total) gait_step_now = 0; // 循环计数

    // 遍历所有腿，生成轨迹
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[moving_leg];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
    }
}

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_Diag::update()
{
    controller();              // 执行主控制器
    balance_controller();      // 执行平衡控制器
    main_inverse_kinematics(); // 执行逆运动学解算
    output_leg_angle();        // 输出腿部关节角度
}
