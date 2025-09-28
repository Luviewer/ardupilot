#include "AP_QuadRuped_Wave.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        25.0f // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_Wave::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Wave, gait_hz, SPEED_HZ_DEFAULT), // 步态频率
    // 步长
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_Wave, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Wave::AP_QuadRuped_Wave(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// 步态初始化
void AP_QuadRuped_Wave::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_Wave init");

    td_smooth[0].init(1000 / gait_hz, 100, 0);
    td_smooth[1].init(1000 / gait_hz, 100, 0);
    td_smooth[2].init(1000 / gait_hz, 100, 0);

    // 设置每条腿的起始步数
    // 对角步态：左前右后同时抬起，右前左后同时抬起
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                       // 右前腿 - 0度相位
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4;     // 左前腿 - 90度相位
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 2;     // 左后腿 - 180度相位
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = 3 * gait_step_total / 4; // 右后腿 - 270度相位

    // 设置步态参数
    gait_travel_divisor = gait_step_total / 2; // 行程除数
    gait_lift_divisor   = 2;                   // 抬腿除数
}

// 轨迹生成
void AP_QuadRuped_Wave::trajectory_generation(uint8_t leg_index)
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
    const uint16_t support_steps       = gait_step_total - centre_offset_steps - lift_steps;

    if (delta_step < centre_offset_steps) {
        // 中心偏移阶段
        handle_centre_offset_phase(leg_index);
    } else if (delta_step < (centre_offset_steps + lift_steps)) {
        // 抬腿阶段 - 使用正弦曲线实现平滑运动
        handle_lift_phase(delta_step, centre_offset_steps, lift_steps, leg_xy_target, leg_z_target);
    } else {
        // 支撑阶段 - 保持稳定姿态
        const uint16_t step_in_support = delta_step - (centre_offset_steps + lift_steps);
        const float    support_s       = (float)step_in_support / (float)support_steps;
        handle_support_phase(support_s, leg_xy_target, leg_z_target);
    }
    // 设置最终腿部位置
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 生成偏航（旋转）轨迹
void AP_QuadRuped_Wave::yaw_trajectory_generation(uint8_t leg_index)
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

void AP_QuadRuped_Wave::handle_centre_offset_phase(uint8_t leg_index)
{
    // 计算当前腿相对于其起始步态的相位偏移
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 处理负相位偏移，确保在[0, gait_step_total)范围内
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }

    const uint16_t centre_offset_steps = gait_step_total / 12;

    // 计算平滑插值比例 (0.0 到 1.0)
    float t = (float)delta_step / centre_offset_steps;

    // 使用平滑的缓动函数 (ease-in-out cubic)
    float smooth_t = t < 0.5f ? 4.0f * t * t * t : 1.0f - powf(-2.0f * t + 2.0f, 3.0f) / 2.0f;

    if (throttle_x_travel < 0) {
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF:
                set_centre_offset(-30.0f * smooth_t, -45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LF:
                set_centre_offset(-30.0f * smooth_t, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LB:
                set_centre_offset(0.0f, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_RB:
                set_centre_offset(0.0f, -45.0f * smooth_t);
                break;
        }
    } else {
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF:
                set_centre_offset(0.0f, -45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LF:
                set_centre_offset(0.0f, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LB:
                set_centre_offset(30.0f * smooth_t, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_RB:
                set_centre_offset(30.0f * smooth_t, -45.0f * smooth_t);
                break;
        }
    }
}

// 处理抬腿阶段
void AP_QuadRuped_Wave::handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps,
                                          uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target)
{
    float delta      = M_2PI * (delta_step - centre_offset_steps) / lift_steps;
    leg_xy_target[0] = throttle_x_travel * (delta - sinf(delta)) / M_2PI;
    leg_xy_target[1] = 0;
    leg_z_target     = -3 * leg_lift_height * (1.0f - cosf(delta));
}

// 处理支撑阶段
void AP_QuadRuped_Wave::handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target)
{
    // 三次缓动：起止速度为0
    const float s   = 3.0f * support_s * support_s - 2.0f * support_s * support_s * support_s;
    leg_xy_target.x = throttle_x_travel * (1.0f - s); // 从前端回拖到 0
    leg_xy_target.y = 0.0f;
    leg_z_target    = 0.0f; // 地面接触
}

// 更新腿部运动
void AP_QuadRuped_Wave::update_leg()
{
    // 更新步态计数器
    gait_step_now++;
    if (gait_step_now >= gait_step_total) gait_step_now = 0; // 循环计数

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

void AP_QuadRuped_Wave::smooth_target()
{
    // 平滑跟随（不清零，不回中位）
    centre_offset.x = td_smooth[0].update(centre_offset_target.x);
    centre_offset.y = td_smooth[1].update(centre_offset_target.y);
    centre_offset.z = td_smooth[2].update(centre_offset_target.z);

    // centre_offset += (centre_offset_target - centre_offset) * alpha;
}

void AP_QuadRuped_Wave::main_inverse_kinematics()
{
    smooth_target();
    AP_QuadRuped_Backend::main_inverse_kinematics();
}

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_Wave::update()
{
    // if ((AP_HAL::millis() - lasttime) < (1000 / gait_hz)) {
    //     return;
    // }

    // // 更新最后执行时间
    // lasttime = AP_HAL::millis();

    // // 检查遥控器通道 6（CH_6）的值是否大于 1500（通常表示开关激活）并且没有解锁
    // if (_frontend.get_mode_channel() > 1800 && !_motors.armed()) {
    //     // 执行主控制器
    //     main_radio_controller();

    //     // 执行平衡控制器
    //     // balance_controller();

    //     // 执行逆运动学解算
    //     main_inverse_kinematics();

    //     // 输出腿部关节角度
    //     output_leg_angle();
    // } else {
    //     hengxiang_up_sleep_leg();
    // }

    main_radio_controller();

    // 执行平衡控制器
    // balance_controller();

    // 执行逆运动学解算
    main_inverse_kinematics();

    // 输出腿部关节角度
    output_leg_angle();

    // 发送数据
    send_servo_cmd();
}

// 平衡控制器 - 简单的平衡控制实现
void AP_QuadRuped_Wave::balance_controller()
{
    // 简单的平衡控制实现
    // 这里可以根据IMU数据调整重心偏移以保持平衡

    // 获取当前姿态数据
    const Vector3f& gyro  = _ahrs.get_gyro();
    const Vector3f& accel = _ahrs.get_accel_ef();

    // 计算需要的重心补偿（示例实现）
    // 这里可以根据实际的平衡控制算法进行调整
    centre_offset.x = constrain_float(gyro.y * 0.1f, -10.0f, 10.0f); // 基于横滚角速度补偿
    centre_offset.y = constrain_float(gyro.x * 0.1f, -10.0f, 10.0f); // 基于俯仰角速度补偿

    // 重心高度补偿（基于Z轴加速度）
    centre_offset.z = constrain_float(accel.z * 0.05f, -5.0f, 5.0f);
}

// gait_step本质上是一个离散化的时间变量，将连续的步态运动分解为多个离散的步骤
// 和逆运动学相互约束，逆运动学解算出相应的关节角，再通过轨迹生成生成轨迹
// 末段时间缩放函数：C2 连续，末端 v=a=0
float AP_QuadRuped_Wave::slow_phi(float s, float s0)
{
    if (s <= s0) return s;
    float sigma = (s - s0) / (1.0f - s0); // 0..1
    float w     = sigma
        + 4.0f * powf(sigma, 3.0f)
        - 7.0f * powf(sigma, 4.0f)
        + 3.0f * powf(sigma, 5.0f); // w(0)=0,w'(0)=1; w(1)=1,w'(1)=0
    return s0 + (1.0f - s0) * w;
}