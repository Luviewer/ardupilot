#include "AP_QuadRuped_ZongXiang.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        25.0f // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_ZongXiang::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_ZongXiang, gait_hz, SPEED_HZ_DEFAULT), // 步态频率
    // 步长
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_ZongXiang, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_ZongXiang::AP_QuadRuped_ZongXiang(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

bool AP_QuadRuped_ZongXiang::init()
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 初始化腿部起始位置 (Initialize leg starting positions)
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        const float hx = (leg_index == AP_QUADRUPED_LEG_RF || leg_index == AP_QUADRUPED_LEG_LF)
            ? (Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN)
            : -(Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN);

        endpoint_leg_pos[leg_index] = Vector3f(hx, 0.0f, Sys_Param.TIBIA_LEN);
    }

    // 初始化腿部框架位置 (Initialize leg frame positions)
    // 计算每条腿的髋关节在机体坐标系中的位置 (Calculate hip joint position in body frame)
    // 使用与腿部位置相同的角度基准 (Using same angle reference as leg positions)
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // X坐标: Sys_Param.FRAME_LEN * sin(角度) - 决定前后位置
        // Y坐标: Sys_Param.FRAME_WIDTH * cos(角度) - 决定左右位置
        // Z坐标: 0 (髋关节与机体在同一平面)
        endpoint_leg_frame[leg_index] = Vector3f(sqrtf(2) * sinf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_LEN * 0.5f,
                                                 sqrtf(2) * cosf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_WIDTH * 0.5f,
                                                 0);
    }

    gait_init(); // 初始化四足机器人逆运动学控制器

    return true;
}

// 步态初始化
void AP_QuadRuped_ZongXiang::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_ZongXiang init");

    // 设置每条腿的起始步数
    // 对角步态：左前右后同时抬起，右前左后同时抬起
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                   // 右前腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = gait_step_total / 2; // 右后腿从中间步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = 0;                   // 左后腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 2; // 左前腿从中间步开始

    // 设置步态参数
    gait_travel_divisor = gait_step_total / 2; // 行程除数
    gait_lift_divisor   = 2;                   // 抬腿除数
}

// 轨迹生成
void AP_QuadRuped_ZongXiang::trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total;

    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 期望的平面行程向量（前后X, 左右Y）

    Vector2f leg_xy_target;
    float    leg_z_target = 0.0f;

    if (p < 0.5f) {                                      // 摆动相
        const float phase      = p * 2.0f;               // 0..1
        const float phase_slow = slow_phi(phase, 0.80f); // 末段减速
        const float delta      = M_2PI * phase_slow;

        // 2D 摆线：对 v 的两个分量都按同一标量函数变换
        const float S   = (delta - sinf(delta)) / M_2PI * 2.0f; // 0..2
        leg_xy_target.x = throttle_x_travel * S - throttle_x_travel;
        leg_z_target    = -leg_lift_height * (1.0f - cosf(delta));
    } else { // 支撑相
        const float phase = (p - 0.5f) * 2.0f;
        const float delta = M_2PI * phase;

        const float S   = (delta - sinf(delta)) / M_2PI * 2.0f;
        leg_xy_target.x = -throttle_x_travel * S + throttle_x_travel;
        leg_z_target    = 0.0f;
    }
    leg_xy_target.y         = 0;
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 生成偏航（旋转）轨迹
void AP_QuadRuped_ZongXiang::yaw_trajectory_generation(uint8_t leg_index)
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

Vector3f AP_QuadRuped_ZongXiang::leg_inverse_kinematics(Vector3f posxyz)
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 存储计算出的关节角度（度）
    Vector3f leg_deg = { 0, 0, 0 };

    // 1. 计算髋关节角度（绕Z轴旋转）
    leg_deg.x = -degrees(atan2f(posxyz.x, 0.0f)); // 使用atan2计算XY平面内的角度

    // 2. 计算从髋关节到末端在XY平面的投影距离
    float trueX = fabsf(posxyz.x) - Sys_Param.COXA_LEN; // 减去髋关节长度

    // 3. 计算从股关节到末端的空间距离
    float im = sqrtf(trueX * trueX + posxyz.z * posxyz.z);

    // 4. 计算股关节角度（使用余弦定理）
    float q1 = atan2f(trueX, posxyz.z); // 股关节与末端连线与垂直方向的夹角

    // 使用余弦定理计算股关节角度
    float d1  = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN + im * im;
    float d2  = 2 * Sys_Param.FEMUR_LEN * im;
    float q2  = acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f)); // 约束在[-1,1]范围内防止数值错误
    leg_deg.y = -(degrees(q1 + q2) - 90);                            // 计算股关节角度并调整坐标系

    // 5. 计算胫关节角度（使用余弦定理）
    d1        = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - im * im + Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN;
    d2        = 2 * Sys_Param.TIBIA_LEN * Sys_Param.FEMUR_LEN;
    leg_deg.z = -(degrees(acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f))) - 90); // 计算胫关节角度

    if (_frontend.get_class() == AP_QUADRUPED_USL_BV2) {
        const float alpha = degrees(atan2f(Sys_Param.Alpha_A, Sys_Param.Alpha_B));
        leg_deg.y -= alpha;
        leg_deg.z += 90.0f - alpha;
    }

    return leg_deg; // 返回{髋关节, 股关节, 胫关节}角度
}

// 主逆运动学计算 - 计算所有腿的关节角度
void AP_QuadRuped_ZongXiang::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 }; // 临时变量，存储腿部末端位置

    // 腿部角度偏移补偿 - 由于机械安装误差，每条腿需要不同的角度补偿
    const Vector3f endpoint_leg_angle_offset[AP_QUADRUPED_LEG_ALL] = {
        { 45, 0, 0 },   // 右前腿：髋关节补偿45度
        { -45, 0, 0 },  // 右后腿：髋关节补偿-45度
        { -135, 0, 0 }, // 左后腿：髋关节补偿-135度
        { -225, 0, 0 }  // 左前腿：髋关节补偿-225度
    }; // 格式：{髋关节角度，股关节角度，胫关节角度} - 只有髋关节需要补偿

    // 遍历所有腿，计算逆运动学
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // 1. 计算腿部末端在机体坐标系中的位置
        ansxyz = body_forward_kinematics(leg_index);
        // 2. 计算逆运动学得到关节角度，并加上补偿值
        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];
        // 3. 将髋关节角度规范到[-180, 180]范围内
        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);
    }

    // 计算步态序列
    // 根据 throttle_travel（前进/后退）和 yaw_travel（旋转）更新步态相位
    // 决定下一步的足端轨迹
    calc_gait_sequence();

    // 保存当前关节角度到上一时刻变量
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

// 更新腿部运动
void AP_QuadRuped_ZongXiang::update_leg()
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

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_ZongXiang::update()
{
    // 执行主控制器
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
void AP_QuadRuped_ZongXiang::balance_controller()
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
float AP_QuadRuped_ZongXiang::slow_phi(float s, float s0)
{
    if (s <= s0) return s;
    float sigma = (s - s0) / (1.0f - s0); // 0..1
    float w     = sigma
        + 4.0f * powf(sigma, 3.0f)
        - 7.0f * powf(sigma, 4.0f)
        + 3.0f * powf(sigma, 5.0f); // w(0)=0,w'(0)=1; w(1)=1,w'(1)=0
    return s0 + (1.0f - s0) * w;
}