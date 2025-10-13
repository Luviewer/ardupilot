#include "AP_QuadRuped_ZongXiang.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

// 构造函数 - 调用基类构造函数
AP_QuadRuped_ZongXiang::AP_QuadRuped_ZongXiang(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_HengXiang(frontend, state, ahrs, motors)
{
    // 参数初始化由基类完成
}

bool AP_QuadRuped_ZongXiang::init()
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 初始化腿部起始位置 - 纵向步态配置
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        const float hx = (leg_index == AP_QUADRUPED_LEG_RF || leg_index == AP_QUADRUPED_LEG_LF)
            ? (Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN)
            : -(Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN);

        endpoint_leg_pos[leg_index] = Vector3f(hx, 0.0f, Sys_Param.TIBIA_LEN);
    }

    // 初始化腿部框架位置 - 与横向步态相同
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
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 4;                   // 左后腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = 3 * gait_step_total / 4; // 左前腿从中间步开始

    // 设置步态参数
    gait_travel_divisor = gait_step_total / 2; // 行程除数
    gait_lift_divisor   = 2;                   // 抬腿除数
}

// 纵向步态的位移向量 - 只使用X轴，Y轴始终为0
Vector2f AP_QuadRuped_ZongXiang::get_throttle_travel() const
{
    return Vector2f(throttle_x_travel, 0.0f);
}

// 逆运动学计算 - 纵向版本，使用X轴计算髋关节角度
Vector3f AP_QuadRuped_ZongXiang::leg_inverse_kinematics(Vector3f posxyz)
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 存储计算出的关节角度（度）
    Vector3f leg_deg = { 0, 0, 0 };

    // 1. 计算髋关节角度（绕Z轴旋转）- 纵向步态使用X轴
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