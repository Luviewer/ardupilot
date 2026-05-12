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

    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                       // 右前腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = 2 * gait_step_total / 4; // 右后腿从中间步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = 3 * gait_step_total / 4;                       // 左后腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4; // 左前腿从中间步开始

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
    return AP_QuadRuped_HengXiang::leg_inverse_kinematics(posxyz);
}