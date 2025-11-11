#include "AP_QuadRuped_Wave_COG.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        AP_QUADRUPED_SPEED_HZ_DEFAULT // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT AP_QUADRUPED_STEP_TOTAL_DEFAULT // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 波浪步态参数表定义
const AP_Param::GroupInfo AP_QuadRuped_Wave_COG::var_info[] = {
    // ==================== 基础步态参数 ====================
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Wave_COG, gait_hz, SPEED_HZ_DEFAULT),
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_Wave_COG, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    // ==================== 轨迹生成模式选择 ====================
    AP_GROUPINFO("TRAJ_MODE", 3, AP_QuadRuped_Wave_COG, trajectory_mode, 0),

    // ==================== 重心偏移参数 ====================
    AP_GROUPINFO("COG_OF", 4, AP_QuadRuped_Wave_COG, centre_offset_ratio, 0.4f),

    // ==================== 贝塞尔曲线控制参数 ====================
    AP_GROUPINFO("BCTRL_H", 5, AP_QuadRuped_Wave_COG, bezier_control_height, 0.3f),
    AP_GROUPINFO("BCTRL_F", 6, AP_QuadRuped_Wave_COG, bezier_control_forward, 0.2f),

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Wave_COG::AP_QuadRuped_Wave_COG(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Wave(frontend, state, ahrs, motors)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// 步态初始化
void AP_QuadRuped_Wave_COG::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_Wave_COG init");

    const int16_t step_total = gait_step_total.get();

    // 维持12.5%摆动相
    gait_lift_divisor = 8;

    // 设置每条腿的起始步数 - 波浪步态：90度相位差，确保单腿摆动
    gait_step_cog_start[AP_QUADRUPED_LEG_RF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 0 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (0 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_RB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 1 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (1 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_LB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 2 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (2 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_LF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 3 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (3 * 2 + 1), 0, 255));
}

// 更新腿部运动
void AP_QuadRuped_Wave_COG::update_leg()
{
    // 更新步态计数器 - 改进为大步数时的连续增长
    gait_step_now++;

    // 使用大整数范围避免频繁循环，减少相位跳跃
    // 只有当步数超过很大值时才重置，避免边界问题
    if (gait_step_now >= 100000000) { // 使用int32_t接近上限的值
        gait_step_now = 0;
        // gait_step_total_cached = -1;
        refresh_steps();
    }

    refresh_steps();

    // 波浪步态当前版本不处理重心偏移
    // center_offset.zero();
    // centre_offset_target.zero();

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        cog_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

void AP_QuadRuped_Wave_COG::cog_generation(uint8_t leg_index)
{
    // 步态相位计算：与贝塞尔曲线轨迹相同的相位处理逻辑
    // 这种统一设计保证了不同轨迹算法之间的相位一致性
    int32_t delta_step = gait_step_now - gait_step_cog_start[leg_index];

    // 相位循环处理：确保步数在有效范围内，避免整数溢出和相位跳跃
    // 先处理负数再取模，保证数学上的正确性和连续性
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位转换：将离散步数映射到连续的[0,1]区间
    // 这个p值是整个轨迹生成的时间基准，驱动后续所有的数学计算
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 单腿重心偏移占 1/8 周期
    const float swing_ratio = 1.0f / (float)gait_lift_divisor;

    const float cog_length = centre_offset_ratio.get();

    Vector2f cog_xy_target, cog_xy_travel;
    float    phase;

    if (p < swing_ratio) {
        phase = constrain_float(p / swing_ratio, 0.0f, 1.0f);

        if (leg_index == AP_QUADRUPED_LEG_RF) {

            cog_xy_travel   = Vector2f(-cog_length, -cog_length);
            cog_xy_target.x = 
            sqrtf(2) * cog_xy_travel.x * cosf(M_PI / 2 * (phase - 0.5)); // X轴正弦变化，起终点相同
            cog_xy_target.y = sqrtf(2) * cog_xy_travel.y * sinf(M_PI / 2 * (phase - 0.5)); // Y轴从+cog_length到-cog_length

        } else if (leg_index == AP_QUADRUPED_LEG_RB) {

            cog_xy_travel   = Vector2f(cog_length, -cog_length);
            cog_xy_target.x = sqrtf(2) * cog_xy_travel.x * sinf(M_PI / 2 * (phase - 0.5)); // X轴正弦变化，起终点相同
            cog_xy_target.y = sqrtf(2) * cog_xy_travel.y * cosf(M_PI / 2 * (phase - 0.5)); // Y轴从+cog_length到-cog_length

        } else if (leg_index == AP_QUADRUPED_LEG_LB) {

            cog_xy_travel   = Vector2f(cog_length, cog_length);
            cog_xy_target.x = sqrtf(2) * cog_xy_travel.x * cosf(M_PI / 2 * (phase - 0.5)); // X轴正弦变化，起终点相同
            cog_xy_target.y = sqrtf(2) * cog_xy_travel.y * sinf(M_PI / 2 * (phase - 0.5)); // Y轴从+cog_length到-cog_length

        } else if (leg_index == AP_QUADRUPED_LEG_LF) {

            cog_xy_travel   = Vector2f(-cog_length, cog_length);
            cog_xy_target.x = sqrtf(2) * cog_xy_travel.x * sinf(M_PI / 2 * (phase - 0.5)); // X轴正弦变化，起终点相同
            cog_xy_target.y = sqrtf(2) * cog_xy_travel.y * cosf(M_PI / 2 * (phase - 0.5)); // Y轴从+cog_length到-cog_length
        }
        set_center_offset(cog_xy_target);
    }
}
