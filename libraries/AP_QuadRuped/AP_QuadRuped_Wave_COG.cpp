#include "AP_QuadRuped_Wave_COG.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        AP_QUADRUPED_SPEED_HZ_DEFAULT   // 默认步态频率（Hz）
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

    // ==================== 重心平滑过渡参数 ====================
    AP_GROUPINFO("COG_TIME", 7, AP_QuadRuped_Wave_COG, cog_transition_time, 0.3f),  // 过渡时间（秒）

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

    // 重心偏移：按完整周期生成连续圆轨迹，避免每个 1/4 圆切换时的突变
    cog_generation(AP_QUADRUPED_LEG_RF);

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

// 主更新函数 - 确保重心过渡始终执行
void AP_QuadRuped_Wave_COG::update()
{
    // 先执行父类的 main_inverse_kinematics，这会设置 move_requested
    main_inverse_kinematics();

    // 无论 move_requested 状态如何，都需要调用 cog_generation
    // 这样才能检测状态变化并启动停止时的过渡
    cog_generation(AP_QUADRUPED_LEG_RF);

    output_leg_angle();
    send_servo_cmd();
}

void AP_QuadRuped_Wave_COG::cog_generation(uint8_t leg_index)
{
    (void)leg_index;

    const int16_t step_total = gait_step_total.get();
    if (step_total <= 0) {
        return;
    }

    // 用完整步态周期生成连续圆轨迹，避免每个 1/4 圆切换时的突变
    const uint16_t step  = (uint16_t)(gait_step_now % step_total);
    const float    phase = (float)step / (float)step_total; // 0..1
    const float    angle = phase * M_2PI;                   // 0..2π

    const float    radius = centre_offset_ratio.get();
    const Vector2f cog_xy_target(radius * cosf(angle + M_PI), radius * sinf(angle + M_PI));

    // 平滑过渡处理：只在开始和结束时平滑
    const uint32_t now_ms = AP_HAL::millis();
    const float transition_duration_ms = cog_transition_time.get() * 1000.0f;

    // 检测移动请求状态变化
    if (move_requested && !cog_last_move_requested) {
        // 开始移动：启动进入过渡
        cog_transitioning = true;
        cog_transition_start = {0.0f, 0.0f};  // 从中心开始
        cog_transition_end = cog_xy_target;    // 目标是当前圆轨迹点
        cog_transition_start_ms = now_ms;
    } else if (!move_requested && cog_last_move_requested) {
        // 停止移动：启动退出过渡
        cog_transitioning = true;
        cog_transition_start = cog_xy_target;  // 从当前圆轨迹点开始
        cog_transition_end = {0.0f, 0.0f};      // 目标是中心
        cog_transition_start_ms = now_ms;
    }
    cog_last_move_requested = move_requested;

    Vector2f cog_output;

    if (cog_transitioning) {
        // 正在过渡中
        const float elapsed = (float)(now_ms - cog_transition_start_ms);
        float t = constrain_value(elapsed / transition_duration_ms, 0.0f, 1.0f);

        // smoothstep 平滑插值
        t = t * t * (3.0f - 2.0f * t);

        cog_output = cog_transition_start * (1.0f - t) + cog_transition_end * t;

        // 过渡完成
        if (elapsed >= transition_duration_ms) {
            cog_transitioning = false;
        }
    } else if (move_requested) {
        // 正常移动中，直接使用圆轨迹
        cog_output = cog_xy_target;
    } else {
        // 静止状态
        cog_output = {0.0f, 0.0f};
    }

    set_center_offset(cog_output);
}
