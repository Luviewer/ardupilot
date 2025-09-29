#pragma once

#include "AC_TD/AC_TD.h"
#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>

// 前向声明
class AP_QuadRuped;

// 对角步态后端实现
class AP_QuadRuped_Wave : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Wave(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_Wave() { }

    // 后端接口实现
    void update() override;
    void update_leg() override;

    void main_inverse_kinematics(void) override;

    void gait_init() override;
    void trajectory_generation(uint8_t leg_index) override;
    void yaw_trajectory_generation(uint8_t leg_index) override;

    uint32_t get_Freq() override { return gait_hz.get(); }

    void smooth_target();

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 波浪步态特定参数
    AP_Int16 gait_step_total;      // 步态周期总步数：控制一个完整步态的离散化精度
    AP_Int16 gait_hz;              // 步态频率（Hz）：控制步态更新的时间分辨率
    AP_Int8  trajectory_mode;      // 轨迹生成模式选择：0=经典正弦轨迹，1=贝塞尔曲线轨迹
    AP_Float bezier_control_height; // 贝塞尔曲线控制点高度系数：调节抬腿高度（相对leg_lift_height的比例）
    AP_Float bezier_control_forward; // 贝塞尔曲线控制点前向偏移系数：调节轨迹前后延伸程度（相对行程长度的比例）

    AC_TD td_smooth[3];            // 三维平滑滤波器：用于中心偏移的平滑过渡

    uint32_t lasttime;

    // 轨迹生成函数
    void handle_centre_offset_phase(uint8_t leg_index);
    void handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps,
                           uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target);
    void handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target);
    void generate_cycloid_trajectory(uint8_t leg_index);   // 正弦轨迹生成器：Wave步态经典实现
    void generate_bezier_trajectory(uint8_t leg_index);    // 贝塞尔曲线轨迹生成器：提供灵活的轨迹形状控制
    Vector3f cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3); // 三次贝塞尔曲线计算核心函数

    // 统一轨迹计算函数（将重心移动整合到轨迹生成中）
    Vector3f calculate_centre_offset_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_adjustment_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_centre_coordination_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_lift_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_centre_stability_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_support_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel);

    // 贝塞尔曲线专用的统一轨迹计算函数
    Vector3f calculate_centre_offset_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_adjustment_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_centre_coordination_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_lift_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_centre_stability_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);
    Vector3f calculate_leg_support_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel);

    float slow_phi(float s, float s0);
    void  balance_controller();
};