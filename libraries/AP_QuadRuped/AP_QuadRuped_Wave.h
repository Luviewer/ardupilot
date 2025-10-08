#pragma once

#include "AC_TD/AC_TD.h"
#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped_Params.h"
// #include "AP_QuadRuped_CentreGait.h" // 波浪步态使用内嵌重心计算
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

protected:
    // 波浪步态特定参数
    AP_Int16 gait_step_total;        // 步态周期总步数：控制一个完整步态的离散化精度
    AP_Int16 gait_hz;                // 步态频率（Hz）：控制步态更新的时间分辨率
    AP_Int8  trajectory_mode;        // 轨迹生成模式选择：0=经典正弦轨迹，1=贝塞尔曲线轨迹
    AP_Float centre_offset_ratio;    // 重心偏移比例：控制重心偏移的程度（0.0-1.0）
    AP_Float bezier_control_height;  // 贝塞尔曲线控制点高度系数：调节抬腿高度（相对leg_lift_height的比例）
    AP_Float bezier_control_forward; // 贝塞尔曲线控制点前向偏移系数：调节轨迹前后延伸程度（相对行程长度的比例）

    uint32_t lasttime;

private:
    // 轨迹生成函数
    void     generate_cycloid_trajectory(uint8_t leg_index);                                                                   // 正弦轨迹生成器：Wave步态经典实现
    void     generate_bezier_trajectory(uint8_t leg_index);                                                                    // 贝塞尔曲线轨迹生成器：提供灵活的轨迹形状控制
    Vector3f cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3); // 三次贝塞尔曲线计算核心函数

    void balance_controller();
    void refresh_phase_offsets(); // 运行时刷新步态步数相关的相位偏移

protected:
    uint8_t get_active_leg_index(); // 获取当前活跃腿的索引

    // 性能优化：相位缓存和三角函数优化
    struct PhaseCache {
        float    angle;
        float    sin_val;
        float    cos_val;
        uint32_t last_update;
        bool     valid;
    };

    PhaseCache phase_cache[AP_QUADRUPED_LEG_ALL]; // 每条腿的相位缓存
    float      get_cached_sin(float angle, uint8_t leg_index);
    float      get_cached_cos(float angle, uint8_t leg_index);
    void       update_phase_cache(float angle, uint8_t leg_index);

    int16_t gait_step_total_cached { -1 };
};
