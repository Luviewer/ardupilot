#pragma once

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
class AP_QuadRuped_Diag : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Diag(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_Diag() { }

    // 后端接口实现
    bool init() override;
    void update() override;
    void gait_init() override;
    void calc_gait_sequence() override;
    void trajectory_generation(uint8_t leg_index) override;
    bool healthy() const override;

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 对角步态特定参数
    AP_Float _step_height;      // 抬腿高度
    AP_Float _step_length;      // 步长
    AP_Float _step_frequency;   // 步频
    AP_Float _duty_factor;      // 占空比 (支撑相比例)
    AP_Float _stability_margin; // 稳定性边距

    // 对角步态特定状态
    bool     _legs_in_air[4];   // 腿部抬升状态
    uint8_t  _diag_phase;       // 对角相位 (0-3)
    float    _gait_cycle_time;  // 步态周期时间
    uint32_t _last_update_time; // 上次更新时间

    // 腿部分组 (对角线分组)
    static const uint8_t DIAGONAL_GROUP_0[2]; // 对角线0: RF+LB
    static const uint8_t DIAGONAL_GROUP_1[2]; // 对角线1: LF+RB

    uint8_t gait_step_leg_start[LEG_ALL]; // 每条腿的步态起始步数

    // 内部辅助函数
    void update_gait_timing();
    void calculate_leg_positions(uint8_t leg_index);
    void apply_stability_control();
    void update_diagonal_phase();

    // 运动学计算
    Vector3f calculate_stance_trajectory(uint8_t leg_index, float phase);
    Vector3f calculate_swing_trajectory(uint8_t leg_index, float phase);
    float    get_leg_phase(uint8_t leg_index) const;
};