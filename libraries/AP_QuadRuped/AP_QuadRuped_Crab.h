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

// 工字步态后端实现
class AP_QuadRuped_Crab : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Crab(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_Crab() {}

    // 后端接口实现
    bool init() override;
    void update() override;
    void gait_init() override;
    void calc_gait_sequence() override;
    void trajectory_generation(uint8_t leg_index) override;
    bool healthy() const override;

    // 重写特定函数
    void yaw_trajectory_generation(uint8_t leg_index) override;

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 工字步态特定参数
    AP_Float _step_height;          // 抬腿高度
    AP_Float _step_length;          // 步长
    AP_Float _step_frequency;       // 步频
    AP_Float _crab_stride;          // 工字步幅
    AP_Float _lateral_spread;        // 横向展开度
    AP_Float _turning_radius;       // 转弯半径

    // 工字步态特定状态
    uint8_t _crab_phase;            // 工字相位 (0-3)
    bool _turning_mode;             // 转弯模式
    float _turn_angle;              // 转弯角度
    uint32_t _last_update_time;     // 上次更新时间

    // 腿部分组 (同侧分组)
    static const uint8_t LATERAL_GROUP_LEFT[2];  // 左侧腿
    static const uint8_t LATERAL_GROUP_RIGHT[2]; // 右侧腿

    // 内部辅助函数
    void update_crab_timing();
    void update_crab_phase();
    void calculate_crab_positions(uint8_t leg_index);
    void apply_turning_control();
    void calculate_lateral_movement();

    // 运动学计算
    Vector3f calculate_crab_trajectory(uint8_t leg_index, float phase);
    Vector3f calculate_turning_trajectory(uint8_t leg_index, float phase);
    float get_leg_crab_phase(uint8_t leg_index) const;
    bool is_leg_in_crab_support(uint8_t leg_index) const;

    // 工字步态特有的运动函数
    Vector2f calculate_lateral_displacement(uint8_t leg_index, float phase);
    float calculate_turn_displacement(uint8_t leg_index, float phase);
};