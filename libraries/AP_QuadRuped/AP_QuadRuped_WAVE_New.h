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

// 波浪步态后端实现
class AP_QuadRuped_WAVE : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_WAVE(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_WAVE() {}

    // 后端接口实现
    bool init() override;
    void update() override;
    void gait_init() override;
    void calc_gait_sequence() override;
    void trajectory_generation(uint8_t leg_index) override;
    bool healthy() const override;

    // 重写特定函数
    void set_centre_offset(float x, float y, float z = 0) override;

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 波浪步态特定参数
    AP_Float _step_height;          // 抬腿高度
    AP_Float _step_length;          // 步长
    AP_Float _step_frequency;       // 步频
    AP_Float _wave_overlap;         // 波浪重叠时间
    AP_Float _body_sway;            // 机身摆动幅度

    // 波浪步态特定状态
    uint8_t _wave_phase[4];         // 每条腿的相位 (0-3)
    uint8_t _current_lead_leg;      // 当前引导腿
    float _centre_offset_x;         // 重心X偏移
    float _centre_offset_y;         // 重心Y偏移
    float _centre_offset_z;         // 重心Z偏移
    uint32_t _last_update_time;     // 上次更新时间

    // 波浪步态序列
    static const uint8_t WAVE_SEQUENCE[4]; // 波浪步态序列

    // 内部辅助函数
    void update_wave_timing();
    void update_wave_phase();
    void calculate_wave_positions(uint8_t leg_index);
    void handle_centre_offset_phase(uint8_t leg_index);
    void handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps,
                          uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target);
    void handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target);
    void apply_body_sway();

    // 运动学计算
    Vector3f calculate_wave_trajectory(uint8_t leg_index, float phase);
    float get_leg_wave_phase(uint8_t leg_index) const;
    bool is_leg_in_wave_support(uint8_t leg_index) const;
};