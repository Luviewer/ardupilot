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
class AP_QuadRuped_Wave : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Wave(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_Wave() { }

    // 后端接口实现
    void update() override;
    void update_leg() override;

    void gait_init() override;
    void trajectory_generation(uint8_t leg_index) override;
    void yaw_trajectory_generation(uint8_t leg_index) override;
    void set_centre_offset(float x, float y, float z = 0) override;

    uint32_t get_Freq() override { return gait_hz.get(); }

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 对角步态特定参数
    AP_Float gait_step_total; // 稳定性边距
    AP_Float gait_hz;

    uint32_t lasttime;

    void handle_centre_offset_phase(uint8_t leg_index);
    void handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps, 
                         uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target);
    void handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target);

    float slow_phi(float s, float s0);
    void  balance_controller();
};