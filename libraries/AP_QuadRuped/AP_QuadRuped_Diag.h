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
    void update_leg() override;

    void gait_init() override;
    void trajectory_generation(uint8_t leg_index) override;
    void yaw_trajectory_generation(uint8_t leg_index) override;

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

private:
    // 对角步态特定参数
    AP_Float gait_step_total; // 稳定性边距
    AP_Float gait_hz;

    uint32_t lasttime;

    float slow_phi(float s, float s0);
};