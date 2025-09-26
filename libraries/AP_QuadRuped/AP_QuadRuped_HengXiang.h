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
class AP_QuadRuped_HengXiang : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_HengXiang(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_HengXiang() { }

    // 后端接口实现
    void update() override;
    void update_leg() override;

    bool     init() override;
    void     gait_init() override;
    void     trajectory_generation(uint8_t leg_index) override;
    void     yaw_trajectory_generation(uint8_t leg_index) override;
    void     main_inverse_kinematics(void) override;
    Vector3f leg_inverse_kinematics(Vector3f posxyz) override;

    uint32_t get_Freq() override { return gait_hz.get(); }

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

    float hip_lock_deg[AP_QUADRUPED_LEG_ALL];
    bool  hip_lock_inited = false;

private:
    // 对角步态特定参数
    AP_Float gait_step_total; // 稳定性边距
    AP_Float gait_hz;

    uint32_t lasttime;

    float slow_phi(float s, float s0);
    void  balance_controller();
};