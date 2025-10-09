#pragma once

#include "AP_QuadRuped_HengXiang.h"
#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>

// 前向声明
class AP_QuadRuped;

// 纵向步态后端实现 - 继承横向步态的通用功能
class AP_QuadRuped_ZongXiang : public AP_QuadRuped_HengXiang {
public:
    // 构造函数
    AP_QuadRuped_ZongXiang(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_ZongXiang() { }

    // 只重写需要不同的函数
    bool init() override;
    void gait_init() override;
    Vector3f leg_inverse_kinematics(Vector3f posxyz) override;

protected:
    // 重写轴向处理 - 纵向步态只使用X轴
    Vector2f get_throttle_travel() const override;
};