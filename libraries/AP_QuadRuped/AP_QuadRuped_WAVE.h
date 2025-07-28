#pragma once

#include "AP_QuadRuped_Base.h"
#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Param/AP_Param.h>
#include <stdio.h>

class AP_QuadRuped_WAVE : public AP_QuadRuped_Base {
public:
    AP_QuadRuped_WAVE(AP_AHRS_View& ahrs, AP_MotorsMulticopter& motors)
        : AP_QuadRuped_Base(ahrs, motors)
    {
    }
    virtual ~AP_QuadRuped_WAVE() { }

    void gait_init() override;
    void update_leg() override;
    void trajectory_generation(uint8_t leg_index) override;
    void yaw_trajectory_generation(uint8_t leg_index) override;

    void set_centre_offset(float x, float y, float z = 0) override;

    void update() override;
};
