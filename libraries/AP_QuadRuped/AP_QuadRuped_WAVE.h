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
    
    void gait_init() override;
};
