#pragma once

#include "AP_QuadRuped_Wave.h"

// 波浪步态（含重心偏移版）
class AP_QuadRuped_Wave_COG : public AP_QuadRuped_Wave {
public:
    AP_QuadRuped_Wave_COG(AP_QuadRuped& frontend,
                          AP_QuadRuped::QuadRuped_State& state,
                          AP_AHRS_View& ahrs,
                          AP_Motors& motors);

    void update_leg() override;

protected:
    void calculate_support_polygon_centre_offset(uint8_t swing_leg) override;
};

