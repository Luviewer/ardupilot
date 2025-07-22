#include "AP_QuadRuped_WAVE.h"

extern const AP_HAL::HAL& hal;

void AP_QuadRuped_WAVE::gait_init()
{
    // 波浪步态设置 - 四条腿依次移动
    // 波浪步态设置 - 四条腿依次移动
    gait_step_leg_start[Leg_RF] = 0;                       // 右前腿
    gait_step_leg_start[Leg_RB] = 3 * gait_step_total / 4; // 左后腿
    gait_step_leg_start[Leg_LB] = gait_step_total / 2;     // 右后腿
    gait_step_leg_start[Leg_LF] = gait_step_total / 4;     // 左前腿

    // 调整步态参数
    gait_travel_divisor = gait_step_total / 2;
    gait_lift_divisor   = 8;
}

