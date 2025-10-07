#include "AP_QuadRuped_Wave_COG.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_QuadRuped_Wave_COG::AP_QuadRuped_Wave_COG(AP_QuadRuped& frontend,
                                             AP_QuadRuped::QuadRuped_State& state,
                                             AP_AHRS_View& ahrs,
                                             AP_Motors& motors)
    : AP_QuadRuped_Wave(frontend, state, ahrs, motors)
{
}

void AP_QuadRuped_Wave_COG::update_leg()
{
    const Vector3f prev_offset       = centre_offset;
    const Vector3f prev_offset_target = centre_offset_target;

    AP_QuadRuped_Wave::update_leg();

    centre_offset        = prev_offset;
    centre_offset_target = prev_offset_target;

    const uint8_t swing_leg = get_active_leg_index();
    calculate_support_polygon_centre_offset(swing_leg);
}

void AP_QuadRuped_Wave_COG::calculate_support_polygon_centre_offset(uint8_t swing_leg)
{
    const float offset_ratio = centre_offset_ratio.get();

    if (offset_ratio <= 0.0f) {
        centre_offset.zero();
        centre_offset_target.zero();
        return;
    }

    Vector3f support_centre(0.0f, 0.0f, 0.0f);
    uint8_t support_leg_count = 0;

    for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
        if (i == swing_leg) {
            continue;
        }

        Vector3f foot = endpoint_leg_frame[i] + endpoint_leg_pos[i] + gait_pos_xyz[i];
        foot.z         = 0.0f;
        support_centre += foot;
        support_leg_count++;
    }

    if (support_leg_count > 0) {
        support_centre /= support_leg_count;
    }

    Vector3f desired_offset = support_centre;
    desired_offset.z        = 0.0f;

    int32_t delta_step = gait_step_now - gait_step_leg_start[swing_leg];
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    const float p                 = (float)delta_step / (float)gait_step_total;
    const float swing_ratio       = 0.25f;
    const float pre_shift_ratio   = 0.15f; // 提前预载重心，落脚前先平移
    const float release_ratio_raw = 1.0f - swing_ratio - pre_shift_ratio;
    const float release_ratio     = (release_ratio_raw > 0.0f) ? release_ratio_raw : 0.0f;

    float offset_weight = 0.0f;

    if ((pre_shift_ratio > 0.0f) && (p >= (1.0f - pre_shift_ratio))) {
        const float phase = constrain_float((p - (1.0f - pre_shift_ratio)) / pre_shift_ratio, 0.0f, 1.0f);
        offset_weight     = 0.5f * (1.0f - cosf(phase * M_PI));

    } else if (p < swing_ratio) {
        // 在抬腿阶段保持重心偏移，避免支撑边界过近
        offset_weight = 1.0f;

    } else if ((release_ratio > 0.0f) && (p < (swing_ratio + release_ratio))) {
        const float phase = constrain_float((p - swing_ratio) / release_ratio, 0.0f, 1.0f);
        offset_weight     = 0.5f * (1.0f + cosf(phase * M_PI));
    }

    centre_offset_target = desired_offset * offset_weight * offset_ratio;

    const uint32_t now_ms = AP_HAL::millis();
    float          dt_s   = 0.0f;
    if (_com_last_ms != 0 && now_ms >= _com_last_ms) {
        dt_s = (now_ms - _com_last_ms) * 0.001f;
    }
    _com_last_ms = now_ms;

    const float fc    = fmaxf(_com_fc, 0.1f);
    const float tau   = 1.0f / (2.0f * M_PI * fc);
    const float alpha = (dt_s > 0.0f) ? constrain_float(dt_s / (tau + dt_s), 0.0f, 1.0f) : 1.0f;

    centre_offset += (centre_offset_target - centre_offset) * alpha;

    constexpr float MAX_OFFSET_X = 30.0f;
    constexpr float MAX_OFFSET_Y = 20.0f;
    constexpr float MAX_OFFSET_Z = 5.0f;

    centre_offset.x = constrain_float(centre_offset.x, -MAX_OFFSET_X, MAX_OFFSET_X);
    centre_offset.y = constrain_float(centre_offset.y, -MAX_OFFSET_Y, MAX_OFFSET_Y);
    centre_offset.z = constrain_float(centre_offset.z, -MAX_OFFSET_Z, MAX_OFFSET_Z);
}
