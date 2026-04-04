#include "Copter.h"

void Copter::tritilt_update()
{
#if AP_SCRIPTING_ENABLED && AP_MOTORS_TRI_TILT_ENABLED
    // TriTilt：使用专用 RCx_OPTION 输入控制俯仰角速度与回零
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_TRI &&
        (AP_Motors::motor_frame_type)g.frame_type.get() == AP_Motors::MOTOR_FRAME_TYPE_TRI_TILT &&
        motors->get_tilt_enable()) {

        RC_Channel *pitch_ctrl_ch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::TRITILT_PITCH_CTRL);
        RC_Channel *return_zero_ch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::TRITILT_RETURN_TO_ZERO);
        const uint16_t pwm_pitch = (pitch_ctrl_ch != nullptr) ? pitch_ctrl_ch->get_radio_in() : 0U;
        const uint16_t pwm_return_zero = (return_zero_ch != nullptr) ? return_zero_ch->get_radio_in() : 0U;

        // 最大俯仰转动角速度（度/秒）- 后续可改为参数
        const float max_rate_deg_per_sec = 10.0f;              // TODO: make this a configurable parameter
        const float return_to_zero_rate_deg_per_sec = 5.0f;   // TODO: make this a configurable parameter

        // 静态变量：保存累计俯仰偏置角与提示状态
        float& pitch_off_deg = _tritilt.pitch_off_deg;
        static bool return_to_zero_latched = false;
        static bool upper_limit_reported = false;
        static bool lower_limit_reported = false;
        static bool zero_position_reported = true;
        static uint32_t wait_pitch_center_warn_ms = 0;
        static uint32_t return_zero_not_low_warn_ms = 0;

        float pitch_rate_deg_per_sec = 0.0f;

        // MAVLink 外部命令处理（MAV_CMD_USER_1）
        if (_tritilt.cmd_pending) {
            _tritilt.cmd_pending = false;
            const float max_lim = MIN(MAX(0.0f, motors->get_tilt_max_deg()), 90.0f);
            if (_tritilt.cmd_is_rate) {
                pitch_rate_deg_per_sec = _tritilt.cmd_value;
            } else {
                pitch_off_deg = constrain_float(_tritilt.cmd_value, -max_lim, max_lim);
            }
        }

        const uint32_t now_ms = AP_HAL::millis();
        const bool pitch_ctrl_valid = (pwm_pitch >= 900U && pwm_pitch <= 2100U);
        const bool return_zero_valid = (pwm_return_zero >= 900U && pwm_return_zero <= 2100U);
        const bool pitch_in_mid_deadzone = pitch_ctrl_valid && (pwm_pitch >= 1450U && pwm_pitch <= 1550U);
        const bool pitch_manual_active = (pwm_pitch > 1000U && pwm_pitch < 2000U) &&
                                         (pwm_pitch < 1450U || pwm_pitch > 1550U);
        const bool return_zero_is_low = return_zero_valid && (pwm_return_zero < 1500U);
        const bool return_to_zero_trigger = return_zero_valid && (pwm_return_zero > 1800U && pwm_return_zero < 2100U);
        // 将无效输入或断开连接也视为回零复位条件
        const bool return_to_zero_reset = !return_zero_valid || (pwm_return_zero < 1500U);

        if (return_to_zero_trigger && !return_to_zero_latched) {
            if (pitch_in_mid_deadzone) {
                return_to_zero_latched = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "TriTilt: return-to-zero triggered");
                wait_pitch_center_warn_ms = 0;
            } else if (now_ms - wait_pitch_center_warn_ms >= 1000U) {
                gcs().send_text(MAV_SEVERITY_WARNING, "TriTilt: waiting pitch control centered before return-to-zero");
                wait_pitch_center_warn_ms = now_ms;
            }
        } else if (return_to_zero_latched && return_to_zero_reset) {
            return_to_zero_latched = false;
            gcs().send_text(MAV_SEVERITY_NOTICE, "TriTilt: return-to-zero reset");
            wait_pitch_center_warn_ms = 0;
        } else if (!return_to_zero_trigger) {
            wait_pitch_center_warn_ms = 0;
        }

        // 处理手动俯仰控制或已锁存的回零模式
        if (pitch_ctrl_valid || return_to_zero_latched) {
            // 将俯仰偏置角限制在配置最大值与 90 度两者中的较小值范围内
            const float max_pitch_off_deg = MIN(MAX(0.0f, motors->get_tilt_max_deg()), 90.0f);
            const float min_pitch_off_deg = -max_pitch_off_deg;
            const float dt = copter.G_Dt;
            const float limit_msg_epsilon_deg = 0.01f;

            if (return_to_zero_latched) {
                // 回零模式优先于手动俯仰输入
                if (pitch_off_deg > 0.0f) {
                    pitch_rate_deg_per_sec = -return_to_zero_rate_deg_per_sec;
                } else if (pitch_off_deg < 0.0f) {
                    pitch_rate_deg_per_sec = return_to_zero_rate_deg_per_sec;
                }
                return_zero_not_low_warn_ms = 0;
            } else if (pitch_manual_active && !return_zero_is_low) {
                if (now_ms - return_zero_not_low_warn_ms >= 1000U) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "TriTilt: return-to-zero switch must be low for manual pitch");
                    return_zero_not_low_warn_ms = now_ms;
                }
            } else if (pwm_pitch > 1000U && pwm_pitch < 2000U) {
                return_zero_not_low_warn_ms = 0;
                // <= 1000 或 >= 2000：角速度为 0
                // 1450 <= 输入 <= 1550：死区内角速度为 0
                // 其余区间线性映射为角速度（-max_rate 到 +max_rate，1500 对应 0）
                if (pwm_pitch < 1450U || pwm_pitch > 1550U) {
                    // 将 1000..2000 映射到 -1..+1（1500 -> 0）
                    float norm = (float(pwm_pitch) - 1500.0f) * (1.0f / 500.0f);
                    norm = constrain_float(norm, -1.0f, 1.0f);
                    pitch_rate_deg_per_sec = norm * max_rate_deg_per_sec;
                }
            } else {
                return_zero_not_low_warn_ms = 0;
            }

            // 检查积分后是否会继续推向限幅边界；若已在边界且仍试图继续增大，则停止积分
            if ((pitch_off_deg >= max_pitch_off_deg && pitch_rate_deg_per_sec > 0.0f) ||
                (pitch_off_deg <= min_pitch_off_deg && pitch_rate_deg_per_sec < 0.0f)) {
                // 到达限幅时停止继续转动
                pitch_rate_deg_per_sec = 0.0f;
            } else if (return_to_zero_latched && !is_zero(pitch_rate_deg_per_sec)) {
                const float next_pitch_off_deg = pitch_off_deg + pitch_rate_deg_per_sec * dt;

                // 回零过程中若本次步进将跨过中心点，则直接吸附到 0 度
                if ((pitch_off_deg > 0.0f && next_pitch_off_deg <= 0.0f) ||
                    (pitch_off_deg < 0.0f && next_pitch_off_deg >= 0.0f)) {
                    pitch_off_deg = 0.0f;
                    pitch_rate_deg_per_sec = 0.0f;
                } else {
                    pitch_off_deg = constrain_float(next_pitch_off_deg, min_pitch_off_deg, max_pitch_off_deg);
                }
            } else {
                // 对角速度积分，得到累计俯仰偏置角
                pitch_off_deg += pitch_rate_deg_per_sec * dt;

                // 积分后再次夹紧到限幅范围内
                pitch_off_deg = constrain_float(pitch_off_deg, min_pitch_off_deg, max_pitch_off_deg);
            }

            // 写入角速度与累计俯仰偏置角
            AP::ins().set_imu_pitch_rot_rate_deg_per_sec(pitch_rate_deg_per_sec);
            AP::ins().set_imu_pitch_rot_deg(pitch_off_deg);
            AP::compass().set_imu_pitch_rot_deg(pitch_off_deg);

#if HAL_LOGGING_ENABLED
            {
                const float pit_virt = AP::ins().get_imu_pitch_rot_deg();
                const float pit_ahrs = degrees(ahrs.get_pitch());
                Log_Write_TriTilt(pitch_off_deg, pit_virt, pit_ahrs,
                                  pit_ahrs + pit_virt,
                                  pitch_rate_deg_per_sec,
                                  degrees(ahrs.get_gyro().y));
            }
#endif

            // 每 200ms 通过 NAMED_VALUE_FLOAT 发送遥测到地面站
            static uint32_t named_float_send_ms = 0;
            if (now_ms - named_float_send_ms >= 200U) {
                named_float_send_ms = now_ms;
                const float pit_true = degrees(ahrs.get_pitch()) + AP::ins().get_imu_pitch_rot_deg();
                gcs().send_named_float("PitOff",  pitch_off_deg);
                gcs().send_named_float("PitTrue", pit_true);
                gcs().send_named_float("RTZCh",   return_zero_is_low ? 0.0f : 1.0f);
            }

            const bool at_upper_limit = (pitch_off_deg >= max_pitch_off_deg - limit_msg_epsilon_deg);
            const bool at_lower_limit = (pitch_off_deg <= min_pitch_off_deg + limit_msg_epsilon_deg);
            const bool at_zero_position = is_zero(pitch_off_deg);

            if (at_upper_limit && !upper_limit_reported) {
                upper_limit_reported = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "TriTilt: pitch upper limit reached");
            } else if (!at_upper_limit) {
                upper_limit_reported = false;
            }

            if (at_lower_limit && !lower_limit_reported) {
                lower_limit_reported = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "TriTilt: pitch lower limit reached");
            } else if (!at_lower_limit) {
                lower_limit_reported = false;
            }

            if (at_zero_position && !zero_position_reported) {
                zero_position_reported = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "TriTilt: pitch returned to zero");
            } else if (!at_zero_position) {
                zero_position_reported = false;
            }
        } else {
            // 没有有效手动输入且回零未激活时，将角速度置 0
            AP::ins().set_imu_pitch_rot_rate_deg_per_sec(0.0f);
        }
    }
#endif
}
