#include "Copter.h"

#if AP_RANGEFINDER_ENABLED
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#endif

static constexpr uint32_t IMPEDANCE_TARE_SETTLE_MS = 600;
static constexpr uint32_t IMPEDANCE_RANGE_TIMEOUT_MS = 300;
static constexpr float IMPEDANCE_RANGE_JUMP_M = 0.2f;
// A forward ray mounted at the probe tip becomes invalid as it enters the
// contact surface.  Permit a tightly bounded handoff to the force sensor only
// after the last valid range was already inside this near-contact zone.
static constexpr float IMPEDANCE_RANGE_HANDOFF_M = 0.10f;
static constexpr uint32_t IMPEDANCE_RANGE_HANDOFF_TIMEOUT_MS = 2000;
static constexpr uint16_t IMPEDANCE_ALT_HOLD_PWM = 1800U;
static constexpr float IMPEDANCE_FORCE_LPF_HZ = 3.0f;
static constexpr float IMPEDANCE_REACQUIRE_SPEED_MS = 0.03f;
static constexpr float IMPEDANCE_CONFIDENCE_TC_S = 0.3f;
static constexpr float IMPEDANCE_FORCE_REF_RATE_NS = 1.0f;
static constexpr float IMPEDANCE_CONTACT_ON_N = 0.5f;
static constexpr float IMPEDANCE_CONTACT_OFF_N = 0.25f;
static constexpr uint8_t IMPEDANCE_CONTACT_SAMPLES = 3;
// Keep force control active through a bounded impact transient before
// declaring a genuine release. Force, range, position and sensor safety
// exits remain active during this window.
static constexpr uint8_t IMPEDANCE_BASELINE_SAMPLES = 20;
static constexpr float IMPEDANCE_NOISE_SIGMA_MULTIPLIER = 4.0f;

const AP_Param::GroupInfo ModeImpedance::var_info[] = {
    // @Param: FREF
    // @DisplayName: 初始接触目标力
    // @Description: 首次确认接触并进入恒力控制时使用的工具轴目标力。俯仰摇杆可在恒力阶段调整该目标。
    // @Range: 1 10
    // @Units: N
    // @User: Standard
    AP_GROUPINFO("FREF", 11, ModeImpedance, _force_ref_default_n, 1.0f),

    // @Param: FREF_MAX
    // @DisplayName: 最大接触目标力
    // @Description: 飞手通过俯仰摇杆能够设定的最大工具轴目标力，用于限制期望接触力。
    // @Range: 1 10
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("FREF_MAX", 12, ModeImpedance, _force_ref_max_n, 10.0f),

    // @Param: F_ABORT
    // @DisplayName: 接触力中止上限
    // @Description: 原始校正力或滤波力达到该值时立即停止增力并进入自动后退。该值必须高于正常目标力。
    // @Range: 1 30
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("F_ABORT", 15, ModeImpedance, _force_abort_n, 12.0f),

    // @Param: F_KP
    // @DisplayName: 接触力比例增益
    // @Description: 每牛顿力误差产生的归一化前向推力指令。增大可加快响应，但过大可能导致振荡或冲击。
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("F_KP", 16, ModeImpedance, _force_kp, 0.05f),

    // @Param: F_KI
    // @DisplayName: 接触力积分增益
    // @Description: 每牛顿秒累计力误差产生的归一化前向推力指令，用于消除稳态力误差。过大可能造成超调或振荡。
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("F_KI", 17, ModeImpedance, _force_ki, 0.02f),

    // @Param: F_IMAX
    // @DisplayName: 接触力积分限幅
    // @Description: 积分项允许贡献的最大归一化前向推力绝对值，用于限制积分累积和接触力超调。
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("F_IMAX", 19, ModeImpedance, _force_i_max, 0.3f),

    // @Param: FX_MAX
    // @DisplayName: 最大前向力指令
    // @Description: 恒力控制器允许输出的最大归一化前向推力绝对值，是接触作业的最终前向输出限幅。
    // @Range: 0.05 1
    // @User: Advanced
    AP_GROUPINFO("FX_MAX", 20, ModeImpedance, _fx_max, 0.4f),

    // @Param: FX_RATE
    // @DisplayName: 前向力变化率
    // @Description: 归一化前向推力指令每秒允许的最大变化量，用于平滑增力和卸力。越小越平缓，但响应越慢。
    // @Range: 0.05 5
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("FX_RATE", 21, ModeImpedance, _fx_slew_rate, 0.5f),

    // @Param: APP_SPD
    // @DisplayName: 接触接近速度
    // @Description: 激光测得的表面距离大于 SLOW_DIST 时，沿锁定工具轴向前接近的速度。
    // @Range: 0.02 0.5
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("APP_SPD", 22, ModeImpedance, _approach_speed_ms, 0.15f),

    // @Param: SLOW_SPD
    // @DisplayName: 近表面接近速度
    // @Description: 激光测得的表面距离不大于 SLOW_DIST 时使用的低速接近速度；自动后退速度也基于该值。
    // @Range: 0.01 0.2
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("SLOW_SPD", 23, ModeImpedance, _slow_speed_ms, 0.05f),

    // @Param: SLOW_DIST
    // @DisplayName: 接近减速距离
    // @Description: 工具端到表面的激光距离不大于该值时，从 APP_SPD 切换到 SLOW_SPD 接近。
    // @Range: 0.05 1
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("SLOW_DIST", 25, ModeImpedance, _slow_distance_m, 0.25f),

    // @Param: RET_DIST
    // @DisplayName: 自动后退距离
    // @Description: 发生故障、超力或接触任务结束后，飞行器沿锁定工具轴反方向自动移动的目标距离。
    // @Range: 0.1 2
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RET_DIST", 26, ModeImpedance, _retreat_distance_m, 0.5f),

    // @Param: APP_TOUT
    // @DisplayName: 接近超时时间
    // @Description: 从开始接近到确认接触允许的最长时间，超过后判定接近失败并进入自动后退。
    // @Range: 1 60
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("APP_TOUT", 27, ModeImpedance, _approach_timeout_s, 10.0f),

    // @Param: RET_TOUT
    // @DisplayName: 后退超时时间
    // @Description: 完成 RET_DIST 自动后退允许的最长时间。超时后停止后退并报告错误。
    // @Range: 1 30
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("RET_TOUT", 28, ModeImpedance, _retreat_timeout_s, 8.0f),

    // @Param: RFND_IDX
    // @DisplayName: 前向测距仪序号
    // @Description: 接触模式用于测量工具轴前方距离的测距仪实例序号，从零开始；所选测距仪必须配置为朝前方向。
    // @Range: 0 9
    // @User: Advanced
    AP_GROUPINFO("RFND_IDX", 29, ModeImpedance, _rangefinder_instance, 0),

    // @Param: SAT_TOUT
    // @DisplayName: 执行器饱和超时
    // @Description: 前向力输出或电机滚转、俯仰、偏航、油门上限持续受限达到该时间后，判定控制能力不足并自动后退。
    // @Range: 0.1 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("SAT_TOUT", 32, ModeImpedance, _saturation_timeout_s, 1.0f),

    // @Param: FS_REV
    // @DisplayName: 接触力方向反转
    // @Description: 当力传感器安装方向导致受压读数符号相反时，反转工具轴力测量值。
    // @Values: 0:正常,1:反转
    // @User: Advanced
    AP_GROUPINFO("FS_REV", 33, ModeImpedance, _force_reverse, 0),

    // @Param: REL_TOUT
    // @DisplayName: 接触释放确认时间
    // @Description: 恒力阶段检测到低于释放阈值后，继续保持力控的确认时间。用于区分碰撞瞬态和真实失触；力上限、测距、位置和传感器故障仍立即退出。
    // @Range: 0.1 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("REL_TOUT", 34, ModeImpedance, _release_confirm_s, 1.0f),

    // @Param: F_OFF
    // @DisplayName: 接触释放力阈值
    // @Description: 恒力阶段滤波力连续低于该阈值时，开始释放确认计时。必须低于接触建立阈值，过高会把低力稳态误判为失触。
    // @Range: 0.01 5
    // @Units: N
    // @User: Advanced
    AP_GROUPINFO("F_OFF", 35, ModeImpedance, _force_off_threshold_n, IMPEDANCE_CONTACT_OFF_N),

    AP_GROUPEND
};

ModeImpedance::ModeImpedance() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeImpedance::init(bool ignore_checks)
{
    auto *att6 = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att6 == nullptr || !motors->get_tilt_enable()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: 6DoF tilt control required");
        return false;
    }

    loiter_nav->set_pilot_desired_acceleration_rad(0.0f, 0.0f);
    loiter_nav->init_target_m(pos_control->get_pos_estimate_NED_m().xy() -
                              pos_control->get_pos_offset_NED_m().xy());
    pos_control->NE_stop_vel_stabilisation();
    loiter_nav->clear_vel_offset_NE_ms();

    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    reset_contact_control();
    _locked_yaw_rad = ahrs.get_yaw();
    _force_ref_n = constrain_float(_force_ref_default_n, 1.0f, _force_ref_max_n);

#if AP_CONTACT_SENSOR_ENABLED
    if (!copter.contact_sensor.healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact sensor not healthy");
        return false;
    }
    if (!copter.contact_sensor.tare()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact sensor tare failed");
        return false;
    }
    _tare_requested = true;
    _tare_start_ms = AP_HAL::millis();
#else
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact sensor unavailable");
    return false;
#endif

    return true;
}

void ModeImpedance::exit()
{
    if (auto *att6 = AC_AttitudeControl_Multi_6DoF::get_singleton()) {
        att6->clear_external_forward();
    }
    motors->set_forward(0.0f);
    loiter_nav->clear_vel_offset_NE_ms();
    loiter_nav->clear_pilot_desired_acceleration();
    reset_contact_control();
}

void ModeImpedance::reset_contact_control()
{
    _state = ContactState::READY;
    _fault_reason = FaultReason::NONE;
    _force_integral = 0.0f;
    _force_p_out = 0.0f;
    _force_i_out = 0.0f;
    _fx_target = 0.0f;
    _fx_command = 0.0f;
    _confidence = 0.0f;
    _noise_mean_n = 0.0f;
    _noise_m2_n = 0.0f;
    _noise_count = 0;
    _previous_tool_distance_m = 0.0f;
    _last_rangefinder_ms = 0;
    _last_valid_tool_distance_m = 0.0f;
    _last_valid_tool_distance_ms = 0;
    const float reset_off_n = constrain_float(_force_off_threshold_n.get(), 0.01f, IMPEDANCE_CONTACT_ON_N);
    _contact_detector.configure(IMPEDANCE_CONTACT_ON_N, reset_off_n, IMPEDANCE_CONTACT_SAMPLES);
    _contact_detector.reset();
    _last_force_sequence = 0;
    _last_force_sample_ms = 0;
    _last_force_dt_s = 0.05f;
    _body_x_velocity_command_ms = 0.0f;
    _state_start_ms = AP_HAL::millis();
    _saturation_start_ms = 0;
    _reacquire_used = false;
    _auto_start_pending = true;
    _tare_requested = false;
    _tare_reported = false;
    _tare_start_ms = 0;
}

void ModeImpedance::set_state(ContactState state, FaultReason reason)
{
    if (_state == state && reason == FaultReason::NONE) {
        return;
    }

    _state = state;
    _state_start_ms = AP_HAL::millis();
    _saturation_start_ms = 0;
    if (reason != FaultReason::NONE) {
        _fault_reason = reason;
    }

    switch (state) {
    case ContactState::READY:
        _force_integral = 0.0f;
        _fx_target = 0.0f;
        _reacquire_used = false;
        set_body_x_velocity(0.0f);
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: ready");
        break;
    case ContactState::APPROACH:
        _fault_reason = FaultReason::NONE;
        _locked_yaw_rad = ahrs.get_yaw();
        _locked_tool_axis_ne = Vector2f{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
        _approach_start_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
        _force_integral = 0.0f;
        _force_ref_n = constrain_float(_force_ref_default_n, 1.0f, _force_ref_max_n);
        _contact_detector.reset();
        loiter_nav->init_target_m(pos_control->get_pos_estimate_NED_m().xy() -
                                  pos_control->get_pos_offset_NED_m().xy());
        pos_control->NE_stop_vel_stabilisation();
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: approach");
        break;
    case ContactState::CONTACT_CONFIRM: {
        const Vector2f approach_delta = pos_control->get_pos_estimate_NED_m().xy().tofloat() - _approach_start_ne_m;
        if (approach_delta.length() > 0.05f) {
            _locked_tool_axis_ne = approach_delta.normalized();
        }
    }
    set_body_x_velocity(0.0f);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: confirming contact");
    break;
    case ContactState::FORCE_HOLD: {
        loiter_nav->clear_vel_offset_NE_ms();
        loiter_nav->init_target_m(pos_control->get_pos_estimate_NED_m().xy() -
                                  pos_control->get_pos_offset_NED_m().xy());
        _contact_start_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
        // Make the velocity-to-force transition bumpless.  The approach
        // controller already has the correct model-specific forward sign;
        // preserve its current actuator demand while the force PI takes over.
        _confidence = 1.0f;
        const float force_error_n = _force_ref_n - _force_filtered_n;
        _force_p_out = _force_kp.get() * force_error_n;
        const float fx_max = constrain_float(_fx_max.get(), 0.0f, 1.0f);
        const float current_forward = constrain_float(motors->get_forward(), -fx_max, fx_max);
        _force_integral = constrain_float(current_forward - _force_p_out,
                                          -_force_i_max.get(), _force_i_max.get());
        _force_i_out = _force_integral;
        _fx_target = constrain_float(_force_p_out + _force_i_out, -fx_max, fx_max);
        _fx_command = _fx_target;
        _release_pending_start_ms = 0;
        _contact_detector.reset(true);
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: force hold");
        break;
    }
    case ContactState::REACQUIRE:
        _force_integral = 0.0f;
        _fx_target = 0.0f;
        _release_pending_start_ms = 0;
        _reacquire_used = true;
        _contact_detector.reset();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact lost, reacquire");
        break;
    case ContactState::RETREAT:
        _force_integral = 0.0f;
        _fx_target = 0.0f;
        _release_pending_start_ms = 0;
        _retreat_start_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
        _retreat_start_distance_m = _tool_distance_healthy ? _tool_distance_m : _last_valid_tool_distance_m;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: retreat reason %u", unsigned(_fault_reason));
        break;
    }
}

bool ModeImpedance::update_force_sample()
{
#if AP_CONTACT_SENSOR_ENABLED
    AP_ContactSensor::ForceSample sample;
    if (!copter.contact_sensor.get_force_sample(sample) || sample.sequence == _last_force_sequence) {
        return false;
    }

    _last_force_sequence = sample.sequence;
    const uint32_t previous_sample_ms = _last_force_sample_ms;
    _last_force_sample_ms = sample.timestamp_ms;
    _force_raw_n = (_force_reverse.get() != 0) ? -sample.tool_force_n : sample.tool_force_n;

    if (_state == ContactState::READY && _noise_count < 200U) {
        _noise_count++;
        const float delta = _force_raw_n - _noise_mean_n;
        _noise_mean_n += delta / float(_noise_count);
        _noise_m2_n += delta * (_force_raw_n - _noise_mean_n);
    }

    const float corrected_force_n = _force_raw_n - _noise_mean_n;
    float sample_dt = 0.05f;
    if (previous_sample_ms != 0 && sample.timestamp_ms > previous_sample_ms) {
        sample_dt = constrain_float((sample.timestamp_ms - previous_sample_ms) * 0.001f, 0.001f, 0.2f);
    }
    _last_force_dt_s = sample_dt;
    // Preserve the physical filter pole at low or irregular sensor rates.
    // Euler alpha approaches unity at 20 Hz, bypassing the intended filter.
    const float alpha = 1.0f - expf(-sample_dt * IMPEDANCE_FORCE_LPF_HZ * M_2PI);
    _force_filtered_n += (corrected_force_n - _force_filtered_n) * alpha;
#if HAL_LOGGING_ENABLED
    // Record actual consumed samples; the flight-loop log repeats held values.
    copter.logger.Write("IMFS", "TimeUS,Seq,SampleMS,Dt,Raw,Filt", "QIIfff",
                        AP_HAL::micros64(), sample.sequence, sample.timestamp_ms,
                        double(sample_dt), double(_force_raw_n), double(_force_filtered_n));
#endif
    return true;
#else
    return false;
#endif
}

void ModeImpedance::update_tool_distance()
{
    _tool_distance_healthy = false;
    _tool_distance_m = 0.0f;
#if AP_RANGEFINDER_ENABLED
    AP_RangeFinder_Backend *backend = copter.rangefinder.get_backend(uint8_t(MAX(_rangefinder_instance.get(), 0)));
    if (backend == nullptr || backend->orientation() != ROTATION_NONE ||
        backend->status() != RangeFinder::Status::Good ||
        AP_HAL::millis() - backend->last_reading_ms() > IMPEDANCE_RANGE_TIMEOUT_MS) {
        return;
    }
    const uint32_t reading_ms = backend->last_reading_ms();
    _tool_distance_m = backend->distance();
    if (reading_ms != _last_rangefinder_ms) {
        const bool distance_jump = _last_rangefinder_ms != 0 &&
                                   fabsf(_tool_distance_m - _previous_tool_distance_m) > IMPEDANCE_RANGE_JUMP_M;
        _last_rangefinder_ms = reading_ms;
        if (distance_jump) {
            return;
        }
        _previous_tool_distance_m = _tool_distance_m;
    } else if (fabsf(_tool_distance_m - _previous_tool_distance_m) > IMPEDANCE_RANGE_JUMP_M) {
        return;
    }
    _tool_distance_healthy = _tool_distance_m >= backend->min_distance() &&
                             _tool_distance_m <= backend->max_distance();
    if (_tool_distance_healthy) {
        _last_valid_tool_distance_m = _tool_distance_m;
        _last_valid_tool_distance_ms = AP_HAL::millis();
    }
#endif
}

bool ModeImpedance::range_contact_handoff_available(uint32_t now_ms) const
{
    if (_tool_distance_healthy) {
        return true;
    }
    if (_last_valid_tool_distance_ms == 0 || _last_valid_tool_distance_m > IMPEDANCE_RANGE_HANDOFF_M) {
        return false;
    }
#if AP_CONTACT_SENSOR_ENABLED
    if (!copter.contact_sensor.healthy()) {
        return false;
    }
    // Once force hold is positively established the force sensor is the
    // primary normal-axis feedback, so the tip ray may remain occluded for the
    // duration of contact.  All pre-contact/reacquire states retain the short
    // handoff deadline.
    return _state == ContactState::FORCE_HOLD ||
           now_ms - _last_valid_tool_distance_ms <= IMPEDANCE_RANGE_HANDOFF_TIMEOUT_MS;
#else
    return false;
#endif
}

float ModeImpedance::contact_on_threshold_n() const
{
    float sigma_n = 0.0f;
    if (_noise_count > 1U) {
        sigma_n = sqrtf(_noise_m2_n / float(_noise_count - 1U));
    }
    return MAX(IMPEDANCE_CONTACT_ON_N, IMPEDANCE_NOISE_SIGMA_MULTIPLIER * sigma_n);
}

void ModeImpedance::update_confidence(float dt)
{
    float requested = 0.0f;
    bool force_sensor_healthy = false;
#if AP_CONTACT_SENSOR_ENABLED
    force_sensor_healthy = copter.contact_sensor.healthy();
#endif
    const bool contact_handoff = range_contact_handoff_available(AP_HAL::millis());
    if (contact_handoff && force_sensor_healthy &&
        (_state == ContactState::CONTACT_CONFIRM || _state == ContactState::FORCE_HOLD || _state == ContactState::REACQUIRE)) {
        const float near_m = MAX(_slow_distance_m.get(), 0.05f);
        const float confidence_distance_m = _tool_distance_healthy ? _tool_distance_m : _last_valid_tool_distance_m;
        if (confidence_distance_m <= near_m) {
            requested = 1.0f;
        } else if (confidence_distance_m < 2.0f * near_m) {
            const float phase = (confidence_distance_m - near_m) / near_m;
            requested = 0.5f * (1.0f + cosf(phase * M_PI));
        }

        // A loaded probe constrains the normal position physically. Estimator
        // drift must not suppress its force loop while contact is confirmed.
        // Retain the displacement guard when force evidence becomes uncertain.
        if (_state == ContactState::FORCE_HOLD && !_contact_detector.contact()) {
            const Vector2f position_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
            const Vector2f forward_ne = _locked_tool_axis_ne;
            const float tool_error_m = fabsf((position_ne_m - _contact_start_ne_m) * forward_ne);
            if (tool_error_m > 0.1f) {
                const float error_phase = constrain_float((tool_error_m - 0.1f) / 0.4f, 0.0f, 1.0f);
                requested *= 0.5f * (1.0f + cosf(error_phase * M_PI));
            }
        }
    }

    const float alpha = constrain_float(dt / IMPEDANCE_CONFIDENCE_TC_S, 0.0f, 1.0f);
    _confidence += (requested - _confidence) * alpha;
}

void ModeImpedance::update_force_controller(float dt, bool new_force_sample)
{
    if (_state != ContactState::FORCE_HOLD) {
        _fx_target = 0.0f;
    } else if (new_force_sample) {
        const float error_n = _force_ref_n - _force_filtered_n;
        _force_p_out = _force_kp.get() * error_n;

        const float fx_max = constrain_float(_fx_max.get(), 0.0f, 1.0f);
        const float unsaturated = _force_p_out + _force_integral;
        const bool drives_out_of_saturation = (unsaturated >= fx_max && error_n < 0.0f) ||
                                              (unsaturated <= -fx_max && error_n > 0.0f);
        if (_confidence > 0.05f && (fabsf(unsaturated) < fx_max || drives_out_of_saturation)) {
            _force_integral += _force_ki.get() * error_n * _last_force_dt_s;
            _force_integral = constrain_float(_force_integral, -_force_i_max.get(), _force_i_max.get());
        }
        _force_i_out = _force_integral;
        _fx_target = constrain_float((_force_p_out + _force_i_out) * _confidence, -fx_max, fx_max);
    }

    const float max_change = MAX(_fx_slew_rate.get(), 0.01f) * dt;
    _fx_command += constrain_float(_fx_target - _fx_command, -max_change, max_change);
}

void ModeImpedance::set_body_x_velocity(float speed_ms)
{
    _body_x_velocity_command_ms = speed_ms;
}

bool ModeImpedance::use_ne_velocity_control() const
{
    return _state == ContactState::APPROACH ||
           _state == ContactState::CONTACT_CONFIRM ||
           _state == ContactState::REACQUIRE ||
           (_state == ContactState::RETREAT && fabsf(_fx_command) <= 0.01f);
}

float ModeImpedance::retreat_distance_done_m() const
{
    const Vector2f position_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
    return -((position_ne_m - _retreat_start_ne_m) * _locked_tool_axis_ne);
}

void ModeImpedance::run_contact_state(float dt, bool new_force_sample)
{
    const uint32_t now_ms = AP_HAL::millis();
    const float contact_on_n = contact_on_threshold_n();
    const float contact_off_n = constrain_float(_force_off_threshold_n.get(), 0.01f, contact_on_n);
    _contact_detector.configure(contact_on_n, contact_off_n, IMPEDANCE_CONTACT_SAMPLES);
    const bool detection_active = _state == ContactState::APPROACH ||
                                  _state == ContactState::CONTACT_CONFIRM ||
                                  _state == ContactState::FORCE_HOLD ||
                                  _state == ContactState::REACQUIRE;
    const AP_ContactDetector::Event contact_event = (new_force_sample && detection_active) ?
                                                    _contact_detector.update(_force_filtered_n) :
                                                    AP_ContactDetector::Event::NONE;

    if (_auto_start_pending && _state == ContactState::READY) {
        bool force_sensor_healthy = false;
#if AP_CONTACT_SENSOR_ENABLED
        force_sensor_healthy = copter.contact_sensor.healthy();
#endif
        if (_tool_distance_healthy && force_sensor_healthy && _tare_reported &&
            _noise_count >= IMPEDANCE_BASELINE_SAMPLES) {
            _auto_start_pending = false;
            set_state(ContactState::APPROACH);
        }
    }

    bool force_sensor_healthy = false;
#if AP_CONTACT_SENSOR_ENABLED
    force_sensor_healthy = copter.contact_sensor.healthy();
#endif
    if (_state != ContactState::READY && _state != ContactState::RETREAT && !force_sensor_healthy) {
        set_state(ContactState::RETREAT, FaultReason::FORCE_SENSOR);
    }
    if (_state != ContactState::READY && _state != ContactState::RETREAT && !copter.position_ok()) {
        set_state(ContactState::RETREAT, FaultReason::POSITION_ESTIMATE);
    }
    const float corrected_raw_force_n = _force_raw_n - _noise_mean_n;
    if (_state != ContactState::READY && _state != ContactState::RETREAT &&
        MAX(corrected_raw_force_n, _force_filtered_n) >= _force_abort_n.get()) {
        set_state(ContactState::RETREAT, FaultReason::FORCE_LIMIT);
    }

    switch (_state) {
    case ContactState::READY:
        set_body_x_velocity(0.0f);
        break;

    case ContactState::APPROACH:
        if (!range_contact_handoff_available(now_ms)) {
            set_state(ContactState::RETREAT, FaultReason::RANGEFINDER);
            break;
        }
        if (now_ms - _state_start_ms > uint32_t(MAX(_approach_timeout_s.get(), 1.0f) * 1000.0f)) {
            set_state(ContactState::RETREAT, FaultReason::APPROACH_TIMEOUT);
            break;
        }
        set_body_x_velocity((!_tool_distance_healthy || _tool_distance_m <= _slow_distance_m.get()) ?
                            _slow_speed_ms.get() : _approach_speed_ms.get());
        if (contact_event == AP_ContactDetector::Event::CONTACT) {
            set_state(ContactState::FORCE_HOLD);
        } else if (_contact_detector.contact_candidate()) {
            set_state(ContactState::CONTACT_CONFIRM);
        }
        break;

    case ContactState::CONTACT_CONFIRM:
        set_body_x_velocity(0.0f);
        if (!range_contact_handoff_available(now_ms)) {
            set_state(ContactState::RETREAT, FaultReason::RANGEFINDER);
            break;
        }
        if (contact_event == AP_ContactDetector::Event::CONTACT) {
            set_state(ContactState::FORCE_HOLD);
        } else if (new_force_sample && !_contact_detector.contact_candidate()) {
            set_state(_reacquire_used ? ContactState::REACQUIRE : ContactState::APPROACH);
        }
        break;

    case ContactState::FORCE_HOLD: {
        set_body_x_velocity(0.0f);
        if (!range_contact_handoff_available(now_ms)) {
            set_state(ContactState::RETREAT, FaultReason::RANGEFINDER);
            break;
        }
        if (contact_event == AP_ContactDetector::Event::RELEASE) {
            _release_pending_start_ms = now_ms;
        } else if (contact_event == AP_ContactDetector::Event::CONTACT) {
            _release_pending_start_ms = 0;
        }
        if (_release_pending_start_ms != 0 &&
            now_ms - _release_pending_start_ms >= uint32_t(MAX(_release_confirm_s.get(), 0.1f) * 1000.0f)) {
            set_state(_reacquire_used ? ContactState::RETREAT : ContactState::REACQUIRE,
                      _reacquire_used ? FaultReason::REACQUIRE_FAILED : FaultReason::NONE);
            break;
        }
        const float fx_limit = constrain_float(_fx_max.get(), 0.0f, 1.0f);
        const bool forward_output_limited = is_positive(fx_limit) && fabsf(_fx_target) >= 0.99f * fx_limit;
        if (forward_output_limited || motors->limit.roll || motors->limit.pitch ||
            motors->limit.yaw || motors->limit.throttle_upper) {
            if (_saturation_start_ms == 0) {
                _saturation_start_ms = now_ms;
            } else if (now_ms - _saturation_start_ms > uint32_t(MAX(_saturation_timeout_s.get(), 0.1f) * 1000.0f)) {
                set_state(ContactState::RETREAT, FaultReason::ACTUATOR_SATURATION);
            }
        } else {
            _saturation_start_ms = 0;
        }
        break;
    }

    case ContactState::REACQUIRE:
        _fx_target = 0.0f;
        if (!range_contact_handoff_available(now_ms)) {
            set_state(ContactState::RETREAT, FaultReason::RANGEFINDER);
            break;
        }
        if (now_ms - _state_start_ms > uint32_t(MIN(MAX(_approach_timeout_s.get(), 1.0f), 3.0f) * 1000.0f)) {
            set_state(ContactState::RETREAT, FaultReason::REACQUIRE_FAILED);
            break;
        }
        set_body_x_velocity(MIN(_slow_speed_ms.get(), IMPEDANCE_REACQUIRE_SPEED_MS));
        if (contact_event == AP_ContactDetector::Event::CONTACT) {
            set_state(ContactState::FORCE_HOLD);
        } else if (_contact_detector.contact_candidate()) {
            set_state(ContactState::CONTACT_CONFIRM);
        }
        break;

    case ContactState::RETREAT:
        if (fabsf(_fx_command) > 0.01f) {
            set_body_x_velocity(0.0f);
            break;
        }
        if (!copter.position_ok()) {
            _fault_reason = FaultReason::POSITION_ESTIMATE;
            set_body_x_velocity(0.0f);
            break;
        }
        set_body_x_velocity(-MAX(_slow_speed_ms.get(), 0.02f));
        // Accept either the requested position retreat or direct physical
        // evidence that the probe has cleared the wall.  The latter protects
        // tilt-frame SITL setups where NE/body-axis mapping is imperfect:
        // force must be released and the forward gap must have opened by a
        // meaningful margin before READY can be declared.
        const bool force_released = fabsf(_force_filtered_n) <= _force_off_threshold_n.get();
        const bool range_clear = _tool_distance_healthy &&
                                 _tool_distance_m >= MAX(_retreat_start_distance_m + 0.15f,
                                         _slow_distance_m.get() + 0.10f);
        if (retreat_distance_done_m() >= _retreat_distance_m.get() || (force_released && range_clear)) {
            set_state(ContactState::READY);
        } else if (now_ms - _state_start_ms > uint32_t(MAX(_retreat_timeout_s.get(), 1.0f) * 1000.0f)) {
            set_body_x_velocity(0.0f);
            if (_fault_reason != FaultReason::RETREAT_TIMEOUT) {
                _fault_reason = FaultReason::RETREAT_TIMEOUT;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Impedance: retreat timeout");
            }
        }
        break;
    }

    update_confidence(dt);
    update_force_controller(dt, new_force_sample);
}

void ModeImpedance::relax_force_axis_position_control()
{
    // Match the axis removed by output_attitude_and_force. Keep tangential
    // holding active without winding up the unused normal position loop.
    const Vector2f normal_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
    Vector2p position_desired = pos_control->get_pos_desired_NED_m().xy();
    const Vector2p position_estimate = pos_control->get_pos_estimate_NED_m().xy() -
                                      pos_control->get_pos_offset_NED_m().xy();
    position_desired += (normal_ne * ((position_estimate - position_desired).tofloat() * normal_ne)).topostype();
    pos_control->set_pos_desired_NE_m(position_desired);

    Vector2f velocity_desired = pos_control->get_vel_desired_NED_ms().xy();
    const Vector2f velocity_estimate = pos_control->get_vel_estimate_NED_ms().xy() -
                                      pos_control->get_vel_offset_NED_ms().xy();
    velocity_desired += normal_ne * ((velocity_estimate - velocity_desired) * normal_ne);
    pos_control->set_vel_desired_NE_ms(velocity_desired);

    auto &velocity_pid = pos_control->NE_get_vel_pid();
    const Vector2f integral = velocity_pid.get_i();
    velocity_pid.set_integrator(integral - normal_ne * (integral * normal_ne));
}

void ModeImpedance::output_attitude_and_force(const Vector3f &thrust_vector, bool force_override)
{
    auto *att6 = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att6 == nullptr) {
        return;
    }

    Vector3f selected_thrust = thrust_vector;
    if (force_override) {
        const Vector2f forward_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
        const float normal_component = selected_thrust.x * forward_ne.x + selected_thrust.y * forward_ne.y;
        selected_thrust.x -= normal_component * forward_ne.x;
        selected_thrust.y -= normal_component * forward_ne.y;
        att6->set_external_forward(_fx_command);
    } else {
        att6->clear_external_forward();
    }
    attitude_control->input_thrust_vector_heading_rad(selected_thrust, _locked_yaw_rad, 0.0f);
}

void ModeImpedance::run()
{
    if (rc().has_valid_input() &&
        channel_pitch->get_radio_in() > IMPEDANCE_ALT_HOLD_PWM) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: RC2 high, ALT_HOLD");
        if (!copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Impedance: ALT_HOLD switch failed");
        }
        return;
    }

    const float dt = constrain_float(pos_control->get_dt_s(), 0.001f, 0.1f);
    const uint32_t now_ms = AP_HAL::millis();
    const bool new_force_sample = update_force_sample();
    update_tool_distance();

    bool tare_complete = false;
#if AP_CONTACT_SENSOR_ENABLED
    tare_complete = copter.contact_sensor.tare_complete();
#endif
    if (_tare_requested && !_tare_reported &&
        now_ms - _tare_start_ms >= IMPEDANCE_TARE_SETTLE_MS &&
        tare_complete) {
        _tare_reported = true;
        _noise_mean_n = 0.0f;
        _noise_m2_n = 0.0f;
        _noise_count = 0;
        _force_filtered_n = 0.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: force tare complete");
    }

    if (_state == ContactState::FORCE_HOLD) {
        float pitch_stick = -channel_pitch->norm_input_dz();
        if (fabsf(pitch_stick) < 0.05f) {
            pitch_stick = 0.0f;
        }
        _force_ref_n += pitch_stick * IMPEDANCE_FORCE_REF_RATE_NS * dt;
        _force_ref_n = constrain_float(_force_ref_n, 1.0f, MAX(_force_ref_max_n.get(), 1.0f));
    }

    float target_roll_rad;
    float unused_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, unused_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, 0.0f);

    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    const AltHoldModeState alt_state = get_alt_hold_state_D_ms(target_climb_rate_ms);
    switch (alt_state) {
    case AltHoldModeState::MotorStopped:
        set_state(ContactState::READY);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);
        loiter_nav->init_target();
        break;
    case AltHoldModeState::Landed_Ground_Idle:
        set_state(ContactState::READY);
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;
    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->D_relax_controller(0.0f);
        loiter_nav->init_target();
        break;
    case AltHoldModeState::Takeoff:
        set_state(ContactState::READY);
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0f, 10.0f));
        }
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        loiter_nav->update();
        break;
    case AltHoldModeState::Flying:
        run_contact_state(dt, new_force_sample);
        if (use_ne_velocity_control()) {
            Vector2f velocity_ne_ms = _locked_tool_axis_ne * _body_x_velocity_command_ms;
            const Vector2f accel_ne_zero;
            pos_control->input_vel_accel_NE_m(velocity_ne_ms, accel_ne_zero, false);
            pos_control->NE_stop_pos_stabilisation();
            pos_control->NE_update_controller();
        } else {
            if (_state == ContactState::FORCE_HOLD) {
                relax_force_axis_position_control();
            }
            loiter_nav->clear_vel_offset_NE_ms();
            loiter_nav->update();
        }
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    pos_control->D_update_controller();
    const bool force_override = _state == ContactState::FORCE_HOLD ||
                                (_state == ContactState::RETREAT && fabsf(_fx_command) > 0.01f);
    output_attitude_and_force(loiter_nav->get_thrust_vector(), force_override);

#if HAL_LOGGING_ENABLED
    copter.Log_Write_Impedance(uint8_t(_state), uint8_t(_fault_reason), _force_ref_n,
                               _force_raw_n, _force_filtered_n, _tool_distance_m,
                               _confidence, 0.0f, _force_p_out,
                               _force_i_out, _fx_command);
#endif

    if (now_ms - _telemetry_ms >= 200U) {
        _telemetry_ms = now_ms;
        const Vector2f velocity_ne_ms = pos_control->get_vel_estimate_NED_ms().xy();
        const Vector2f forward_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
        gcs().send_named_float("Contact", force_override ? 1.0f : 0.0f);
        gcs().send_named_float("ForceN", _force_filtered_n);
        gcs().send_named_float("ForceRef", _force_ref_n);
        gcs().send_named_float("ToolDist", _tool_distance_healthy ? _tool_distance_m : -1.0f);
        gcs().send_named_float("ToolVel", velocity_ne_ms * forward_ne);
        gcs().send_named_float("ContState", float(uint8_t(_state)));
#if AP_CONTACT_SENSOR_ENABLED
        gcs().send_named_float("ForceType", float(uint8_t(copter.contact_sensor.type())));
#endif
    }
}

float ModeImpedance::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeImpedance::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}
