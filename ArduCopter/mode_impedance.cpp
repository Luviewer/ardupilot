#include "Copter.h"

#if AP_RANGEFINDER_ENABLED
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#endif

// Impedance (IMPD) is a Guided-derived contact mode.
// 阻抗/接触模式：继承 Guided，本文件只做接触决策，飞控走 Guided 位置/速度接口。
//
// Split of responsibility:
// 职责划分：
//   1. This file chooses a body-X speed (_guided_body_x_ms) and locked yaw.
//      本文件只决定机头方向速度和锁定航向。
//   2. apply_guided_velocity() converts that into NED velocity + climb.
//      apply_guided_velocity() 把它换成 NED 速度，并带上油门爬升率。
//   3. ModeGuided::run() flies the vehicle.
//      ModeGuided::run() 负责真正的位置/速度环和电机输出。
//
// Entry: flying only. Take off in Loiter/AltHold, then switch to IMPD.
// 进模式：只能在空中切入。地面先用 Loiter/AltHold 起飞，再切 IMPD。
//
// IMPD_STAGE selects how much of the pipeline is live:
// IMPD_STAGE 决定跑到哪一步：
//   0 ApproachOnly: fly forward until the forward rangefinder <= IMPD_STOP_CM, then hover.
//     只接近：朝前飞到测距小于 STOP_CM 后刹停悬停。
//   1 FullContact: approach -> confirm contact force -> hold force / reacquire / retreat.
//     完整接触：接近 -> 确认接触力 -> 恒力 / 再贴 / 后退。
//
// Contact states (ContState telemetry):
// 接触状态（地面站 ContState）：
//   0 READY          wait for rangefinder (and force tare if STAGE=1)
//                    等待朝前测距健康（完整流程还要等力传感器去皮）
//   1 APPROACH       fly along locked yaw toward the surface
//                    沿锁定航向接近表面
//   2 CONTACT_CONFIRM hover while force samples confirm contact
//                    悬停，等力传感器连续确认接触
//   3 FORCE_HOLD     hold contact force (STAGE=1; not yet mapped to Guided vel)
//                    恒力（STAGE=1；力环还没接到 Guided 速度口）
//   4 REACQUIRE      creep forward once after a lost contact
//                    失接触后低速再贴一次
//   5 RETREAT        back away along the tool axis
//                    沿工具轴后退
//   6 APPROACH_HOLD  STAGE=0 brake/hover after rangefinder stop
//                    只接近阶段：测距到达后刹车悬停

static constexpr uint32_t IMPEDANCE_TARE_SETTLE_MS = 600;       // wait after software tare
                                                                // 软件去皮后再等一会儿再采噪声
static constexpr uint32_t IMPEDANCE_RANGE_TIMEOUT_MS = 300;     // stale forward-rangefinder timeout
                                                                // 朝前测距超过该时间无新数据则判失效
static constexpr uint32_t IMPEDANCE_RANGE_REPORT_MS = 1000;     // GCS forward-range report interval
                                                                // 每秒向地面站报告一次朝前距离
static constexpr float IMPEDANCE_RANGE_JUMP_M = 0.2f;           // reject a sudden range step
                                                                // 相邻两拍距离跳变过大则丢掉
static constexpr float IMPEDANCE_PILOT_ABORT_STICK = 0.6f;      // pitch-forward abort to AltHold
                                                                // 俯仰杆前推超过该值切回 AltHold
static constexpr float IMPEDANCE_FORCE_LPF_HZ = 3.0f;           // force low-pass
                                                                // 接触力低通截止频率
static constexpr float IMPEDANCE_REACQUIRE_SPEED_MS = 0.03f;    // max creep after lost contact
                                                                // 失接触后再贴的最大速度
static constexpr float IMPEDANCE_CONFIDENCE_TC_S = 0.3f;        // confidence filter time constant
                                                                // 接触置信度滤波时间常数
static constexpr float IMPEDANCE_FORCE_REF_RATE_NS = 1.0f;      // pitch-stick force-ref rate
                                                                // 俯仰杆改目标力的速率
static constexpr float IMPEDANCE_CONTACT_ON_N = 0.5f;           // minimum contact-on force
                                                                // 判定接触的最小力
static constexpr float IMPEDANCE_CONTACT_OFF_N = 0.25f;         // contact-release force
                                                                // 判定脱离接触的力
static constexpr uint8_t IMPEDANCE_CONTACT_SAMPLES = 3;         // consecutive samples to confirm
                                                                // 连续几拍才确认接触/脱离
static constexpr uint8_t IMPEDANCE_BASELINE_SAMPLES = 20;       // force-noise samples before approach
                                                                // 完整流程开始接近前至少采这么多噪声点
static constexpr float IMPEDANCE_NOISE_SIGMA_MULTIPLIER = 4.0f; // contact threshold = max(0.5N, 4*sigma)
                                                                // 接触阈值取 0.5N 与 4 倍噪声标准差的较大值

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

    // @Param: STAGE
    // @DisplayName: Contact test stage
    // @Description: Which part of the contact pipeline is enabled. 0 tests forward-rangefinder approach and brake only.
    // @Values: 0:ApproachOnly,1:FullContact
    // @User: Standard
    AP_GROUPINFO("STAGE", 13, ModeImpedance, _stage, 0),

    // @Param: STOP_CM
    // @DisplayName: Approach stop distance
    // @Description: In approach-only stage, brake and hold when the forward rangefinder distance is at or below this value.
    // @Range: 5 100
    // @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STOP_CM", 14, ModeImpedance, _stop_distance_cm, 20),

    AP_GROUPEND
};

ModeImpedance::ModeImpedance()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Reject ground entry, then start Guided velocity control and tare the force sensor if needed.
// 拒绝地面切入；通过后初始化 Guided 速度控制，完整流程还要给力传感器去皮。
bool ModeImpedance::init(bool ignore_checks)
{
    // Flying only: must already be airborne in another mode.
    // 只能空中切入：未解锁、未自动解锁或仍判定在地，一律拒绝。
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: must be flying");
        return false;
    }

    auto *att6 = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att6 == nullptr || !motors->get_tilt_enable()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: 6DoF tilt control required");
        return false;
    }

    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    // Keep Guided XY speed close to approach speed so braking is not WPNAV-fast.
    // 水平速度上限贴近接近速度，避免 Guided 按航点速度猛刹。
    set_speed_NE_ms(MAX(_approach_speed_ms.get(), 0.5f));

    reset_contact_control();
    _locked_yaw_rad = ahrs.get_yaw();
    _force_ref_n = constrain_float(_force_ref_default_n, 1.0f, _force_ref_max_n);
    _guided_body_x_ms = 0.0f;

#if AP_CONTACT_SENSOR_ENABLED
    if (approach_only()) {
        // Approach-only does not need the force sensor. Tare it if present, but do not block.
        // 只接近不依赖力传感器。有传感器就顺手去皮，失败也不挡进模式。
        if (copter.contact_sensor.healthy() && copter.contact_sensor.tare()) {
            _tare_requested = true;
            _tare_start_ms = AP_HAL::millis();
        }
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: approach-only stop %.0fcm",
                      double(MAX(_stop_distance_cm.get(), 1.0f)));
    } else {
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
    }
#else
    if (!approach_only()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact sensor unavailable");
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: approach-only stop %.0fcm",
                  double(MAX(_stop_distance_cm.get(), 1.0f)));
#endif

    return true;
}

// Leave a zero Guided velocity command so the next mode does not inherit a forward speed.
// 退出时清掉前向速度指令，避免带进下一个模式。
void ModeImpedance::exit()
{
    if (auto *att6 = AC_AttitudeControl_Multi_6DoF::get_singleton()) {
        att6->clear_external_forward();
    }
    motors->set_forward(0.0f);
    _guided_body_x_ms = 0.0f;
    hold_position();
    reset_contact_control();
}

// Reset the contact state machine and force/rangefinder bookkeeping.
// 复位接触状态机，以及力和测距的内部记录。
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
    _contact_detector.configure(IMPEDANCE_CONTACT_ON_N, IMPEDANCE_CONTACT_OFF_N, IMPEDANCE_CONTACT_SAMPLES);
    _contact_detector.reset();
    _last_force_sequence = 0;
    _last_force_sample_ms = 0;
    _last_force_dt_s = 0.05f;
    _state_start_ms = AP_HAL::millis();
    _saturation_start_ms = 0;
    _rangefinder_report_ms = _state_start_ms;
    _reacquire_used = false;
    _auto_start_pending = true;
    _tare_requested = false;
    _tare_reported = false;
    _tare_start_ms = 0;
    _guided_body_x_ms = 0.0f;
}

// Enter a new contact state. Side effects (lock yaw, zero integrator, GCS text) happen here.
// 切入新接触状态。锁航向、清积分、报 GCS 都在这里做，行为不要散落到 run() 里。
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
        // Lock yaw now so the whole approach stays on the current tool axis.
        // 此刻锁航向，整段接近都沿当前工具轴走，中途不再跟飞手偏航。
        _fault_reason = FaultReason::NONE;
        _locked_yaw_rad = ahrs.get_yaw();
        _force_integral = 0.0f;
        _force_ref_n = constrain_float(_force_ref_default_n, 1.0f, _force_ref_max_n);
        _contact_detector.reset();
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: approach");
        break;
    case ContactState::CONTACT_CONFIRM:
        set_body_x_velocity(0.0f);
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: confirming contact");
        break;
    case ContactState::FORCE_HOLD:
        // Remember where contact started so confidence can drop if we drift along the tool axis.
        // 记下刚贴上时的水平位置，后面沿工具轴漂太远会压低置信度。
        _contact_start_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
        _force_integral = 0.0f;
        _contact_detector.reset(true);
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: force hold");
        break;
    case ContactState::REACQUIRE:
        _force_integral = 0.0f;
        _fx_target = 0.0f;
        _reacquire_used = true;
        _contact_detector.reset();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: contact lost, reacquire");
        break;
    case ContactState::RETREAT:
        _force_integral = 0.0f;
        _fx_target = 0.0f;
        _retreat_start_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: retreat reason %u", unsigned(_fault_reason));
        break;
    case ContactState::APPROACH_HOLD:
        set_body_x_velocity(0.0f);
        if (reason == FaultReason::NONE) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Impedance: approach hold %.0fcm",
                          double(_tool_distance_m * 100.0f));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: approach hold reason %u", unsigned(reason));
        }
        break;
    }
}

// Pull one new force sample if the sequence advanced. Returns false when the sensor has no new data.
// 只有力传感器序号前进才算新样本。飞行环比传感器快，旧样本不要重复拿去判接触。
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

    // Welford running mean/variance while hovering in READY. Used as a zero offset and noise floor.
    // READY 悬停时用 Welford 累计均值和方差，后面当零点和噪声底。
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
    const float alpha = constrain_float(sample_dt * IMPEDANCE_FORCE_LPF_HZ * M_2PI, 0.0f, 1.0f);
    _force_filtered_n += (corrected_force_n - _force_filtered_n) * alpha;
    return true;
#else
    return false;
#endif
}

// Read the forward rangefinder (IMPD_RFND_IDX, orientation must be ROTATION_NONE).
// 读朝前测距（IMPD_RFND_IDX，朝向必须是机头前 ROTATION_NONE）。
// Hold the last valid distance through brief status dropouts or jumps. A persistent
// failure still becomes unhealthy after IMPEDANCE_RANGE_TIMEOUT_MS.
// 短时状态异常或跳变沿用上次有效距离；持续超过超时时间才判为失效。
void ModeImpedance::update_tool_distance()
{
    _tool_distance_healthy = false;
    _tool_distance_m = 0.0f;
#if AP_RANGEFINDER_ENABLED
    AP_RangeFinder_Backend *backend = copter.rangefinder.get_backend(uint8_t(MAX(_rangefinder_instance.get(), 0)));
    if (backend == nullptr || backend->orientation() != ROTATION_NONE) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t reading_ms = backend->last_reading_ms();
    const float distance_m = backend->distance();
    const bool reading_fresh = now_ms - reading_ms <= IMPEDANCE_RANGE_TIMEOUT_MS;
    const bool distance_valid = distance_m >= backend->min_distance() &&
                                distance_m <= backend->max_distance();

    if (backend->status() == RangeFinder::Status::Good && reading_fresh && distance_valid) {
        const bool distance_jump = _last_rangefinder_ms != 0 &&
                                   reading_ms != _last_rangefinder_ms &&
                                   fabsf(distance_m - _previous_tool_distance_m) > IMPEDANCE_RANGE_JUMP_M;
        if (!distance_jump) {
            _last_rangefinder_ms = reading_ms;
            _previous_tool_distance_m = distance_m;
            _tool_distance_m = distance_m;
            _tool_distance_healthy = true;
            return;
        }
    }

    if (_last_rangefinder_ms != 0 &&
        now_ms - _last_rangefinder_ms <= IMPEDANCE_RANGE_TIMEOUT_MS) {
        _tool_distance_m = _previous_tool_distance_m;
        _tool_distance_healthy = true;
    }
#endif
}

// Contact-on threshold: at least 0.5 N, or 4-sigma of the READY noise, whichever is larger.
// 接触阈值：至少 0.5 N，若 READY 噪声更大则用 4 倍标准差，避免在噪声里误触发。
float ModeImpedance::contact_on_threshold_n() const
{
    float sigma_n = 0.0f;
    if (_noise_count > 1U) {
        sigma_n = sqrtf(_noise_m2_n / float(_noise_count - 1U));
    }
    return MAX(IMPEDANCE_CONTACT_ON_N, IMPEDANCE_NOISE_SIGMA_MULTIPLIER * sigma_n);
}

// Confidence scales the force PI. Near the surface and not drifted along the tool axis -> 1.
// 置信度用来缩放力环输出。离表面近且没沿工具轴漂走就接近 1，否则往下压。
void ModeImpedance::update_confidence(float dt)
{
    float requested = 0.0f;
    bool force_sensor_healthy = false;
#if AP_CONTACT_SENSOR_ENABLED
    force_sensor_healthy = copter.contact_sensor.healthy();
#endif
    if (_tool_distance_healthy && force_sensor_healthy &&
        (_state == ContactState::CONTACT_CONFIRM || _state == ContactState::FORCE_HOLD || _state == ContactState::REACQUIRE)) {
        const float near_m = MAX(_slow_distance_m.get(), 0.05f);
        if (_tool_distance_m <= near_m) {
            requested = 1.0f;
        } else if (_tool_distance_m < 2.0f * near_m) {
            const float phase = (_tool_distance_m - near_m) / near_m;
            requested = 0.5f * (1.0f + cosf(phase * M_PI));
        }

        if (_state == ContactState::FORCE_HOLD) {
            const Vector2f position_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
            const Vector2f forward_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
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

// Force PI on the tool axis. Only runs in FORCE_HOLD. Output is still _fx_command (not Guided vel yet).
// 工具轴力 PI，只在恒力阶段算。输出还是 _fx_command，目前没有接到 Guided 速度口。
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

// Remember the requested body-X speed. Guided is updated once per run() after the state machine.
// 只记下机头方向速度。等状态机跑完，run() 里再一次性交给 Guided。
void ModeImpedance::set_body_x_velocity(float speed_ms)
{
    _guided_body_x_ms = speed_ms;
}

// Body-X along locked yaw, pilot climb on Z (NED down is negative climb). Yaw is held by Guided.
// 机头速度投到锁定航向；油门爬升率放在 NED 的 Z（向下为正，所以取负）。航向由 Guided 锁住。
void ModeImpedance::apply_guided_velocity(float climb_rate_ms)
{
    Vector3f vel_ned_ms;
    vel_ned_ms.x = _guided_body_x_ms * cosf(_locked_yaw_rad);
    vel_ned_ms.y = _guided_body_x_ms * sinf(_locked_yaw_rad);
    vel_ned_ms.z = -climb_rate_ms;
    set_vel_NED_ms(vel_ned_ms, true, _locked_yaw_rad, false, 0.0f, false, false);
}

// IMPD_STAGE 0 is the current flight-test path: rangefinder approach and brake only.
// IMPD_STAGE=0 是现在的试飞路径：只做测距接近和刹车。
bool ModeImpedance::approach_only() const
{
    return _stage.get() == 0;
}

float ModeImpedance::stop_distance_m() const
{
    return MAX(_stop_distance_cm.get(), 1.0f) * 0.01f;
}

// How far we have already moved backward along the locked tool axis.
// 已经沿锁定工具轴后退了多少。正值表示在往远离表面的方向走。
float ModeImpedance::retreat_distance_done_m() const
{
    const Vector2f position_ne_m = pos_control->get_pos_estimate_NED_m().xy().tofloat();
    const Vector2f forward_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
    return -((position_ne_m - _retreat_start_ne_m) * forward_ne);
}

// Contact state machine. Only writes _guided_body_x_ms; Guided flies it later in run().
// 接触状态机。这里只改机头速度，真正飞是后面 run() 里交给 Guided。
void ModeImpedance::run_contact_state(float dt, bool new_force_sample)
{
    const uint32_t now_ms = AP_HAL::millis();
    _contact_detector.configure(contact_on_threshold_n(), IMPEDANCE_CONTACT_OFF_N, IMPEDANCE_CONTACT_SAMPLES);
    const bool detection_active = _state == ContactState::APPROACH ||
                                  _state == ContactState::CONTACT_CONFIRM ||
                                  _state == ContactState::FORCE_HOLD ||
                                  _state == ContactState::REACQUIRE;
    const AP_ContactDetector::Event contact_event = (new_force_sample && detection_active) ?
                                                    _contact_detector.update(_force_filtered_n) :
                                                    AP_ContactDetector::Event::NONE;

    // Auto-start once after mode entry. Approach-only only needs a healthy forward rangefinder.
    // 进模式后自动开始一次。只接近只要朝前测距健康；完整流程还要去皮完成并采够噪声。
    if (_auto_start_pending && _state == ContactState::READY) {
        bool force_sensor_healthy = false;
#if AP_CONTACT_SENSOR_ENABLED
        force_sensor_healthy = copter.contact_sensor.healthy();
#endif
        const bool approach_ready = _tool_distance_healthy &&
                                    (approach_only() ||
                                     (force_sensor_healthy && _tare_reported &&
                                      _noise_count >= IMPEDANCE_BASELINE_SAMPLES));
        if (approach_ready) {
            _auto_start_pending = false;
            set_state(ContactState::APPROACH);
        }
    }

    // Full-contact faults retreat. Approach-only brakes in place instead of backing up.
    // 完整流程故障就后退。只接近不后退，位置估计丢了就地刹停。
    if (!approach_only()) {
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
    } else if (_state != ContactState::READY &&
               _state != ContactState::APPROACH_HOLD &&
               !copter.position_ok()) {
        set_state(ContactState::APPROACH_HOLD, FaultReason::POSITION_ESTIMATE);
    }

    switch (_state) {
    case ContactState::READY:
        set_body_x_velocity(0.0f);
        break;

    case ContactState::APPROACH:
        // Fly forward. STAGE=0 stops on rangefinder. STAGE=1 waits for force contact.
        // 向前飞。STAGE=0 看测距刹车；STAGE=1 等力传感器确认接触。
        if (!_tool_distance_healthy) {
            set_state(approach_only() ? ContactState::APPROACH_HOLD : ContactState::RETREAT,
                      FaultReason::RANGEFINDER);
            break;
        }
        if (now_ms - _state_start_ms > uint32_t(MAX(_approach_timeout_s.get(), 1.0f) * 1000.0f)) {
            set_state(approach_only() ? ContactState::APPROACH_HOLD : ContactState::RETREAT,
                      FaultReason::APPROACH_TIMEOUT);
            break;
        }
        if (approach_only() && _tool_distance_m <= stop_distance_m()) {
            set_state(ContactState::APPROACH_HOLD);
            break;
        }
        set_body_x_velocity((_tool_distance_m <= _slow_distance_m.get()) ? _slow_speed_ms.get() : _approach_speed_ms.get());
        if (!approach_only()) {
            if (contact_event == AP_ContactDetector::Event::CONTACT) {
                set_state(ContactState::FORCE_HOLD);
            } else if (_contact_detector.contact_candidate()) {
                set_state(ContactState::CONTACT_CONFIRM);
            }
        }
        break;

    case ContactState::CONTACT_CONFIRM:
        // Hover while the detector finishes its consecutive-sample confirmation.
        // 先刹停，等检测器凑满连续几拍再确认接触；力又掉下去就回到接近。
        set_body_x_velocity(0.0f);
        if (!_tool_distance_healthy) {
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
        // Hold XY. Force PI still updates _fx_command but does not yet command Guided velocity.
        // 水平刹停。力 PI 仍在算 _fx_command，但还没有变成 Guided 速度。
        set_body_x_velocity(0.0f);
        if (!_tool_distance_healthy) {
            set_state(ContactState::RETREAT, FaultReason::RANGEFINDER);
            break;
        }
        if (contact_event == AP_ContactDetector::Event::RELEASE) {
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
        // One slow retry after lost contact. Fail once and we retreat instead of looping.
        // 失接触后只低速再贴一次。再失败就后退，避免来回蹭。
        _fx_target = 0.0f;
        if (!_tool_distance_healthy) {
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

    case ContactState::APPROACH_HOLD:
        set_body_x_velocity(0.0f);
        break;

    case ContactState::RETREAT:
        // Unload any leftover force command first, then back up RET_DIST along the tool axis.
        // 先把残留前向力指令卸掉，再沿工具轴后退 RET_DIST。
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
        if (retreat_distance_done_m() >= _retreat_distance_m.get()) {
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

    if (!approach_only()) {
        update_confidence(dt);
        update_force_controller(dt, new_force_sample);
    }
}

// 400 Hz-class flight loop: sensors -> contact decision -> Guided velocity -> Guided::run().
// 飞行主循环：传感器 -> 接触决策 -> 写 Guided 速度 -> 交给 Guided 飞。
void ModeImpedance::run()
{
    // Pitch stick forward is the pilot abort. Do this before commanding any approach speed.
    // 俯仰杆前推是飞手中止，必须在发接近速度之前处理。
    const bool pilot_abort = rc().has_valid_input() &&
                             channel_pitch->norm_input_dz() > IMPEDANCE_PILOT_ABORT_STICK;
    if (pilot_abort) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: pilot abort");
    }

    const float dt = constrain_float(pos_control->get_dt_s(), 0.001f, 0.1f);
    const uint32_t now_ms = AP_HAL::millis();
    const bool new_force_sample = update_force_sample();
    update_tool_distance();
    if (now_ms - _rangefinder_report_ms >= IMPEDANCE_RANGE_REPORT_MS) {
        _rangefinder_report_ms = now_ms;
        if (_tool_distance_healthy) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Impedance: range %.0fcm",
                          double(_tool_distance_m * 100.0f));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Impedance: range unavailable");
        }
    }

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

    // In force hold, pitch stick trims the force reference instead of pitching the airframe.
    // 恒力阶段俯仰杆改目标力，不再直接打俯仰。
    if (!pilot_abort && _state == ContactState::FORCE_HOLD) {
        float pitch_stick = -channel_pitch->norm_input_dz();
        if (fabsf(pitch_stick) < 0.05f) {
            pitch_stick = 0.0f;
        }
        _force_ref_n += pitch_stick * IMPEDANCE_FORCE_REF_RATE_NS * dt;
        _force_ref_n = constrain_float(_force_ref_n, 1.0f, MAX(_force_ref_max_n.get(), 1.0f));
    }

    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    if (is_disarmed_or_landed()) {
        // Mode cannot be entered on the ground. If we land while already in IMPD, just sit safe.
        // 地面不能切入本模式。如果已经在 IMPD 里落地，只做安全停车，不再起飞。
        set_state(ContactState::READY);
        _guided_body_x_ms = 0.0f;
        _auto_start_pending = true;
        _locked_yaw_rad = ahrs.get_yaw();
        ModeGuided::run();
    } else {
        if (pilot_abort) {
            set_body_x_velocity(0.0f);
        } else {
            run_contact_state(dt, new_force_sample);
        }
        apply_guided_velocity(target_climb_rate_ms);
        ModeGuided::run();
    }

    const bool in_contact = !approach_only() &&
                           (_state == ContactState::FORCE_HOLD ||
                            (_state == ContactState::RETREAT && fabsf(_fx_command) > 0.01f));

#if HAL_LOGGING_ENABLED
    copter.Log_Write_Impedance(uint8_t(_state), uint8_t(_fault_reason), _force_ref_n,
                               _force_raw_n, _force_filtered_n, _tool_distance_m,
                               _confidence, 0.0f, _force_p_out,
                               _force_i_out, _fx_command);
#endif

    // Named floats for GCS: ContState, ToolDist (m, -1 if invalid), ToolVel along locked yaw.
    // 地面站命名浮点：ContState、ToolDist（米，无效为 -1）、沿锁定航向的 ToolVel。
    if (now_ms - _telemetry_ms >= 200U) {
        _telemetry_ms = now_ms;
        const Vector2f velocity_ne_ms = pos_control->get_vel_estimate_NED_ms().xy();
        const Vector2f forward_ne{cosf(_locked_yaw_rad), sinf(_locked_yaw_rad)};
        gcs().send_named_float("Contact", in_contact ? 1.0f : 0.0f);
        gcs().send_named_float("ForceN", _force_filtered_n);
        gcs().send_named_float("ForceRef", _force_ref_n);
        gcs().send_named_float("ToolDist", _tool_distance_healthy ? _tool_distance_m : -1.0f);
        gcs().send_named_float("ToolVel", velocity_ne_ms * forward_ne);
        gcs().send_named_float("ContState", float(uint8_t(_state)));
#if AP_CONTACT_SENSOR_ENABLED
        gcs().send_named_float("ForceType", float(uint8_t(copter.contact_sensor.type())));
#endif
    }

    // Finish this cycle's position-controller update before changing mode.
    // 先跑完本周期位置控制器，再切模式，避免下一周期误报 flow_of_control。
    if (pilot_abort) {
        set_mode(Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND);
    }
}
