/*
   天蝎座有三个可独立倾转的旋翼，输出顺序为：
     Motor1 / tilt-right：前右旋翼（俯视顺时针）
     Motor2 / tilt-rear： 后旋翼（俯视逆时针）
     Motor3 / tilt-left： 前左旋翼（俯视顺时针）

   电机推力的垂直分量控制升力、横滚和俯仰；倾转产生的水平分量
   控制前向力、侧向力和偏航。初始化时根据真实几何生成混控系数，
   飞行循环中采用与其他 ArduPilot 多旋翼一致的分轴系数叠加方式。
 */

#include "AP_MotorsScorpio.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_MotorsScorpio::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // 参数编号 1～5 由早期的天蝎座几何参数保留占用。

    // @Param: SC_F_ANG
    // @DisplayName: Scorpio front tilt axis angle
    // @Description: Absolute yaw installation angle of the front rotor tilt mechanisms
    // @Units: deg
    // @Range: 1 89
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_F_ANG", 6, AP_MotorsScorpio, _front_axis_angle_deg, 30.0f),

    // @Param: SC_R_ANG
    // @DisplayName: Scorpio rear tilt axis angle
    // @Description: Yaw installation angle of the rear rotor tilt mechanism
    // @Units: deg
    // @Range: 1 179
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_R_ANG", 7, AP_MotorsScorpio, _rear_axis_angle_deg, 90.0f),

    // @Param: SC_TILT_MAX
    // @DisplayName: Scorpio maximum rotor tilt
    // @Description: Maximum commanded tilt angle from the neutral vertical position
    // @Units: deg
    // @Range: 5 60
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("SC_TILT_MAX", 8, AP_MotorsScorpio, _tilt_max_deg, 45.0f),

    // @Param: SC_XY_GAIN
    // @DisplayName: Scorpio horizontal force gain
    // @Description: Scales forward and lateral force requests before allocation
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("SC_XY_GAIN", 9, AP_MotorsScorpio, _xy_gain, 1.0f),

    // 参数编号 10 原为 SC_YAW_ARM；现在直接根据真实几何归一化偏航力臂。

    // 参数编号 11 原为 SC_REACT；旋翼反扭矩作为扰动，由偏航闭环抑制。

    // @Param: SC_FR_X
    // @DisplayName: Scorpio front-right rotor X position
    // @Description: Front-right rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_FR_X", 12, AP_MotorsScorpio, _front_right_x, 0.18811f),

    // @Param: SC_FR_Y
    // @DisplayName: Scorpio front-right rotor Y position
    // @Description: Front-right rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_FR_Y", 13, AP_MotorsScorpio, _front_right_y, 0.20529f),

    // 参数编号 14 为已删除的 SC_FR_Z 保留。

    // @Param: SC_FL_X
    // @DisplayName: Scorpio front-left rotor X position
    // @Description: Front-left rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_FL_X", 15, AP_MotorsScorpio, _front_left_x, 0.18811f),

    // @Param: SC_FL_Y
    // @DisplayName: Scorpio front-left rotor Y position
    // @Description: Front-left rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_FL_Y", 16, AP_MotorsScorpio, _front_left_y, -0.20529f),

    // 参数编号 17 为已删除的 SC_FL_Z 保留。

    // @Param: SC_R_X
    // @DisplayName: Scorpio rear rotor X position
    // @Description: Rear rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_R_X", 18, AP_MotorsScorpio, _rear_x, -0.22917f),

    // @Param: SC_R_Y
    // @DisplayName: Scorpio rear rotor Y position
    // @Description: Rear rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SC_R_Y", 19, AP_MotorsScorpio, _rear_y, 0.0f),

    // 参数编号 20 为已删除的 SC_R_Z 保留。

    AP_GROUPEND
};

void AP_MotorsScorpio::setup_tilt_outputs()
{
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRear, CH_4);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_5);

    const float angle_cd = constrain_float(_tilt_max_deg, 5.0f, 60.0f) * 100.0f;
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, angle_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, angle_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, angle_cd);
}

void AP_MotorsScorpio::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        add_motor_num(i);
        motor_enabled[i] = true;
    }
    setup_tilt_outputs();
    _allocation_valid = setup_allocation_factors();
    set_update_rate(_speed_hz);
    _mav_type = MAV_TYPE_TRICOPTER;
    set_initialised_ok(frame_class == MOTOR_FRAME_SCORPIO);
}

void AP_MotorsScorpio::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    setup_tilt_outputs();
    _allocation_valid = setup_allocation_factors();
    set_initialised_ok(frame_class == MOTOR_FRAME_SCORPIO);
}

void AP_MotorsScorpio::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;
    rc_set_freq((1U << FRONT_RIGHT) | (1U << REAR) | (1U << FRONT_LEFT), speed_hz);
}

uint32_t AP_MotorsScorpio::get_motor_mask()
{
    const uint32_t motor_mask = (1U << FRONT_RIGHT) | (1U << REAR) | (1U << FRONT_LEFT);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);
    mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_tiltMotorRight);
    mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_tiltMotorRear);
    mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_tiltMotorLeft);
    mask |= AP_MotorsMulticopter::get_motor_mask();
    return mask;
}

// 计算矩阵无穷范数，用于快速判断分配矩阵是否接近奇异。
static float matrix_inf_norm(const Matrix3f &matrix)
{
    return MAX(fabsf(matrix.a.x) + fabsf(matrix.a.y) + fabsf(matrix.a.z),
               MAX(fabsf(matrix.b.x) + fabsf(matrix.b.y) + fabsf(matrix.b.z),
                   fabsf(matrix.c.x) + fabsf(matrix.c.y) + fabsf(matrix.c.z)));
}

bool AP_MotorsScorpio::setup_allocation_factors()
{
    // ArduPilot 机体系 Y 轴向右，Gazebo RSDF 的 Y 轴向左，因此这里需要转换符号。
    // 正倾转方向与 RSDF 倾转关节局部坐标系的 -Y 轴一致。
    _vertical_throttle_factor.zero();
    _vertical_roll_factor.zero();
    _vertical_pitch_factor.zero();
    _horizontal_forward_factor.zero();
    _horizontal_lateral_factor.zero();
    _horizontal_yaw_factor.zero();

    if (!isfinite(_front_axis_angle_deg) || !isfinite(_rear_axis_angle_deg) ||
        _front_axis_angle_deg < 1.0f || _front_axis_angle_deg > 89.0f ||
        _rear_axis_angle_deg < 1.0f || _rear_axis_angle_deg > 179.0f) {
        return false;
    }

    Vector3f pos_x;
    Vector3f pos_y;
    get_rotor_positions(pos_x, pos_y);
    Vector3f direction_x;
    Vector3f direction_y;
    const Vector3f axis_angle(radians(_front_axis_angle_deg),
                              radians(_rear_axis_angle_deg),
                              -radians(_front_axis_angle_deg));
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        direction_x[i] = -cosf(axis_angle[i]);
        direction_y[i] = sinf(axis_angle[i]);
    }

    // 垂直向上的推力 T 在机体系产生：Mx=-y*T，My=x*T。
    // 对该矩阵求逆，只是为了在初始化阶段生成 throttle/roll/pitch 系数。
    Matrix3f vertical(Vector3f(1.0f, 1.0f, 1.0f),
                      Vector3f(-pos_y.x, -pos_y.y, -pos_y.z),
                      pos_x);
    Matrix3f vertical_inverse;
    if (!vertical.inverse(vertical_inverse)) {
        return false;
    }

    // 每个旋翼只有一个沿自身倾转方向的水平力 q。
    // 水平矩阵把三个 q 映射为机体系 Fx、Fy 和偏航力矩 Mz。
    const Vector3f yaw_arm(pos_x.x * direction_y.x - pos_y.x * direction_x.x,
                           pos_x.y * direction_y.y - pos_y.y * direction_x.y,
                           pos_x.z * direction_y.z - pos_y.z * direction_x.z);
    Matrix3f horizontal(direction_x, direction_y, yaw_arm);
    Matrix3f horizontal_inverse;
    if (!horizontal.inverse(horizontal_inverse)) {
        return false;
    }

    // 有些几何虽然数学上可求逆，但接近奇异时会把很小的控制量放大成很大的
    // 电机或舵机指令。条件指标超过阈值时禁止使用，并在解锁检查中报告。
    constexpr float condition_limit = 100.0f;
    if (matrix_inf_norm(vertical) * matrix_inf_norm(vertical_inverse) > condition_limit ||
        matrix_inf_norm(horizontal) * matrix_inf_norm(horizontal_inverse) > condition_limit) {
        return false;
    }

    const Vector3f collective = vertical_inverse * Vector3f(1.0f, 0.0f, 0.0f);
    const float collective_max = MAX(collective.x, MAX(collective.y, collective.z));
    const float collective_min = MIN(collective.x, MIN(collective.y, collective.z));
    const float roll_arm = MAX(pos_y.x, MAX(pos_y.y, pos_y.z)) - MIN(pos_y.x, MIN(pos_y.y, pos_y.z));
    const float pitch_arm = MAX(pos_x.x, MAX(pos_x.y, pos_x.z)) - MIN(pos_x.x, MIN(pos_x.y, pos_x.z));
    const float yaw_arm_max = MAX(fabsf(yaw_arm.x), MAX(fabsf(yaw_arm.y), fabsf(yaw_arm.z)));
    if (collective_max <= 0.0f || collective_min < 0.0f ||
        roll_arm <= 0.0f || pitch_arm <= 0.0f || yaw_arm_max <= 0.0f) {
        return false;
    }

    // 将逆矩阵的每一列展开为标准 mixer factor：
    // vertical[i] = throttle*T[i] + roll*R[i] + pitch*P[i]
    // horizontal[i] = forward*F[i] + lateral*L[i] + yaw*Y[i]
    _vertical_throttle_factor = collective / collective_max;
    _vertical_roll_factor = vertical_inverse * Vector3f(0.0f, roll_arm, 0.0f);
    _vertical_pitch_factor = vertical_inverse * Vector3f(0.0f, 0.0f, pitch_arm);
    _horizontal_forward_factor = horizontal_inverse * Vector3f(1.0f, 0.0f, 0.0f);
    _horizontal_lateral_factor = horizontal_inverse * Vector3f(0.0f, 1.0f, 0.0f);
    _horizontal_yaw_factor = horizontal_inverse * Vector3f(0.0f, 0.0f, yaw_arm_max);

    return !_vertical_throttle_factor.is_nan() && !_vertical_throttle_factor.is_inf() &&
           !_vertical_roll_factor.is_nan() && !_vertical_roll_factor.is_inf() &&
           !_vertical_pitch_factor.is_nan() && !_vertical_pitch_factor.is_inf() &&
           !_horizontal_forward_factor.is_nan() && !_horizontal_forward_factor.is_inf() &&
           !_horizontal_lateral_factor.is_nan() && !_horizontal_lateral_factor.is_inf() &&
           !_horizontal_yaw_factor.is_nan() && !_horizontal_yaw_factor.is_inf();
}

void AP_MotorsScorpio::get_rotor_positions(Vector3f &pos_x,
        Vector3f &pos_y) const
{
    // 所有 Vector3f 的元素顺序固定为：前右、后、前左。
    pos_x = Vector3f(_front_right_x, _rear_x, _front_left_x);
    pos_y = Vector3f(_front_right_y, _rear_y, _front_left_y);
}

bool AP_MotorsScorpio::horizontal_mix_is_feasible(const Vector3f &vertical,
        const Vector3f &horizontal) const
{
    // 同时检查两类物理限制：
    // 1. 水平/垂直分量之比不能超过最大倾转角；
    // 2. 两个分量合成后的电机总推力不能超过 1。
    const float max_angle = radians(constrain_float(_tilt_max_deg, 5.0f, 60.0f));
    const float tan_max = tanf(max_angle);
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        const float vertical_i = vertical[i];
        const float horizontal_i = horizontal[i];
        if (vertical_i < 0.0f || fabsf(horizontal_i) > vertical_i * tan_max) {
            return false;
        }
        if (sq(vertical_i) + sq(horizontal_i) > 1.0f) {
            return false;
        }
    }
    return true;
}

float AP_MotorsScorpio::horizontal_mix_scale(const Vector3f &vertical,
        const Vector3f &base,
        const Vector3f &addition) const
{
    // base 是已经保留的高优先级水平分量，addition 是准备加入的低优先级分量。
    // 若直接叠加会饱和，则用二分法寻找 addition 可使用的最大比例。
    if (horizontal_mix_is_feasible(vertical, base + addition)) {
        return 1.0f;
    }

    float low = 0.0f;
    float high = 1.0f;
    for (uint8_t i = 0; i < 14; i++) {
        const float mid = 0.5f * (low + high);
        if (horizontal_mix_is_feasible(vertical, base + addition * mid)) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

void AP_MotorsScorpio::mix_y3_vertical(float throttle, float throttle_avg_max,
                                       float roll, float pitch,
                                       Vector3f &vertical, float &throttle_out)
{
    // 标准 Y3 风格：先分别计算 roll、pitch 对三个旋翼垂直推力的贡献，再叠加。
    Vector3f roll_pitch;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        roll_pitch[i] = roll * _vertical_roll_factor[i] + pitch * _vertical_pitch_factor[i];
    }

    // 与标准多旋翼 mixer 一致，先在合适的集体升力附近为姿态控制预留空间，
    // 再尽量靠近高度控制器要求的油门。这样油门接近上下限时仍优先保姿态。
    const float throttle_best_rp = MIN(0.5f, throttle_avg_max);
    float rp_scale = 1.0f;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        const float base = throttle_best_rp * _vertical_throttle_factor[i];
        if (roll_pitch[i] > 0.0f) {
            rp_scale = MIN(rp_scale, (1.0f - base) / roll_pitch[i]);
        } else if (roll_pitch[i] < 0.0f) {
            rp_scale = MIN(rp_scale, -base / roll_pitch[i]);
        }
    }
    rp_scale = constrain_float(rp_scale, 0.0f, 1.0f);
    if (rp_scale < 1.0f) {
        limit.roll = true;
        limit.pitch = true;
    }
    roll_pitch *= rp_scale;

    // 在保持已分配 roll/pitch 的前提下，计算集体油门还能移动的上下边界。
    float throttle_lower = 0.0f;
    float throttle_upper = _throttle_thrust_max * thr_lin.get_compensation_gain();
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        const float throttle_factor = _vertical_throttle_factor[i];
        if (throttle_factor > 0.0f) {
            throttle_lower = MAX(throttle_lower, -roll_pitch[i] / throttle_factor);
            throttle_upper = MIN(throttle_upper, (1.0f - roll_pitch[i]) / throttle_factor);
        }
    }

    const float throttle_limited = constrain_float(throttle, throttle_lower, throttle_upper);
    if (throttle > throttle_upper) {
        limit.throttle_upper = true;
    }
    // 最终垂直分量 = 集体升力贡献 + 横滚/俯仰贡献。
    throttle_out = throttle_limited;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        vertical[i] = throttle_limited * _vertical_throttle_factor[i] + roll_pitch[i];
    }
}

void AP_MotorsScorpio::mix_tilt_horizontal(float throttle, float yaw,
        const Vector3f &vertical,
        Vector3f &horizontal)
{
    // 水平通道的优先级为 yaw > forward/lateral。
    // 达到最大倾转角或电机总推力上限时，先保留偏航，再缩小平移力。
    const Vector3f yaw_out = _horizontal_yaw_factor * (yaw * throttle);
    const float yaw_scale = horizontal_mix_scale(vertical, Vector3f(), yaw_out);
    horizontal = yaw_out * yaw_scale;
    if (yaw_scale < 1.0f && !is_zero(yaw)) {
        limit.yaw = true;
    }

    // 前向与侧向控制量分别乘各自 factor，随后线性叠加为平移水平力。
    Vector3f translation;
    const float forward = get_forward() * throttle * _xy_gain;
    const float lateral = get_lateral() * throttle * _xy_gain;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        translation[i] = forward * _horizontal_forward_factor[i] +
                         lateral * _horizontal_lateral_factor[i];
    }
    horizontal += translation * horizontal_mix_scale(vertical, horizontal, translation);
}

void AP_MotorsScorpio::combine_actuator_vectors(const Vector3f &vertical,
        const Vector3f &horizontal)
{
    // 不能直接线性叠加最终油门和倾转角，必须先在力分量空间叠加，再转换：
    // motor = sqrt(vertical^2 + horizontal^2)
    // tilt  = atan2(horizontal, vertical)
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        _motor_thrust[i] = constrain_float(sqrtf(sq(vertical[i]) + sq(horizontal[i])), 0.0f, 1.0f);
        _tilt_angle_rad[i] = atan2f(horizontal[i], vertical[i]);
    }
}

void AP_MotorsScorpio::output_armed_stabilizing()
{
    // 每个控制周期清除上周期的限幅状态，由本周期混控结果重新设置。
    limit.roll = false;
    limit.pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    const float compensation_gain = thr_lin.get_compensation_gain();
    const float roll = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch = (_pitch_in + _pitch_in_ff) * compensation_gain;
    const float yaw = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle = get_throttle() * compensation_gain;
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    if (throttle <= 0.0f) {
        throttle = 0.0f;
        limit.throttle_lower = true;
    }
    const float throttle_thrust_max = _throttle_thrust_max * compensation_gain;
    if (throttle >= throttle_thrust_max) {
        throttle = throttle_thrust_max;
        limit.throttle_upper = true;
    }
    throttle_avg_max = constrain_float(throttle_avg_max, throttle,
                                       throttle_thrust_max);

    if (!_allocation_valid) {
        set_limit_flag_pitch_roll_yaw(true);
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            _motor_thrust[i] = 0.0f;
            _tilt_angle_rad[i] = 0.0f;
        }
        return;
    }

    // 第一步：throttle/roll/pitch 叠加为三个旋翼的垂直力分量。
    Vector3f vertical;
    float throttle_limited;
    mix_y3_vertical(throttle, throttle_avg_max, roll, pitch, vertical, throttle_limited);

    // 第二步：yaw/forward/lateral 叠加为三个旋翼的水平力分量。
    Vector3f horizontal;
    mix_tilt_horizontal(throttle_limited, yaw, vertical, horizontal);

    // 第三步：把垂直、水平力分量转换为电机推力和倾转角。
    combine_actuator_vectors(vertical, horizontal);

    _throttle_out = throttle_limited / compensation_gain;
}

void AP_MotorsScorpio::output_to_motors()
{
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            _actuator[i] = 0.0f;
            _tilt_angle_rad[i] = 0.0f;
        }
        break;
    case SpoolState::GROUND_IDLE:
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
            _tilt_angle_rad[i] = 0.0f;
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_motor_thrust[i]));
        }
        break;
    }

    rc_write(FRONT_RIGHT, output_to_pwm(_actuator[FRONT_RIGHT]));
    rc_write(REAR, output_to_pwm(_actuator[REAR]));
    rc_write(FRONT_LEFT, output_to_pwm(_actuator[FRONT_LEFT]));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, degrees(_tilt_angle_rad[FRONT_RIGHT]) * 100.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, degrees(_tilt_angle_rad[REAR]) * 100.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, degrees(_tilt_angle_rad[FRONT_LEFT]) * 100.0f);
}

void AP_MotorsScorpio::thrust_compensation()
{
    if (_thrust_compensation_callback != nullptr) {
        _thrust_compensation_callback(_motor_thrust, ACTUATOR_COUNT);
    }
}

void AP_MotorsScorpio::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    switch (motor_seq) {
    case 1:
        rc_write(FRONT_RIGHT, pwm);
        break;
    case 2:
        rc_write(REAR, pwm);
        break;
    case 3:
        rc_write(FRONT_LEFT, pwm);
        break;
    default:
        break;
    }
}

float AP_MotorsScorpio::get_roll_factor(uint8_t i)
{
    if (i < ACTUATOR_COUNT && _allocation_valid) {
        return _vertical_roll_factor[i];
    }
    return 0.0f;
}

bool AP_MotorsScorpio::arming_checks(size_t buflen, char *buffer) const
{
    if (!SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight) ||
        !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear) ||
        !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft)) {
        hal.util->snprintf(buffer, buflen, "Scorpio tilt outputs not assigned");
        return false;
    }
    if (!_allocation_valid) {
        hal.util->snprintf(buffer, buflen, "Scorpio invalid rotor geometry");
        return false;
    }
    return AP_MotorsMulticopter::arming_checks(buflen, buffer);
}
