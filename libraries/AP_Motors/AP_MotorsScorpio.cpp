/*
   Scorpio has three independently tilting rotors.  Motor order is:
     Motor1 / tilt-right: front right (CW viewed from above)
     Motor2 / tilt-rear:  rear        (CCW viewed from above)
     Motor3 / tilt-left:  front left  (CW viewed from above)

   Rotor magnitude controls vertical force, roll and pitch.  Rotor tilt
   controls forward force, lateral force and yaw. The allocator uses the
   measured planar rotor coordinates and normalises the resulting moment arms.
 */

#include "AP_MotorsScorpio.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_MotorsScorpio::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // 1 to 5 are reserved for an early Scorpio geometry layout.

    // @Param: SC_F_ANG
    // @DisplayName: Scorpio front tilt axis angle
    // @Description: Absolute yaw installation angle of the front rotor tilt mechanisms
    // @Units: deg
    // @Range: 1 89
    // @User: Advanced
    AP_GROUPINFO("SC_F_ANG", 6, AP_MotorsScorpio, _front_axis_angle_deg, 30.0f),

    // @Param: SC_R_ANG
    // @DisplayName: Scorpio rear tilt axis angle
    // @Description: Yaw installation angle of the rear rotor tilt mechanism
    // @Units: deg
    // @Range: 1 179
    // @User: Advanced
    AP_GROUPINFO("SC_R_ANG", 7, AP_MotorsScorpio, _rear_axis_angle_deg, 90.0f),

    // @Param: SC_TILT_MAX
    // @DisplayName: Scorpio maximum rotor tilt
    // @Description: Maximum commanded tilt angle from the neutral vertical position
    // @Units: deg
    // @Range: 5 60
    // @User: Standard
    AP_GROUPINFO("SC_TILT_MAX", 8, AP_MotorsScorpio, _tilt_max_deg, 45.0f),

    // @Param: SC_XY_GAIN
    // @DisplayName: Scorpio horizontal force gain
    // @Description: Scales forward and lateral force requests before allocation
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("SC_XY_GAIN", 9, AP_MotorsScorpio, _xy_gain, 1.0f),

    // 10 was SC_YAW_ARM. Yaw allocation is normalised directly from geometry.

    // 11 was SC_REACT. Rotor reaction torque is treated as a disturbance
    // and rejected by the closed-loop yaw controller, like other motor types.

    // @Param: SC_FR_X
    // @DisplayName: Scorpio front-right rotor X position
    // @Description: Front-right rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_FR_X", 12, AP_MotorsScorpio, _front_right_x, 0.18811f),

    // @Param: SC_FR_Y
    // @DisplayName: Scorpio front-right rotor Y position
    // @Description: Front-right rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_FR_Y", 13, AP_MotorsScorpio, _front_right_y, 0.20529f),

    // 14 is reserved for the removed SC_FR_Z parameter.

    // @Param: SC_FL_X
    // @DisplayName: Scorpio front-left rotor X position
    // @Description: Front-left rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_FL_X", 15, AP_MotorsScorpio, _front_left_x, 0.18811f),

    // @Param: SC_FL_Y
    // @DisplayName: Scorpio front-left rotor Y position
    // @Description: Front-left rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_FL_Y", 16, AP_MotorsScorpio, _front_left_y, -0.20529f),

    // 17 is reserved for the removed SC_FL_Z parameter.

    // @Param: SC_R_X
    // @DisplayName: Scorpio rear rotor X position
    // @Description: Rear rotor centre X coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_R_X", 18, AP_MotorsScorpio, _rear_x, -0.22917f),

    // @Param: SC_R_Y
    // @DisplayName: Scorpio rear rotor Y position
    // @Description: Rear rotor centre Y coordinate relative to the body origin in the ArduPilot forward-right-down body frame
    // @Units: m
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("SC_R_Y", 19, AP_MotorsScorpio, _rear_y, 0.0f),

    // 20 is reserved for the removed SC_R_Z parameter.

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
    set_update_rate(_speed_hz);
    _mav_type = MAV_TYPE_TRICOPTER;
    set_initialised_ok(frame_class == MOTOR_FRAME_SCORPIO);
}

void AP_MotorsScorpio::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    setup_tilt_outputs();
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

bool AP_MotorsScorpio::calculate_matrices(Matrix3f &vertical_inverse,
                                          Matrix3f &horizontal_inverse,
                                          Vector3f &tilt_direction_x,
                                          Vector3f &tilt_direction_y) const
{
    // ArduPilot body Y points right.  The RSDF uses Gazebo Y-left, hence the
    // signs below.  Positive tilt follows the RSDF joint's local -Y axis.
    Vector3f pos_x;
    Vector3f pos_y;
    get_rotor_positions(pos_x, pos_y);
    const Vector3f axis_angle(radians(_front_axis_angle_deg),
                              radians(_rear_axis_angle_deg),
                              -radians(_front_axis_angle_deg));
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        tilt_direction_x[i] = -cosf(axis_angle[i]);
        tilt_direction_y[i] = sinf(axis_angle[i]);
    }

    // Upward vertical thrust produces Mx=-y*T and My=x*T.
    Matrix3f vertical(Vector3f(1.0f, 1.0f, 1.0f),
                      Vector3f(-pos_y.x, -pos_y.y, -pos_y.z),
                      pos_x);
    if (!vertical.inverse(vertical_inverse)) {
        return false;
    }

    // Horizontal allocation maps per-rotor horizontal force q to Fx,Fy,Mz.
    const Vector3f yaw_arm(pos_x.x * tilt_direction_y.x - pos_y.x * tilt_direction_x.x,
                           pos_x.y * tilt_direction_y.y - pos_y.y * tilt_direction_x.y,
                           pos_x.z * tilt_direction_y.z - pos_y.z * tilt_direction_x.z);
    Matrix3f horizontal(tilt_direction_x, tilt_direction_y, yaw_arm);
    return horizontal.inverse(horizontal_inverse);
}

void AP_MotorsScorpio::get_rotor_positions(Vector3f &pos_x,
                                            Vector3f &pos_y) const
{
    // Vector element order is front-right, rear, front-left.
    pos_x = Vector3f(_front_right_x, _rear_x, _front_left_x);
    pos_y = Vector3f(_front_right_y, _rear_y, _front_left_y);
}

bool AP_MotorsScorpio::horizontal_mix_is_feasible(float scale,
                                                   const Vector3f &vertical,
                                                   const Vector3f &vertical_correction,
                                                   const Vector3f &horizontal) const
{
    const float max_angle = radians(constrain_float(_tilt_max_deg, 5.0f, 60.0f));
    const float tan_max = tanf(max_angle);
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        const float vertical_i = vertical[i] + scale * vertical_correction[i];
        const float horizontal_i = scale * horizontal[i];
        if (vertical_i < 0.0f || fabsf(horizontal_i) > vertical_i * tan_max) {
            return false;
        }
        if (sq(vertical_i) + sq(horizontal_i) > 1.0f) {
            return false;
        }
    }
    return true;
}

void AP_MotorsScorpio::output_armed_stabilizing()
{
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

    if (throttle <= 0.0f) {
        throttle = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle >= _throttle_thrust_max) {
        throttle = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    Matrix3f vertical_inverse;
    Matrix3f horizontal_inverse;
    Vector3f direction_x;
    Vector3f direction_y;
    if (!calculate_matrices(vertical_inverse, horizontal_inverse, direction_x, direction_y)) {
        set_limit_flag_pitch_roll_yaw(true);
        for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
            _motor_thrust[i] = 0.0f;
            _tilt_angle_rad[i] = 0.0f;
        }
        return;
    }

    Vector3f pos_x;
    Vector3f pos_y;
    get_rotor_positions(pos_x, pos_y);

    // Derive the hover distribution and normalised roll/pitch authority from
    // the entered rotor coordinates. This also supports a mildly asymmetric
    // real vehicle without requiring hand-written mixer factors.
    Vector3f collective_distribution = vertical_inverse * Vector3f(1.0f, 0.0f, 0.0f);
    const float collective_max = MAX(collective_distribution.x,
                                     MAX(collective_distribution.y, collective_distribution.z));
    const float collective_min = MIN(collective_distribution.x,
                                     MIN(collective_distribution.y, collective_distribution.z));
    if (collective_max <= 0.0f || collective_min < 0.0f) {
        set_limit_flag_pitch_roll_yaw(true);
        return;
    }
    Vector3f vertical = collective_distribution * (throttle / collective_max);
    const float roll_arm = MAX(pos_y.x, MAX(pos_y.y, pos_y.z)) -
                           MIN(pos_y.x, MIN(pos_y.y, pos_y.z));
    const float pitch_arm = MAX(pos_x.x, MAX(pos_x.y, pos_x.z)) -
                            MIN(pos_x.x, MIN(pos_x.y, pos_x.z));
    const Vector3f attitude_delta = vertical_inverse *
        Vector3f(0.0f, roll * roll_arm, pitch * pitch_arm);

    // Preserve the requested roll/pitch direction while fitting all vertical
    // thrusts into the unidirectional motor range.
    float attitude_scale = 1.0f;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        if (attitude_delta[i] > 0.0f) {
            attitude_scale = MIN(attitude_scale, (1.0f - vertical[i]) / attitude_delta[i]);
        } else if (attitude_delta[i] < 0.0f) {
            attitude_scale = MIN(attitude_scale, -vertical[i] / attitude_delta[i]);
        }
    }
    attitude_scale = constrain_float(attitude_scale, 0.0f, 1.0f);
    if (attitude_scale < 1.0f) {
        limit.roll = true;
        limit.pitch = true;
    }
    vertical += attitude_delta * attitude_scale;

    // Fx/Fy come from AC_AttitudeControl_Multi_6DoF. Yaw is generated by
    // vectored thrust. Rotor drag torque is rejected by the yaw PID rather
    // than relying on a difficult-to-identify physical feed-forward value.
    const float forward_force = get_forward() * throttle * _xy_gain;
    const float lateral_force = get_lateral() * throttle * _xy_gain;
    const Vector3f yaw_arm(pos_x.x * direction_y.x - pos_y.x * direction_x.x,
                           pos_x.y * direction_y.y - pos_y.y * direction_x.y,
                           pos_x.z * direction_y.z - pos_y.z * direction_x.z);
    const float yaw_normalisation_arm = MAX(fabsf(yaw_arm.x),
                                            MAX(fabsf(yaw_arm.y), fabsf(yaw_arm.z)));
    const float yaw_moment = yaw * throttle * yaw_normalisation_arm;
    Vector3f horizontal = horizontal_inverse * Vector3f(forward_force, lateral_force, yaw_moment);

    // With only planar rotor coordinates configured, no height-offset moment
    // compensation is applied here.
    const Vector3f vertical_correction;

    float horizontal_scale = 1.0f;
    if (!horizontal_mix_is_feasible(horizontal_scale, vertical, vertical_correction, horizontal)) {
        float low = 0.0f;
        float high = 1.0f;
        for (uint8_t i = 0; i < 14; i++) {
            const float mid = 0.5f * (low + high);
            if (horizontal_mix_is_feasible(mid, vertical, vertical_correction, horizontal)) {
                low = mid;
            } else {
                high = mid;
            }
        }
        horizontal_scale = low;
        if (!is_zero(yaw)) {
            limit.yaw = true;
        }
    }

    vertical += vertical_correction * horizontal_scale;
    horizontal *= horizontal_scale;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        _motor_thrust[i] = constrain_float(sqrtf(sq(vertical[i]) + sq(horizontal[i])), 0.0f, 1.0f);
        _tilt_angle_rad[i] = atan2f(horizontal[i], vertical[i]);
    }

    _throttle_out = throttle / compensation_gain;
}

void AP_MotorsScorpio::output_to_motors()
{
    setup_tilt_outputs();

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
    if (i == FRONT_RIGHT) {
        return -1.0f;
    }
    if (i == FRONT_LEFT) {
        return 1.0f;
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
    return AP_MotorsMulticopter::arming_checks(buflen, buffer);
}
