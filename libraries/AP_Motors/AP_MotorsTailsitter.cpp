/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include "AP_MotorsTailsitter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

extern float aim_pitch_deg;
extern float tilt_cdeg_R, tilt_cdeg_L;
extern float tilt2_cdeg_R, tilt2_cdeg_L;

extern float yaw_factor_f;

#define SERVO1_OUTPUT_RANGE    9000
#define SERVO2_OUTPUT_RANGE    13500
#define SEIRAL_SERVO_MAX_ANGLE 150
#define SERVO2_FACTOR          0.5f
#define SERVO1_FACTOR          0.75f

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) || SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tilt2MotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tilt2MotorRight, SERVO2_OUTPUT_RANGE + 50);
    SRV_Channels::set_output_min_max(SRV_Channel::k_tilt2MotorRight, 500, 2500);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tilt2MotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tilt2MotorLeft, SERVO2_OUTPUT_RANGE + 50);
    SRV_Channels::set_output_min_max(SRV_Channel::k_tilt2MotorLeft, 500, 2500);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO1_OUTPUT_RANGE + 50);
    SRV_Channels::set_output_min_max(SRV_Channel::k_tiltMotorRight, 500, 2500);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO1_OUTPUT_RANGE + 50);
    SRV_Channels::set_output_min_max(SRV_Channel::k_tiltMotorLeft, 500, 2500);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorLeft, 9000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorRight, 9000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 9000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 9000);

    // hiwonder_l = AP_Hiwonder_L::get_singleton();
    // hiwonder_r = AP_Hiwonder_R::get_singleton();

    // hiwonder_r->set_position(SERVO_1, 1500, 0);
    // hiwonder_r->set_position(SERVO_2, 1500, 0);
    // hiwonder_l->set_position(SERVO_3, 1500, 0);
    // hiwonder_l->set_position(SERVO_4, 1500, 0);

    _mav_type = MAV_TYPE_VTOL_DUOROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}

/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t speed_hz)
    : AP_MotorsMulticopter(speed_hz)
{
    set_update_rate(speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    float throttle = get_throttle() * 2.0f;

    if (throttle > 1.0f)
        throttle = 1.0f;
    else if (throttle < 0)
        throttle = 0;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0]           = 0.0f;
            _actuator[1]           = 0.0f;
            _actuator[2]           = 0.0f;
            _external_min_throttle = 0.0;
            break;

        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            _external_min_throttle = 0.0;
            break;

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thr_lin.thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thr_lin.thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thr_lin.thrust_to_actuator(_throttle));
            break;
    }

    // float tilt2_l = tilt2_cdeg_L + aim_pitch_deg * 100 + _forward_in * SERVO2_OUTPUT_RANGE * throttle;
    // float tilt2_r = tilt2_cdeg_R - aim_pitch_deg * 100 - _forward_in * SERVO2_OUTPUT_RANGE * throttle;

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2] * 100);

    float tilt_left  = _tilt_left * SERVO1_OUTPUT_RANGE * SERVO1_FACTOR * throttle + tilt_cdeg_L;
    float tilt_right = _tilt_right * SERVO1_OUTPUT_RANGE * SERVO1_FACTOR * throttle + tilt_cdeg_R;

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

    static uint16_t cnt = 0;

    float tilt_2_left = _tilt_2_left*SERVO2_OUTPUT_RANGE;
    float tilt_2_right = _tilt_2_right*SERVO2_OUTPUT_RANGE;

    if ((++cnt % 2) == 0) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorLeft, tilt_2_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tilt2MotorRight, tilt_2_right);
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t  chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float roll_thrust;     // roll thrust input value, +/- 1.0
    float pitch_thrust;    // pitch thrust input value, +/- 1.0
    float yaw_thrust;      // yaw thrust input value, +/- 1.0
    float throttle_thrust; // throttle thrust input value, 0.0 - 1.0
    float thrust_max;      // highest motor value
    float thrust_min;      // lowest motor value
    float thr_adj = 0.0f;  // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain  = thr_lin.get_compensation_gain();
    roll_thrust                    = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust                   = _pitch_in + _pitch_in_ff;
    yaw_thrust                     = _yaw_in + _yaw_in_ff;
    throttle_thrust                = get_throttle() * compensation_gain;

    extern float _aim_pitch_deg;
    Matrix3f rot;
    rot.from_euler312(0, _aim_pitch_deg, 0.0f);

    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above min and below current limited throttle
    if (throttle_thrust <= min_throttle_out) {
        throttle_thrust      = min_throttle_out;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= max_throttle_out) {
        throttle_thrust      = max_throttle_out;
        limit.throttle_upper = true;
    }

    if (roll_thrust >= 1.0) {
        // cannot split motor outputs by more than 1
        roll_thrust = 1;
        limit.roll  = true;
    }

    Vector3f rpy_vec = rot * Vector3f(roll_thrust, pitch_thrust, yaw_thrust);

    float n1s2, n1c2, n2s2, n2c2;
    n1s2 = rpy_vec[1] / 2 + rpy_vec[2] / 2;
    n1c2 = -rpy_vec[0] / 2 + throttle_thrust / 2;
    n2s2 = -rpy_vec[1] / 2 + rpy_vec[2] / 2;
    n2c2 = rpy_vec[0] / 2 + throttle_thrust / 2;

    _thrust_right = sqrtf(n1s2 * n1s2 + n1c2 * n1c2); 
    _thrust_left  = sqrtf(n2s2 * n2s2 + n2c2 * n2c2);
    if (throttle_thrust < 0.1) {
        _tilt_right = 0;
        _tilt_right = 0;
    } else {
        _tilt_right = atan2f(n1s2, n1c2);
        _tilt_left  = atan2f(n2s2, n2c2);
    }

    // calculate left and right throttle outputs
    // _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    // _thrust_right = throttle_thrust - roll_thrust * 0.5f;

    thrust_max = MAX(_thrust_right, _thrust_left);
    thrust_min = MIN(_thrust_right, _thrust_left);
    if (thrust_max > 1.0f) {
        // if max thrust is more than one reduce average throttle
        thr_adj              = 1.0f - thrust_max;
        limit.throttle_upper = true;
    } else if (thrust_min < 0.0) {
        // if min thrust is less than 0 increase average throttle
        // but never above max boost
        thr_adj = -thrust_min;
        if ((throttle_thrust + thr_adj) > max_boost_throttle) {
            thr_adj = MAX(max_boost_throttle - throttle_thrust, 0.0);
            // in this case we throw away some roll output, it will be uneven
            // constraining the lower motor more than the upper
            // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
            // so its worth keeping the higher roll control power at a minor cost to yaw
            limit.roll = true;
        }
        limit.throttle_lower = true;
    }

    // Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);

    _throttle = throttle_thrust;

    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    if (_has_diff_thrust) {
        _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
    } else {
        _throttle_out = throttle_thrust / compensation_gain;
    }

    // thrust vectoring
    // pitch_thrust *= 0.75f;
    // yaw_thrust *= yaw_factor_f;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // _tilt_left  = -pitch_thrust + yaw_thrust;
    // _tilt_right = pitch_thrust + yaw_thrust;
#else
    // _tilt_left  = pitch_thrust - yaw_thrust;
    // _tilt_right = -pitch_thrust - yaw_thrust;
#endif

    forward_thrust = get_forward();

    _tilt_2_right = forward_thrust;
    _tilt_2_left = _tilt_2_right;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tilt2MotorRight, pwm);
            // hiwonder_r->set_position(SERVO_2, pwm, 0);
            break;

        case 3:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            // hiwonder_r->set_position(SERVO_1, pwm, 0);
            break;

        case 4:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;

        case 5:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tilt2MotorLeft, pwm);
            // hiwonder_l->set_position(SERVO_4, pwm, 0);
            break;
        case 6:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            // hiwonder_l->set_position(SERVO_3, pwm, 0);
            break;

        default:
            // do nothing
            break;
    }
}
