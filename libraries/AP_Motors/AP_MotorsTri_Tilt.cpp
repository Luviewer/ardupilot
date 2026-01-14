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
   Coaxial Y6B Tricopter with Tiltable Rotors (5DOF control)
   
   Motor Layout:
   - Motors 0-1: Front-right coaxial pair (upper CW, lower CCW)
   - Motors 2-3: Rear coaxial pair (upper CW, lower CCW)  
   - Motors 4-5: Front-left coaxial pair (upper CW, lower CCW)
   
   Control allocation based on MATLAB derivation:
   Static matrix F_alloc maps 5DOF [Fx, Fz, Mx, My, Mz] to 6 intermediate 
   variables [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
*/

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_MotorsTri_Tilt.h"

extern const AP_HAL::HAL& hal;

// Parameters
const AP_Param::GroupInfo AP_MotorsTri_Tilt::var_info[] = {
    // @Param: TRI_TILT_LX
    // @DisplayName: Front arm X distance (normalized)
    // @Description: Front rotor arm X-axis distance from CG, normalized
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LX", 1, AP_MotorsTri_Tilt, _lfront_x, 0.5f),

    // @Param: TRI_TILT_LY
    // @DisplayName: Front arm Y distance (normalized)
    // @Description: Front rotor arm Y-axis distance from CG, normalized
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LY", 2, AP_MotorsTri_Tilt, _lfront_y, 0.5f),

    // @Param: TRI_TILT_LREAR
    // @DisplayName: Rear arm length (normalized)
    // @Description: Rear rotor arm length from CG, normalized
    // @Range: 0.1 2.0
    // @User: Advanced
    AP_GROUPINFO("TILT_LREAR", 3, AP_MotorsTri_Tilt, _lrear, 1.0f),

    // @Param: TRI_TILT_ANG_MAX
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum tilt angle for rotors in degrees
    // @Range: 0 135
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("TILT_ANG_MAX", 4, AP_MotorsTri_Tilt, _servo_angle_max, 135.0f),

    // @Param: TRI_TILT_YAW_FAC
    // @DisplayName: Yaw torque factor
    // @Description: Factor for yaw torque via differential thrust on coaxial pairs
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_FAC", 8, AP_MotorsTri_Tilt, _yaw_torque_factor, 0.15f),

    // @Param: TRI_TILT_YAW_DIR
    // @DisplayName: Yaw direction
    // @Description: Set to -1 if yaw response is reversed (vehicle yaws opposite to commanded direction)
    // @Values: -1:Reversed, 1:Normal
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_DIR", 9, AP_MotorsTri_Tilt, _yaw_dir, 1),

    // @Param: TRI_TILT_SVO_FR_REV
    // @DisplayName: Front-right tilt servo reverse
    // @Description: Set to 1 to reverse front-right tilt servo direction
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FR_REV", 11, AP_MotorsTri_Tilt, _tilt_servo_fr_rev, 0),

    // @Param: TRI_TILT_SVO_REAR_REV
    // @DisplayName: Rear tilt servo reverse
    // @Description: Set to 1 to reverse rear tilt servo direction
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_REAR_REV", 12, AP_MotorsTri_Tilt, _tilt_servo_rear_rev, 0),

    // @Param: TRI_TILT_SVO_FL_REV
    // @DisplayName: Front-left tilt servo reverse
    // @Description: Set to 1 to reverse front-left tilt servo direction
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FL_REV", 13, AP_MotorsTri_Tilt, _tilt_servo_fl_rev, 0),

    // @Param: TRI_TILT_PIT_OFF_MAX
    // @DisplayName: Max pitch attitude offset (RC7)
    // @Description: Maximum pitch attitude offset in degrees commanded by RC7 (1500->0, 1000->-max, 2000->+max)
    // @Range: 0 45
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PIT_OFF_MAX", 14, AP_MotorsTri_Tilt, _tilt_pitch_off_max_deg, 20.0f),

    AP_GROUPEND
};

// init
void AP_MotorsTri_Tilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Enable 6 motors for coaxial Y6B configuration
    add_motor_num(AP_MOTORS_MOT_1);  // Front-right upper
    add_motor_num(AP_MOTORS_MOT_2);  // Front-right lower
    add_motor_num(AP_MOTORS_MOT_3);  // Rear upper
    add_motor_num(AP_MOTORS_MOT_4);  // Rear lower
    add_motor_num(AP_MOTORS_MOT_5);  // Front-left upper
    add_motor_num(AP_MOTORS_MOT_6);  // Front-left lower

    // Set update rate for motors
    set_update_rate(_speed_hz);

    // Enable motor flags for calibration
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // setup default motor and servo mappings (can be overridden via SERVOx_FUNCTION)
    // Map: a1->front-right, a2->rear, a3->front-left
    const bool ok_fr = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, AP_MOTORS_TRI_TILT_SERVO_FR);
    const bool ok_rear = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRear, AP_MOTORS_TRI_TILT_SERVO_REAR);
    const bool ok_fl = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, AP_MOTORS_TRI_TILT_SERVO_FL);

    // Set default PWM range for tilt servos (hardware: 500~2500us)
    // This only changes defaults; user-set SERVOx_MIN/MAX will not be overridden.
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRight, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRear, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorLeft, 500, 2500);

    // Set angular range for tilt servos (centi-degrees)
    // If _servo_angle_max is 0, we treat it as "no software clamp" and set a wide range here.
    const float ang_max_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
    const int16_t servo_range_cd = int16_t(constrain_float(ang_max_deg, 0.0f, float(AP_MOTORS_TRI_TILT_ANGLE_MAX)) * 100.0f);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, servo_range_cd);

    // Check if servos are assigned (either via defaults or user mapping)
    _servos_assigned =
        (ok_fr || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)) &&
        (ok_rear || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear)) &&
        (ok_fl || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));

    // Setup motors and allocation matrix
    setup_motors(frame_class, frame_type);

    _mav_type = MAV_TYPE_TRICOPTER;

    // Record successful initialization
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI && _servos_assigned);
}

// set frame class and type
void AP_MotorsTri_Tilt::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Reinitialize if frame changes
    if (frame_class != _active_frame_class || frame_type != _active_frame_type) {
        _active_frame_class = frame_class;
        _active_frame_type = frame_type;
        setup_motors(frame_class, frame_type);
    }

    set_initialised_ok((frame_class == MOTOR_FRAME_TRI) && _servos_assigned);
}

// set update rate to motors
void AP_MotorsTri_Tilt::set_update_rate(uint16_t speed_hz)
{
    // Record requested speed
    _speed_hz = speed_hz;

    // Set update rate for all 6 motors
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6;
    rc_set_freq(mask, _speed_hz);
}

// setup motors - configures the static allocation matrix
void AP_MotorsTri_Tilt::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // NOTE:
    // Do NOT call AP_MotorsMatrix::remove_motor() here.
    // This backend drives motor outputs directly via rc_write() and uses motor_enabled[]
    // for spool-state behaviour. remove_motor() clears motor_enabled[] which would
    // stop outputs (user reports motor outputs stuck at 0).
    //
    // Motor channel defaults are set via add_motor_num() in init().
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // Initialize matrices to zero
    memset(_alloc_matrix, 0, sizeof(_alloc_matrix));
    memset(_alloc_matrix_pinv, 0, sizeof(_alloc_matrix_pinv));
    memset(_thrust, 0, sizeof(_thrust));
    memset(_tilt_angle, 0, sizeof(_tilt_angle));
    memset(_intermediate, 0, sizeof(_intermediate));

    // Calculate allocation matrix and its pseudo-inverse
    calculate_allocation_matrix();
    calculate_allocation_matrix_pinv();

    _frame_class_string = "TRI_TILT";
    _frame_type_string = "Coaxial-Y6B";
}

// get motor mask
uint32_t AP_MotorsTri_Tilt::get_motor_mask()
{
    uint32_t mask = AP_MotorsMatrix::get_motor_mask();

    // add tilt servos
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRight, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRear, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorLeft, chan)) {
        mask |= 1U << chan;
    }

    return mask;
}

// Run arming checks
bool AP_MotorsTri_Tilt::arming_checks(size_t buflen, char *buffer) const
{
    // Check that servos are assigned
    if (!_servos_assigned) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Servos not assigned");
        return false;
    }

    // Check geometric parameters are valid
    if (_lfront_x <= 0.0f || _lfront_y <= 0.0f || _lrear <= 0.0f) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Invalid geometry params");
        return false;
    }

    // Check angle limits:
    // _servo_angle_max == 0 means "no software clamp" (still constrained by SERVOx_MIN/MAX).
    // otherwise enforce a minimum to avoid tiny ranges causing excessive scaling/saturation.
    const float ang_max = _servo_angle_max;
    if ((!is_zero(ang_max) && ang_max < AP_MOTORS_TRI_TILT_ANGLE_MIN) ||
        ang_max < 0.0f ||
        ang_max > AP_MOTORS_TRI_TILT_ANGLE_MAX) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Invalid angle limits");
        return false;
    }

    return true;
}

// thrust compensation
void AP_MotorsTri_Tilt::thrust_compensation(void)
{
    // Call parent class thrust compensation
    AP_MotorsMatrix::thrust_compensation();
}

// output_armed_stabilizing - main control allocation
// Implements: 5DOF input -> static matrix -> 6 intermediate variables -> thrust + tilt angles
void AP_MotorsTri_Tilt::output_armed_stabilizing()
{
    // Get voltage and altitude compensation gain
    const float compensation_gain = thr_lin.get_compensation_gain();

    // Prepare 5DOF control inputs
    float desired[5];

    // NOTE:
    // - Fx (forward force) must come from get_forward() (AP_Motors::set_forward caller), not from _pitch_in.
    // - _pitch_in is the pitch moment request (My).
    float throttle_thrust = get_throttle() * compensation_gain;
    // throttle is constrained in the motor library domain [0,1]
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= 1.0f) {
        throttle_thrust = 1.0f;
        limit.throttle_upper = true;
    }

    // scale forward with throttle (matches 6DoF scripting mixer behaviour)
    const float forward_thrust = get_forward() * throttle_thrust;

    // IMPORTANT SIGN CONVENTION (matches cal_alloc_tri.m):
    // The derived allocation uses body Z positive DOWN (NED).
    // With rotor thrust pointing "up", the resulting body-force is negative Fz.
    // Therefore, for normal multicopter throttle (upwards), desired Fz must be negative.
    //
    // Also Fx row is defined as Fx = -(F1*sin(a1)+F2*sin(a2)+F3*sin(a3)),
    // so a positive forward thrust command should map with a negative sign here.
    desired[0] = -forward_thrust;     // Fx (forward force, +x forward)
    desired[1] = -throttle_thrust;    // Fz (down positive, so up-thrust is negative)
    // Mx (Roll moment)
    desired[2] = (_roll_in + _roll_in_ff) * compensation_gain;
    // My (Pitch moment)
    desired[3] = (_pitch_in + _pitch_in_ff) * compensation_gain;
    // Mz (Yaw moment): handled by coaxial differential thrust in output_to_motors()
    desired[4] = 0.0f;

    // Calculate 6 intermediate variables using pseudo-inverse matrix
    // _intermediate = [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
    for (int i = 0; i < 6; i++) {
        _intermediate[i] = 0.0f;
        for (int j = 0; j < 5; j++) {
            _intermediate[i] += _alloc_matrix_pinv[i][j] * desired[j];
        }
    }

    // Solve for thrust and tilt angles from intermediate variables
    // Motor 1: Front-right
    float f1_sin = _intermediate[0];
    float f1_cos = _intermediate[1];
    _thrust[0] = sqrtf(f1_sin * f1_sin + f1_cos * f1_cos);
    _tilt_angle[0] = atan2f(f1_sin, f1_cos);

    // Motor 2: Rear
    float f2_sin = _intermediate[2];
    float f2_cos = _intermediate[3];
    _thrust[1] = sqrtf(f2_sin * f2_sin + f2_cos * f2_cos);
    _tilt_angle[1] = atan2f(f2_sin, f2_cos);

    // Motor 3: Front-left
    float f3_sin = _intermediate[4];
    float f3_cos = _intermediate[5];
    _thrust[2] = sqrtf(f3_sin * f3_sin + f3_cos * f3_cos);
    _tilt_angle[2] = atan2f(f3_sin, f3_cos);

    // Apply constraints
    // If _servo_angle_max == 0 => no software clamp on tilt angles
    const bool clamp_tilt = (_servo_angle_max > 0.0f);
    const float max_angle_rad = clamp_tilt ? radians(_servo_angle_max) : radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX));

    for (int i = 0; i < 3; i++) {
        if (clamp_tilt) {
            _tilt_angle[i] = constrain_float(_tilt_angle[i], -max_angle_rad, max_angle_rad);
        } else {
            // still keep it sane to avoid NaNs/overflow downstream
            _tilt_angle[i] = constrain_float(_tilt_angle[i], -radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX)), radians(float(AP_MOTORS_TRI_TILT_ANGLE_MAX)));
        }

        // Constrain thrust [0, 1]
        _thrust[i] = constrain_float(_thrust[i], 0.0f, 1.0f);

        // Check limits
        if (_thrust[i] >= 1.0f) {
            limit.throttle_upper = true;
        }
        if (_thrust[i] <= 0.0f) {
            limit.throttle_lower = true;
        }
    }

    // Apply thrust scaling if any motor is saturated
    float max_thrust = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (_thrust[i] > max_thrust) {
            max_thrust = _thrust[i];
        }
    }

    // Scale down if over limit
    if (max_thrust > 1.0f) {
        float scale = 1.0f / max_thrust;
        for (int i = 0; i < 3; i++) {
            _thrust[i] *= scale;
        }
        limit.throttle_upper = true;
    }
}

// output_to_motors - sends commands to motors and servos
void AP_MotorsTri_Tilt::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // Sends minimum values out to motors
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                    rc_write(i, get_pwm_output_min());
                }
            }
            // Center servos
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            break;
        }

        case SpoolState::GROUND_IDLE: {
            // Sends output to motors when armed but not flying
            float spin_up = actuator_spin_up_to_ground_idle();
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], spin_up);
                    rc_write(i, output_to_pwm(_actuator[i]));
                }
            }
            // Center servos
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            break;
        }

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN: {
            // Set motor output based on thrust requests
            // Apply yaw differential on coaxial pairs
            
            // Calculate yaw thrust component
            float yaw_thrust = (_yaw_in + _yaw_in_ff) * _yaw_torque_factor * float(_yaw_dir.get());
            yaw_thrust = constrain_float(yaw_thrust, -0.5f, 0.5f);

            // Front-right coaxial pair (motors 0-1)
            apply_coaxial_yaw(AP_MOTORS_MOT_1, AP_MOTORS_MOT_2, _thrust[0], yaw_thrust);

            // Rear coaxial pair (motors 2-3)
            apply_coaxial_yaw(AP_MOTORS_MOT_3, AP_MOTORS_MOT_4, _thrust[1], yaw_thrust);

            // Front-left coaxial pair (motors 4-5)
            apply_coaxial_yaw(AP_MOTORS_MOT_5, AP_MOTORS_MOT_6, _thrust[2], yaw_thrust);

            // Output tilt servo angles (centi-degrees)
            // If _servo_angle_max == 0 => no software clamp (still limited by SERVOx_MIN/MAX).
            const float lim_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
            const int16_t fr_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[0]), -lim_deg, lim_deg) * 100);
            const int16_t fl_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[2]), -lim_deg, lim_deg) * 100);
            const int16_t rear_angle_cd = int16_t(constrain_float(degrees(_tilt_angle[1]), -lim_deg, lim_deg) * 100);

            // scaled output uses centi-degrees when SRV_Channels::set_angle() has been configured
            const bool fr_rev = (_tilt_servo_fr_rev.get() != 0);
            const bool rear_rev = (_tilt_servo_rear_rev.get() != 0);
            const bool fl_rev = (_tilt_servo_fl_rev.get() != 0);

            const int16_t fr_out_cd = fr_rev ? -fr_angle_cd : fr_angle_cd;
            const int16_t rear_out_cd = rear_rev ? -rear_angle_cd : rear_angle_cd;
            const int16_t fl_out_cd = fl_rev ? -fl_angle_cd : fl_angle_cd;

            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, fr_out_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, rear_out_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, fl_out_cd);
            break;
        }
    }
}

// output test sequence
void AP_MotorsTri_Tilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // Map motor sequence to actual motor number
    uint8_t motor_num;
    switch (motor_seq) {
        case 1:
            motor_num = AP_MOTORS_MOT_1;
            break;
        case 2:
            motor_num = AP_MOTORS_MOT_2;
            break;
        case 3:
            motor_num = AP_MOTORS_MOT_3;
            break;
        case 4:
            motor_num = AP_MOTORS_MOT_4;
            break;
        case 5:
            motor_num = AP_MOTORS_MOT_5;
            break;
        case 6:
            motor_num = AP_MOTORS_MOT_6;
            break;
        default:
            return;
    }

    // Output PWM to motor
    if (motor_enabled[motor_num]) {
        rc_write(motor_num, pwm);
    }
}

// Apply yaw torque via differential thrust on coaxial pairs
void AP_MotorsTri_Tilt::apply_coaxial_yaw(uint8_t motor_upper, uint8_t motor_lower, 
                                           float base_thrust, float yaw_thrust)
{
    // Upper motor is CW, lower is CCW
    // For positive yaw (CW rotation), increase CCW motor, decrease CW motor
    float thrust_upper = base_thrust - yaw_thrust;
    float thrust_lower = base_thrust + yaw_thrust;

    // Constrain to valid range [0, 1]
    thrust_upper = constrain_float(thrust_upper, 0.0f, 1.0f);
    thrust_lower = constrain_float(thrust_lower, 0.0f, 1.0f);

    // Convert thrust to actuator value
    float actuator_upper = thr_lin.thrust_to_actuator(thrust_upper);
    float actuator_lower = thr_lin.thrust_to_actuator(thrust_lower);

    // Output to motors
    rc_write(motor_upper, output_to_pwm(actuator_upper));
    rc_write(motor_lower, output_to_pwm(actuator_lower));
}

// Calculate static allocation matrix F_alloc[5][6]
// Based on MATLAB derivation from tricopter_allocation/cal_alloc_tri.m
// Maps [Fx, Fz, Mx, My, Mz] -> [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
void AP_MotorsTri_Tilt::calculate_allocation_matrix()
{
    float lx = _lfront_x;
    float ly = _lfront_y;
    float lr = _lrear;

    // Row 0: Fx (Forward force)
    // Fx = -(F1*sin(a1) + F2*sin(a2) + F3*sin(a3))
    _alloc_matrix[0][0] = -1.0f;  // F1*sin(a1)
    _alloc_matrix[0][1] = 0.0f;   // F1*cos(a1)
    _alloc_matrix[0][2] = -1.0f;  // F2*sin(a2)
    _alloc_matrix[0][3] = 0.0f;   // F2*cos(a2)
    _alloc_matrix[0][4] = -1.0f;  // F3*sin(a3)
    _alloc_matrix[0][5] = 0.0f;   // F3*cos(a3)

    // Row 1: Fz (Vertical force / Throttle)
    // Fz = -(F1*cos(a1) + F2*cos(a2) + F3*cos(a3))
    _alloc_matrix[1][0] = 0.0f;   // F1*sin(a1)
    _alloc_matrix[1][1] = -1.0f;  // F1*cos(a1)
    _alloc_matrix[1][2] = 0.0f;   // F2*sin(a2)
    _alloc_matrix[1][3] = -1.0f;  // F2*cos(a2)
    _alloc_matrix[1][4] = 0.0f;   // F3*sin(a3)
    _alloc_matrix[1][5] = -1.0f;  // F3*cos(a3)

    // Row 2: Mx (Roll moment)
    // Mx = -ly*F1*cos(a1) + ly*F3*cos(a3)
    _alloc_matrix[2][0] = 0.0f;       // F1*sin(a1)
    _alloc_matrix[2][1] = -ly;        // F1*cos(a1)
    _alloc_matrix[2][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[2][3] = 0.0f;       // F2*cos(a2)
    _alloc_matrix[2][4] = 0.0f;       // F3*sin(a3)
    _alloc_matrix[2][5] = ly;         // F3*cos(a3)

    // Row 3: My (Pitch moment)
    // My = lx*F1*cos(a1) - lr*F2*cos(a2) - lx*F3*cos(a3)
    _alloc_matrix[3][0] = 0.0f;       // F1*sin(a1)
    _alloc_matrix[3][1] = lx;         // F1*cos(a1)
    _alloc_matrix[3][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[3][3] = -lr;        // F2*cos(a2)
    _alloc_matrix[3][4] = 0.0f;       // F3*sin(a3)
    _alloc_matrix[3][5] = -lx;        // F3*cos(a3)

    // Row 4: Mz (Yaw moment)
    // Mz = ly*F1*sin(a1) - ly*F3*sin(a3)
    _alloc_matrix[4][0] = ly;         // F1*sin(a1)
    _alloc_matrix[4][1] = 0.0f;       // F1*cos(a1)
    _alloc_matrix[4][2] = 0.0f;       // F2*sin(a2)
    _alloc_matrix[4][3] = 0.0f;       // F2*cos(a2)
    _alloc_matrix[4][4] = -ly;        // F3*sin(a3)
    _alloc_matrix[4][5] = 0.0f;       // F3*cos(a3)
}

// Calculate pseudo-inverse of allocation matrix
// Based on MATLAB: simplify(pinv(F_alloc))
// Maps [Fx, Fz, Mx, My, Mz] -> [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
void AP_MotorsTri_Tilt::calculate_allocation_matrix_pinv()
{
    float lx = _lfront_x;
    float ly = _lfront_y;
    float lr = _lrear;

    // Prevent division by zero
    if (fabsf(ly) < 0.01f || fabsf(lr) < 0.01f) {
        // Set to safe defaults if parameters are invalid
        ly = 0.5f;
        lr = 1.0f;
    }

    // Row 0: F1*sin(a1)
    _alloc_matrix_pinv[0][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[0][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[0][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[0][3] = 0.0f;                    // My
    _alloc_matrix_pinv[0][4] = 1.0f/(2.0f*ly);          // Mz

    // Row 1: F1*cos(a1)
    _alloc_matrix_pinv[1][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[1][1] = -1.0f/2.0f;              // Fz
    _alloc_matrix_pinv[1][2] = (lx - lr)/(2.0f*ly*lr);  // Mx
    _alloc_matrix_pinv[1][3] = 1.0f/(2.0f*lr);          // My
    _alloc_matrix_pinv[1][4] = 0.0f;                    // Mz

    // Row 2: F2*sin(a2)
    _alloc_matrix_pinv[2][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[2][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[2][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[2][3] = 0.0f;                    // My
    _alloc_matrix_pinv[2][4] = 0.0f;                    // Mz

    // Row 3: F2*cos(a2)
    _alloc_matrix_pinv[3][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[3][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[3][2] = -lx/(ly*lr);             // Mx
    _alloc_matrix_pinv[3][3] = -1.0f/lr;                // My
    _alloc_matrix_pinv[3][4] = 0.0f;                    // Mz

    // Row 4: F3*sin(a3)
    _alloc_matrix_pinv[4][0] = -1.0f/3.0f;              // Fx
    _alloc_matrix_pinv[4][1] = 0.0f;                    // Fz
    _alloc_matrix_pinv[4][2] = 0.0f;                    // Mx
    _alloc_matrix_pinv[4][3] = 0.0f;                    // My
    _alloc_matrix_pinv[4][4] = -1.0f/(2.0f*ly);         // Mz

    // Row 5: F3*cos(a3)
    _alloc_matrix_pinv[5][0] = 0.0f;                    // Fx
    _alloc_matrix_pinv[5][1] = -1.0f/2.0f;              // Fz
    _alloc_matrix_pinv[5][2] = (lx + lr)/(2.0f*ly*lr);  // Mx
    _alloc_matrix_pinv[5][3] = 1.0f/(2.0f*lr);          // My
    _alloc_matrix_pinv[5][4] = 0.0f;                    // Mz
}

#endif  // AP_MOTORS_TRI_TILT_ENABLED
