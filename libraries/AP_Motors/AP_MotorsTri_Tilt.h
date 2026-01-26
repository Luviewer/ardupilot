/// @file	AP_MotorsTri_Tilt.h
/// @brief	Motor control class for Coaxial Tricopters with Tiltable Rotors
#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMatrix.h"

// Default servo outputs for tilt servos (can be overridden via SERVOx_FUNCTION)
// Note: CH_7 means SERVO7 (0-based channel index 6)
#define AP_MOTORS_TRI_TILT_SERVO_FR     CH_7    // Front-right tilt servo (k_tiltMotorRight)
#define AP_MOTORS_TRI_TILT_SERVO_REAR   CH_8    // Rear tilt servo (k_tiltMotorRear)
#define AP_MOTORS_TRI_TILT_SERVO_FL     CH_9    // Front-left tilt servo (k_tiltMotorLeft)

// Default angle limits
#define AP_MOTORS_TRI_TILT_ANGLE_MIN    5       // minimum tilt angle in degrees
#define AP_MOTORS_TRI_TILT_ANGLE_MAX    135     // maximum tilt angle in degrees (physical limit)

/// @class      AP_MotorsTri_Tilt
/// @brief      Coaxial Y6B Tricopter with tiltable rotors (5DOF control)
/// @details    Uses static allocation matrix to map 5DOF commands to 6 intermediate
///             variables [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
///             Then solves for thrust and tilt angles
class AP_MotorsTri_Tilt : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsTri_Tilt(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

    // output_to_motors - sends commands to motors and servos
    void output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    uint32_t get_motor_mask() override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // Max pitch attitude offset (deg) applied via RC7
    float tilt_pitch_offset_max_deg() const { return _tilt_pitch_off_max_deg; }

    void set_roll_pitch(float roll_deg, float pitch_deg) override;

protected:
    // setup motors - configures the static allocation matrix
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // output - sends commands to the motors
    // Implements: 5DOF input -> static matrix -> 6 intermediate variables -> thrust + tilt angles
    void output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "TRI_TILT"; }
    const char* get_type_string() const override { return "Coaxial-Y6B"; }

    // output_test_seq - spin a motor at the pwm value specified
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // Calculate the static allocation matrix F_alloc based on geometry
    void calculate_allocation_matrix();

    // Calculate the pseudo-inverse of the allocation matrix
    void calculate_allocation_matrix_pinv();

    // Current offset angles, radians
    float _roll_offset;
    float _pitch_offset;
    
private:
    // Geometric parameters (normalized arm lengths)
    AP_Float _lfront_x;         // Front arm X distance (normalized)
    AP_Float _lfront_y;         // Front arm Y distance (normalized)
    AP_Float _lrear;            // Rear arm length (normalized)

    // Servo parameters
    AP_Float _servo_angle_max;  // Maximum tilt angle in degrees

    // Coaxial yaw parameters
    AP_Float _yaw_torque_factor; // Factor for yaw torque via differential thrust
    AP_Int8  _yaw_dir;          // +1 normal yaw direction, -1 reversed
    AP_Int8  _tilt_servo_fr_rev;   // front-right tilt servo reverse (0 normal, 1 reversed)
    AP_Int8  _tilt_servo_rear_rev; // rear tilt servo reverse (0 normal, 1 reversed)
    AP_Int8  _tilt_servo_fl_rev;   // front-left tilt servo reverse (0 normal, 1 reversed)
    AP_Float _tilt_pitch_off_max_deg; // max pitch offset (deg) commanded by RC7

    // Static allocation matrix F_alloc[5][6]
    // Maps [Fx, Fz, Mx, My, Mz] -> [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
    float _alloc_matrix[5][6];

    // Pseudo-inverse of allocation matrix [6][5]
    // Maps [Fx, Fz, Mx, My, Mz] -> [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
    float _alloc_matrix_pinv[6][5];

    // Calculated outputs
    float _thrust[3];           // Thrust for each rotor pair [F1, F2, F3]
    float _tilt_angle_rad[3];       // Tilt angle for each rotor pair [a1, a2, a3] in radians

    // Intermediate variables from allocation matrix
    float _intermediate[6];     // [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]

    // 准备 5DOF 控制输入
    float desired[5];
    float f_sin[3], f_cos[3];
    
    // Forward thrust input (for 5DOF control)
    float _forward_thrust;

    // Check if servos are assigned
    bool _servos_assigned;
};

#endif  // AP_MOTORS_TRI_TILT_ENABLED
