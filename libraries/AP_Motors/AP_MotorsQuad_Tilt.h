/// @file	AP_MotorsQuad_Tilt.h
/// @brief	Motor control class for Coaxial X-frame Octocopters with Tiltable Rotors
#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_QUAD_TILT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMatrix.h"

// 是否启用限制警告提示（设置为1启用，0禁用）
#define AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS 0
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// Default servo outputs for tilt servos (can be overridden via SERVOx_FUNCTION)
// Note: CH_7 means SERVO7 (0-based channel index 6)
#define AP_MOTORS_QUAD_TILT_SERVO_RF     CH_7    // Front-right tilt servo (k_tiltMotorRight)
#define AP_MOTORS_QUAD_TILT_SERVO_RR     CH_8    // Rear-right tilt servo (k_tiltMotorRear)
#define AP_MOTORS_QUAD_TILT_SERVO_LR     CH_10   // Rear-left tilt servo (k_tiltMotorRearLeft)
#define AP_MOTORS_QUAD_TILT_SERVO_LF     CH_9    // Front-left tilt servo (k_tiltMotorLeft)

// Default angle limits
#define AP_MOTORS_QUAD_TILT_ANGLE_MIN    5       // minimum tilt angle in degrees
#define AP_MOTORS_QUAD_TILT_ANGLE_MAX    270     // maximum tilt angle in degrees (physical limit)

/// @class      AP_MotorsQuad_Tilt
/// @brief      Coaxial X-frame Octocopter with tiltable rotors
/// @details    4 coaxial pairs (RF, RR, LR, LF), upper CCW / lower CW.
///             Yaw via differential coaxial thrust, forward/lateral via tilt.
class AP_MotorsQuad_Tilt : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsQuad_Tilt(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
#if AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS
        , _limit_warn_state{}
#endif
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    
    enum MotorIndex {
        RF_UP   = 0, // Front-right upper (CCW)
        RF_DOWN = 1, // Front-right lower (CW)
        RR_UP   = 2, // Rear-right upper (CCW)
        RR_DOWN = 3, // Rear-right lower (CW)
        LR_UP   = 4, // Rear-left upper (CCW)
        LR_DOWN = 5, // Rear-left lower (CW)
        LF_UP   = 6, // Front-left upper (CCW)
        LF_DOWN = 7, // Front-left lower (CW)
        MotorIndex_COUNT,
    };

    enum TiltIndex {
        RF = 0, // Front-right tilt
        RR = 1, // Rear-right tilt
        LR = 2, // Rear-left tilt
        LF = 3, // Front-left tilt
        TiltIndex_COUNT,
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

    virtual int8_t get_tilt_enable() override { return _tilt_enable.get(); }
    virtual int8_t get_lateral_enable() override { return _lateral_enable.get(); }
    virtual float get_tilt_max_deg() override { return _tilt_pitch_off_max_deg.get(); }
    virtual float get_bicopter_pitch_P_factor() override;
    float get_est_body_x_thrust_ratio() const override { return _est_body_x_thrust_ratio; }
    void servoOutput(enum TiltIndex servo_index, float svo_out_cd);

protected:
    // setup motors - configures the static allocation matrix
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // output - sends commands to the motors
    // Implements: 5DOF input -> static matrix -> 6 intermediate variables -> thrust + tilt angles
    void output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "QUAD_TILT"; }
    const char* get_type_string() const override { return "Coaxial-X8"; }

    // output_test_seq - spin a motor at the pwm value specified
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

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

    AP_Int8  _tilt_servo_rf_rev;   // front-left tilt servo reverse (0 normal, 1 reversed)
    AP_Int8  _tilt_servo_rr_rev; // rear-right tilt servo reverse (0 normal, 1 reversed)
    AP_Int8  _tilt_servo_lr_rev;   // rear-left tilt servo reverse (0 normal, 1 reversed)
    AP_Int8  _tilt_servo_lf_rev;   // front-left tilt servo reverse (0 normal, 1 reversed)

    AP_Float _tilt_pitch_off_max_deg; // max pitch offset (deg) commanded by RC7
    AP_Int8 _tilt_enable;
    AP_Int8 _lateral_enable;
    AP_Float _forward_factor;

    AP_Float _svo_rf_offset, _svo_rr_offset, _svo_lr_offset, _svo_lf_offset;

    AP_Float _anti_yaw_factor;
    AP_Float _bicopter_pitch_P_factor;
    AP_Float _lateral_factor;
    
    // 三旋翼推力分配
    float _thrust_tricopter[TiltIndex_COUNT];

    // 双旋翼推力分配
    float _thrust_bicopter[TiltIndex_COUNT];

    // 双旋翼倾转控制
    float _tilt_bicopter[TiltIndex_COUNT];

    float tilt_mixed[TiltIndex_COUNT];

    // Calculated outputs
    float _thrust[MotorIndex_COUNT];           // Thrust for each rotor pair [F1, F2, F3]
    float _tilt_angle_rad[TiltIndex_COUNT];       // Tilt angle for each rotor pair [a1, a2, a3] in radians
    
    // 临时存储混合后的 RPY 输出（不含 throttle）
    float _rpy_out[MotorIndex_COUNT];

    float _est_body_x_thrust_ratio;

#if AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS
    // GCS 提示状态记录（记录上次的限制状态和时间戳）
    struct {
        uint32_t throttle_lower_ms;
        uint32_t throttle_upper_ms;
        uint32_t yaw_ms;
        uint32_t roll_ms;
        uint32_t pitch_ms;
        uint32_t rpy_all_ms;  // RPY同时受限
        bool throttle_lower_last;
        bool throttle_upper_last;
        bool yaw_last;
        bool roll_last;
        bool pitch_last;
        bool rpy_all_last;
    } _limit_warn_state;
#endif

    bool _servos_assigned;
};

#endif  // AP_MOTORS_QUAD_TILT_ENABLED
