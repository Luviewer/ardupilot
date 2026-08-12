/// @file AP_MotorsScorpio.h
/// @brief Motor and tilt allocation for the Scorpio tilting tricopter
#pragma once

#include "AP_MotorsMulticopter.h"

class AP_MotorsScorpio : public AP_MotorsMulticopter {
public:
    explicit AP_MotorsScorpio(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_update_rate(uint16_t speed_hz) override;
    void output_to_motors() override;
    uint32_t get_motor_mask() override;
    bool arming_checks(size_t buflen, char *buffer) const override;
    float get_roll_factor(uint8_t i) override;

    static const AP_Param::GroupInfo var_info[];

protected:
    void output_armed_stabilizing() override;
    void thrust_compensation() override;
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    const char *_get_frame_string() const override { return "SCORPIO"; }
    const char *get_type_string() const override { return "tilt-tri"; }

private:
    enum Actuator : uint8_t {
        FRONT_RIGHT = AP_MOTORS_MOT_1,
        REAR = AP_MOTORS_MOT_2,
        FRONT_LEFT = AP_MOTORS_MOT_3,
        ACTUATOR_COUNT = 3,
    };

    bool calculate_matrices(Matrix3f &vertical_inverse, Matrix3f &horizontal_inverse,
                            Vector3f &tilt_direction_x, Vector3f &tilt_direction_y) const;
    bool horizontal_mix_is_feasible(float scale, const Vector3f &vertical,
                                    const Vector3f &vertical_correction,
                                    const Vector3f &horizontal) const;
    void get_rotor_positions(Vector3f &pos_x, Vector3f &pos_y) const;
    void setup_tilt_outputs();

    // Installation angles match the yaw angles used by the Gazebo RSDF.
    AP_Float _front_axis_angle_deg;
    AP_Float _rear_axis_angle_deg;

    AP_Float _tilt_max_deg;
    AP_Float _xy_gain;

    // Rotor positions relative to the body origin in ArduPilot FRD axes.
    AP_Float _front_right_x;
    AP_Float _front_right_y;
    AP_Float _front_left_x;
    AP_Float _front_left_y;
    AP_Float _rear_x;
    AP_Float _rear_y;

    float _motor_thrust[ACTUATOR_COUNT] {};
    float _tilt_angle_rad[ACTUATOR_COUNT] {};
};
