/// @file AP_MotorsScorpio.h
/// @brief 天蝎座三倾转飞行器的电机与倾转舵机分配器
#pragma once

#include "AP_MotorsMulticopter.h"

class AP_MotorsScorpio : public AP_MotorsMulticopter
{
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

    const char *_get_frame_string() const override
    {
        return "SCORPIO";
    }
    const char *get_type_string() const override
    {
        return "tilt-tri";
    }

private:
    enum Actuator : uint8_t {
        FRONT_RIGHT = AP_MOTORS_MOT_1,
        REAR = AP_MOTORS_MOT_2,
        FRONT_LEFT = AP_MOTORS_MOT_3,
        ACTUATOR_COUNT = 3,
    };

    bool setup_allocation_factors();
    void mix_y3_vertical(float throttle, float throttle_avg_max, float roll, float pitch,
                         Vector3f &vertical, float &throttle_out);
    void mix_tilt_horizontal(float throttle, float yaw, const Vector3f &vertical,
                             Vector3f &horizontal);
    void combine_actuator_vectors(const Vector3f &vertical, const Vector3f &horizontal);
    bool horizontal_mix_is_feasible(const Vector3f &vertical, const Vector3f &horizontal) const;
    float horizontal_mix_scale(const Vector3f &vertical, const Vector3f &base,
                               const Vector3f &addition) const;
    void get_rotor_positions(Vector3f &pos_x, Vector3f &pos_y) const;
    void setup_tilt_outputs();

    // 倾转轴安装角，与 Gazebo RSDF 中各倾转关节的偏航安装角一致。
    AP_Float _front_axis_angle_deg;
    AP_Float _rear_axis_angle_deg;

    AP_Float _tilt_max_deg;
    AP_Float _xy_gain;

    // 各旋翼中心相对机体原点的位置，使用 ArduPilot FRD（前、右、下）坐标系。
    AP_Float _front_right_x;
    AP_Float _front_right_y;
    AP_Float _front_left_x;
    AP_Float _front_left_y;
    AP_Float _rear_x;
    AP_Float _rear_y;

    // 初始化时根据真实几何生成各控制轴的混控系数。
    // 飞行循环中只按 ArduPilot 标准方式进行系数叠加，不再实时求逆矩阵。
    Vector3f _vertical_throttle_factor;
    Vector3f _vertical_roll_factor;
    Vector3f _vertical_pitch_factor;
    Vector3f _horizontal_forward_factor;
    Vector3f _horizontal_lateral_factor;
    Vector3f _horizontal_yaw_factor;
    bool _allocation_valid { false };

    float _motor_thrust[ACTUATOR_COUNT] {};
    float _tilt_angle_rad[ACTUATOR_COUNT] {};
};
