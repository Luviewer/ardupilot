#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

enum {
    Leg_RF = 0,
    Leg_RB,
    Leg_LB,
    Leg_LF,

    Leg_ALL,
};

class AP_Quadruped {

protected:
    bool move_requested;

    uint8_t gait_type;
    uint8_t gait_step;
    uint8_t gait_step_total;
    uint8_t gait_step_leg_start[Leg_ALL];
    uint8_t gait_lifted_steps;
    uint8_t gait_down_steps;
    uint8_t gait_lift_divisor;
    uint8_t gait_half_lift_height;
    uint8_t gait_travel_divisor;

    Vector3f gait_pos_xyz[Leg_ALL];

    float gait_rot_z[Leg_ALL];

    float leg_lift_height; // leg lift height(in mm) while walking

    float COXA_LEN;    // distance (in mm) from coxa (aka hip) servo to femur servo
    float FEMUR_LEN;   // distance (in mm) from femur servo to tibia servo
    float TIBIA_LEN;   // distance (in mm) from tibia servo to foot
    float FRAME_LEN;   // frame length in mm
    float FRAME_WIDTH; // frame width in mm

    Vector3f endpoint_leg_pos[Leg_ALL];
    Vector3f endpoint_leg_frame[Leg_ALL];

    Vector3f endpoint_leg_angle[Leg_ALL];
    Vector3f endpoint_leg_angle_last[Leg_ALL];

    Vector3f body_rot_xyz_deg;

    float throttle_travel;
    float z_travel;

    float yaw_travel;

public:
    AP_Quadruped();
    ~AP_Quadruped() { };

    void init();

    void     gait_select();
    void     calc_gait_sequence(void);
    void     update_leg(uint8_t moving_leg);
    void     main_inverse_kinematics();
    Vector3f body_forward_kinematics(uint8_t leg_index);
    Vector3f leg_inverse_kinematics(Vector3f posxyz);

    void set_throttle_travel(int16_t _throttle) { throttle_travel = _throttle; }
    void set_yaw_travel(int16_t _yaw_travel) { yaw_travel = _yaw_travel; }
    // Vector3f get_endpoint_leg(uint8_t leg) { return endpoint_leg[leg]; }

    void output_leg_angle();
};
