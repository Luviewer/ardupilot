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

    uint8_t gait_rot_z[Leg_ALL];

    uint8_t leg_lift_height = 50; // leg lift height(in mm) while walking

    uint16_t COXA_LEN    = 53;  // distance (in mm) from coxa (aka hip) servo to femur servo
    uint16_t FEMUR_LEN   = 85;  // distance (in mm) from femur servo to tibia servo
    uint16_t TIBIA_LEN   = 128; // distance (in mm) from tibia servo to foot
    uint16_t FRAME_LEN   = 180; // frame length in mm
    uint16_t FRAME_WIDTH = 180; // frame width in mm
    
    Vector3f endpoint_leg[Leg_ALL];

public:
    AP_Quadruped();
    ~AP_Quadruped();

    void     gait_select(uint8_t gait_type);
    void     calc_gait_sequence(int16_t x_travel, int16_t y_travel, int16_t yaw_travel);
    void     update_leg(int16_t x_travel, int16_t y_travel, int16_t yaw_travel, uint8_t moving_leg);
    void     main_inverse_kinematics();
    Vector3f body_forward_kinematics(Vector3f xyz_leg, float Zrot_deg, Vector2f XYdist, Vector3f body_rot_xyz_deg);
    Vector3f leg_inverse_kinematics(Vector3f posxyz);
};

AP_Quadruped::~AP_Quadruped()
{
}
