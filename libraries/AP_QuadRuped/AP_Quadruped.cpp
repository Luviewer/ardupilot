#include "AP_Quadruped.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

AP_Quadruped::AP_Quadruped()
{
    gait_type      = 0;
    move_requested = false;

    leg_lift_height = 50; // leg lift height(in mm) while walking

    COXA_LEN    = 48;  // distance (in mm) from coxa (aka hip) servo to femur servo
    FEMUR_LEN   = 83;  // distance (in mm) from femur servo to tibia servo
    TIBIA_LEN   = 135; // distance (in mm) from tibia servo to foot
    FRAME_LEN   = 180; // frame length in mm
    FRAME_WIDTH = 180; // frame width in mm
}

#define START_COXA_ANGLE 45

void AP_Quadruped::init(void)
{
    // starting positions of the legs
    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        endpoint_leg_pos[leg_index] = Vector3f(cosf(radians(START_COXA_ANGLE - leg_index * 90)) * (COXA_LEN + FEMUR_LEN),
                                               sinf(radians(START_COXA_ANGLE - leg_index * 90)) * (COXA_LEN + FEMUR_LEN),
                                               TIBIA_LEN);
    }

    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        endpoint_leg_frame[leg_index] = Vector3f(cosf(radians(START_COXA_ANGLE - leg_index * 90)) * FRAME_LEN,
                                                 sinf(radians(START_COXA_ANGLE - leg_index * 90)) * FRAME_WIDTH,
                                                 0);
    }

    gait_select();

    // for (uint8_t i = 0; i < 12; i++) {
    //     // SRV_Channels::set_angle((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + i), SERVO_OUTPUT_RANGE);
    //     // SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + i), 50);
    // }
}

void AP_Quadruped::gait_select(void)
{
    if (gait_type == 0) {
        gait_step_total       = 6;
        gait_lifted_steps     = 2;
        gait_down_steps       = 1;
        gait_lift_divisor     = 2;
        gait_half_lift_height = 1;
        gait_travel_divisor   = 4;

        gait_step_leg_start[Leg_RF] = 1;
        gait_step_leg_start[Leg_RB] = 4;
        gait_step_leg_start[Leg_LB] = 1;
        gait_step_leg_start[Leg_LF] = 4;

    } else if (gait_type == 1) {
    }
}

void AP_Quadruped::calc_gait_sequence(void)
{
    const float travel_dz = 5;

    if ((fabsf(throttle_travel) > travel_dz) || (fabsf(yaw_travel) > travel_dz / 2))
        move_requested = true;
    else
        move_requested = false;

    if (move_requested == true) {
        for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
            update_leg(leg_index);
        }
        gait_step += 1;

        if (gait_step > gait_step_total) {
            gait_step = 1;
        }

    } else {
        for (uint8_t moving_leg = 0; moving_leg < Leg_ALL; moving_leg++) {
            gait_pos_xyz[moving_leg] = { 0, 0, 0 };
            gait_rot_z[moving_leg]   = 0;
        }
    }
}

void AP_Quadruped::update_leg(uint8_t moving_leg)
{
    int8_t leg_step = gait_step - gait_step_leg_start[moving_leg];

    switch (leg_step) {
        case 0:
            gait_pos_xyz[moving_leg] = { 0, 0, -(float)leg_lift_height };
            gait_rot_z[moving_leg]   = 0;
            break;

        case 1:
            gait_pos_xyz[moving_leg] = Vector3f(throttle_travel / gait_lift_divisor,
                                                0,
                                                -3.0f * leg_lift_height / (3.0f + gait_half_lift_height));

            gait_rot_z[moving_leg] = yaw_travel / gait_lift_divisor;
            break;

        default:
            gait_pos_xyz[moving_leg] = Vector3f(gait_pos_xyz[moving_leg].x - (throttle_travel / gait_travel_divisor),
                                                gait_pos_xyz[moving_leg].y,
                                                0.0f);

            gait_rot_z[moving_leg] = gait_rot_z[moving_leg] - (yaw_travel / gait_travel_divisor);
            break;
    }
}

Vector3f AP_Quadruped::body_forward_kinematics(uint8_t leg_index)
{
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    totaldist_xyz.z += z_travel;

    Quaternion quat = { 1, 0, 0, 0 };

    body_rot_xyz_deg.x = 0;
    body_rot_xyz_deg.y = 0;
    body_rot_xyz_deg.z = radians(gait_rot_z[leg_index]);

    quat.from_euler(-body_rot_xyz_deg);

    Vector3f totaldist_xyz_rot = quat * totaldist_xyz;

    return (totaldist_xyz_rot - endpoint_leg_frame[leg_index]);
}

Vector3f AP_Quadruped::leg_inverse_kinematics(Vector3f posxyz)
{
    Vector3f leg_deg = { 0, 0, 0 };

    leg_deg.x = degrees(atan2f(posxyz.y, posxyz.x));

    float trueX = sqrtf(posxyz.x * posxyz.x + posxyz.y * posxyz.y) - COXA_LEN;
    float im    = sqrtf(trueX * trueX + posxyz.z * posxyz.z);
    float q1    = atan2f(trueX, posxyz.z);
    float d1    = FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN + im * im;
    float d2    = 2 * FEMUR_LEN * im;
    float q2    = acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f));
    leg_deg.y   = -(degrees(q1 + q2) - 90);

    d1        = FEMUR_LEN * FEMUR_LEN - im * im + TIBIA_LEN * TIBIA_LEN;
    d2        = 2 * TIBIA_LEN * FEMUR_LEN;
    leg_deg.z = -(degrees(acosf(d1 / d2)) - 90);

    return leg_deg;
}

void AP_Quadruped::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 };

    throttle_travel = (rc().RC_Channels::get_throttle_channel().get_radio_in() - 1500) / 500.0f * 100;
    yaw_travel      = (rc().RC_Channels::get_yaw_channel().get_radio_in() - 1500) / 500.0f * 30;
    z_travel        = (rc().RC_Channels::get_pitch_channel().get_radio_in() - 1500) / 500.0f * 50;

    const Vector3f endpoint_leg_angle_offset[Leg_ALL] = {
        { -45, 0, 0 },
        { 45, 0, 0 },
        { 135, 0, 0 },
        { 225, 0, 0 }
    };

    const float endpoint_leg_angle_dir[Leg_ALL] = { 1, 1, 1, 1 };

    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        ansxyz = body_forward_kinematics(leg_index);

        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];

        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);

        endpoint_leg_angle[leg_index].x *= endpoint_leg_angle_dir[leg_index];
    }

    calc_gait_sequence();

    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

void AP_Quadruped::output_leg_angle(void)
{
    uint16_t pwm_coxa = 1500, pwm_femur = 1500, pwm_tibia = 1500;

    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        pwm_coxa  = endpoint_leg_angle[leg_index].x * 1000 / 90 + 1500;
        pwm_femur = endpoint_leg_angle[leg_index].y * 1000 / 90 + 1500;
        pwm_tibia = endpoint_leg_angle[leg_index].z * 1000 / 90 + 1500;

        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + leg_index * 3), pwm_coxa);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_2 + leg_index * 3), pwm_femur);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_3 + leg_index * 3), pwm_tibia);
    }
}
