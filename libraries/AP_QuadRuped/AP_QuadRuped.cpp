#include "AP_QuadRuped.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

#define COXA_LEN_DEFAULT        47.1f
#define FEMUR_LEN_DEFAULT       133.0f
#define TIBIA_LEN_DEFAULT       144.1f
#define FRAME_LEN_DEFAULT       185.0f
#define FRAME_WIDTH_DEFAULT     185.0f
#define LIFT_HEIGHT_DEFAULT     50.0f
#define SPEED_HZ_DEFAULT        25.0f
#define MAX_THROTTLE_DEFAULT    200.0f
#define GAIT_STEP_TOTAL_DEFAULT 10.0f

const AP_Param::GroupInfo AP_QuadRuped::var_info[] = {
    AP_GROUPINFO("_COXA", 1, AP_QuadRuped, COXA_LEN, COXA_LEN_DEFAULT),
    AP_GROUPINFO("_FEMUR", 2, AP_QuadRuped, FEMUR_LEN, FEMUR_LEN_DEFAULT),
    AP_GROUPINFO("_TIBIA", 3, AP_QuadRuped, TIBIA_LEN, TIBIA_LEN_DEFAULT),

    AP_GROUPINFO("_FX", 4, AP_QuadRuped, FRAME_LEN, FRAME_LEN_DEFAULT),
    AP_GROUPINFO("_FY", 5, AP_QuadRuped, FRAME_WIDTH, FRAME_WIDTH_DEFAULT),

    AP_GROUPINFO("_LIFT", 6, AP_QuadRuped, leg_lift_height, LIFT_HEIGHT_DEFAULT),
    AP_GROUPINFO("_Hz", 7, AP_QuadRuped, gait_hz, SPEED_HZ_DEFAULT),

    AP_GROUPINFO("_THR", 8, AP_QuadRuped, throttle_max, MAX_THROTTLE_DEFAULT),
    AP_GROUPINFO("_STEP", 9, AP_QuadRuped, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    AP_GROUPINFO("_COXA1", 10, AP_QuadRuped, leg_coxa_direction[Leg_RF], 1),
    AP_GROUPINFO("_COXA2", 12, AP_QuadRuped, leg_coxa_direction[Leg_RB], 1),
    AP_GROUPINFO("_COXA3", 13, AP_QuadRuped, leg_coxa_direction[Leg_LB], 1),
    AP_GROUPINFO("_COXA4", 14, AP_QuadRuped, leg_coxa_direction[Leg_LF], 1),

    AP_GROUPINFO("_FEMU1", 15, AP_QuadRuped, leg_femur_direction[Leg_RF], 1),
    AP_GROUPINFO("_FEMU2", 16, AP_QuadRuped, leg_femur_direction[Leg_RB], 1),
    AP_GROUPINFO("_FEMU3", 17, AP_QuadRuped, leg_femur_direction[Leg_LB], 1),
    AP_GROUPINFO("_FEMU4", 18, AP_QuadRuped, leg_femur_direction[Leg_LF], 1),

    AP_GROUPINFO("_TIBI1", 19, AP_QuadRuped, leg_tibia_direction[Leg_RF], 1),
    AP_GROUPINFO("_TIBI2", 20, AP_QuadRuped, leg_tibia_direction[Leg_RB], 1),
    AP_GROUPINFO("_TIBI3", 21, AP_QuadRuped, leg_tibia_direction[Leg_LB], 1),
    AP_GROUPINFO("_TIBI4", 22, AP_QuadRuped, leg_tibia_direction[Leg_LF], 1),

    AP_GROUPEND
};

AP_QuadRuped::AP_QuadRuped(AP_AHRS_View*& ahrs, AP_MotorsMulticopter*& motors)
    : _ahrs(ahrs)
    , _motors(motors)
{
    gait_type      = 0;
    move_requested = false;

    AP_Param::setup_object_defaults(this, var_info);

    // leg_lift_height = 80; // leg lift height(in mm) while walking

    // COXA_LEN  = 47.1; // distance (in mm) from coxa (aka hip) servo to femur servo
    // FEMUR_LEN = 133;  // distance (in mm) from femur servo to tibia servo
    // TIBIA_LEN = 144;  // distance (in mm) from tibia servo to foot

    // FRAME_LEN   = 185; // frame length in mm
    // FRAME_WIDTH = 185; // frame width in mm
}

#define START_COXA_ANGLE 45

void AP_QuadRuped::init(void)
{
    // starting positions of the legs
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_pos[leg_index] = Vector3f(sinf(radians(START_COXA_ANGLE - leg_index * 90)) * (COXA_LEN + FEMUR_LEN),
                                               cosf(radians(START_COXA_ANGLE - leg_index * 90)) * (COXA_LEN + FEMUR_LEN),
                                               TIBIA_LEN);
    }

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_frame[leg_index] = Vector3f(sinf(radians(START_COXA_ANGLE - leg_index * 90)) * FRAME_LEN,
                                                 cosf(radians(START_COXA_ANGLE - leg_index * 90)) * FRAME_WIDTH,
                                                 0);
    }

    gait_select();

    // for (uint8_t i = 0; i < 12; i++) {
    //     // SRV_Channels::set_angle((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + i), SERVO_OUTPUT_RANGE);
    //     // SRV_Channels::set_rc_frequency((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + i), 50);
    // }
}

void AP_QuadRuped::gait_select(void)
{
    // gait_step_total       = 6;
    // gait_lifted_steps     = 2;
    // gait_down_steps       = 1;
    // gait_lift_divisor     = 2;
    // gait_half_lift_height = 1;
    // gait_travel_divisor   = 4;

    // gait_step_leg_start[Leg_RF] = 1;
    // gait_step_leg_start[Leg_RB] = 4;
    // gait_step_leg_start[Leg_LB] = 1;
    // gait_step_leg_start[Leg_LF] = 4;
}

void AP_QuadRuped::calc_gait_sequence(void)
{
    const float travel_dz = 5;

    if ((fabsf(throttle_travel) > travel_dz) || (fabsf(yaw_travel) > travel_dz / 2))
        move_requested = true;
    else
        move_requested = false;

    if (move_requested == true) {
        update_leg();
    } else {
        reset_leg();
    }
}

Vector3f AP_QuadRuped::trajectory_generation()
{
    float delta = M_2PI * (gait_step_now - gait_step_total) / gait_step_total;

    Vector2f leg_xy_target = Vector2f(throttle_travel, 0) * (delta - sinf(delta)) / M_2PI;

    float leg_z_target = -leg_lift_height * (1.0f - cosf(delta)) / 2.0f;

    return Vector3f(leg_xy_target, leg_z_target);
}

void AP_QuadRuped::update_leg()
{
    gait_step_now++;
    if (gait_step_now > (gait_step_total * 2)) gait_step_now = 0;

    float dir = 1;
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        if (moving_leg == Leg_RF || moving_leg == Leg_LB) {
            dir = 1;
        } else {
            dir = -1;
        }

        if (gait_step_now < gait_step_total) {
            gait_pos_xyz[moving_leg] = trajectory_generation() * dir;
        } else {
            gait_pos_xyz[moving_leg] = trajectory_generation() * -dir;
        }
    }
}

// void AP_QuadRuped::update_leg(uint8_t moving_leg)
// {
//     int8_t leg_step = gait_step - gait_step_leg_start[moving_leg];

//     switch (leg_step) {
//         case 0:
//             gait_pos_xyz[moving_leg] = { 0, 0, -(float)leg_lift_height };
//             gait_rot_z[moving_leg]   = 0;
//             break;

//         case 1:
//             gait_pos_xyz[moving_leg] = Vector3f(throttle_travel / gait_lift_divisor,
//                                                 0,
//                                                 -3.0f * leg_lift_height / (3.0f + gait_half_lift_height));

//             gait_rot_z[moving_leg] = yaw_travel / gait_lift_divisor;
//             break;

//         default:
//             gait_pos_xyz[moving_leg] = Vector3f(gait_pos_xyz[moving_leg].x - (throttle_travel / gait_travel_divisor),
//                                                 gait_pos_xyz[moving_leg].y,
//                                                 0.0f);

//             gait_rot_z[moving_leg] = gait_rot_z[moving_leg] - (yaw_travel / gait_travel_divisor);
//             break;
//     }
// }

Vector3f AP_QuadRuped::body_forward_kinematics(uint8_t leg_index)
{
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    totaldist_xyz.z += z_travel;

    Quaternion quat = { 1, 0, 0, 0 };

    body_rot_xyz_deg.x = -radians(roll_travel);
    body_rot_xyz_deg.y = -radians(pitch_travel);
    body_rot_xyz_deg.z = radians(gait_rot_z[leg_index]);

    quat.from_euler(body_rot_xyz_deg);

    Vector3f totaldist_xyz_rot = quat * totaldist_xyz;

    return (totaldist_xyz_rot - endpoint_leg_frame[leg_index]);
}

Vector3f AP_QuadRuped::leg_inverse_kinematics(Vector3f posxyz)
{
    Vector3f leg_deg = { 0, 0, 0 };

    leg_deg.x = -degrees(atan2f(posxyz.x, posxyz.y));

    float trueX = sqrtf(posxyz.x * posxyz.x + posxyz.y * posxyz.y) - COXA_LEN;
    float im    = sqrtf(trueX * trueX + posxyz.z * posxyz.z);
    float q1    = atan2f(trueX, posxyz.z);
    float d1    = FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN + im * im;
    float d2    = 2 * FEMUR_LEN * im;
    float q2    = acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f));
    leg_deg.y   = -(degrees(q1 + q2) - 90);

    d1        = FEMUR_LEN * FEMUR_LEN - im * im + TIBIA_LEN * TIBIA_LEN;
    d2        = 2 * TIBIA_LEN * FEMUR_LEN;
    leg_deg.z = -(degrees(acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f))) - 90);

    return leg_deg;
}

void AP_QuadRuped::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 };

    const Vector3f endpoint_leg_angle_offset[LEG_ALL] = {
        { 45, 0, 0 },
        { -45, 0, 0 },
        { -135, 0, 0 },
        { -225, 0, 0 }
    };

    contoller();

    // const float endpoint_leg_angle_dir[LEG_ALL] = { 1, 1, 1, 1 };

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        ansxyz = body_forward_kinematics(leg_index);

        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];

        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);

        // endpoint_leg_angle[leg_index].x *= endpoint_leg_angle_dir[leg_index];
    }

    // if (servo_estimate()) {
    // start_time = AP_HAL::millis();

    calc_gait_sequence();

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
    // }
}

bool AP_QuadRuped::servo_estimate(void)
{
    uint32_t target_time = AP_HAL::millis();

    if ((target_time - start_time) >= (1000.0f / gait_hz)) {
        return true;
    }
    return false;
}

void AP_QuadRuped::left_sleep_leg(void)
{
    uint16_t pwm_coxa = 1500, pwm_femur = 1500, pwm_tibia = 1500;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {

        pwm_coxa  = leg_coxa_direction[leg_index] * 45 * 500 / 120 + 1500;
        pwm_femur = leg_femur_direction[leg_index] * -45 * 500 / 120 + 1500;
        pwm_tibia = leg_tibia_direction[leg_index] * 45 * 500 / 120 + 1500;

        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + leg_index * 3), pwm_coxa);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_2 + leg_index * 3), pwm_femur);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_3 + leg_index * 3), pwm_tibia);
    }
}

void AP_QuadRuped::reset_leg(void)
{
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 };
        gait_rot_z[moving_leg]   = 0;
    }
}

void AP_QuadRuped::output_leg_angle(void)
{
    uint16_t pwm_coxa = 1500, pwm_femur = 1500, pwm_tibia = 1500;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        pwm_coxa  = leg_coxa_direction[leg_index] * endpoint_leg_angle[leg_index].x * 500 / 120 + 1500;
        pwm_femur = leg_femur_direction[leg_index] * endpoint_leg_angle[leg_index].y * 500 / 120 + 1500;
        pwm_tibia = leg_tibia_direction[leg_index] * endpoint_leg_angle[leg_index].z * 500 / 120 + 1500;

        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_1 + leg_index * 3), pwm_coxa);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_2 + leg_index * 3), pwm_femur);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_3 + leg_index * 3), pwm_tibia);
    }
}

void AP_QuadRuped::contoller()
{
    float temp_rc;

    temp_rc = constrain_value((float)rc().RC_Channels::get_yaw_channel().get_radio_in(), (float)1000, (float)2000);
    temp_rc = (temp_rc - 1500) / 500.0f * 35.0f;

    // float yaw_out = yaw_pid.update_all(temp_rc, _ahrs->get_gyro().z, 1.0 / 50.0f);

    yaw_travel = temp_rc;

    temp_rc         = constrain_value((float)rc().RC_Channels::get_throttle_channel().get_radio_in(), (float)1000, (float)2000);
    throttle_travel = (temp_rc - 1500) / 500.0f * throttle_max;
    // throttle_travel = 20;
    // temp_rc     = constrain_value((float)rc().RC_Channels::get_roll_channel().get_radio_in(), (float)1000, (float)2000);
    // roll_travel = (temp_rc - 1500) / 500.0f * 15.0f;

    // temp_rc      = constrain_value((float)rc().RC_Channels::get_pitch_channel().get_radio_in(), (float)1000, (float)2000);
    // pitch_travel = (temp_rc - 1500) / 500.0f * 5.0f;

    temp_rc  = constrain_value((float)rc().RC_Channels::get_pitch_channel().get_radio_in(), (float)1000, (float)2000);
    z_travel = (temp_rc - 1500) / 500.0f * 120.0f;
}
