#include "AP_Quadruped.h"

AP_Quadruped::AP_Quadruped()
{
    // starting positions of the legs
    endpoint_leg[Leg_RF] = Vector3f(cosf(radians(45)) * (COXA_LEN + FEMUR_LEN), sinf(radians(45)) * (COXA_LEN + FEMUR_LEN), TIBIA_LEN);
    endpoint_leg[Leg_RB] = Vector3f(cosf(radians(45)) * (COXA_LEN + FEMUR_LEN), sinf(radians(-45)) * (COXA_LEN + FEMUR_LEN), TIBIA_LEN);
    endpoint_leg[Leg_LB] = Vector3f(-cosf(radians(45)) * (COXA_LEN + FEMUR_LEN), sinf(radians(-45)) * (COXA_LEN + FEMUR_LEN), TIBIA_LEN);
    endpoint_leg[Leg_LF] = Vector3f(-cosf(radians(45)) * (COXA_LEN + FEMUR_LEN), sinf(radians(45)) * (COXA_LEN + FEMUR_LEN), TIBIA_LEN);
}

void AP_Quadruped::gait_select(uint8_t gait_type)
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
        gait_step_leg_start[Leg_RF] = 4;

    } else if (gait_type == 1) {
    }
}

void AP_Quadruped::calc_gait_sequence(int16_t x_travel, int16_t y_travel, int16_t yaw_travel)
{
    const float travel_dz = 10;

    bool move_requested = (abs(x_travel) > travel_dz)
        || (abs(x_travel) > travel_dz)
        || (abs(yaw_travel) > travel_dz);

    if (move_requested == true) {
        for (uint8_t leg_index = 1; leg_index < Leg_ALL; leg_index++) {
            update_leg(x_travel, y_travel, yaw_travel, leg_index);
        }
        gait_step += 1;

        if (gait_step > gait_step_total) {
            gait_step = 1;
        }

    } else {
        for (uint8_t moving_leg = 0; moving_leg < Leg_ALL; moving_leg++) {
            gait_pos_x[moving_leg] = 0;
            gait_pos_z[moving_leg] = 0;
            gait_pos_y[moving_leg] = 0;
            gait_rot_z[moving_leg] = 0;
        }
    }
}

void AP_Quadruped::update_leg(int16_t x_travel, int16_t y_travel, int16_t yaw_travel, uint8_t moving_leg)
{
    int8_t leg_step = gait_step - gait_step_leg_start[moving_leg];

    switch (leg_step) {
        case 0:
            gait_pos_x[moving_leg] = 0;
            gait_pos_z[moving_leg] = -leg_lift_height;
            gait_pos_y[moving_leg] = 0;
            gait_rot_z[moving_leg] = 0;
            break;

        case 1:
            gait_pos_x[moving_leg] = x_travel / gait_lift_divisor;
            gait_pos_z[moving_leg] = -3 * leg_lift_height / (3 + gait_half_lift_height);
            gait_pos_y[moving_leg] = y_travel / gait_lift_divisor;
            gait_rot_z[moving_leg] = yaw_travel / gait_lift_divisor;
            break;

        case 2:
            gait_pos_x[moving_leg] = x_travel * 0.5;
            gait_pos_z[moving_leg] = -leg_lift_height * 0.5;
            gait_pos_y[moving_leg] = y_travel * 0.5;
            gait_rot_z[moving_leg] = yaw_travel * 0.5;
            break;

            // case -2:
            //     gait_pos_x[moving_leg] = -x_travel * 0.5;
            //     gait_pos_z[moving_leg] = -leg_lift_height * 0.5;
            //     gait_pos_y[moving_leg] = -y_travel * 0.5;
            //     gait_rot_z[moving_leg] = -yaw_travel * 0.5;
            //     break;

        default:
            gait_pos_x[moving_leg] = gait_pos_x[moving_leg] - (x_travel / gait_travel_divisor);
            gait_pos_z[moving_leg] = 0;
            gait_pos_y[moving_leg] = gait_pos_y[moving_leg] - (y_travel / gait_travel_divisor);
            gait_rot_z[moving_leg] = gait_rot_z[moving_leg] - (yaw_travel / gait_travel_divisor);
            break;
    }
}

Vector3f AP_Quadruped::body_forward_kinematics(Vector3f xyz_leg, float Zrot_deg, Vector2f XYdist, Vector3f body_rot_xyz_deg)
{
    float totaldist_x = xyz_leg.x + XYdist.x;
    float totaldist_y = xyz_leg.y + XYdist.y;

    float distBodyCenterFeet = sqrtf(totaldist_x * totaldist_x + totaldist_y * totaldist_y);

    float AngleBodyCenter = atan2f(totaldist_y, totaldist_x);

    float rolly  = tanf(radians(body_rot_xyz_deg.y)) * totaldist_x;
    float pitchy = tanf(radians(body_rot_xyz_deg.x)) * totaldist_y;

    Vector3f ansxyz { 0, 0, 0 };
    ansxyz.x = cosf(AngleBodyCenter + radians(body_rot_xyz_deg.z + Zrot_deg)) * distBodyCenterFeet - totaldist_x;
    ansxyz.y = sinf(AngleBodyCenter + radians(body_rot_xyz_deg.z + Zrot_deg)) * distBodyCenterFeet - totaldist_y;
    ansxyz.z = rolly + pitchy + xyz_leg.z;

    return ansxyz;
}

Vector3f AP_Quadruped::leg_inverse_kinematics(Vector3f posxyz)
{
    Vector3f leg_deg { 0, 0, 0 };

    leg_deg.x = degrees(atan2f(posxyz.x, posxyz.y));

    float trueX = sqrtf(posxyz.x * posxyz.x + posxyz.y * posxyz.y) - COXA_LEN;
    float im    = sqrtf(trueX * trueX + posxyz.z * posxyz.z);
    float q1    = -atan2f(posxyz.z, trueX);
    float d1    = FEMUR_LEN * FEMUR_LEN - TIBIA_LEN * TIBIA_LEN + im * im;
    float d2    = 2 * FEMUR_LEN * im;
    float q2    = acosf(d1 / d2);
    leg_deg.y   = degrees(q1 + q2);

    d1        = FEMUR_LEN * FEMUR_LEN - im * im + TIBIA_LEN * TIBIA_LEN;
    d2        = 2 * TIBIA_LEN * FEMUR_LEN;
    leg_deg.z = degrees(acosf(d1 / d2) - radians(90));

    return leg_deg;
}

void AP_Quadruped::main_inverse_kinematics(void)
{

    Vector3f body_rot_xyz_deg = { 0, 0, 0 };

    int8_t neg_pos_sig_x[Leg_ALL] = { 1, -1, 1, -1 };
    int8_t neg_pos_sig_y[Leg_ALL] = { 1, -1, 1, -1 };

    for (uint8_t leg_index = 0; leg_index < Leg_ALL; leg_index++) {
        Vector3f ans_leg = body_forward_kinematics(endpoint_leg[leg_index] + gait_pos_xyz[leg_index],
                                                   gait_rot_z[leg_index],
                                                   Vector2f(neg_pos_sig_x[leg_index] * FRAME_LEN * 0.5,
                                                            neg_pos_sig_y[leg_index] * FRAME_WIDTH * 0.5),
                                                   body_rot_xyz_deg);
    }
}
