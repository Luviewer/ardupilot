#include "AP_QuadRuped.h"
#include "AP_DroneCAN/AP_DroneCAN.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define COXA_LEN_DEFAULT        47.1f
#define FEMUR_LEN_DEFAULT       133.0f
#define TIBIA_LEN_DEFAULT       144.1f
#define FRAME_LEN_DEFAULT       185.0f
#define FRAME_WIDTH_DEFAULT     185.0f
#define LIFT_HEIGHT_DEFAULT     50.0f
#define SPEED_HZ_DEFAULT        25.0f
#define MAX_THROTTLE_DEFAULT    200.0f
#define GAIT_STEP_TOTAL_DEFAULT 12

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

    AP_GROUPINFO("_COX1OF", 23, AP_QuadRuped, coxa_offset[Leg_RF], 0),
    AP_GROUPINFO("_COX2OF", 24, AP_QuadRuped, coxa_offset[Leg_RB], 0),
    AP_GROUPINFO("_COX3OF", 25, AP_QuadRuped, coxa_offset[Leg_LB], 0),
    AP_GROUPINFO("_COX4OF", 26, AP_QuadRuped, coxa_offset[Leg_LF], 0),

    AP_GROUPINFO("_FEM1OF", 27, AP_QuadRuped, femur_offset[Leg_RF], 0),
    AP_GROUPINFO("_FEM2OF", 28, AP_QuadRuped, femur_offset[Leg_RB], 0),
    AP_GROUPINFO("_FEM3OF", 29, AP_QuadRuped, femur_offset[Leg_LB], 0),
    AP_GROUPINFO("_FEM4OF", 30, AP_QuadRuped, femur_offset[Leg_LF], 0),

    AP_GROUPINFO("_TIB1OF", 31, AP_QuadRuped, tibia_offset[Leg_RF], 0),
    AP_GROUPINFO("_TIB2OF", 32, AP_QuadRuped, tibia_offset[Leg_RB], 0),
    AP_GROUPINFO("_TIB3OF", 33, AP_QuadRuped, tibia_offset[Leg_LB], 0),
    AP_GROUPINFO("_TIB4OF", 34, AP_QuadRuped, tibia_offset[Leg_LF], 0),

    AP_GROUPINFO("_THRCH", 35, AP_QuadRuped, throttle_channel, -1),
    AP_GROUPINFO("_POSCH", 36, AP_QuadRuped, zpos_channel, -1),
    AP_GROUPINFO("_YAWCH", 37, AP_QuadRuped, yaw_channel, -1),

    AP_GROUPINFO("_GAITCH", 38, AP_QuadRuped, gait_channel, -1),
    AP_GROUPINFO("_zHeight", 39, AP_QuadRuped, zfactor, 1),
    AP_GROUPINFO("_ROLLCH", 40, AP_QuadRuped, roll_channel, -1),
    AP_GROUPINFO("_pitchCH", 41, AP_QuadRuped, pitch_channel, -1),

    AP_SUBGROUPINFO(yaw_pid, "_YAW_", 42, AP_QuadRuped, AC_PID),
    AP_GROUPEND
};

AP_QuadRuped::AP_QuadRuped(AP_AHRS_View*& ahrs, AP_MotorsMulticopter*& motors)
    : _ahrs(ahrs)     //_ahrs(ahrs)：将传入的 ahrs 指针赋给类的私有成员 _ahrs（姿态传感器接口）
    , _motors(motors) //_motors(motors)：将传入的 motors 指针赋给类的私有成员 _motors（电机控制接口）
    , yaw_pid(0.1f, 0.05f, 0.01f, 0.0f, 50.0f, 0.0f, 0.0f, 0.0f, 0.0f)
// 使用引用传递指针（*&）确保外部传入的指针在类内部可被修改
{
    gait_type      = 0;
    move_requested = false;
    max_yaw_rate   = radians(30.0f); // 限制最大25°/s

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
    // gait_half_lift_height = 1;
    // gait_travel_divisor   = 4;
    if (gait_type == GAIT_DIAGONAL) {
        gait_step_leg_start[Leg_RF] = 1;
        gait_step_leg_start[Leg_RB] = gait_step_total / 2 + 1;
        gait_step_leg_start[Leg_LB] = 1;
        gait_step_leg_start[Leg_LF] = gait_step_total / 2 + 1;

        gait_travel_divisor = gait_step_total / 2 + 1;
        gait_lift_divisor   = 2;
    } else if (gait_type == GAIT_WAVE) {
        // 波浪步态设置 - 四条腿依次移动
        gait_step_leg_start[Leg_RF] = 1;
        gait_step_leg_start[Leg_LF] = gait_step_total / 4 + 1;
        gait_step_leg_start[Leg_LB] = gait_step_total / 2 + 1;
        gait_step_leg_start[Leg_RB] = 3 * gait_step_total / 4 + 1;

        gait_travel_divisor = gait_step_total / 4 + 1;
        gait_lift_divisor   = 4;
    }
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

void AP_QuadRuped::trajectory_generation(uint8_t leg_index)
{
    float    delta;
    Vector2f leg_xy_target;
    float    leg_z_target = 0;

    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    if (delta_step < 0) {
        delta_step = gait_step_total + delta_step + 1;
    }

    if (gait_type == GAIT_DIAGONAL) {
        if (delta_step <= (gait_step_total / 2)) {
            delta = M_2PI * delta_step / gait_step_total * 2.0f;
            // leg_xy_target = Vector2f(throttle_travel, 0) * (delta - sinf(delta)) / M_2PI * 2.0f - Vector2f(throttle_travel, 0);
            leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f - throttle_travel;
            leg_xy_target[1] = 0;
            leg_z_target     = -leg_lift_height * (1.0f - cosf(delta)) * zfactor;
        } else {
            delta = M_2PI * (delta_step - gait_step_total / 2) / gait_step_total * 2.0f;
            // leg_xy_target = -Vector2f(throttle_travel, 0) * (delta - sinf(delta)) / M_2PI * 2.0f + Vector2f(throttle_travel, 0);
            leg_xy_target[0] = -throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f + throttle_travel;
            leg_xy_target[1] = 0;
            leg_z_target     = 0;
        }
    }

    else if (gait_type == GAIT_WAVE) {
        // 波浪步态轨迹生成
        if (delta_step <= (gait_step_total / 4)) {
            delta            = M_2PI * delta_step / (gait_step_total / 4);
            leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 4.0f - throttle_travel; // 钟型曲线，由于1-cosf(delta)的导数特性，运动开始和结束时的速度为0    M_2PI * 4.0f：适配波浪步态的1/4周期时间窗口
            leg_xy_target[1] = 0;
            leg_z_target     = -leg_lift_height * (1.0f - cosf(delta)) * zfactor; // 抬升之后放下
        } else {
            delta            = M_2PI * (delta_step - gait_step_total / 4) / (3 * gait_step_total / 4);
            leg_xy_target[0] = -throttle_travel * (delta - sinf(delta)) / M_2PI * (4.0f / 3.0f) + throttle_travel;
            leg_xy_target[1] = 0;
            leg_z_target     = 0;
        }
    }
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

void AP_QuadRuped::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    // float progress = (float)delta_step / gait_step_total;
    if (gait_type == GAIT_DIAGONAL) {
        switch (delta_step) {
            case 0:
                gait_rot_z[leg_index] = 0;
                break;

            case 1:
                gait_rot_z[leg_index] = yaw_travel / gait_lift_divisor;
                break;

            default:
                gait_rot_z[leg_index] = gait_rot_z[leg_index] - (yaw_travel / gait_travel_divisor);
                break;
        }
    }

    else if (gait_type == GAIT_WAVE) {
        // 根据腿部运动阶段分配航向角
        // 根据腿部运动阶段平滑分配航向角
        switch (delta_step) {
            case 0:
                gait_rot_z[leg_index] = 0;
                break;

            case 1:
                gait_rot_z[leg_index] = yaw_travel / gait_lift_divisor;
                break;

            default:
                gait_rot_z[leg_index] = gait_rot_z[leg_index] - (yaw_travel / gait_travel_divisor);
                break;
        }
    }
}

void AP_QuadRuped::update_leg()
{
    gait_step_now++;
    if (gait_step_now > gait_step_total) gait_step_now = 0;
    // float dir = 1;
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
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
    controller();
    balance_controller();

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
        pwm_femur = leg_femur_direction[leg_index] * -65 * 500 / 120 + 1500;
        pwm_tibia = leg_tibia_direction[leg_index] * 30 * 500 / 120 + 1500;

        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
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

        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped::balance_controller()
{

    // float temp_rc;
    float target_roll  = 0;
    float target_pitch = 0;

    float current_roll  = degrees(_ahrs->roll);
    float current_pitch = degrees(_ahrs->pitch);

    if (roll_channel != -1) {
        // temp_rc    = constrain_value((float)rc().RC_Channels::get_radio_in(roll_channel - 1), (float)1000, (float)2000);   //通道索引通常从 0 开始，这里设置的 yaw_channel从 1 开始编号
        // target_roll = (temp_rc - 1500) / 500.0f * 15.0f;      //将遥控器输入转换为目标偏航角（-180°到+180°）
        // roll_travel = (temp_rc - 1500) / 500.0f * 15.0f;
        float roll_error = wrap_180(current_roll - target_roll); // 计算偏航角误差（将弧度值规范到[-π, π]区间）
        // // hal.console->printf("yaw_error=%f,target_yaw=%f,current_yaw=%f\n",yaw_error,target_yaw,current_yaw)
        roll_travel = roll_pid.update_all(0, roll_error, 1.0f / gait_hz);

    } else {
        roll_travel = 0;
    }
    if (pitch_channel != -1) {
        // temp_rc    = constrain_value((float)rc().RC_Channels::get_radio_in(pitch_channel - 1), (float)1000, (float)2000);   //通道索引通常从 0 开始，这里设置的 yaw_channel从 1 开始编号
        // target_pitch = (temp_rc - 1500) / 500.0f * 15.0f;      //将遥控器输入转换为目标偏航角（-180°到+180°）
        float pitch_error = wrap_180(current_pitch - target_pitch); // 计算偏航角误差（将弧度值规范到[-π, π]区间）
        // hal.console->printf("yaw_error=%f,target_yaw=%f,current_yaw=%f\n",yaw_error,target_yaw,current_yaw)
        pitch_travel = pitch_pid.update_all(0, pitch_error, 1.0f / gait_hz);
    } else {
        pitch_travel = 0;
    }
}

void AP_QuadRuped::controller()
{

    float temp_rc;
    float target_yaw  = 0;
    float current_yaw = degrees(_ahrs->yaw);

    if (yaw_channel != -1) {
        temp_rc         = constrain_value((float)rc().RC_Channels::get_radio_in(yaw_channel - 1), (float)1000, (float)2000); // 通道索引通常从 0 开始，这里设置的 yaw_channel从 1 开始编号
        target_yaw      = (temp_rc - 1500) / 500.0f * 180.0f;                                                                // 将遥控器输入转换为目标偏航角（-180°到+180°）
        float yaw_error = wrap_180(current_yaw - target_yaw);                                                                // 计算偏航角误差（将弧度值规范到[-π, π]区间）
        // hal.console->printf("yaw_error=%f,target_yaw=%f,current_yaw=%f\n",yaw_error,target_yaw,current_yaw)
        yaw_travel = yaw_pid.update_all(0, yaw_error, 1.0f / gait_hz);
    } else {
        yaw_travel = 0;
    }

    if (throttle_channel != -1) {
        temp_rc         = constrain_value((float)rc().RC_Channels::get_radio_in(throttle_channel - 1), (float)1000, (float)2000);
        throttle_travel = (temp_rc - 1500) / 500.0f * throttle_max;
    } else {
        throttle_travel = 0;
    }

    if (zpos_channel != -1) {
        temp_rc  = constrain_value((float)rc().RC_Channels::get_radio_in(zpos_channel - 1), (float)1000, (float)2000);
        z_travel = (temp_rc - 1500) / 500.0f * 120.0f - 50;
    } else {
        z_travel = -50;
    }

    // 添加步态切换控制
    if (gait_channel != -1) {
        float   gait_switch   = constrain_value((float)rc().RC_Channels::get_radio_in(gait_channel - 1), (float)1000, (float)2000);
        uint8_t new_gait_type = (gait_switch > 1500) ? GAIT_WAVE : GAIT_DIAGONAL;

        if (new_gait_type != gait_type) {
            gait_type = new_gait_type;
            gait_select(); // 步态变化时重新初始化步态参数
        }
    }
}

bool AP_QuadRuped::hw_set_servo_cmd()
{
    com_usl_ServoCmd msg {};

    msg.cmd.len = 12;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        msg.cmd.data[leg_index * 3 + 0] = servo_output_cmd[leg_index].x + coxa_offset[leg_index];
        msg.cmd.data[leg_index * 3 + 1] = servo_output_cmd[leg_index].y + femur_offset[leg_index];
        msg.cmd.data[leg_index * 3 + 2] = servo_output_cmd[leg_index].z + tibia_offset[leg_index];

        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_coxa + leg_index * 3), servo_output_cmd[leg_index].x + coxa_offset[leg_index]);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_femu + leg_index * 3), servo_output_cmd[leg_index].y + femur_offset[leg_index]);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_tibi + leg_index * 3), servo_output_cmd[leg_index].z + tibia_offset[leg_index]);
    }

    // broadcast the message on all ifaces
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    bool ok = false;
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto* dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan != nullptr) {
            ok |= dronecan->com_usl_servocmd.broadcast(msg);
        }
    }
    return ok;
}