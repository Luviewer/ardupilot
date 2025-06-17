#include "AP_QuadRuped.h"
#include "AP_DroneCAN/AP_DroneCAN.h"
#include "AP_QuadRuped_Params.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define LEG_MOTOR_MAX_DEG       (120)
#define LEG_MOTOR_MAX_PWM       (500)
#define LEG_MOTOR_PWM_MIDDLE    (1500)

#define LIFT_HEIGHT_DEFAULT     50.0f
#define SPEED_HZ_DEFAULT        25.0f
#define MAX_THROTTLE_DEFAULT    200.0f
#define GAIT_STEP_TOTAL_DEFAULT 12

const AP_Param::GroupInfo AP_QuadRuped::var_info[] = {

    AP_GROUPINFO("_LIFT", 1, AP_QuadRuped, leg_lift_height, LIFT_HEIGHT_DEFAULT),
    AP_GROUPINFO("_Hz", 2, AP_QuadRuped, gait_hz, SPEED_HZ_DEFAULT),

    AP_GROUPINFO("_THR", 3, AP_QuadRuped, throttle_max, MAX_THROTTLE_DEFAULT),
    AP_GROUPINFO("_STEP", 4, AP_QuadRuped, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    AP_SUBGROUPINFO(Sys_Param, "_SYS", 5, AP_QuadRuped, AP_QuadRuped_SYS_Params),

    AP_SUBGROUPINFO(channel, "_CH_", 6, AP_QuadRuped, AP_QuadRuped_CHANNEL_Params),

    AP_SUBGROUPINFO(leg_param[Leg_RF], "_RF_", 7, AP_QuadRuped, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_RB], "_RB_", 8, AP_QuadRuped, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_LB], "_LB_", 9, AP_QuadRuped, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_LF], "_LF_", 10, AP_QuadRuped, AP_QuadRuped_Params),

    AP_SUBGROUPINFO(roll_pid, "_RLL_", 11, AP_QuadRuped, AC_PID),
    AP_SUBGROUPINFO(pitch_pid, "_PIT_", 12, AP_QuadRuped, AC_PID),

    AP_SUBGROUPINFO(diag_yaw_pid, "_DYAW_", 13, AP_QuadRuped, AC_PID),
    AP_SUBGROUPINFO(wave_yaw_pid, "_WYAW_", 14, AP_QuadRuped, AC_PID),

    AP_GROUPEND
};

AP_QuadRuped::AP_QuadRuped(AP_AHRS_View*& ahrs, AP_MotorsMulticopter*& motors)
    : _ahrs(ahrs)     //_ahrs(ahrs)：将传入的 ahrs 指针赋给类的私有成员 _ahrs（姿态传感器接口）
    , _motors(motors) //_motors(motors)：将传入的 motors 指针赋给类的私有成员 _motors（电机控制接口）
// 使用引用传递指针（*&）确保外部传入的指针在类内部可被修改
{
    gait_type      = 0;
    move_requested = false;
    max_yaw_rate   = radians(30.0f); // 限制最大30°/s

    AP_Param::setup_object_defaults(this, var_info);

    // leg_lift_height = 80; // leg lift height(in mm) while walking

    // Sys_Param.COXA_LEN  = 47.1; // distance (in mm) from coxa (aka hip) servo to femur servo
    // Sys_Param.FEMUR_LEN = 133;  // distance (in mm) from femur servo to tibia servo
    // Sys_Param.TIBIA_LEN = 144;  // distance (in mm) from tibia servo to foot

    // Sys_Param.FRAME_LEN   = 185; // frame length in mm
    // Sys_Param.FRAME_WIDTH = 185; // frame width in mm
}

#define START_COXA_ANGLE 45

void AP_QuadRuped::init(void)
{
    // 初始化腿部起始位置 (Initialize leg starting positions)
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // X坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * sin(角度)
        // Y坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * cos(角度)
        // Z坐标: Sys_Param.TIBIA_LEN (胫骨长度决定初始高度)
        endpoint_leg_pos[leg_index] = Vector3f(sinf(radians(START_COXA_ANGLE + leg_index * 90)) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               cosf(radians(START_COXA_ANGLE + leg_index * 90)) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               Sys_Param.TIBIA_LEN);
    }

    // 初始化腿部框架位置 (Initialize leg frame positions)
    // 计算每条腿的髋关节在机体坐标系中的位置 (Calculate hip joint position in body frame)
    // 使用与腿部位置相同的角度基准 (Using same angle reference as leg positions)
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // X坐标: Sys_Param.FRAME_LEN * sin(角度) - 决定前后位置
        // Y坐标: Sys_Param.FRAME_WIDTH * cos(角度) - 决定左右位置
        // Z坐标: 0 (髋关节与机体在同一平面)
        endpoint_leg_frame[leg_index] = Vector3f(sqrtf(2) * sinf(radians(START_COXA_ANGLE + leg_index * 90)) * Sys_Param.FRAME_LEN * 0.5f,
                                                 sqrtf(2) * cosf(radians(START_COXA_ANGLE + leg_index * 90)) * Sys_Param.FRAME_WIDTH * 0.5f,
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
    if (gait_type == GAIT_DIAGONAL) {
        gait_step_leg_start[Leg_RF] = 1;
        gait_step_leg_start[Leg_RB] = gait_step_total / 2 + 1;
        gait_step_leg_start[Leg_LB] = 1;
        gait_step_leg_start[Leg_LF] = gait_step_total / 2 + 1;

        gait_travel_divisor = gait_step_total / 2 + 1;
        gait_lift_divisor   = 2;
    } else if (gait_type == GAIT_WAVE) {
        // 波浪步态设置 - 四条腿依次移动
        // 波浪步态设置 - 四条腿依次移动
        gait_step_leg_start[Leg_RF] = 0;                       // 右前腿
        gait_step_leg_start[Leg_LF] = gait_step_total / 4;     // 左前腿
        gait_step_leg_start[Leg_LB] = gait_step_total / 2;     // 左后腿
        gait_step_leg_start[Leg_RB] = 3 * gait_step_total / 4; // 右后腿

        // 调整步态参数
        gait_travel_divisor = gait_step_total / 4;
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

// 设置目标重心偏移
void AP_QuadRuped::set_centre_offset(float x, float y, float z = 0)
{
    centre_offset = Vector3f(x, y, z);
}

// gait_step本质上是一个离散化的时间变量，将连续的步态运动分解为多个离散的步骤
void AP_QuadRuped::trajectory_generation(uint8_t leg_index)
{
    float    delta;
    Vector2f leg_xy_target;
    float    leg_z_target = 0;

    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index]; // 计算当前腿相对于其起始步态的相位偏移
    // gait_step_total一个完整步态周期的总步数，值越大，步态分解越精细，运动越平滑
    // gait_step_leg_start[leg_index]每条腿的步态起始相位，使不同腿的运动产生相位差，例如对角步态中，两条腿的起始相位相差 gait_step_total/2

    if (delta_step < 0) {
        delta_step = gait_step_total + delta_step + 1; // 如果结果为负，通过加上总步态周期数来修正
    }

    if (gait_type == GAIT_DIAGONAL) {
        if (delta_step <= (gait_step_total / 2)) {                                                       // 抬起移动阶段
            delta            = M_2PI * delta_step / gait_step_total * 2.0f;                              // 将当前步态相位映射到0-2π范围
            leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f - throttle_travel; // 最终将范围平移为[-throttle_travel → +throttle_travel]，将轨迹中心从throttle_travel移动到坐标系原点（0点）
            leg_xy_target[1] = 0;
            leg_z_target     = -leg_lift_height * (1.0f - cosf(delta)) * 1.0f; // 形成山峰形状
        } else {                                                               // 支撑返回阶段
            delta            = M_2PI * (delta_step - gait_step_total / 2) / gait_step_total * 2.0f;
            leg_xy_target[0] = -throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f + throttle_travel; //[+throttle_travel → -throttle_travel]，核心作用是通过坐标平移实现运动方向反转和相位同步
            leg_xy_target[1] = 0;
            leg_z_target     = 0;
        }
    }

    else if (gait_type == GAIT_WAVE) {
        if (delta_step <= gait_step_total / 4) {
            // 抬腿 + 前移阶段
            delta            = M_2PI * delta_step / gait_step_total * 4.0f;                              // 将当前步态相位映射到0-2π范围
            leg_xy_target[0] = throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f - throttle_travel; // 光滑前移
            leg_xy_target[1] = 0;
            leg_z_target     = -leg_lift_height * (delta - cosf(delta)) / 1.0f; // 平滑抬腿（从0升到最大）
        } else if (delta_step <= gait_step_total * 3 / 4) {
            // 向后移、落腿阶段
            delta            = M_2PI * (delta_step - gait_step_total / 4) / (3 * gait_step_total / 4); // 归一化到 [0, 1]
            leg_xy_target[0] = -throttle_travel * (delta - sinf(delta)) / M_2PI * 2.0f + throttle_travel;              // 平滑后移
            leg_xy_target[1] = 0;
            leg_z_target     = 0; // 平滑落地（从最大降到0）
        }
        gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target); // x：横向移动（如左右踏步）    y：前后移动（如前进/后退）
    }
}

void AP_QuadRuped::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }
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
        // 更平滑的偏航控制曲线

        if (delta_step <= (gait_step_total / 4)) {
            // 抬腿阶段(重心已提前转移)
            float phase               = (float)delta_step / (gait_step_total / 4);
            gait_pos_xyz[leg_index].x = throttle_travel * (phase - sinf(phase * M_2PI) / M_2PI) * 2.0f;
            gait_pos_xyz[leg_index].z = -leg_lift_height * (1.0f - cosf(phase * M_PI)) * 1.8f;
        } else if (delta_step <= (gait_step_total * 3 / 4)) {
            // 移动阶段
            float phase               = (float)(delta_step - gait_step_total / 4) / (gait_step_total / 2);
            gait_pos_xyz[leg_index].x = throttle_travel * (1.0f - phase);
        } else {
            // 放下阶段
            float phase               = (float)(delta_step - gait_step_total * 3 / 4) / (gait_step_total / 4);
            gait_pos_xyz[leg_index].x = -throttle_travel * phase;
            gait_pos_xyz[leg_index].z = -leg_lift_height * (1.0f - cosf((1.0f - phase) * M_PI)) * 0.5f;
        }

        // 根据腿的位置调整偏航量
        switch (leg_index) {
            case Leg_RF:                       // 右前腿
            case Leg_LB:                       // 左后腿
                gait_rot_z[leg_index] *= 1.2f; // 增加主力腿的偏航贡献
                break;
            case Leg_LF:                       // 左前腿
            case Leg_RB:                       // 右后腿
                gait_rot_z[leg_index] *= 0.8f; // 减少辅助腿的偏航贡献
                break;
        }
    }
}

void AP_QuadRuped::update_leg()
{
    gait_step_now++;
    if (gait_step_now > gait_step_total) gait_step_now = 0;

    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[moving_leg];
        if (delta_step < 0) delta_step += gait_step_total;
        // update_centre_offset(moving_leg);
        // set_centre_offset(offset_xy.x, offset_xy.y);
        trajectory_generation(moving_leg);
        yaw_trajectory_generation(moving_leg);
    }
}

void AP_QuadRuped::update_centre_offset(uint8_t leg_index)
{
    if (gait_type != GAIT_WAVE) {
        return; // 仅对波浪步态应用
    }
    switch (leg_index) {
        case Leg_RF: // 右前腿
            offset_xy.x += -60;
            offset_xy.y += -60;
            break;
        case Leg_LB: // 左后腿
            offset_xy.x += 60;
            offset_xy.y += 60;
            break;
        case Leg_LF: // 左前腿
            offset_xy.x += -60;
            offset_xy.y += 60;
            break;
        case Leg_RB: // 右后腿
            offset_xy.x += 60;
            offset_xy.y += -60;
            break;
    }
    hal.console->printf("offset.x=%f\n,offset.y=%f\n", offset_xy.x, offset_xy.y);
}

Vector3f AP_QuadRuped::body_forward_kinematics(uint8_t leg_index)
{
    // gait_pos_xyz：步态生成的目标位置
    // endpoint_leg_pos：腿部初始展开位置（代码中初始化为(Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN)*sin(45°), ...）
    // endpoint_leg_frame：机体框架几何尺寸（机体的几何偏移）（如FRAME_LEN和FRAME_WIDTH）
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    // 添加重心偏移补偿
    totaldist_xyz -= centre_offset;

    totaldist_xyz.z += z_travel;

    Quaternion quat = { 1, 0, 0, 0 };

    body_rot_xyz_deg.x = -radians(roll_travel);  // 横滚角（绕X轴）
    body_rot_xyz_deg.y = -radians(pitch_travel); // 俯仰角（绕Y轴）
    // hal.console->printf("pitch_travel_1=%f\n", pitch_travel);
    body_rot_xyz_deg.z = radians(gait_rot_z[leg_index]); // 偏航角（绕Z轴）

    quat.from_euler(body_rot_xyz_deg);

    Vector3f totaldist_xyz_rot = quat * totaldist_xyz;

    return (totaldist_xyz_rot - endpoint_leg_frame[leg_index]);
}

// posxyz：表示腿部末端执行器相对于髋关节的位置坐标(x, y, z)     x: 末端在机器人前进方向的前后  y: 末端在机器人前进方向的左右   z: 末端的高度位置
// leg_deg：包含三个关节的角度值(度)     x: 髋关节(coxal joint)旋转角度  y: 股关节(femur joint)角度  z: 胫关节(tibia joint)角度

Vector3f AP_QuadRuped::leg_inverse_kinematics(Vector3f posxyz)
{
    Vector3f leg_deg = { 0, 0, 0 };

    leg_deg.x = -degrees(atan2f(posxyz.x, posxyz.y));

    float trueX = sqrtf(posxyz.x * posxyz.x + posxyz.y * posxyz.y) - Sys_Param.COXA_LEN;                           // trueX: 从髋关节到末端在XY平面投影的直线距离减去大腿长度
    float im    = sqrtf(trueX * trueX + posxyz.z * posxyz.z);                                                      // im: 从股关节到末端的直线距离(空间距离)
    float q1    = atan2f(trueX, posxyz.z);                                                                         // q1: 临时角度变量1 - 股关节与末端连线与垂直方向的夹角
    float d1    = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN + im * im; // q2: 临时角度变量2 - 余弦定理计算得到的角度
    float d2    = 2 * Sys_Param.FEMUR_LEN * im;                                                                    // d1, d2: 余弦定理计算中的中间变量
    float q2    = acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f));
    leg_deg.y   = -(degrees(q1 + q2) - 90);

    d1        = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - im * im + Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN;
    d2        = 2 * Sys_Param.TIBIA_LEN * Sys_Param.FEMUR_LEN;
    leg_deg.z = -(degrees(acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f))) - 90);

    return leg_deg;
}

void AP_QuadRuped::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 };

    const Vector3f endpoint_leg_angle_offset[LEG_ALL] = {
        { 45, 0, 0 },
        { 135, 0, 0 },
        { 225, 0, 0 },
        { 315, 0, 0 }
    }; // {髋关节角度， 股关节角度， 胫关节角度}   只有髋关节需要补偿
    controller();

    // const float endpoint_leg_angle_dir[LEG_ALL] = { 1, 1, 1, 1 };

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {

        ansxyz = body_forward_kinematics(leg_index);

        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];

        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);

        // endpoint_leg_angle[leg_index].x *= endpoint_leg_angle_dir[leg_index];
    }

    // if (servo_estimate()) {
    // start_time = AP_HAL::millis();

    calc_gait_sequence(); // 根据 throttle_travel 和 yaw_travel 更新步态相位（gait_step_now）    决定下一步的足端轨迹（gait_pos_xyz）

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
    // }
}

bool AP_QuadRuped::servo_estimate(void) // 定时触发器，用于判断是否到达预定的控制周期时间点
{
    uint32_t target_time = AP_HAL::millis(); // 获取系统运行的毫秒数，硬件抽象层(HAL)提供的跨平台时间函数

    if ((target_time - start_time) >= (1000.0f / gait_hz)) { // 1000.0f / gait_hz将频率(Hz)转换为周期时间(ms)     检查是否已经过完整的控制周期，使用>=而非==确保不会错过周期（即使有微小延迟）
        return true;
    }
    return false;
}

void AP_QuadRuped::right_sleep_leg(void)
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {

        pwm_coxa  = leg_param[leg_index].COXA_DIR * 45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * -65 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 30 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped::reset_leg(void)
{
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 }; // 重置位置坐标为原点
        gait_rot_z[moving_leg]   = 0;           // 重置旋转角度为0
    }
}

void AP_QuadRuped::output_leg_angle(void)
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        pwm_coxa  = leg_param[leg_index].COXA_DIR * endpoint_leg_angle[leg_index].x * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * endpoint_leg_angle[leg_index].y * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * endpoint_leg_angle[leg_index].z * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 存储PWM命令
        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped::balance_controller()
{
    float temp_rc;
    float target_roll  = 0;
    float target_pitch = 0;

    float current_roll  = degrees(_ahrs->roll);
    float current_pitch = degrees(_ahrs->pitch);
    float current_yaw   = degrees(_ahrs->yaw);

    if (channel.roll_channel != -1) {
        temp_rc          = constrain_value((float)rc().RC_Channels::get_radio_in(channel.roll_channel - 1), (float)1000, (float)2000); // 通道索引通常从 0 开始，这里设置的 yaw_channel从 1 开始编号
        target_roll      = (temp_rc - 1500) / 500.0f * 15.0f;                                                                          // 将遥控器输入转换为目标偏航角（-180°到+180°）
        float roll_error = wrap_180(current_roll - target_roll);                                                                       // 计算偏航角误差（将弧度值规范到[-π, π]区间）
        // // hal.console->printf("yaw_error=%f,target_yaw=%f,current_yaw=%f\n",yaw_error,target_yaw,current_yaw)
        roll_travel = roll_pid.update_all(0, roll_error, 1.0f / gait_hz);
    } else {
        roll_travel = 0;
    }

    if (channel.pitch_channel != -1) {
        temp_rc           = constrain_value((float)rc().RC_Channels::get_radio_in(channel.pitch_channel - 1), (float)1000, (float)2000); // 通道索引通常从 0 开始，这里设置的 yaw_channel从 1 开始编号
        target_pitch      = (temp_rc - 1500) / 500.0f * 15.0f;                                                                           // 将遥控器输入转换为目标偏航角（-180°到+180°）
        float pitch_error = wrap_180(current_pitch - target_pitch);                                                                      // 计算偏航角误差（将弧度值规范到[-π, π]区间）
        // hal.console->printf("yaw_error=%f,target_yaw=%f,current_yaw=%f\n",yaw_error,target_yaw,current_yaw)
        pitch_travel = pitch_pid.update_all(0, pitch_error, 1.0f / gait_hz);
        // hal.console->printf("pitch_error=%f,pitch_travel=%f\n", pitch_error, pitch_travel);
    } else {
        pitch_travel = 0;
    }

    if (channel.yaw_channel != -1) {
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.yaw_channel - 1), (float)1000, (float)2000);
        if (temp_rc < 1550 && temp_rc > 1450) temp_rc = 1500;

        // 将遥控器输入转换为目标角速度（-max_yaw_rate到+max_yaw_rate）
        float delta_yaw = (temp_rc - 1500) / 500.0f * 1.0f;
        // 计算角度误差
        target_yaw += delta_yaw;
        float yaw_error = wrap_180(current_yaw - target_yaw);

        if (fabsf(yaw_error) < radians(1.0f)) yaw_error = 0.0f;

        // 使用PID控制器计算角速度增量
        if (gait_type == GAIT_DIAGONAL) {
            yaw_travel = diag_yaw_pid.update_all(0, yaw_error, 1.0f / gait_hz);
        } else {
            yaw_travel = wave_yaw_pid.update_all(0, yaw_error, 1.0f / gait_hz);
        }
    } else {
        yaw_travel = 0;
    }

    // 添加新的遥控通道处理
    if (channel.centre_offset_x_channel != -1) {
        float val = constrain_value((float)rc().get_radio_in(channel.centre_offset_x_channel - 1), (float)1000, (float)2000);
        if (val > 1475 && val < 1525) { // 死区检测
            val = 1500;
        }
        offset_xy.x = (val - 1500) / 500.0f * 100.0f; // ±100mm范围
    } else {
        offset_xy.x = 0;
    }
    if (channel.centre_offset_y_channel != -1) {
        float val = constrain_value((float)rc().get_radio_in(channel.centre_offset_y_channel - 1), (float)1000, (float)2000);
        if (val > 1475 && val < 1525) { // 死区检测
            val = 1500;
        } // 死区检测
        offset_xy.y = (val - 1500) / 500.0f * 100.0f; // ±100mm范围
    } else {
        offset_xy.y = 0;
    }
}

void AP_QuadRuped::controller()
{
    float temp_rc;

    if (channel.throttle_channel != -1) {
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.throttle_channel - 1), (float)1000, (float)2000);
        if (temp_rc < 1550 && temp_rc > 1450) temp_rc = 1500;

        throttle_travel = (temp_rc - 1500) / 500.0f * throttle_max;
    } else {
        throttle_travel = 0;
    }

    if (channel.height_channel != -1) {
        temp_rc  = constrain_value((float)rc().RC_Channels::get_radio_in(channel.height_channel - 1), (float)1000, (float)2000);
        z_travel = (temp_rc - 1500) / 500.0f * 120.0f - 50;
    } else {
        z_travel = -50;
    }

    // 添加步态切换控制
    if (channel.gait_channel != -1) {
        float   gait_switch   = constrain_value((float)rc().RC_Channels::get_radio_in(channel.gait_channel - 1), (float)1000, (float)2000);
        uint8_t new_gait_type = (gait_switch > 1500) ? GAIT_WAVE : GAIT_DIAGONAL;

        if (new_gait_type != gait_type) {
            gait_type = new_gait_type;
            gait_select(); // 步态变化时重新初始化步态参数
        }
    }
}

bool AP_QuadRuped::hw_set_servo_cmd()
{
    com_usl_ServoCmd msg {}; // 创建DroneCAN伺服控制消息结构体
    msg.cmd.len = 12;        // 设置消息长度(4条腿×3个关节)

    // 遍历所有腿部(LEG_ALL=4)
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        msg.cmd.data[leg_index * 3 + 0] = servo_output_cmd[leg_index].x + leg_param[leg_index].COXA_OFS;
        msg.cmd.data[leg_index * 3 + 1] = servo_output_cmd[leg_index].y + leg_param[leg_index].FEMU_OFS;
        msg.cmd.data[leg_index * 3 + 2] = servo_output_cmd[leg_index].z + leg_param[leg_index].TIBI_OFS;

        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_coxa + leg_index * 3),
                                     servo_output_cmd[leg_index].x + leg_param[leg_index].COXA_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_femu + leg_index * 3),
                                     servo_output_cmd[leg_index].y + leg_param[leg_index].FEMU_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_tibi + leg_index * 3),
                                     servo_output_cmd[leg_index].z + leg_param[leg_index].TIBI_OFS);
    }

    // 这段代码的主要功能是在所有可用的CAN总线接口上广播伺服控制命令(com_usl_servocmd消息)。
    uint8_t can_num_drivers = AP::can().get_num_drivers(); // 获取CAN驱动数量
    bool    ok              = false;                       // 成功标志

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto* dronecan = AP_DroneCAN::get_dronecan(i); // 获取第i个CAN驱动实例
        if (dronecan != nullptr) {
            // 尝试广播消息，使用|=确保只要有一个接口成功就返回true
            ok |= dronecan->com_usl_servocmd.broadcast(msg); // 访问该CAN实例的伺服命令接口
        }
    }
    return ok; // 返回广播结果
}
