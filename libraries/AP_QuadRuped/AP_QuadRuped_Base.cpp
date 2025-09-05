#include "AP_QuadRuped_Base.h"
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
#define GAIT_STEP_TOTAL_DEFAULT 24

#define START_COXA_ANGLE        45

const AP_Param::GroupInfo AP_QuadRuped_Base::var_info[] = {

    AP_GROUPINFO("LIFT", 1, AP_QuadRuped_Base, leg_lift_height, LIFT_HEIGHT_DEFAULT),
    AP_GROUPINFO("Hz", 2, AP_QuadRuped_Base, gait_hz, SPEED_HZ_DEFAULT),

    AP_GROUPINFO("THR", 3, AP_QuadRuped_Base, throttle_max, MAX_THROTTLE_DEFAULT),
    AP_GROUPINFO("STEP", 4, AP_QuadRuped_Base, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    AP_SUBGROUPINFO(Sys_Param, "SYS", 5, AP_QuadRuped_Base, AP_QuadRuped_SYS_Params),

    AP_SUBGROUPINFO(channel, "CH_", 6, AP_QuadRuped_Base, AP_QuadRuped_CHANNEL_Params),

    AP_SUBGROUPINFO(leg_param[Leg_RF], "RF_", 7, AP_QuadRuped_Base, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_RB], "RB_", 8, AP_QuadRuped_Base, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_LB], "LB_", 9, AP_QuadRuped_Base, AP_QuadRuped_Params),
    AP_SUBGROUPINFO(leg_param[Leg_LF], "LF_", 10, AP_QuadRuped_Base, AP_QuadRuped_Params),

    // AP_SUBGROUPINFO(roll_pid, "_RLL_", 11, AP_QuadRuped_Base, AC_PID),
    // AP_SUBGROUPINFO(pitch_pid, "_PIT_", 12, AP_QuadRuped_Base, AC_PID),

    // AP_SUBGROUPINFO(diag_yaw_pid, "_DYAW_", 13, AP_QuadRuped_Base, AC_PID),
    // AP_SUBGROUPINFO(wave_yaw_pid, "_WYAW_", 14, AP_QuadRuped_Base, AC_PID),

    AP_GROUPEND
};

AP_QuadRuped_Base::AP_QuadRuped_Base(AP_AHRS_View& ahrs, AP_Motors& motors)
    : _ahrs(ahrs)     //_ahrs(ahrs)：将传入的 ahrs 指针赋给类的私有成员 _ahrs（姿态传感器接口）
    , _motors(motors) //_motors(motors)：将传入的 motors 指针赋给类的私有成员 _motors（电机控制接口）
{
    move_requested = false;
    max_yaw_rate   = radians(30.0f); // 限制最大30°/s

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_QuadRuped_Base::init(void)
{
    // 初始化腿部起始位置 (Initialize leg starting positions)
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // X坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * sin(角度)
        // Y坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * cos(角度)
        // Z坐标: Sys_Param.TIBIA_LEN (胫骨长度决定初始高度)
        endpoint_leg_pos[leg_index] = Vector3f(sinf(radians(START_COXA_ANGLE - leg_index * 90)) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               cosf(radians(START_COXA_ANGLE - leg_index * 90)) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               Sys_Param.TIBIA_LEN);
    }

    // 初始化腿部框架位置 (Initialize leg frame positions)
    // 计算每条腿的髋关节在机体坐标系中的位置 (Calculate hip joint position in body frame)
    // 使用与腿部位置相同的角度基准 (Using same angle reference as leg positions)
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // X坐标: Sys_Param.FRAME_LEN * sin(角度) - 决定前后位置
        // Y坐标: Sys_Param.FRAME_WIDTH * cos(角度) - 决定左右位置
        // Z坐标: 0 (髋关节与机体在同一平面)
        endpoint_leg_frame[leg_index] = Vector3f(sqrtf(2) * sinf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_LEN * 0.5f,
                                                 sqrtf(2) * cosf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_WIDTH * 0.5f,
                                                 0);
    }

    gait_init(); // 初始化四足机器人逆运动学控制器
}

void AP_QuadRuped_Base::calc_gait_sequence()
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

void AP_QuadRuped_Base::reset_leg()
{
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 }; // 重置位置坐标为原点
        gait_rot_z[moving_leg]   = 0;           // 重置旋转角度为0
    }
}

void AP_QuadRuped_Base::right_sleep_leg()
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

void AP_QuadRuped_Base::x_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        pwm_coxa  = leg_param[leg_index].COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped_Base::x_up_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        pwm_coxa  = leg_param[leg_index].COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * -75 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 60 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped_Base::hengxiang_up_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE;
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE;

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        if (leg_index == Leg_RB || leg_index == Leg_LF)
            pwm_coxa = leg_param[leg_index].COXA_DIR * -45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        else
            pwm_coxa = leg_param[leg_index].COXA_DIR * 45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        pwm_femur = leg_param[leg_index].FEMU_DIR * -75 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 60 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;
        servo_output_cmd[leg_index].y = pwm_femur;
        servo_output_cmd[leg_index].z = pwm_tibia;
    }
}

void AP_QuadRuped_Base::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 };

    const Vector3f endpoint_leg_angle_offset[LEG_ALL] = {
        { 45, 0, 0 },
        { -45, 0, 0 },
        { -135, 0, 0 },
        { -225, 0, 0 }
    }; // {髋关节角度， 股关节角度， 胫关节角度}   只有髋关节需要补偿

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        ansxyz                          = body_forward_kinematics(leg_index);
        endpoint_leg_angle[leg_index]   = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];
        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);
    }

    // if (servo_estimate()) {
    // start_time = AP_HAL::millis();

    calc_gait_sequence(); // 根据 throttle_travel 和 yaw_travel 更新步态相位（gait_step_now）    决定下一步的足端轨迹（gait_pos_xyz）

    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

Vector3f AP_QuadRuped_Base::leg_inverse_kinematics(Vector3f posxyz)
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

// 已知当前腿的位置（相对机体的位置 + gait目标位置），求在考虑机体姿态（俯仰、横滚、偏航）和重心偏移情况下，腿部在机体坐标系下的位置变化。
Vector3f AP_QuadRuped_Base::body_forward_kinematics(uint8_t leg_index)
{
    // gait_pos_xyz：步态生成的目标位置
    // endpoint_leg_pos：腿部初始展开位置（代码中初始化为(Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN)*sin(45°), ...）
    // endpoint_leg_frame：机体框架几何尺寸（机体的几何偏移）（如FRAME_LEN和FRAME_WIDTH）
    // totaldist_xyz：从机体几何中心出发，考虑步态变化、腿的展开长度和腿在机体上的安装位置后，腿末端相对于机体坐标系的实际位置。
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    // 添加重心偏移补偿
    totaldist_xyz -= centre_offset; // 减去 centre_offset 本质上是：腿末端的位置没变，但机体的参考点变了，所有腿的相对位置都需要根据新的参考中心重新表示。

    totaldist_xyz.z += z_travel;

    Quaternion quat = { 1, 0, 0, 0 };

    body_rot_xyz_deg.x = -radians(roll_travel);  // 横滚角（绕X轴）
    body_rot_xyz_deg.y = -radians(pitch_travel); // 俯仰角（绕Y轴）
    // hal.console->printf("pitch_travel_1=%f\n", pitch_travel);
    body_rot_xyz_deg.z = radians(gait_rot_z[leg_index]); // 偏航角（绕Z轴）   gait_rot_z[leg_index] 表示机器人在某一条腿的步态相位中，绕机身 Z 轴（垂直轴）的期望偏航角度补偿，最终影响该腿末端轨迹的旋转偏移。

    quat.from_euler(body_rot_xyz_deg);

    Vector3f totaldist_xyz_rot = quat * totaldist_xyz;

    return (totaldist_xyz_rot - endpoint_leg_frame[leg_index]);
}

void AP_QuadRuped_Base::set_centre_offset(float x, float y, float z = 0)
{
    centre_offset = Vector3f(x, y, z);
}

void AP_QuadRuped_Base::controller()
{
    float temp_rc;

    if (channel.throttle_channel != -1) {
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.throttle_channel - 1), (float)1000, (float)2000);
        if (temp_rc < 1550 && temp_rc > 1450) {
            temp_rc = 1500;
            set_centre_offset(0.0, 0.0, 0.0);
        }
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
}

void AP_QuadRuped_Base::balance_controller()
{
    float temp_rc;
    float target_roll  = 0;
    float target_pitch = 0;

    float current_yaw   = _ahrs.yaw;
    float current_pitch = _ahrs.pitch;
    float current_roll  = _ahrs.roll;

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
        if (temp_rc < 1550 && temp_rc > 1450) {
            temp_rc = 1500;
            // 遥控器信号归零时，重置目标偏航角为当前偏航角
            target_yaw = current_yaw;
        }
        // 将遥控器输入转换为目标角速度（-max_yaw_rate到+max_yaw_rate）
        float delta_yaw = (temp_rc - 1500) / 500.0f * 1.0f;
        // 计算角度误差
        target_yaw += delta_yaw;
        float yaw_error = wrap_180(current_yaw - target_yaw);

        if (fabsf(yaw_error) < radians(1.0f)) yaw_error = 0.0f;

        // 使用PID控制器计算角速度增量
        yaw_travel = yaw_pid.update_all(0, yaw_error, 1.0f / gait_hz);

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

void AP_QuadRuped_Base::output_leg_angle(void)
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

bool AP_QuadRuped_Base::hw_set_servo_cmd()
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
