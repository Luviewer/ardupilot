#include "AP_HexRuped.h"
#include "AP_HexRuped_Backend.h"
#include "AP_HexRuped_Defines.h"
#include <AP_HAL/AP_HAL.h>

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 右侧睡眠姿态 - 将机器人调整为右侧卧倒的睡眠姿态
void AP_HexRuped_Backend::right_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，计算睡眠姿态的关节角度
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 计算各关节的PWM值
        // 髋关节：45度
        pwm_coxa = leg_param.COXA_DIR * 45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 股关节：-65度（向后弯曲）
        pwm_femur = leg_param.FEMU_DIR * -65 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 胫关节：30度（支撑）
        pwm_tibia = leg_param.TIBI_DIR * 30 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
    servo_output_valid = true;
}

// X形睡眠姿态 - 将所有腿收拢，呈X形站立姿态
void AP_HexRuped_Backend::x_sleep_leg()
{
    default_claw_leg(0.0f);
}

// X形抬升睡眠姿态 - 将机器人调整为X形且抬高的姿态
void AP_HexRuped_Backend::x_up_sleep_leg()
{
    default_claw_leg(1.0f);
}

// 默认爪开合: 0 为地面展开(全关节 0°), 1 为飞行收起(股 -75°/胫 60°)
void AP_HexRuped_Backend::default_claw_leg(float close_norm)
{
    close_norm = constrain_float(close_norm, 0.0f, 1.0f);
    default_claw_joints(-75.0f * close_norm, 60.0f * close_norm);
}

void AP_HexRuped_Backend::default_claw_joints(float femur_deg, float tibia_deg)
{
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);
        servo_output_cmd[leg_index].x = leg_param.COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        servo_output_cmd[leg_index].y = leg_param.FEMU_DIR * femur_deg * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        servo_output_cmd[leg_index].z = leg_param.TIBI_DIR * tibia_deg * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
    }
    servo_output_valid = true;
}

// 横向抬升睡眠姿态 - 将机器人调整为横向展开且抬高的姿态
void AP_HexRuped_Backend::hengxiang_claw_leg(float angle_value)
{
    hengxiang_claw_joints(angle_value, 0.0f);
}

void AP_HexRuped_Backend::hengxiang_claw_joints(float femur_deg, float tibia_deg)
{
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 伺服 0° 是沿安装轴(前腿 +45°,后腿 -45°,中腿 0°)。横向爪要收到纯侧向:
        // 前腿往后转、后腿往前转,同侧前后符号相反;中腿已经朝向侧向,髋关节不动。
        float coxa_angle = 0.0f;
        switch (leg_index) {
        case AP_HEXRUPED_LEG_RF:
        case AP_HEXRUPED_LEG_LB:
            coxa_angle = 45.0f;
            break;
        case AP_HEXRUPED_LEG_RB:
        case AP_HEXRUPED_LEG_LF:
            coxa_angle = -45.0f;
            break;
        default:
            break;
        }

        servo_output_cmd[leg_index].x = leg_param.COXA_DIR * coxa_angle * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        servo_output_cmd[leg_index].y = leg_param.FEMU_DIR * femur_deg * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        servo_output_cmd[leg_index].z = leg_param.TIBI_DIR * tibia_deg * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
    }
    servo_output_valid = true;
}
