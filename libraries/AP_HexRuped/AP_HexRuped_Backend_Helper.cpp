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
    // 发送数据
    send_servo_cmd();
}

// X形睡眠姿态 - 将所有腿收拢，呈X形站立姿态
void AP_HexRuped_Backend::x_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置X形姿态
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 所有关节角度设为0度（中间位置）
        pwm_coxa  = leg_param.COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param.FEMU_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param.TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
    // 发送数据
    send_servo_cmd();
}

// X形抬升睡眠姿态 - 将机器人调整为X形且抬高的姿态
void AP_HexRuped_Backend::x_up_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置X形抬升姿态
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 计算各关节的PWM值
        // 髋关节：0度（保持中间位置）
        pwm_coxa = leg_param.COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 股关节：-75度（向上弯曲）
        pwm_femur = leg_param.FEMU_DIR * -75 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 胫关节：60度（形成支撑）
        pwm_tibia = leg_param.TIBI_DIR * 60 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
    // 发送数据
    send_servo_cmd();
}

// 横向抬升睡眠姿态 - 将机器人调整为横向展开且抬高的姿态
void AP_HexRuped_Backend::hengxiang_claw_leg(float angle_value)
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置横向抬升姿态
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 六足横向收纳：左右两侧镜像折叠，中腿与同侧前后腿保持一致。
        const float coxa_angle = is_right_leg(leg_index) ? 45.0f : -45.0f;
        pwm_coxa = leg_param.COXA_DIR * coxa_angle * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 股关节和胫关节统一设置
        pwm_femur = leg_param.FEMU_DIR * angle_value * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param.TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE; // 形成支撑

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
    // 发送数据
    send_servo_cmd();
}

// 纵向抬升睡眠姿态 - 将机器人调整为纵向展开且抬高的姿态
void AP_HexRuped_Backend::zhongxiang_claw_leg(float angle_value)
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置横向抬升姿态
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 六足纵向收纳方向与横向收纳相反。
        const float coxa_angle = is_right_leg(leg_index) ? -45.0f : 45.0f;
        pwm_coxa = leg_param.COXA_DIR * coxa_angle * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 股关节和胫关节统一设置
        pwm_femur = leg_param.FEMU_DIR * angle_value * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param.TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE; // 形成支撑

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
    // 发送数据
    send_servo_cmd();
}
