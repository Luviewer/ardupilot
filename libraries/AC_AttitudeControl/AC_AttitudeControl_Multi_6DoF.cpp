#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi_6DoF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// 6自由度控制是从现有的多旋翼代码中提取的，将期望的角度视为推力角度，而不是飞行器姿态。
// 飞行器姿态随后单独设置，通常飞行器会保持0度的滚转和俯仰。
// 速率命令使飞行器表现得像一个普通的多旋翼飞行器。

// 运行最低级别的机体框架速率控制器，并将输出发送到电机
void AC_AttitudeControl_Multi_6DoF::rate_controller_run() {

    // 将当前偏移量传递给电机，并运行基类控制器
    // 电机需要偏移量来知道“向上”方向
    float roll_deg = roll_offset_deg; // 当前滚转偏移角度
    float pitch_deg = pitch_offset_deg; // 当前俯仰偏移角度

    // 如果启用了横向控制，始终将滚转角度指向正上方
    // 这可以防止由于目标姿态和真实姿态之间的误差导致的水平漂移
    if (lateral_enable) {
        roll_deg = degrees(AP::ahrs().get_roll()); // 获取当前滚转角度
    }
    // 如果启用了前向控制，始终将俯仰角度指向正上方
    if (forward_enable) {
        pitch_deg = degrees(AP::ahrs().get_pitch()); // 获取当前俯仰角度
    }

    // 设置电机的滚转和俯仰角度
    _motors.set_roll_pitch(roll_deg, pitch_deg);

    // 调用基类的方法以继续运行速率控制器
    AC_AttitudeControl_Multi::rate_controller_run();
}

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
    重写无人机姿态控制器的所有输入，将期望的角度转换为推力角并进行替换
*/

// 命令欧拉滚转和俯仰角以及欧拉航向速率，并进行角速度前馈和平滑处理
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    // 设置前向和横向的控制目标
    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    // 调用基类函数以输入欧拉角和航向速率
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// 命令欧拉滚转、俯仰和航向角，并进行角速度前馈和平滑处理
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) {
    // 设置前向和横向的控制目标
    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    // 调用基类函数以输入欧拉角和航向角
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}

// 命令推力向量和航向速率
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // 将推力向量转换为滚转角和俯仰角
    // 这会抵消使用推力向量控制的优势，但仍然可以正常工作
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    // 调用函数以输入目标的滚转角、俯仰角和航向速率
    input_euler_angle_roll_pitch_euler_rate_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_rate_cds);
}

// 命令推力向量、航向角和航向速率
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // 将推力向量转换为滚转角和俯仰角
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    // 注意这里我们丢弃了航向速率
    input_euler_angle_roll_pitch_yaw(degrees(angle_target.x) * 100.0f, degrees(angle_target.y) * 100.0f, heading_angle_cd, true);
}

void AC_AttitudeControl_Multi_6DoF::set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd)
{
    // 处理前向运动的控制逻辑
    if (forward_enable) { // 检查前向运动是否启用
        // 根据俯仰角计算前向推力，并设置给电机
        // euler_pitch_angle_cd * 0.01f 是为了将角度缩放到合适的范围
        _motors.set_forward(-sinf(radians(euler_pitch_angle_cd * 0.01f))); 
        // 将俯仰角更新为偏移量的100倍
        euler_pitch_angle_cd = pitch_offset_deg * 100.0f; 
    } else {
        // 如果前向运动未启用，设置前向推力为0
        _motors.set_forward(0.0f); 
        // 增加俯仰角的偏移量
        euler_pitch_angle_cd += pitch_offset_deg * 100.0f; 
    }
    // 确保俯仰角在[-180, 180]度范围内
    euler_pitch_angle_cd = wrap_180_cd(euler_pitch_angle_cd); 

    // 处理侧向运动的控制逻辑
    if (lateral_enable) { // 检查侧向运动是否启用
        // 根据滚转角计算侧向推力，并设置给电机
        // euler_roll_angle_cd * 0.01f 是为了将角度缩放到合适的范围
        _motors.set_lateral(sinf(radians(euler_roll_angle_cd * 0.01f))); 
        // 将滚转角更新为偏移量的100倍
        euler_roll_angle_cd = roll_offset_deg * 100.0f; 
    } else {
        // 如果侧向运动未启用，设置侧向推力为0
        _motors.set_lateral(0.0f); 
        // 增加滚转角的偏移量
        euler_roll_angle_cd += roll_offset_deg * 100.0f; 
    }
    // 确保滚转角在[-180, 180]度范围内
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd); 
}

/*
    all other input functions should zero thrust vectoring
    无人机的姿态控制器中重写所有输入函数，以实现零推力矢量
*/

// 命令欧拉偏航速率和俯仰角，滚转角在机体坐标系中指定
// （仅用于tailsitter quadplanes飞机）
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    // 将横向运动的速度设置为0.0f，确保无人机在控制过程中不进行横向移动
    _motors.set_lateral(0.0f);
    // 将前向运动的速度设置为0.0f，确保无人机在控制过程中不进行前向移动
    _motors.set_forward(0.0f);

    // 调用父类 AC_AttitudeControl_Multi 的同名函数
    // 将传入的参数传递给父类进行处理
    AC_AttitudeControl_Multi::input_euler_rate_yaw_euler_angle_pitch_bf_roll(plane_controls, euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// 命令欧拉角的滚转、俯仰和偏航速率，使用角速度前馈和控制平滑
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) {
    // 将横向运动的速度设置为0.0f，确保无人机在控制过程中不进行横向移动
    _motors.set_lateral(0.0f);
    // 将前向运动的速度设置为0.0f，确保无人机在控制过程中不进行前向移动
    _motors.set_forward(0.0f);

    // 调用父类 AC_AttitudeControl_Multi 的同名函数
    // 将传入的欧拉角滚转、俯仰和偏航速率传递给父类进行处理
    AC_AttitudeControl_Multi::input_euler_rate_roll_pitch_yaw(euler_roll_rate_cds, euler_pitch_rate_cds, euler_yaw_rate_cds);
}

// 命令一个角速度，使用角速度前馈和控制平滑
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    // 将横向运动的速度设置为0.0f，确保无人机在控制过程中不进行横向移动
    _motors.set_lateral(0.0f);
    // 将前向运动的速度设置为0.0f，确保无人机在控制过程中不进行前向移动
    _motors.set_forward(0.0f);

    // 调用父类 AC_AttitudeControl_Multi 的同名函数
    // 将传入的滚转、俯仰和偏航速率传递给父类进行处理
    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// 命令一个角速度，使用角速度前馈和控制平滑
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    // 将横向运动的速度设置为0.0f，确保无人机在控制过程中不进行横向移动
    _motors.set_lateral(0.0f);
    // 将前向运动的速度设置为0.0f，确保无人机在控制过程中不进行前向移动
    _motors.set_forward(0.0f);

    // 调用父类 AC_AttitudeControl_Multi 的同名函数
    // 将传入的滚转、俯仰和偏航速率传递给父类进行处理
    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_2(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// 命令一个角速度，使用速率环进行平滑控制，并集成速率误差稳定
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    // 将横向运动的速度设置为0.0f，确保无人机在控制过程中不进行横向移动
    _motors.set_lateral(0.0f);
    // 将前向运动的速度设置为0.0f，确保无人机在控制过程中不进行前向移动
    _motors.set_forward(0.0f);

    // 调用父类 AC_AttitudeControl_Multi 的同名函数
    // 将传入的滚转、俯仰和偏航速率传递给父类进行处理
    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_3(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// 在机体坐标系中命令一个角度变化（即角度步进）
void AC_AttitudeControl_Multi_6DoF::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) {
    // 将电机的横向和前向控制设置为零
    // 这可能是为了在进行角度调整时避免电机的意外操作
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    // 调用基类的方法处理角度步进输入
    // 这允许基类实现的逻辑处理姿态控制
    AC_AttitudeControl_Multi::input_angle_step_bf_roll_pitch_yaw(roll_angle_step_bf_cd, pitch_angle_step_bf_cd, yaw_angle_step_bf_cd);
}

// 输入四元数姿态控制的函数，支持前馈和光滑处理
// attitude_desired_quat: 在每个时间步长 (_dt) 中通过角速度的积分更新
void AC_AttitudeControl_Multi_6DoF::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body) {
    // 如果当前编译为 SITL 模式，触发 panic，提示此函数未实现
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("input_quaternion not implemented AC_AttitudeControl_Multi_6DoF");
#endif

    // 将电机的横向和前向控制设置为零
    // 这可能是为了在未实现的情况下避免电机的意外操作
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    // 调用基类的方法处理四元数输入
    // 这允许基类实现的逻辑处理姿态控制
    AC_AttitudeControl_Multi::input_quaternion(attitude_desired_quat, ang_vel_body);
}

// 单例模式的静态成员变量，指向该类的唯一实例
AC_AttitudeControl_Multi_6DoF *AC_AttitudeControl_Multi_6DoF::_singleton = nullptr;

#endif // AP_SCRIPTING_ENABLED
