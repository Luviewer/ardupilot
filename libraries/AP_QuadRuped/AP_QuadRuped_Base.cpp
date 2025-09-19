#include "AP_QuadRuped_Base.h"
#include "AP_DroneCAN/AP_DroneCAN.h"
#include "AP_QuadRuped_Params.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// 腿部电机参数定义
#define LEG_MOTOR_MAX_DEG       (120)  // 腿部电机最大角度（度）
#define LEG_MOTOR_MAX_PWM       (500)  // 腿部电机最大PWM值
#define LEG_MOTOR_PWM_MIDDLE    (1500) // 腿部电机中间PWM值（1500μs）

// 默认参数定义
#define LIFT_HEIGHT_DEFAULT     50.0f  // 默认抬腿高度（mm）
#define SPEED_HZ_DEFAULT        25.0f  // 默认步态频率（Hz）
#define MAX_THROTTLE_X_DEFAULT  200.0f // 默认最大油门行程（mm）
#define MAX_THROTTLE_Y_DEFAULT  200.0f // 默认最大油门行程（mm）
#define GAIT_STEP_TOTAL_DEFAULT 24     // 默认步态总步数

#define START_COXA_ANGLE        45 // 起始髋关节角度（度）

// 参数定义表 - 用于参数系统配置
const AP_Param::GroupInfo AP_QuadRuped_Base::var_info[] = {

    // 基本运动参数
    AP_GROUPINFO("Hz", 2, AP_QuadRuped_Base, gait_hz, SPEED_HZ_DEFAULT), // 步态频率

    // 行程参数
    AP_GROUPINFO("THR_X", 3, AP_QuadRuped_Base, throttle_x_max, MAX_THROTTLE_X_DEFAULT),  // 最大x方向油门行程
    AP_GROUPINFO("THR_Y", 4, AP_QuadRuped_Base, throttle_y_max, MAX_THROTTLE_Y_DEFAULT),  // 最大y方向油门行程
    AP_GROUPINFO("STEP", 5, AP_QuadRuped_Base, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数

    // 系统参数组
    AP_SUBGROUPINFO(Sys_Param, "SYS", 6, AP_QuadRuped_Base, AP_QuadRuped_SYS_Params), // 系统参数

    // 遥控通道参数组
    AP_SUBGROUPINFO(channel, "CH_", 7, AP_QuadRuped_Base, AP_QuadRuped_CHANNEL_Params), // 通道配置

    // 四条腿的参数组
    AP_SUBGROUPINFO(leg_param[Leg_RF], "RF_", 8, AP_QuadRuped_Base, AP_QuadRuped_Params),  // 右前腿参数
    AP_SUBGROUPINFO(leg_param[Leg_RB], "RB_", 9, AP_QuadRuped_Base, AP_QuadRuped_Params),  // 右后腿参数
    AP_SUBGROUPINFO(leg_param[Leg_LB], "LB_", 10, AP_QuadRuped_Base, AP_QuadRuped_Params), // 左后腿参数
    AP_SUBGROUPINFO(leg_param[Leg_LF], "LF_", 11, AP_QuadRuped_Base, AP_QuadRuped_Params), // 左前腿参数

    // PID控制器参数（已注释）
    // AP_SUBGROUPINFO(roll_pid, "_RLL_", 11, AP_QuadRuped_Base, AC_PID),                // 横滚PID
    // AP_SUBGROUPINFO(pitch_pid, "_PIT_", 12, AP_QuadRuped_Base, AC_PID),                // 俯仰PID

    // 偏航PID控制器参数（已注释）
    // AP_SUBGROUPINFO(diag_yaw_pid, "_DYAW_", 13, AP_QuadRuped_Base, AC_PID),            // 对角步态偏航PID
    // AP_SUBGROUPINFO(wave_yaw_pid, "_WYAW_", 14, AP_QuadRuped_Base, AC_PID),            // 波浪步态偏航PID

    AP_GROUPEND
};

// 构造函数 - 初始化四足机器人基类
AP_QuadRuped_Base::AP_QuadRuped_Base(AP_AHRS_View& ahrs, AP_Motors& motors)
    : _ahrs(ahrs)     // 初始化姿态航向参考系统接口
    , _motors(motors) // 初始化电机控制接口
{
    // 初始化成员变量
    move_requested = false;          // 移动请求标志，初始为静止
    max_yaw_rate   = radians(30.0f); // 最大偏航角速度限制为30度/秒

    // 设置参数默认值
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

// 计算步态序列 - 判断是否需要移动并执行相应动作
void AP_QuadRuped_Base::calc_gait_sequence()
{
    const float travel_dz = 5; // 移动死区阈值，防止微小抖动

    // 判断是否有移动请求（前进/后退或旋转）
    if ((fabsf(throttle_x_travel) > travel_dz) || (fabsf(throttle_y_travel) > travel_dz) || (fabsf(yaw_travel) > travel_dz / 2))
        move_requested = true; // 需要移动
    else
        move_requested = false; // 保持静止

    // 根据移动请求执行相应动作
    if (move_requested == true) {
        update_leg(); // 更新腿部运动（执行步态）
    } else {
        reset_leg(); // 重置腿部到初始位置
    }
}

// 重置腿部位置 - 将所有腿恢复到初始状态
void AP_QuadRuped_Base::reset_leg()
{
    // 遍历所有腿，重置其位置和旋转
    for (uint8_t moving_leg = 0; moving_leg < LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 }; // 重置位置坐标为原点（相对于初始位置）
        gait_rot_z[moving_leg]   = 0;           // 重置旋转角度为0（无旋转）
    }
}

// 右侧睡眠姿态 - 将机器人调整为右侧卧倒的睡眠姿态
void AP_QuadRuped_Base::right_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，计算睡眠姿态的关节角度
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 计算各关节的PWM值
        // 髋关节：45度
        pwm_coxa = leg_param[leg_index].COXA_DIR * 45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 股关节：-65度（向后弯曲）
        pwm_femur = leg_param[leg_index].FEMU_DIR * -65 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 胫关节：30度（支撑）
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 30 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// X形睡眠姿态 - 将所有腿收拢，呈X形站立姿态
void AP_QuadRuped_Base::x_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置X形姿态
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 所有关节角度设为0度（中间位置）
        pwm_coxa  = leg_param[leg_index].COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// X形抬升睡眠姿态 - 将机器人调整为X形且抬高的姿态
void AP_QuadRuped_Base::x_up_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    // 遍历所有腿，设置X形抬升姿态
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 计算各关节的PWM值
        // 髋关节：0度（保持中间位置）
        pwm_coxa = leg_param[leg_index].COXA_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 股关节：-75度（向上弯曲）
        pwm_femur = leg_param[leg_index].FEMU_DIR * -75 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        // 胫关节：60度（形成支撑）
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 60 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// 横向抬升睡眠姿态 - 将机器人调整为横向展开且抬高的姿态
void AP_QuadRuped_Base::hengxiang_up_sleep_leg()
{
    uint16_t pwm_coxa  = LEG_MOTOR_PWM_MIDDLE; // 髋关节PWM值
    uint16_t pwm_femur = LEG_MOTOR_PWM_MIDDLE; // 股关节PWM值
    uint16_t pwm_tibia = LEG_MOTOR_PWM_MIDDLE; // 胫关节PWM值

    float angle_value = (float)(hal.rcin->read(CH_7) - 1500) / 500.0f * 90.0f;

    // 遍历所有腿，设置横向抬升姿态
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 根据腿的位置设置不同的髋关节角度
        if (leg_index == Leg_RB || leg_index == Leg_LF) // 右后腿和左前腿
            pwm_coxa = leg_param[leg_index].COXA_DIR * -45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        else // 右前腿和左后腿
            pwm_coxa = leg_param[leg_index].COXA_DIR * 45 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 股关节和胫关节统一设置
        pwm_femur = leg_param[leg_index].FEMU_DIR * angle_value * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * 0 * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE; // 形成支撑

        // 将计算出的PWM值存入输出命令数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// 主逆运动学计算 - 计算所有腿的关节角度
void AP_QuadRuped_Base::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 }; // 临时变量，存储腿部末端位置

    // 腿部角度偏移补偿 - 由于机械安装误差，每条腿需要不同的角度补偿
    const Vector3f endpoint_leg_angle_offset[LEG_ALL] = {
        { 45, 0, 0 },   // 右前腿：髋关节补偿45度
        { -45, 0, 0 },  // 右后腿：髋关节补偿-45度
        { -135, 0, 0 }, // 左后腿：髋关节补偿-135度
        { -225, 0, 0 }  // 左前腿：髋关节补偿-225度
    }; // 格式：{髋关节角度，股关节角度，胫关节角度} - 只有髋关节需要补偿

    // 遍历所有腿，计算逆运动学
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 1. 计算腿部末端在机体坐标系中的位置
        ansxyz = body_forward_kinematics(leg_index);
        // 2. 计算逆运动学得到关节角度，并加上补偿值
        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz) + endpoint_leg_angle_offset[leg_index];
        // 3. 将髋关节角度规范到[-180, 180]范围内
        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);
    }

    // 计算步态序列
    // 根据 throttle_travel（前进/后退）和 yaw_travel（旋转）更新步态相位
    // 决定下一步的足端轨迹
    calc_gait_sequence();

    // 保存当前关节角度到上一时刻变量
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

// 腿部逆运动学计算 - 根据腿部末端位置计算关节角度
Vector3f AP_QuadRuped_Base::leg_inverse_kinematics(Vector3f posxyz)
{
    Vector3f leg_deg = { 0, 0, 0 }; // 存储计算出的关节角度（度）

    // 1. 计算髋关节角度（绕Z轴旋转）
    leg_deg.x = -degrees(atan2f(posxyz.x, posxyz.y)); // 使用atan2计算XY平面内的角度

    // 2. 计算从髋关节到末端在XY平面的投影距离
    float trueX = sqrtf(posxyz.x * posxyz.x + posxyz.y * posxyz.y) - Sys_Param.COXA_LEN; // 减去髋关节长度

    // 3. 计算从股关节到末端的空间距离
    float im = sqrtf(trueX * trueX + posxyz.z * posxyz.z);

    // 4. 计算股关节角度（使用余弦定理）
    float q1 = atan2f(trueX, posxyz.z); // 股关节与末端连线与垂直方向的夹角
    // 使用余弦定理计算股关节角度
    float d1  = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN + im * im;
    float d2  = 2 * Sys_Param.FEMUR_LEN * im;
    float q2  = acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f)); // 约束在[-1,1]范围内防止数值错误
    leg_deg.y = -(degrees(q1 + q2) - 90);                            // 计算股关节角度并调整坐标系

    // 5. 计算胫关节角度（使用余弦定理）
    d1        = Sys_Param.FEMUR_LEN * Sys_Param.FEMUR_LEN - im * im + Sys_Param.TIBIA_LEN * Sys_Param.TIBIA_LEN;
    d2        = 2 * Sys_Param.TIBIA_LEN * Sys_Param.FEMUR_LEN;
    leg_deg.z = -(degrees(acosf(constrain_value(float(d1 / d2), -1.0f, 1.0f))) - 90); // 计算胫关节角度

    ////////////////////////////////
    const float a = 55.82;
    const float b = 48.58;

    float alpha = degrees(atan2f(b, a));
    leg_deg.y += alpha;
    leg_deg.z -= 90 - alpha;
    return leg_deg; // 返回{髋关节, 股关节, 胫关节}角度
}

// 机体正向运动学 - 计算考虑机体姿态和重心偏移后的腿部末端位置
Vector3f AP_QuadRuped_Base::body_forward_kinematics(uint8_t leg_index)
{
    // 计算腿部末端在机体坐标系中的总位置
    // gait_pos_xyz：步态生成的目标位置（相对于初始位置的偏移）
    // endpoint_leg_pos：腿部初始展开位置（髋关节+股关节长度在45度方向的投影）
    // endpoint_leg_frame：机体框架几何尺寸（腿在机体上的安装位置）
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    // 添加重心偏移补偿
    // 减去 centre_offset 是因为：当重心偏移时，机体参考点改变，所有腿的相对位置需要重新计算
    totaldist_xyz -= centre_offset;
    totaldist_xyz -= centre_offset_move;

    // 添加Z轴高度偏移（机体升降）
    totaldist_xyz.z += z_travel;

    // 创建四元数用于旋转变换
    Quaternion quat = { 1, 0, 0, 0 };

    // 设置机体旋转角度
    body_rot_xyz_deg.x = -radians(roll_travel);  // 横滚角（绕X轴）- 取负值是因为坐标系定义
    body_rot_xyz_deg.y = -radians(pitch_travel); // 俯仰角（绕Y轴）- 取负值是因为坐标系定义
    // 偏航角（绕Z轴）- 来自步态生成的旋转补偿，影响腿末端轨迹的旋转偏移
    body_rot_xyz_deg.z = radians(gait_rot_z[leg_index]);

    // 根据欧拉角创建四元数
    quat.from_euler(body_rot_xyz_deg);

    // 应用旋转变换
    Vector3f totaldist_xyz_rot = quat * totaldist_xyz;

    // 返回相对于髋关节的位置（减去机体框架偏移）
    return (totaldist_xyz_rot - endpoint_leg_frame[leg_index]);
}

// 设置重心偏移 - 调整机器人的重心位置
void AP_QuadRuped_Base::set_centre_offset(float x, float y, float z = 0)
{
    centre_offset = Vector3f(x, y, z); // 设置X、Y、Z三个方向的偏移量
}

// 主控制器 - 处理遥控器输入并转换为运动指令
void AP_QuadRuped_Base::controller()
{
    float temp_rc; // 临时存储遥控器值

    // 处理油门通道（前进/后退）
    if (channel.throttle_x_channel != -1) {
        // 读取遥控器输入并约束在[1000, 2000]范围内
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.throttle_x_channel - 1), (float)1000, (float)2000);
        // 死区处理：当摇杆在中间位置附近时，认为无输入
        if (temp_rc < 1550 && temp_rc > 1450) {
            temp_rc = 1500;
            set_centre_offset(0.0, 0.0, 0.0); // 重置重心偏移
        }
        // 将遥控器输入转换为前进/后退行程
        throttle_x_travel = (temp_rc - 1500) / 500.0f * throttle_x_max;
    } else {
        throttle_x_travel = 0; // 无通道配置时保持静止
    }

    // 处理横移通道（向右为正，向左为负）
    if (channel.throttle_y_channel != -1) {
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.throttle_y_channel - 1), 1000.0f, 2000.0f);
        // 死区：1450~1550
        if (temp_rc < 1550 && temp_rc > 1450) {
            temp_rc = 1500;
        }
        // 将遥控器输入转换为横移行程（mm）
        // 建议与 throttle_travel 一样的线性映射
        throttle_y_travel = (temp_rc - 1500) / 500.0f * throttle_y_max;
    } else {
        throttle_y_travel = 0.0f;
    }

    // // 处理高度通道（机体升降）
    // if (channel.height_channel != -1) {
    //     // 读取遥控器输入
    //     temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.height_channel - 1), (float)1000, (float)2000);
    //     // 转换为高度偏移：范围-50mm到+70mm
    //     z_travel = (temp_rc - 1500) / 500.0f * 120.0f - 50;
    // } else {
    // }
    z_travel = (float)channel.height_channel; // 默认高度

    if (channel.lift_channel != -1) {
        // 读取遥控器输入
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.lift_channel - 1), (float)1000, (float)2000);
        // 转换为高度偏移：范围-50mm到+70mm
        leg_lift_height = (temp_rc - 1500) / 10.0f + 25;
    } else {
        leg_lift_height = 25; // 默认高度
    }

    leg_lift_height = (float)channel.lift_channel; // 默认高度
}

// 平衡控制器 - 处理姿态控制和重心偏移
void AP_QuadRuped_Base::balance_controller()
{
    float temp_rc;          // 临时存储遥控器值
    float target_roll  = 0; // 目标横滚角
    float target_pitch = 0; // 目标俯仰角

    // 获取当前姿态
    float current_yaw   = _ahrs.yaw;   // 当前偏航角
    float current_pitch = _ahrs.pitch; // 当前俯仰角
    float current_roll  = _ahrs.roll;  // 当前横滚角

    // 处理横滚通道
    if (channel.roll_channel != -1) {
        // 读取遥控器输入
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.roll_channel - 1), (float)1000, (float)2000);
        // 转换为目标横滚角（±15度范围）
        target_roll = (temp_rc - 1500) / 500.0f * 15.0f;
        // 计算横滚角误差并规范到[-180, 180]度
        float roll_error = wrap_180(current_roll - target_roll);
        // 使用PID控制器计算横滚补偿
        roll_travel = roll_pid.update_all(0, roll_error, 1.0f / gait_hz);
    } else {
        roll_travel = 0; // 无通道配置时无补偿
    }

    // 处理俯仰通道
    if (channel.pitch_channel != -1) {
        // 读取遥控器输入
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.pitch_channel - 1), (float)1000, (float)2000);
        // 转换为目标俯仰角（±15度范围）
        target_pitch = (temp_rc - 1500) / 500.0f * 15.0f;
        // 计算俯仰角误差并规范到[-180, 180]度
        float pitch_error = wrap_180(current_pitch - target_pitch);
        // 使用PID控制器计算俯仰补偿
        pitch_travel = pitch_pid.update_all(0, pitch_error, 1.0f / gait_hz);
    } else {
        pitch_travel = 0; // 无通道配置时无补偿
    }

    // 处理偏航通道
    if (channel.yaw_channel != -1) {
        // 读取遥控器输入
        temp_rc = constrain_value((float)rc().RC_Channels::get_radio_in(channel.yaw_channel - 1), (float)1000, (float)2000);
        // 死区处理
        if (temp_rc < 1550 && temp_rc > 1450) {
            temp_rc = 1500;
            // 遥控器信号归零时，重置目标偏航角为当前偏航角
            target_yaw = current_yaw;
        }
        // 将遥控器输入转换为角速度增量
        float delta_yaw = (temp_rc - 1500) / 500.0f * 1.0f;
        // 更新目标偏航角
        target_yaw += delta_yaw;
        // 计算偏航角误差
        float yaw_error = wrap_180(current_yaw - target_yaw);

        // 小角度误差处理
        if (fabsf(yaw_error) < radians(1.0f)) yaw_error = 0.0f;

        // 使用PID控制器计算偏航补偿
        yaw_travel = yaw_pid.update_all(0, yaw_error, 1.0f / gait_hz);
    } else {
        yaw_travel = 0; // 无通道配置时无补偿
    }

    // 处理重心X轴偏移通道
    if (channel.centre_offset_x_channel != -1) {
        float val = constrain_value((float)rc().get_radio_in(channel.centre_offset_x_channel - 1), (float)1000, (float)2000);
        // 死区检测（±25μs）
        if (val > 1475 && val < 1525) {
            val = 1500;
        }
        // 转换为X轴偏移（±100mm范围）
        centre_offset.x = (val - 1500) / 500.0f * 100.0f;
    } else {
        centre_offset.x = 0;
    }

    // 处理重心Y轴偏移通道
    if (channel.centre_offset_y_channel != -1) {
        float val = constrain_value((float)rc().get_radio_in(channel.centre_offset_y_channel - 1), (float)1000, (float)2000);
        // 死区检测（±25μs）
        if (val > 1475 && val < 1525) {
            val = 1500;
        }
        // 转换为Y轴偏移（±100mm范围）
        centre_offset.y = (val - 1500) / 500.0f * 100.0f;
    } else {
        centre_offset.y = 0;
    }
}

// 输出腿部关节角度 - 将计算出的关节角度转换为PWM信号
void AP_QuadRuped_Base::output_leg_angle(void)
{
    uint16_t pwm_coxa;  // 髋关节PWM值
    uint16_t pwm_femur; // 股关节PWM值
    uint16_t pwm_tibia; // 胫关节PWM值

    // 遍历所有腿，计算每个关节的PWM值
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 将角度转换为PWM值
        // 公式：PWM = 方向系数 × 角度 × PWM范围/角度范围 + 中间值
        pwm_coxa  = leg_param[leg_index].COXA_DIR * endpoint_leg_angle[leg_index].x * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param[leg_index].FEMU_DIR * endpoint_leg_angle[leg_index].y * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param[leg_index].TIBI_DIR * endpoint_leg_angle[leg_index].z * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 存储PWM命令到输出数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// 硬件伺服命令设置 - 通过CAN总线发送PWM控制信号到舵机
bool AP_QuadRuped_Base::hw_set_servo_cmd()
{
    com_usl_ServoCmd msg {}; // 创建DroneCAN伺服控制消息结构体
    msg.cmd.len = 12;        // 设置消息长度(4条腿×3个关节 = 12个数据)

    // 遍历所有腿部，准备发送数据
    for (uint8_t leg_index = 0; leg_index < LEG_ALL; leg_index++) {
        // 填充CAN消息数据（添加偏移补偿）
        msg.cmd.data[leg_index * 3 + 0] = servo_output_cmd[leg_index].x + leg_param[leg_index].COXA_OFS; // 髋关节
        msg.cmd.data[leg_index * 3 + 1] = servo_output_cmd[leg_index].y + leg_param[leg_index].FEMU_OFS; // 股关节
        msg.cmd.data[leg_index * 3 + 2] = servo_output_cmd[leg_index].z + leg_param[leg_index].TIBI_OFS; // 胫关节

        // 同时设置PWM输出通道（直接输出模式）
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_coxa + leg_index * 3),
                                     servo_output_cmd[leg_index].x + leg_param[leg_index].COXA_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_femu + leg_index * 3),
                                     servo_output_cmd[leg_index].y + leg_param[leg_index].FEMU_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_tibi + leg_index * 3),
                                     servo_output_cmd[leg_index].z + leg_param[leg_index].TIBI_OFS);
    }

    // 在所有可用的CAN总线接口上广播伺服控制命令
    // 获取CAN驱动数量
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    // 发送成功标志
    bool ok = false;

    // 遍历所有CAN接口
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto* dronecan = AP_DroneCAN::get_dronecan(i); // 获取第i个CAN驱动实例
        if (dronecan != nullptr) {
            // 尝试广播消息，使用|=确保只要有一个接口成功就返回true
            ok |= dronecan->com_usl_servocmd.broadcast(msg); // 发送伺服控制命令
        }
    }
    return ok; // 返回广播结果
}
