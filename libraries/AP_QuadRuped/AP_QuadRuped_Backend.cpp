#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped.h"
#include "AP_QuadRuped_Defines.h"

// 构造函数
AP_QuadRuped_Backend::AP_QuadRuped_Backend(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : _frontend(frontend)
    , _state(state)
    , _ahrs(ahrs)
    , _motors(motors)
{
}

bool AP_QuadRuped_Backend::init()
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 初始化腿部起始位置 (Initialize leg starting positions)
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
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
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // X坐标: Sys_Param.FRAME_LEN * sin(角度) - 决定前后位置
        // Y坐标: Sys_Param.FRAME_WIDTH * cos(角度) - 决定左右位置
        // Z坐标: 0 (髋关节与机体在同一平面)
        endpoint_leg_frame[leg_index] = Vector3f(sqrtf(2) * sinf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_LEN * 0.5f,
                                                 sqrtf(2) * cosf(radians(START_COXA_ANGLE - leg_index * 90)) * Sys_Param.FRAME_WIDTH * 0.5f,
                                                 0);
    }

    gait_init(); // 初始化四足机器人逆运动学控制器

    return true;
}

// 重置腿部位置 - 将所有腿恢复到初始状态
void AP_QuadRuped_Backend::reset_leg()
{
    // 遍历所有腿，重置其位置和旋转
    for (uint8_t moving_leg = 0; moving_leg < AP_QUADRUPED_LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 }; // 重置位置坐标为原点（相对于初始位置）
        gait_rot_z[moving_leg]   = 0;           // 重置旋转角度为0（无旋转）
    }
}

// 计算步态序列 - 判断是否需要移动并执行相应动作
void AP_QuadRuped_Backend::calc_gait_sequence()
{
    const float travel_dz = 5.0f / 500.0f; // 移动死区阈值，防止微小抖动

    // 判断是否有移动请求（前进/后退或旋转）
    if ((fabsf(_frontend.get_throttle_x()) > travel_dz) || (fabsf(_frontend.get_throttle_y()) > travel_dz) || (fabsf(_frontend.get_yaw_rate()) > travel_dz / 2))
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

// 腿部逆运动学计算
Vector3f AP_QuadRuped_Backend::leg_inverse_kinematics(Vector3f posxyz)
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 存储计算出的关节角度（度）
    Vector3f leg_deg = { 0, 0, 0 };

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

#ifdef ENABLE_LEG_ALPHA_COMP
    {
        const float alpha = degrees(atan2f(LEG_ALPHA_B, LEG_ALPHA_A));
        leg_deg.y -= alpha;
        leg_deg.z += 90.0f - alpha;
    }
#endif
    return leg_deg; // 返回{髋关节, 股关节, 胫关节}角度
}

// 机体正向运动学 - 计算考虑机体姿态和重心偏移后的腿部末端位置
Vector3f AP_QuadRuped_Backend::body_forward_kinematics(uint8_t leg_index)
{
    // 计算腿部末端在机体坐标系中的总位置
    // gait_pos_xyz：步态生成的目标位置（相对于初始位置的偏移）
    // endpoint_leg_pos：腿部初始展开位置（髋关节+股关节长度在45度方向的投影）
    // endpoint_leg_frame：机体框架几何尺寸（腿在机体上的安装位置）
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    // 添加重心偏移补偿
    // 减去 centre_offset 是因为：当重心偏移时，机体参考点改变，所有腿的相对位置需要重新计算
    // totaldist_xyz -= centre_offset;
    // totaldist_xyz -= centre_offset_move;

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

// 主逆运动学计算 - 计算所有腿的关节角度
void AP_QuadRuped_Backend::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 }; // 临时变量，存储腿部末端位置

    // 腿部角度偏移补偿 - 由于机械安装误差，每条腿需要不同的角度补偿
    const Vector3f endpoint_leg_angle_offset[AP_QUADRUPED_LEG_ALL] = {
        { 45, 0, 0 },   // 右前腿：髋关节补偿45度
        { -45, 0, 0 },  // 右后腿：髋关节补偿-45度
        { -135, 0, 0 }, // 左后腿：髋关节补偿-135度
        { -225, 0, 0 }  // 左前腿：髋关节补偿-225度
    }; // 格式：{髋关节角度，股关节角度，胫关节角度} - 只有髋关节需要补偿

    // 遍历所有腿，计算逆运动学
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
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
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

// 主控制器 - 处理遥控器输入并转换为运动指令
void AP_QuadRuped_Backend::main_radio_controller()
{
    const AP_QuadRuped_CHANNEL_Params& channel = _frontend.get_channel_params();

    // 处理油门通道（前进/后退）
    if (channel.throttle_x_channel != -1) {
        // 将遥控器输入转换为前进/后退行程
        throttle_x_travel = _frontend.get_throttle_x() * channel.throttle_x_max;
    } else {
        throttle_x_travel = 0; // 无通道配置时保持静止
    }

    // 处理横移通道（向右为正，向左为负）
    if (channel.throttle_y_channel != -1) {
        throttle_y_travel = _frontend.get_throttle_y() * channel.throttle_y_max;
    } else {
        throttle_y_travel = 0.0f;
    }

    z_travel        = channel.body_height; // 默认高度
    leg_lift_height = channel.left_lift;   // 抬腿高度
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// 输出腿部关节角度 - 将计算出的关节角度转换为PWM信号
void AP_QuadRuped_Backend::output_leg_angle(void)
{
    uint16_t pwm_coxa;  // 髋关节PWM值
    uint16_t pwm_femur; // 股关节PWM值
    uint16_t pwm_tibia; // 胫关节PWM值

    // 遍历所有腿，计算每个关节的PWM值
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_QuadRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 将角度转换为PWM值
        // 公式：PWM = 方向系数 × 角度 × PWM范围/角度范围 + 中间值
        pwm_coxa  = leg_param.COXA_DIR * endpoint_leg_angle[leg_index].x * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_femur = leg_param.FEMU_DIR * endpoint_leg_angle[leg_index].y * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;
        pwm_tibia = leg_param.TIBI_DIR * endpoint_leg_angle[leg_index].z * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG + LEG_MOTOR_PWM_MIDDLE;

        // 存储PWM命令到输出数组
        servo_output_cmd[leg_index].x = pwm_coxa;  // 髋关节PWM
        servo_output_cmd[leg_index].y = pwm_femur; // 股关节PWM
        servo_output_cmd[leg_index].z = pwm_tibia; // 胫关节PWM
    }
}

// 硬件伺服命令设置 - 通过CAN总线发送PWM控制信号到舵机
bool AP_QuadRuped_Backend::send_servo_cmd()
{
    com_usl_ServoCmd msg {}; // 创建DroneCAN伺服控制消息结构体
    msg.cmd.len = 12;        // 设置消息长度(4条腿×3个关节 = 12个数据)

    // 遍历所有腿部，准备发送数据
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_QuadRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 填充CAN消息数据（添加偏移补偿）
        msg.cmd.data[leg_index * 3 + 0] = servo_output_cmd[leg_index].x + leg_param.COXA_OFS; // 髋关节
        msg.cmd.data[leg_index * 3 + 1] = servo_output_cmd[leg_index].y + leg_param.FEMU_OFS; // 股关节
        msg.cmd.data[leg_index * 3 + 2] = servo_output_cmd[leg_index].z + leg_param.TIBI_OFS; // 胫关节

        // 同时设置PWM输出通道（直接输出模式）
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_coxa + leg_index * 3),
                                     servo_output_cmd[leg_index].x + leg_param.COXA_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_femu + leg_index * 3),
                                     servo_output_cmd[leg_index].y + leg_param.FEMU_OFS);
        SRV_Channels::set_output_pwm((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_legmotor_rf_tibi + leg_index * 3),
                                     servo_output_cmd[leg_index].z + leg_param.TIBI_OFS);
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

// 辅助函数：角度转PWM
uint16_t AP_QuadRuped_Backend::radians_to_pwm(float angle_rad)
{
    // 假设PWM范围1000-2000对应-90到90度
    float angle_deg = degrees(angle_rad);
    return uint16_t(1500 + (angle_deg / 90.0f) * 500);
}