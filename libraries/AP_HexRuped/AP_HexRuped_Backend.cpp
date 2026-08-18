#include "AP_HexRuped_Backend.h"
#include "AP_HexRuped.h"
#include "AP_HexRuped_Defines.h"

static uint16_t angle_to_servo_pwm(float angle_deg, float direction)
{
    const float pwm = direction * angle_deg * LEG_MOTOR_MAX_PWM / LEG_MOTOR_MAX_DEG +
                      LEG_MOTOR_PWM_MIDDLE;
    return uint16_t(constrain_float(pwm, LEG_MOTOR_PWM_MIN, LEG_MOTOR_PWM_MAX));
}

static uint16_t apply_servo_offset(uint16_t pwm, float offset)
{
    return uint16_t(constrain_float(float(pwm) + offset,
                                    LEG_MOTOR_PWM_MIN,
                                    LEG_MOTOR_PWM_MAX));
}

// 构造函数
AP_HexRuped_Backend::AP_HexRuped_Backend(AP_HexRuped& frontend, AP_HexRuped::HexRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : _frontend(frontend)
    , _state(state)
    , _ahrs(ahrs)
    , _motors(motors)
{
}

float AP_HexRuped_Backend::get_leg_mount_yaw_deg(uint8_t leg_index) const
{
    const AP_HexRuped_SYS_Params& sys = _frontend.get_sys_params();
    switch (leg_index) {
    case AP_HEXRUPED_LEG_RF: return sys.FRONT_YAW;
    case AP_HEXRUPED_LEG_RB: return sys.REAR_YAW;
    case AP_HEXRUPED_LEG_LB: return -180.0f - sys.REAR_YAW;
    case AP_HEXRUPED_LEG_LF: return 180.0f - sys.FRONT_YAW;
    case AP_HEXRUPED_LEG_RM: return sys.MIDDLE_YAW;
    case AP_HEXRUPED_LEG_LM: return 180.0f - sys.MIDDLE_YAW;
    default: return 0.0f;
    }
}

bool AP_HexRuped_Backend::is_right_leg(uint8_t leg_index) const
{
    return leg_index == AP_HEXRUPED_LEG_RF ||
           leg_index == AP_HEXRUPED_LEG_RB ||
           leg_index == AP_HEXRUPED_LEG_RM;
}

Vector3f AP_HexRuped_Backend::get_leg_frame_position(uint8_t leg_index) const
{
    const AP_HexRuped_SYS_Params& sys = _frontend.get_sys_params();
    const float half_length = sys.FRAME_LEN * 0.5f;
    const float half_width = sys.FRAME_WIDTH * 0.5f;

    switch (leg_index) {
    case AP_HEXRUPED_LEG_RF: return { half_length, half_width, 0.0f };
    case AP_HEXRUPED_LEG_RB: return { -half_length, half_width, 0.0f };
    case AP_HEXRUPED_LEG_LB: return { -half_length, -half_width, 0.0f };
    case AP_HEXRUPED_LEG_LF: return { half_length, -half_width, 0.0f };
    case AP_HEXRUPED_LEG_RM: return { sys.MIDDLE_X, half_width, 0.0f };
    case AP_HEXRUPED_LEG_LM: return { sys.MIDDLE_X, -half_width, 0.0f };
    default: return {};
    }
}

bool AP_HexRuped_Backend::init()
{
    const AP_HexRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 初始化六条腿的起始位置。安装角显式定义，避免假设等角度分布。
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // X坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * sin(角度)
        // Y坐标: (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN) * cos(角度)
        // Z坐标: Sys_Param.TIBIA_LEN (胫骨长度决定初始高度)
        const float mount_yaw = radians(get_leg_mount_yaw_deg(leg_index));
        endpoint_leg_pos[leg_index] = Vector3f(sinf(mount_yaw) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               cosf(mount_yaw) * (Sys_Param.COXA_LEN + Sys_Param.FEMUR_LEN),
                                               Sys_Param.TIBIA_LEN);
    }

    // 初始化六个髋关节在机体上的安装位置。
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        endpoint_leg_frame[leg_index] = get_leg_frame_position(leg_index);
    }

    // 后端可能在步态切换后复用，初始化时必须清除上一次的收步状态。
    reset_leg();
    stop_state = StopState::IDLE;
    settle_step = 0;
    gait_step_total_cached = -1;
    refresh_steps(); // 校验步数参数并初始化当前步态

    // 上电安全姿态：没有任何RC输入或有效步态目标时，18路舵机均保持1500us。
    // send_servo_cmd()会再次叠加通道偏移，因此这里预先扣除偏移，确保实际输出仍为1500us。
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);
        servo_output_cmd[leg_index].x = uint16_t(constrain_float(LEG_MOTOR_PWM_MIDDLE - leg_param.COXA_OFS,
                                                                  LEG_MOTOR_PWM_MIN, LEG_MOTOR_PWM_MAX));
        servo_output_cmd[leg_index].y = uint16_t(constrain_float(LEG_MOTOR_PWM_MIDDLE - leg_param.FEMU_OFS,
                                                                  LEG_MOTOR_PWM_MIN, LEG_MOTOR_PWM_MAX));
        servo_output_cmd[leg_index].z = uint16_t(constrain_float(LEG_MOTOR_PWM_MIDDLE - leg_param.TIBI_OFS,
                                                                  LEG_MOTOR_PWM_MIN, LEG_MOTOR_PWM_MAX));
    }
    // 允许100Hz调度器从启动后的第一帧开始周期发送中位命令。
    servo_output_valid = true;

    return true;
}

// 重置腿部位置 - 将所有腿恢复到初始状态
void AP_HexRuped_Backend::reset_leg()
{
    // 遍历所有腿，重置其位置和旋转
    for (uint8_t moving_leg = 0; moving_leg < AP_HEXRUPED_LEG_ALL; moving_leg++) {
        gait_pos_xyz[moving_leg] = { 0, 0, 0 }; // 重置位置坐标为原点（相对于初始位置）
        gait_rot_z[moving_leg]   = 0;           // 重置旋转角度为0（无旋转）
    }
}

void AP_HexRuped_Backend::refresh_steps()
{
    const int16_t step_total = constrain_int16(gait_step_total.get(),
                               AP_HEXRUPED_STEP_TOTAL_MIN,
                               AP_HEXRUPED_STEP_TOTAL_MAX);
    if (step_total != gait_step_total.get()) {
        // 参数元数据只限制地面站输入，运行时仍需防御脚本或旧参数文件中的非法值。
        gait_step_total.set(step_total);
    }

    if (step_total == gait_step_total_cached) {
        return;
    }

    gait_step_total_cached = step_total;

    gait_init();
}

// 计算步态序列 - 判断是否需要移动并执行相应动作
void AP_HexRuped_Backend::calc_gait_sequence()
{
    const float travel_dz = 0.01f; // 移动死区阈值，防止微小抖动

    // 判断是否有移动请求（前进/后退或旋转）
    if ((fabsf(_frontend.get_throttle_x()) > travel_dz) || (fabsf(_frontend.get_throttle_y()) > travel_dz) || (fabsf(_frontend.get_yaw_rate()) > travel_dz / 2)) {
        move_requested = true;    // 需要移动
    } else {
        move_requested = false;    // 保持静止
    }

    // RC失联属于安全事件，不等待收步，立即回到对称站姿。
    if (_frontend.is_rc_failsafe()) {
        reset_leg();
        stop_state = StopState::IDLE;
        settle_step = 0;
        return;
    }

    if (move_requested) {
        stop_throttle_x = throttle_x_travel;
        stop_throttle_y = throttle_y_travel;
        stop_yaw = yaw_travel;
        stop_state = StopState::WALKING;
        update_leg();
        return;
    }

    if (stop_state == StopState::IDLE) {
        reset_leg();
        return;
    }

    if (stop_state == StopState::WALKING) {
        stop_state = StopState::FINISHING_STEP;
    }

    if (stop_state == StopState::FINISHING_STEP) {
        // 摇杆已经回中，但轨迹仍用最后有效行程走到最近的全腿着地边界。
        const float input_x = throttle_x_travel;
        const float input_y = throttle_y_travel;
        const float input_yaw = yaw_travel;
        throttle_x_travel = stop_throttle_x;
        throttle_y_travel = stop_throttle_y;
        yaw_travel = stop_yaw;
        update_leg();
        throttle_x_travel = input_x;
        throttle_y_travel = input_y;
        yaw_travel = input_yaw;

        bool all_legs_down = true;
        for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
            if (fabsf(gait_pos_xyz[leg_index].z) > 0.01f) {
                all_legs_down = false;
                break;
            }
        }
        if (!all_legs_down) {
            return;
        }

        for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
            settle_start_pos[leg_index] = gait_pos_xyz[leg_index];
            settle_start_yaw[leg_index] = gait_rot_z[leg_index];
        }
        settle_step = 0;
        settle_step_total = MAX(static_cast<uint16_t>(1), static_cast<uint16_t>(gait_step_total.get() / 2));
        stop_state = StopState::SETTLING;
    }

    if (stop_state == StopState::SETTLING) {
        settle_step = MIN(static_cast<uint16_t>(settle_step + 1), settle_step_total);
        const float t = static_cast<float>(settle_step) / static_cast<float>(settle_step_total);
        const float blend = t * t * (3.0f - 2.0f * t); // smoothstep: 两端速度为零
        for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
            gait_pos_xyz[leg_index] = settle_start_pos[leg_index] * (1.0f - blend);
            gait_rot_z[leg_index] = settle_start_yaw[leg_index] * (1.0f - blend);
        }
        if (settle_step >= settle_step_total) {
            reset_leg();
            stop_state = StopState::IDLE;
        }
    }
}

// 腿部逆运动学计算
Vector3f AP_HexRuped_Backend::leg_inverse_kinematics(Vector3f posxyz)
{
    const AP_HexRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

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

    if (_frontend.get_class() == AP_HEXRUPED_USL_BV2) {
        const float alpha = degrees(atan2f(Sys_Param.Alpha_A, Sys_Param.Alpha_B));
        leg_deg.y -= alpha;
        leg_deg.z += 90.0f - alpha;
    }

    return leg_deg; // 返回{髋关节, 股关节, 胫关节}角度
}

// 机体正向运动学 - 计算考虑机体姿态和重心偏移后的腿部末端位置
Vector3f AP_HexRuped_Backend::body_forward_kinematics(uint8_t leg_index)
{
    // 计算腿部末端在机体坐标系中的总位置
    // gait_pos_xyz：步态生成的目标位置（相对于初始位置的偏移）
    // endpoint_leg_pos：腿部初始展开位置（髋关节+股关节长度在45度方向的投影）
    // endpoint_leg_frame：机体框架几何尺寸（腿在机体上的安装位置）
    Vector3f totaldist_xyz = gait_pos_xyz[leg_index] + endpoint_leg_pos[leg_index] + endpoint_leg_frame[leg_index];

    // 添加重心偏移补偿
    // 减去 center_offset 是因为：当重心偏移时，机体参考点改变，所有腿的相对位置需要重新计算
    // center_offset 保留为静态校准和未来支撑多边形补偿接口；普通步态默认保持为零。
    totaldist_xyz -= center_offset;

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
void AP_HexRuped_Backend::main_inverse_kinematics(void)
{
    Vector3f ansxyz = { 0, 0, 0 }; // 临时变量，存储腿部末端位置

    // 腿部角度偏移补偿 - 由于机械安装误差，每条腿需要不同的角度补偿
    // 遍历所有腿，计算逆运动学
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 1. 计算腿部末端在机体坐标系中的位置
        ansxyz = body_forward_kinematics(leg_index);
        // 2. 计算逆运动学得到关节角度，并加上补偿值
        endpoint_leg_angle[leg_index] = leg_inverse_kinematics(ansxyz);
        endpoint_leg_angle[leg_index].x += get_leg_mount_yaw_deg(leg_index);
        // 3. 将髋关节角度规范到[-180, 180]范围内
        endpoint_leg_angle[leg_index].x = wrap_180(endpoint_leg_angle[leg_index].x);
    }

    // 保存当前关节角度到上一时刻变量
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        endpoint_leg_angle_last[leg_index] = endpoint_leg_angle[leg_index];
    }
}

// 偏航轨迹生成
void AP_HexRuped_Backend::yaw_trajectory_generation(uint8_t leg_index)
{
    // 计算当前腿的步数偏移
    int32_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 相位循环处理：确保步数在有效范围内，避免整数溢出和相位跳跃
    // 先处理负数再取模，保证数学上的正确性和连续性
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    const float p    = (float)delta_step / (float)gait_step_total;
    const float peak = yaw_travel / (float)gait_lift_divisor;

    if (p < (1.0f / 12.0f)) {
        gait_rot_z[leg_index] = 0.0f;
    } else if (p < (1.0f / 6.0f)) {
        gait_rot_z[leg_index] = peak;
    } else {
        const float t         = (p - (1.0f / 6.0f)) / (5.0f / 6.0f);
        gait_rot_z[leg_index] = peak * (1.0f - t);
    }
}

// 主控制器 - 处理遥控器输入并转换为运动指令
void AP_HexRuped_Backend::main_radio_controller()
{
    const AP_HexRuped_CHANNEL_Params& channel = _frontend.get_channel_params();
    AP_HexRuped_CTRL_Params&          ctrl    = _frontend.get_ctrl_params();

    //////////////////////////////////////////////////////////////////////////////////
    // 处理油门通道（前进/后退）
    if (channel.throttle_x_channel != -1) {
        // 将遥控器输入转换为前进/后退行程
        throttle_x_travel = _frontend.get_throttle_x() * channel.throttle_x_max;
    } else {
        throttle_x_travel = 0; // 无通道配置时保持静止
    }

    //////////////////////////////////////////////////////////////////////////////////
    // 处理横移通道。与前进相同，当前为摇杆到足端行程的开环映射。
    if (channel.throttle_y_channel != -1) {
        throttle_y_travel = _frontend.get_throttle_y() * channel.throttle_y_max;
    } else {
        throttle_y_travel = 0.0f;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // 处理偏航通道
    if (channel.yaw_channel != -1) {
        yaw_travel = _frontend.get_yaw_rate() * channel.throttle_yaw_max;
    } else {
        yaw_travel = 0.0f;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // 处理滚转通道（向右为正，向左为负）
    if (channel.roll_channel != -1) {
        roll_target        = _frontend.get_throttle_roll() * channel.throttle_roll_max;
        float roll_current = degrees(_ahrs.roll);

        // 使用PI控制器计算输出
        float dt    = 0.1f; // 计算实际时间步长
        roll_travel = ctrl.roll_pid.update_all(roll_target, roll_current, dt);
    } else {
        roll_travel = 0.0f;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // 处理俯仰通道
    if (channel.pitch_channel != -1) {
        pitch_target        = _frontend.get_throttle_pitch() * channel.throttle_pitch_max;
        float pitch_current = degrees(_ahrs.pitch);

        // 使用PI控制器计算输出
        float dt     = 0.1f; // 计算实际时间步长
        pitch_travel = ctrl.pitch_pid.update_all(pitch_target, pitch_current, dt);

        // hal.console->printf("pitch_target = %.2f, pitch_current = %.2f",
        //                     pitch_target,
        //                     pitch_current);
    } else {
        pitch_travel = 0.0f;
    }

    z_travel        = channel.body_height; // 默认高度
    leg_lift_height = channel.left_lift;   // 抬腿高度
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// 输出腿部关节角度 - 将计算出的关节角度转换为PWM信号
void AP_HexRuped_Backend::output_leg_angle(void)
{
    // 遍历所有腿，计算每个关节的PWM值
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 将角度转换为PWM值
        // 公式：PWM = 方向系数 × 角度 × PWM范围/角度范围 + 中间值
        servo_output_cmd[leg_index].x = angle_to_servo_pwm(endpoint_leg_angle[leg_index].x,
                                        leg_param.COXA_DIR);
        servo_output_cmd[leg_index].y = angle_to_servo_pwm(endpoint_leg_angle[leg_index].y,
                                        leg_param.FEMU_DIR);
        servo_output_cmd[leg_index].z = angle_to_servo_pwm(endpoint_leg_angle[leg_index].z,
                                        leg_param.TIBI_DIR);
    }
    servo_output_valid = true;
}

// 硬件伺服命令设置 - 通过CAN总线发送PWM控制信号到舵机
bool AP_HexRuped_Backend::send_servo_cmd()
{
    if (!servo_output_valid) {
        return false;
    }

    com_usl_ServoCmd msg {}; // 创建DroneCAN伺服控制消息结构体
    msg.cmd.len = AP_HEXRUPED_SERVO_COUNT; // 六条腿×三个关节 = 18个数据

    // 遍历所有腿部，准备发送数据
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        // 获取腿部参数
        const AP_HexRuped_Params& leg_param = _frontend.get_leg_params(leg_index);

        // 填充CAN消息数据（添加偏移补偿）
        const uint16_t coxa_pwm = apply_servo_offset(servo_output_cmd[leg_index].x,
                                  leg_param.COXA_OFS);
        const uint16_t femur_pwm = apply_servo_offset(servo_output_cmd[leg_index].y,
                                   leg_param.FEMU_OFS);
        const uint16_t tibia_pwm = apply_servo_offset(servo_output_cmd[leg_index].z,
                                   leg_param.TIBI_OFS);
        const uint8_t channel = leg_index * AP_HEXRUPED_JOINTS_PER_LEG;

        msg.cmd.data[channel + 0] = coxa_pwm;
        msg.cmd.data[channel + 1] = femur_pwm;
        msg.cmd.data[channel + 2] = tibia_pwm;

        // SRV只保存18路逻辑输出，不要求飞控具备18个物理PWM通道。只有通过
        // SERVOx_FUNCTION显式映射的功能才会输出到引脚；实机和Scorpio SITL
        // 均以下方DroneCAN消息作为腿部执行链路，SRV镜像用于遥测和可选PWM调试。
        SRV_Channels::set_output_pwm(static_cast<SRV_Channel::Aux_servo_function_t>(SRV_Channel::k_hexleg_rf_coxa + channel),
                                     coxa_pwm);
        SRV_Channels::set_output_pwm(static_cast<SRV_Channel::Aux_servo_function_t>(SRV_Channel::k_hexleg_rf_femu + channel),
                                     femur_pwm);
        SRV_Channels::set_output_pwm(static_cast<SRV_Channel::Aux_servo_function_t>(SRV_Channel::k_hexleg_rf_tibi + channel),
                                     tibia_pwm);
    }

    // 在所有可用的CAN总线接口上广播伺服控制命令
    // 获取CAN驱动数量
    const uint8_t can_num_drivers = AP::can().get_num_drivers();

    // 发送成功标志
    bool ok = false;

    // 遍历所有CAN接口
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto *dronecan = AP_DroneCAN::get_dronecan(i); // 获取第i个CAN驱动实例
        if (dronecan != nullptr) {
            const bool sent = dronecan->com_usl_servocmd.broadcast(msg);
            if (sent) {
                dronecan->log_hiwonder_servo_command(msg);
            }
            // 使用|=确保只要有一个接口成功就返回true
            ok |= sent;
        }
    }
    return ok; // 返回广播结果
}
