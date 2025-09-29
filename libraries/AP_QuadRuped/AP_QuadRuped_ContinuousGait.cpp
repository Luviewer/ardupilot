#include "AP_QuadRuped_ContinuousGait.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

// 外部HAL实例 - 提供对硬件抽象层的访问
extern const AP_HAL::HAL& hal;

#define SPEED_HZ_DEFAULT 25.0f // 默认步态频率25Hz，平衡速度和稳定性

// 参数表定义 - 用于配置连续步态系统的参数
// 这些参数可以通过地面站或配置文件进行调整
const AP_Param::GroupInfo AP_QuadRuped_ContinuousGait::var_info[] = {
    // @Param: Hz
    // @DisplayName: 步态频率
    // @Description: 连续步态的执行频率(Hz)，控制运动速度
    // @Range: 0.5 5.0
    // @User: Advanced
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_ContinuousGait, gait_hz, SPEED_HZ_DEFAULT), // 步态频率参数

    AP_GROUPEND // 参数表结束标记
};

// 构造函数 - 初始化连续对角步态控制器
AP_QuadRuped_ContinuousGait::AP_QuadRuped_ContinuousGait(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state,
                                                         AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors) // 调用基类构造函数，传递必要的接口
{
    // 设置参数默认值 - 确保参数系统正确初始化
    // setup_object_defaults会为var_info中定义的参数设置默认值
    AP_Param::setup_object_defaults(this, var_info);

    // 将参数表注册到状态对象中 - 使参数系统能够找到和使用这些参数
    _state.var_info = var_info;
}

// 步态初始化 - 连续对角步态
// 这是系统启动时调用的关键函数，设置整个连续步态系统的基础参数
void AP_QuadRuped_ContinuousGait::gait_init()
{
    // 发送初始化信息到地面站 - 用于调试和状态监控
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_ContinuousGait (Diagonal) init");

    // 计算完整步态周期时间 - 一个完整步态循环所需的时间(秒)
    // 例如：25Hz频率 = 0.04秒周期时间
    float cycle_time = 1.0f / gait_hz;

    // 设置每条腿的连续相位偏移（对角步态配置）
    // 对角步态策略：对角线上的腿同时运动，提供最佳稳定性
    // 右前腿和左后腿为一组，左前腿和右后腿为另一组
    _leg_phase_offset[AP_QUADRUPED_LEG_RF] = 0.0f;              // 右前腿 - 0秒偏移，作为基准
    _leg_phase_offset[AP_QUADRUPED_LEG_LB] = 0.0f;              // 左后腿 - 0秒偏移，与右前腿同步运动
    _leg_phase_offset[AP_QUADRUPED_LEG_LF] = cycle_time * 0.5f; // 左前腿 - 半周期偏移，与第一组形成对角
    _leg_phase_offset[AP_QUADRUPED_LEG_RB] = cycle_time * 0.5f; // 右后腿 - 半周期偏移，与左前腿同步运动

    // 初始化连续时间变量 - 记录系统时间，用于相对时间计算
    _init_time = AP_HAL::millis();  // 记录初始化时刻
    _current_time = _init_time;     // 当前时间从初始化时刻开始

    // 设置连续步态时序参数 - 将时间分配给支撑相和摆动相
    _stance_duration   = cycle_time * 0.5f; // 支撑相占60%时间 - 腿着地推动身体前进
    _transfer_duration = cycle_time * 0.5f; // 摆动相占40%时间 - 腿抬起向前摆动

    // 初始化每条腿的状态 - 设置每条腿的开始时间和激活状态
    for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
        _leg_start_time[i] = _leg_phase_offset[i]; // 每条腿从其相位偏移时间开始运动
        _leg_active[i]     = true;                 // 所有腿都激活，准备参与步态运动
    }
}

// 连续轨迹生成 - 实现对角步态的核心算法
// 为指定腿计算连续的目标位置，保证运动的平滑性和连续性
void AP_QuadRuped_ContinuousGait::trajectory_generation(uint8_t leg_index)
{
    // 计算每条腿的相对时间（考虑相位偏移）
    // 每条腿有自己的时间起点，实现对角步态的相位差
    // 注意：需要将毫秒转换为秒，并减去初始化时间
    float elapsed_time = (_current_time - _init_time) * 0.001f; // 毫秒转秒
    float leg_time = elapsed_time - _leg_start_time[leg_index];
    if (leg_time < 0) leg_time = 0; // 确保时间非负，防止初始时刻的负值

    // 计算完整步态周期时间 = 支撑相时间 + 摆动相时间
    float cycle_time = _stance_duration + _transfer_duration;

    // 计算当前腿在周期内的相位时间 - 使用模运算保证周期性
    // 这样确保运动是循环的，不会无限增长
    float phase_time = fmod(leg_time, cycle_time);

    // 声明目标位置变量
    Vector2f leg_xy_target;       // XY平面目标位置
    float    leg_z_target = 0.0f; // Z轴目标高度

    // 获取期望的平面行程向量 - 来自遥控器输入
    // throttle_x_travel: 前后方向行程，throttle_y_travel: 左右方向行程
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    // 判断当前腿部处于哪个运动阶段
    if (phase_time < _stance_duration) {
        // ========== 支撑相阶段 ==========
        // 腿部着地，相对身体向后移动，推动身体前进

        // 计算支撑相进度 (0.0 到 1.0)
        float progress = phase_time / _stance_duration;

        // 计算摆线参数 (0 到 2π)
        // 摆线曲线保证起止速度为0，提供平滑运动
        float delta = M_2PI * progress;

        // 使用摆线算法计算位移 - 与AP_QuadRuped_Diag保持一致
        // S值从0变化到2，提供平滑的加减速
        const float S = (delta - sinf(delta)) / M_2PI * 2.0f;

        // 支撑相：腿部从throttle_travel位置移动到-throttle_travel位置
        // 即从最前端移动到最后端，推动身体前进
        leg_xy_target = -throttle_travel * S + throttle_travel;
        leg_z_target  = 0.0f; // 支撑相保持地面接触，高度为0
    } else {
        // ========== 摆动相阶段 ==========
        // 腿部抬起，向前摆动，准备下一次着地

        // 计算摆动相的相对时间（减去支撑相时间）
        float transfer_time = phase_time - _stance_duration;

        // 计算摆动相进度 (0.0 到 1.0)
        float progress = transfer_time / _transfer_duration;

        // 计算摆线参数 (0 到 2π)
        float delta = M_2PI * progress;

        // 使用相同的摆线算法保证运动连续性
        const float S = (delta - sinf(delta)) / M_2PI * 2.0f;

        // 摆动相：腿部从-throttle_travel位置移动到throttle_travel位置
        // 即从最后端移动到最前端，完成向前摆动
        leg_xy_target = throttle_travel * S - throttle_travel;

        // 垂直方向使用正弦曲线实现抬腿动作
        // (1 - cosf(delta))确保起止高度为0，中间达到最高点
        leg_z_target = -leg_lift_height * (1.0f - cosf(delta));
    }

    // 将计算出的目标位置存储到全局变量中，供逆运动学使用
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 生成偏航（旋转）轨迹 - 连续版本
// 计算腿部在转向时的旋转补偿，保证转向时的身体平衡
void AP_QuadRuped_ContinuousGait::yaw_trajectory_generation(uint8_t leg_index)
{
    // 计算每条腿的相对时间（考虑相位偏移）
    // 与轨迹生成函数使用相同的时间计算方法
    float elapsed_time = (_current_time - _init_time) * 0.001f; // 毫秒转秒
    float leg_time = elapsed_time - _leg_start_time[leg_index];
    if (leg_time < 0) leg_time = 0; // 确保时间非负

    // 计算完整步态周期时间
    float cycle_time = _stance_duration + _transfer_duration;

    // 计算当前腿在周期内的相位时间
    float phase_time = fmod(leg_time, cycle_time);

    // 计算整个周期的进度比例 (0.0 到 1.0)
    float progress = phase_time / cycle_time;

    // 计算旋转峰值 - 来自偏航输入的最大旋转角度
    // 除以2是为了限制旋转幅度，防止过度旋转
    const float peak = yaw_travel / 2.0f;

    // 分段实现旋转轨迹 - 在摆动相初期施加旋转
    if (progress < (1.0f / 12.0f)) {
        // 前1/12时间段：无旋转
        // 腿部刚进入支撑相，保持稳定，不施加旋转
        gait_rot_z[leg_index] = 0.0f;
    } else if (progress < (1.0f / 6.0f)) {
        // 1/12到1/6时间段：快速达到峰值旋转
        // 腿部准备进入摆动相，施加最大旋转补偿
        gait_rot_z[leg_index] = peak;
    } else {
        // 剩余5/6时间段：线性衰减到0
        // 旋转补偿逐渐减小，避免突然变化
        const float t         = (progress - (1.0f / 6.0f)) / (5.0f / 6.0f); // 归一化到 [0,1)
        gait_rot_z[leg_index] = peak * (1.0f - t);                          // 线性衰减
    }
}

// 更新腿部运动 - 连续版本
// 这是步态系统的核心更新函数，在主循环中被周期性调用
void AP_QuadRuped_ContinuousGait::update_leg()
{
    // 更新连续时间 - 使用ArduPilot系统时间
    // 这样确保与整个系统同步，避免时间漂移
    _current_time = AP_HAL::millis();       // 获取当前系统时间(毫秒)

    // 为每条腿生成连续轨迹 - 并行处理所有四条腿
    // 对角步态的特性通过相位偏移自动实现，不需要额外逻辑
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // 生成位置轨迹 - 计算腿的XY平面位置和Z轴高度
        trajectory_generation(leg_index);

        // 生成旋转轨迹 - 计算转向时的旋转补偿
        yaw_trajectory_generation(leg_index);
    }
}

// 主更新函数 - 连续步态系统的顶层控制流程
// 这是ArduPilot系统调用的主要接口，协调所有控制模块
void AP_QuadRuped_ContinuousGait::update()
{
    // 执行主控制器 - 处理遥控器输入，转化为运动指令
    // 包括油门、转向、俯仰、横滚等控制信号的解析
    main_radio_controller();

    // 执行逆运动学解算 - 将足端目标位置转换为关节角度
    // 考虑身体姿态、重心偏移等因素的影响
    main_inverse_kinematics();

    // 输出腿部关节角度 - 将计算出的角度转换为PWM信号
    // 准备发送到舵机控制器
    output_leg_angle();

    // 发送舵机控制命令 - 通过CAN总线或PWM输出到舵机
    // 实际驱动电机运动到目标位置
    send_servo_cmd();
}

// 连续相位计算 - 计算指定腿在步态周期中的归一化相位
// 返回值范围：0.0 到 1.0，表示腿在完整步态周期中的进度
float AP_QuadRuped_ContinuousGait::calculate_continuous_phase(uint8_t leg_index)
{
    // 计算完整步态周期时间
    float cycle_time = _stance_duration + _transfer_duration;

    // 计算指定腿的相对运行时间
    // 考虑每条腿的相位偏移，实现对角步态的时间差
    float elapsed_time = (_current_time - _init_time) * 0.001f; // 毫秒转秒
    float leg_time = elapsed_time - _leg_start_time[leg_index];

    // 确保时间非负，防止初始时刻出现负值
    if (leg_time < 0) leg_time = 0;

    // 使用模运算计算在当前周期中的时间，然后归一化到 [0, 1)
    // 这样可以得到腿在步态周期中的精确位置
    return fmod(leg_time, cycle_time) / cycle_time;
}

// 检查腿部轨迹是否完成 - 判断腿是否到达支撑相末期
// 主要用于判断是否可以开始摆动相（对连续步态可能不需要）
bool AP_QuadRuped_ContinuousGait::is_leg_trajectory_complete(uint8_t leg_index)
{
    // 计算完整步态周期时间
    float cycle_time = _stance_duration + _transfer_duration;

    // 计算指定腿的相对运行时间
    float elapsed_time = (_current_time - _init_time) * 0.001f; // 毫秒转秒
    float leg_time = elapsed_time - _leg_start_time[leg_index];

    // 如果时间还为负，说明腿还未开始运动，返回未完成
    if (leg_time < 0) return false;

    // 计算在当前周期中的相位时间
    float phase_time = fmod(leg_time, cycle_time);

    // 在支撑相末期检查是否可以开始摆动
    // 支撑相的90%到100%时间段被认为是完成点
    // 这样设计是为了给摆动相留出准备时间
    return (phase_time > _stance_duration * 0.9f) && (phase_time < _stance_duration);
}

// 启动下一条腿的轨迹 - 用于离散步态的腿切换逻辑
// 在连续步态中，所有腿通过时间偏移自动控制，不需要显式切换
void AP_QuadRuped_ContinuousGait::start_next_leg_trajectory(uint8_t completed_leg_index)
{
    // 对角步态：成对启动，不需要单独启动
    // 连续步态系统中，所有腿已经在初始化时启动
    // 通过相位偏移(_leg_phase_offset)自动实现对角步态的时间差
    // 这个函数保留以兼容接口，但在连续系统中实际不会被调用
}