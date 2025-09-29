#include "AP_QuadRuped_Wave.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        25.0f // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 波浪步态参数表定义
// 使用AP_Param框架实现参数的持久化存储和运行时调整
// 参数前缀QUAD_WAVE_由ArduPilot参数系统自动添加
const AP_Param::GroupInfo AP_QuadRuped_Wave::var_info[] = {
    // ==================== 基础步态参数 ====================

    // 步态频率控制：决定波浪步态的时间分辨率和运动平滑性
    // 参数名：QUAD_WAVE_Hz，范围：1-100Hz，默认：25Hz
    // 波浪步态特性：频率越高，腿部交替越流畅，但计算负载越大
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Wave, gait_hz, SPEED_HZ_DEFAULT),

    // 步态总步数：决定波浪步态周期的离散化精度和相位控制
    // 参数名：QUAD_WAVE_STEP，范围：8-100步，默认：24步
    // 波浪步态设计：24步提供90度相位差的精确控制，确保四条腿依次运动
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_Wave, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    // ==================== 轨迹生成模式选择 ====================

    // 轨迹生成算法选择：为波浪步态提供多种轨迹生成方式
    // 参数名：QUAD_WAVE_TRAJ_MODE，选项：0=正弦轨迹，1=贝塞尔曲线轨迹，默认：0
    // 波浪步态考量：
    // - 正弦轨迹：符合波浪步态的自然特性，运动平滑
    // - 贝塞尔曲线：提供更灵活的轨迹控制，适合特殊地形需求
    AP_GROUPINFO("TRAJ_MODE", 3, AP_QuadRuped_Wave, trajectory_mode, 0),

    // ==================== 贝塞尔曲线控制参数 ====================
    // 以下参数仅在TRAJ_MODE=1时生效，专门用于波浪步态的贝塞尔曲线调节

    // 波浪步态贝塞尔曲线控制点高度系数：调节波浪步态的抬腿高度
    // 参数名：QUAD_WAVE_BCTRL_H，范围：0.1-2.0，默认：0.3
    // 波浪步态特性：相比对角步态，波浪步态需要更协调的抬腿高度
    AP_GROUPINFO("BCTRL_H", 4, AP_QuadRuped_Wave, bezier_control_height, 0.3f),

    // 波浪步态贝塞尔曲线控制点前向偏移系数：调节轨迹的前后延伸
    // 参数名：QUAD_WAVE_BCTRL_F，范围：0.0-1.0，默认：0.2
    // 波浪步态设计：由于是依次运动，需要考虑腿部间的运动协调性
    AP_GROUPINFO("BCTRL_F", 5, AP_QuadRuped_Wave, bezier_control_forward, 0.2f),

    // 参数表结束标记，AP_Param框架要求的固定格式
    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Wave::AP_QuadRuped_Wave(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// 波浪步态初始化函数
// 配置波浪步态的基本参数，建立四条腿的相位关系，初始化平滑滤波器
// 波浪步态特性：四条腿依次运动，形成波浪式推进效果，稳定性好，适应性强
void AP_QuadRuped_Wave::gait_init()
{
    // 系统启动日志：记录波浪步态模块的初始化状态
    // 便于调试和系统状态监控
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_Wave init");

    // ==================== 波浪步态平滑滤波器初始化 ====================
    // 初始化三维平滑滤波器，用于中心偏移的平滑过渡
    // 波浪步态特点：需要平滑的重心转移来保证运动稳定性
    // 滤波器参数：时间常数=1000/gait_hz，截止频率=100Hz，输出限幅=0
    td_smooth[0].init(1000 / gait_hz, 100, 0); // X轴平滑滤波器
    td_smooth[1].init(1000 / gait_hz, 100, 0); // Y轴平滑滤波器
    td_smooth[2].init(1000 / gait_hz, 100, 0); // Z轴平滑滤波器

    // ==================== 波浪步态相位配置 ====================
    // 波浪步态核心：四条腿依次运动，相位差90度，形成波浪推进效果
    // 相位设计原则：确保任何时刻都有三条腿支撑地面，保证稳定性

    // 右前腿（RF）：0度相位 - 波浪步态的起始腿
    // 作为波浪运动的起始点，其他腿依次跟随
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                       // 右前腿 - 0度相位

    // 左前腿（LF）：90度相位 - 延迟1/4周期
    // 在右前腿运动到1/4位置时开始抬腿，形成连续的波浪效果
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4;     // 左前腿 - 90度相位

    // 左后腿（LB）：180度相位 - 延迟1/2周期
    // 在右前腿运动到1/2位置时开始抬腿，保持波浪的连续性
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 2;     // 左后腿 - 180度相位

    // 右后腿（RB）：270度相位 - 延迟3/4周期
    // 在右前腿运动到3/4位置时开始抬腿，完成波浪循环
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = 3 * gait_step_total / 4; // 右后腿 - 270度相位

    // 波浪步态稳定性分析：
    // - 任何时刻都有3条腿支撑地面，1条腿摆动
    // - 支撑腿形成稳定的三角形支撑结构
    // - 摆动腿依次运动，减少冲击和振动

    // ==================== 波浪步态运动参数配置 ====================
    // 设置步态运动学参数，影响步长和抬腿高度

    // 行程除数：控制步态周期的行程分配
    // 波浪步态设计：将总步数分为2等份，平衡推进效率和稳定性
    gait_travel_divisor = gait_step_total / 2; // 行程除数

    // 抬腿除数：控制抬腿高度的比例系数
    // 波浪步态特点：使用适中的抬腿高度，兼顾越障能力和稳定性
    gait_lift_divisor = 2;                   // 抬腿除数

    // 波浪步态优势：
    // 1. 稳定性高：始终有三点支撑，适合复杂地形
    // 2. 推进效率好：波浪式推进，能量利用效率高
    // 3. 适应性强：可调节相位差适应不同负载和速度需求
    // 4. 控制简单：相位关系固定，易于实现和控制
}

// 波浪步态轨迹生成统一接口函数
// 作为波浪步态控制系统的核心调度器，根据用户参数选择合适的轨迹生成算法
// 波浪步态特性：四条腿依次运动（0°, 90°, 180°, 270°相位差），形成波浪式推进效果
void AP_QuadRuped_Wave::trajectory_generation(uint8_t leg_index)
{
    // 轨迹模式决策：根据参数选择轨迹生成算法
    // 波浪步态支持两种轨迹模式以适应不同应用场景
    if (trajectory_mode.get() == 1) {
        // ==================== 贝塞尔曲线轨迹模式 ====================
        // 适用场景：复杂地形、需要精细控制波浪步态的轨迹形状
        // 优势：为波浪步态的依次运动特性提供更灵活的轨迹控制
        // 特点：可以单独调节每条腿的轨迹，实现更自然的波浪运动
        generate_bezier_trajectory(leg_index);

    } else {
        // ==================== 默认正弦轨迹模式 ====================
        // 适用场景：标准波浪步态、追求运动的自然性和平滑性
        // 优势：符合波浪步态的物理特性，运动流畅，计算效率高
        // 特点：使用正弦曲线和缓动函数，实现经典的波浪步态效果
        generate_cycloid_trajectory(leg_index);
    }

    // 设计原理说明：
    // 1. 波浪步态特性：四条腿依次运动，每条腿相位差90度
    // 2. 模式切换：通过参数实时切换，适应不同地形需求
    // 3. 接口统一：保证不同算法间的相位一致性和运动协调性
    // 4. 扩展性：为后续添加更多波浪步态变体预留接口
}

// 波浪步态偏航（旋转）轨迹生成函数
// 控制波浪步态在进行偏航运动时各条腿的旋转轨迹
// 波浪步态旋转特性：通过协调的腿部旋转实现平滑的转向，保持运动稳定性
void AP_QuadRuped_Wave::yaw_trajectory_generation(uint8_t leg_index)
{
    // ==================== 波浪步态偏航相位计算 ====================
    // 计算当前腿相对于其起始步数的相位偏移
    // 波浪步态旋转：每条腿的旋转轨迹与摆动轨迹同步，保证运动协调性
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

    // 归一化进度：将步数映射到[0,1)区间，作为旋转轨迹的时间基准
    const float p = (float)delta_step / (float)gait_step_total; // 步态进度 ∈ [0,1)

    // 旋转峰值计算：根据用户输入的旋转行程计算最大旋转角度
    // 波浪步态旋转设计：适中的旋转角度，兼顾转向效率 and 稳定性
    const float peak = yaw_travel / (float)gait_lift_divisor;      // 旋转峰值

    // ==================== 波浪步态三阶段旋转控制 ====================
    // 波浪步态旋转采用分段控制：准备阶段、执行阶段、衰减阶段
    // 这种设计确保旋转运动的平滑性和可控性

    if (p < (1.0f / 12.0f)) {
        // ==================== 准备阶段（前1/12周期）====================
        // 无旋转阶段：为旋转运动做准备，确保运动平稳开始
        // 波浪步态设计：在旋转开始前有一个准备期，提高运动稳定性
        gait_rot_z[leg_index] = 0.0f;

    } else if (p < (1.0f / 6.0f)) {
        // ==================== 执行阶段（1/12到1/6周期）====================
        // 峰值旋转阶段：快速达到设定的旋转角度
        // 波浪步态特点：在抬腿阶段执行旋转，减少地面摩擦
        gait_rot_z[leg_index] = peak;

    } else {
        // ==================== 衰减阶段（剩余5/6周期）====================
        // 线性衰减阶段：从峰值旋转角度平滑衰减到0
        // 波浪步态优化：使用长衰减时间，确保旋转运动的平滑结束

        // 线性衰减参数计算：将衰减阶段映射到[0,1)区间
        const float t = (p - (1.0f / 6.0f)) / (5.0f / 6.0f); // t ∈ [0,1)

        // 线性衰减公式：rotation = peak * (1 - t)
        // 特性：t=0时rotation=peak，t=1时rotation=0
        // 波浪步态设计：直接计算不依赖上一帧，避免累积误差
        gait_rot_z[leg_index] = peak * (1.0f - t);
    }

    // 波浪步态旋转设计原理：
    // 1. 时序协调：旋转运动与摆动运动同步，在抬腿阶段执行主要旋转
    // 2. 平滑性：三阶段控制确保旋转的起止平滑，减少冲击
    // 3. 稳定性：长衰减时间保证旋转运动的稳定性
    // 4. 效率：在支撑阶段保持稳定，在摆动阶段执行旋转，提高能量利用效率
}

// 波浪步态统一轨迹生成器（重新设计版本）
// 将重心移动完全整合到轨迹生成过程中，实现腿部运动和重心调整的协调统一
// 新的时间分配：重心准备阶段(1/6) + 抬腿阶段(1/3) + 支撑阶段(1/2)
void AP_QuadRuped_Wave::generate_cycloid_trajectory(uint8_t leg_index)
{
    // 波浪步态相位计算：统一的相位处理逻辑
    int32_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 相位循环处理：确保步数在合理范围内
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位：将离散步数转换为连续的相位值[0,1]
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 用户指令解析：将遥控器输入转换为期望的平面位移向量
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    Vector3f leg_target; // 存储计算得到的腿部目标位置

    // ==================== 相位差别的重心移动策略 ====================
    // 计算每条腿的重心移动相位偏移，实现波浪式的重心传递
    // 这样可以避免所有腿同时进行重心移动，提高稳定性
    const float leg_phase_offset = (float)gait_step_leg_start[leg_index] / (float)gait_step_total;
    const float centre_phase_offset = leg_phase_offset * 0.25f; // 减小偏移幅度，避免过度分散

    // 动态时间分配：根据腿的相位调整重心移动的时间比例
    // 实现更自然的心脏式重心传递效果
    const float prepare_ratio = 0.167f + 0.033f * sinf(centre_phase_offset * M_2PI); // 1/6 ± 0.033
    const float lift_ratio = 0.333f + 0.025f * cosf(centre_phase_offset * M_2PI);   // 1/3 ± 0.025
    const float support_ratio = 1.0f - prepare_ratio - lift_ratio;                // 剩余时间

    // ==================== 波浪步态动态时间分配 =====================
    // 基于腿相位调整的时间分配，实现波浪式的重心传递：
    // - 重心准备阶段：动态比例，重心调整 + 腿部微调
    // - 抬腿阶段：动态比例，腿部离地运动 + 重心协调
    // - 支撑阶段：剩余比例，地面推进 + 重心稳定

    if (p < prepare_ratio) {
        // ==================== 重心准备阶段（动态比例）====================
        const float phase = p / prepare_ratio; // 将[0,prepare_ratio]映射到[0,1]

        // 重心移动轨迹：根据运动方向和腿部位置计算重心偏移
        Vector3f centre_offset_calc = calculate_centre_offset_wave(leg_index, phase, throttle_travel);

        // 应用重心偏移：直接设置目标重心位置
        // 取消平滑滤波，实现精确的轨迹控制
        centre_offset = centre_offset_calc;

        // 腿部微调轨迹：在重心调整时进行微小的腿部位置调整
        // 这种设计确保重心移动和腿部运动的协调性
        leg_target = calculate_leg_adjustment_wave(leg_index, phase, throttle_travel);

    } else if (p < (prepare_ratio + lift_ratio)) {
        // ==================== 抬腿阶段（动态比例）====================
        const float phase = (p - prepare_ratio) / lift_ratio; // 将[prepare_ratio, prepare_ratio+lift_ratio]映射到[0,1]

        // 重心协调轨迹：在抬腿阶段继续协调重心位置
        Vector3f centre_offset_calc = calculate_centre_coordination_wave(leg_index, phase, throttle_travel);
        centre_offset = centre_offset_calc;

        // 腿部抬升轨迹：使用正弦曲线实现平滑的抬腿运动
        leg_target = calculate_leg_lift_wave(leg_index, phase, throttle_travel);

    } else {
        // ==================== 支撑阶段（动态比例）====================
        const float phase = (p - prepare_ratio - lift_ratio) / support_ratio; // 将剩余时间映射到[0,1]

        // 重心稳定轨迹：在支撑阶段保持重心稳定
        Vector3f centre_offset_calc = calculate_centre_stability_wave(leg_index, phase, throttle_travel);
        centre_offset = centre_offset_calc;

        // 腿部支撑轨迹：使用三次缓动函数实现平滑的支撑推进
        leg_target = calculate_leg_support_wave(leg_index, phase, throttle_travel);
    }

    // 设置最终腿部位置
    gait_pos_xyz[leg_index] = leg_target;
}

// 波浪步态中心偏移阶段处理函数
// 负责调整机器人重心，为波浪步态的运动做准备
// 这是波浪步态特有的三阶段控制的第一阶段
void AP_QuadRuped_Wave::handle_centre_offset_phase(uint8_t leg_index)
{
    // 计算当前腿相对于其起始步态的相位偏移
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 处理负相位偏移，确保在[0, gait_step_total)范围内
    if (delta_step < 0) {
        delta_step += gait_step_total;
    }

    const uint16_t centre_offset_steps = gait_step_total / 12;

    // 计算平滑插值比例 (0.0 到 1.0)
    float t = (float)delta_step / centre_offset_steps;

    // 使用平滑的缓动函数 (ease-in-out cubic)
    // 这种缓动函数提供更自然的重心转移效果
    float smooth_t = t < 0.5f ? 4.0f * t * t * t : 1.0f - powf(-2.0f * t + 2.0f, 3.0f) / 2.0f;

    // 波浪步态重心偏移策略：根据运动方向调整不同腿的重心偏移
    // 这种设计确保了波浪步态的稳定性和推进效率
    if (throttle_x_travel < 0) {
        // 后退运动时的重心偏移配置
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF: // 右前腿
                set_centre_offset(-30.0f * smooth_t, -45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LF: // 左前腿
                set_centre_offset(-30.0f * smooth_t, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LB: // 左后腿
                set_centre_offset(0.0f, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_RB: // 右后腿
                set_centre_offset(0.0f, -45.0f * smooth_t);
                break;
        }
    } else {
        // 前进运动时的重心偏移配置
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF: // 右前腿
                set_centre_offset(0.0f, -45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LF: // 左前腿
                set_centre_offset(0.0f, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_LB: // 左后腿
                set_centre_offset(30.0f * smooth_t, 45.0f * smooth_t);
                break;
            case AP_QUADRUPED_LEG_RB: // 右后腿
                set_centre_offset(30.0f * smooth_t, -45.0f * smooth_t);
                break;
        }
    }
}

// 波浪步态抬腿阶段处理函数
// 使用改进的摆线公式实现平滑的抬腿轨迹，确保起止点的速度和加速度连续
// 这是波浪步态摆动相的核心算法，直接影响运动的平滑性和机械寿命
void AP_QuadRuped_Wave::handle_lift_phase(int16_t delta_step, uint16_t centre_offset_steps,
                                          uint16_t lift_steps, Vector2f& leg_xy_target, float& leg_z_target)
{
    // 角度参数计算：将时间映射到[0,2π]范围，完成一个完整的摆线周期
    // 波浪步态优化：使用相对于抬腿阶段开始时间的偏移量
    float delta = M_2PI * (delta_step - centre_offset_steps) / lift_steps;

    // ==================== 波浪步态抬腿阶段摆线轨迹数学原理 ====================
    // X方向轨迹：使用摆线公式实现前后方向的运动
    // 公式：x = throttle_x_travel * (δ - sin(δ)) / 2π
    // 特性：δ=0时x=0，δ=π时x=throttle_x_travel/2，δ=2π时x=throttle_x_travel
    // 摆线特性：起点和终点速度为0，加速度连续，减少机械冲击
    leg_xy_target[0] = throttle_x_travel * (delta - sinf(delta)) / M_2PI;

    // Y方向轨迹：波浪步态中横向运动为0，专注于前后推进
    // 这种设计简化了波浪步态的控制，提高运动稳定性
    leg_xy_target[1] = 0;

    // Z方向轨迹：使用余弦函数实现抬腿高度的平滑变化
    // 公式：z = -3 * leg_lift_height * (1 - cos(δ))
    // 特性：
    // - δ=0时，z=0（地面起始）
    // - δ=π时，z=-6*leg_lift_height（最高抬腿点）
    // - δ=2π时，z=0（地面落地）
    // 系数3的设计：提供更高的抬腿高度，增强越障能力
    // 负号表示向上为负方向（符合机器人坐标系）
    leg_z_target = -3 * leg_lift_height * (1.0f - cosf(delta));
}

// 波浪步态支撑阶段处理函数
// 使用三次缓动函数实现平滑的支撑相运动，确保推进的稳定性
// 这是波浪步态中最长的阶段，占总时间的10/12，直接影响推进效率
void AP_QuadRuped_Wave::handle_support_phase(float support_s, Vector2f& leg_xy_target, float& leg_z_target)
{
    // ==================== 波浪步态支撑阶段三次缓动函数 ====================
    // 三次缓动公式：s = 3t² - 2t³，其中t∈[0,1]
    // 特性：
    // - t=0时，s=0，ds/dt=0（起始速度为0）
    // - t=1时，s=1，ds/dt=0（终止速度为0）
    // - 加速度连续，减少冲击力
    const float s = 3.0f * support_s * support_s - 2.0f * support_s * support_s * support_s;

    // X方向推进轨迹：从最大位移位置平滑回拖到原点
    // 公式：x = throttle_x_travel * (1 - s)
    // 物理意义：腿部从前支撑位置向后移动，推动机器人前进
    // 这种设计实现了波浪步态的推进效果
    leg_xy_target.x = throttle_x_travel * (1.0f - s); // 从前端回拖到 0

    // Y方向稳定性：保持横向位置不变，提高运动稳定性
    // 波浪步态特点：专注于前后推进，横向运动由其他机制控制
    leg_xy_target.y = 0.0f;

    // Z方向地面接触：强制为0，确保支撑相期间腿部始终与地面接触
    // 这是机器人稳定行走的必要条件，提供持续的地面反作用力
    leg_z_target = 0.0f; // 地面接触
}

// 波浪步态腿部运动更新函数
// 管理波浪步态的时序控制，为四条腿生成协调的轨迹
// 波浪步态特性：通过精确的时序控制实现连续的波浪推进效果
void AP_QuadRuped_Wave::update_leg()
{
    // ==================== 波浪步态时序控制 ====================
    // 更新步态计数器：波浪步态的全局时间基准
    // 改进为大步数时的连续增长，避免频繁循环造成的相位跳跃
    gait_step_now++;

    // 波浪步态循环处理：使用大整数范围避免频繁重置
    // 只有当步数接近int32上限时才重置，减少边界问题
    if (gait_step_now >= 100000000) { // 使用int32_t接近上限的值
        gait_step_now = 0;
        // 重置时同步调整所有腿的起始步数，保持相位关系
        for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
            gait_step_leg_start[i] = 0;
        }
        // 重新设置波浪步态的90度相位差
        gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4;     // 左前腿 - 90度相位
        gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 2;     // 左后腿 - 180度相位
        gait_step_leg_start[AP_QUADRUPED_LEG_RB] = 3 * gait_step_total / 4; // 右后腿 - 270度相位
    }

    // ==================== 波浪步态轨迹生成 ====================
    // 遍历所有四条腿，为每条腿生成协调的位置和旋转轨迹
    // 波浪步态特点：四条腿依次运动，形成连续的波浪推进效果
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        // 为每条腿生成位置轨迹：根据选择的算法（正弦/贝塞尔曲线）
        trajectory_generation(leg_index);

        // 为每条腿生成旋转轨迹：控制偏航运动时的腿部旋转
        // 波浪步态旋转特性：通过协调的腿部旋转实现平滑的转向
        yaw_trajectory_generation(leg_index);
    }

    // 波浪步态时序设计原理：
    // 1. 连续性：使用大整数计数器避免频繁重置，保证相位连续性
    // 2. 协调性：四条腿的轨迹生成在同一个时间步进行，确保运动同步
    // 3. 可扩展性：为后续添加更多控制功能预留时间接口
    // 4. 实时性：高效的循环结构，满足实时控制要求
}

void AP_QuadRuped_Wave::smooth_target()
{
    // 平滑跟随（不清零，不回中位）
    centre_offset.x = td_smooth[0].update(centre_offset_target.x);
    centre_offset.y = td_smooth[1].update(centre_offset_target.y);
    centre_offset.z = td_smooth[2].update(centre_offset_target.z);

    // centre_offset += (centre_offset_target - centre_offset) * alpha;
}

void AP_QuadRuped_Wave::main_inverse_kinematics()
{
    smooth_target();
    AP_QuadRuped_Backend::main_inverse_kinematics();
}

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_Wave::update()
{
    // if ((AP_HAL::millis() - lasttime) < (1000 / gait_hz)) {
    //     return;
    // }

    // // 更新最后执行时间
    // lasttime = AP_HAL::millis();

    // // 检查遥控器通道 6（CH_6）的值是否大于 1500（通常表示开关激活）并且没有解锁
    // if (_frontend.get_mode_channel() > 1800 && !_motors.armed()) {
    //     // 执行主控制器
    //     main_radio_controller();

    //     // 执行平衡控制器
    //     // balance_controller();

    //     // 执行逆运动学解算
    //     main_inverse_kinematics();

    //     // 输出腿部关节角度
    //     output_leg_angle();
    // } else {
    //     hengxiang_up_sleep_leg();
    // }

    main_radio_controller();

    // 执行平衡控制器
    // balance_controller();

    // 执行逆运动学解算
    main_inverse_kinematics();

    // 输出腿部关节角度
    output_leg_angle();

    // 发送数据
    send_servo_cmd();
}

// 平衡控制器 - 简单的平衡控制实现
void AP_QuadRuped_Wave::balance_controller()
{
    // 简单的平衡控制实现
    // 这里可以根据IMU数据调整重心偏移以保持平衡

    // 获取当前姿态数据
    const Vector3f& gyro  = _ahrs.get_gyro();
    const Vector3f& accel = _ahrs.get_accel_ef();

    // 计算需要的重心补偿（示例实现）
    // 这里可以根据实际的平衡控制算法进行调整
    centre_offset.x = constrain_float(gyro.y * 0.1f, -10.0f, 10.0f); // 基于横滚角速度补偿
    centre_offset.y = constrain_float(gyro.x * 0.1f, -10.0f, 10.0f); // 基于俯仰角速度补偿

    // 重心高度补偿（基于Z轴加速度）
    centre_offset.z = constrain_float(accel.z * 0.05f, -5.0f, 5.0f);
}

// ==================== 波浪步态统一轨迹计算函数 ====================
// 将重心移动完全整合到轨迹生成过程中的专用计算函数

// 波浪步态重心偏移计算函数（重心准备阶段）
// 根据腿部位置和运动方向计算重心偏移轨迹，实现重心和腿部的协调运动
Vector3f AP_QuadRuped_Wave::calculate_centre_offset_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f offset;

    // 使用平滑的缓动函数实现重心偏移的平滑过渡
    float smooth_phase = phase < 0.5f ? 4.0f * phase * phase * phase : 1.0f - powf(-2.0f * phase + 2.0f, 3.0f) / 2.0f;

    // 根据运动方向和腿部位置计算重心偏移
    if (throttle_travel.x < 0) {
        // 后退运动时的重心偏移策略
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF: // 右前腿
                offset = Vector3f(-30.0f * smooth_phase, -45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_LF: // 左前腿
                offset = Vector3f(-30.0f * smooth_phase, 45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_LB: // 左后腿
                offset = Vector3f(0.0f, 45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_RB: // 右后腿
                offset = Vector3f(0.0f, -45.0f * smooth_phase, 0.0f);
                break;
        }
    } else {
        // 前进运动时的重心偏移策略
        switch (leg_index) {
            case AP_QUADRUPED_LEG_RF: // 右前腿
                offset = Vector3f(0.0f, -45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_LF: // 左前腿
                offset = Vector3f(0.0f, 45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_LB: // 左后腿
                offset = Vector3f(30.0f * smooth_phase, 45.0f * smooth_phase, 0.0f);
                break;
            case AP_QUADRUPED_LEG_RB: // 右后腿
                offset = Vector3f(30.0f * smooth_phase, -45.0f * smooth_phase, 0.0f);
                break;
        }
    }

    return offset;
}

// 波浪步态腿部微调计算函数（重心准备阶段）
// 在重心调整时进行微小的腿部位置调整，确保重心移动的稳定性
Vector3f AP_QuadRuped_Wave::calculate_leg_adjustment_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f leg_position;

    // 在重心准备阶段，腿部进行微小的位置调整
    // 使用二次缓动函数实现平滑的微调运动
    float smooth_phase = phase * phase; // 二次缓动，开始慢后面快

    // 根据腿部位置计算微调量
    float adjustment_magnitude = 5.0f * smooth_phase; // 微调幅度

    switch (leg_index) {
        case AP_QUADRUPED_LEG_RF: // 右前腿
            leg_position = Vector3f(-adjustment_magnitude, -adjustment_magnitude * 0.5f, 0.0f);
            break;
        case AP_QUADRUPED_LEG_LF: // 左前腿
            leg_position = Vector3f(-adjustment_magnitude, adjustment_magnitude * 0.5f, 0.0f);
            break;
        case AP_QUADRUPED_LEG_LB: // 左后腿
            leg_position = Vector3f(adjustment_magnitude, adjustment_magnitude * 0.5f, 0.0f);
            break;
        case AP_QUADRUPED_LEG_RB: // 右后腿
            leg_position = Vector3f(adjustment_magnitude, -adjustment_magnitude * 0.5f, 0.0f);
            break;
    }

    return leg_position;
}

// 波浪步态重心协调计算函数（抬腿阶段）
// 在抬腿阶段继续协调重心位置，为腿部离地运动提供支撑
Vector3f AP_QuadRuped_Wave::calculate_centre_coordination_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f offset;

    // 在抬腿阶段，重心需要向支撑腿中心移动
    // 使用正弦函数实现平滑的重心协调运动
    float coordination_phase = sinf(M_PI * phase); // 正弦协调

    // 计算支撑腿的重心偏移量
    if (leg_index == AP_QUADRUPED_LEG_RF || leg_index == AP_QUADRUPED_LEG_LF) {
        // 前腿抬腿时，重心向后移动
        offset = Vector3f(20.0f * coordination_phase, 0.0f, 0.0f);
    } else {
        // 后腿抬腿时，重心向前移动
        offset = Vector3f(-20.0f * coordination_phase, 0.0f, 0.0f);
    }

    return offset;
}

// 波浪步态腿部抬升计算函数（抬腿阶段）
// 使用改进的摆线公式实现平滑的抬腿轨迹，确保起止点的速度和加速度连续
Vector3f AP_QuadRuped_Wave::calculate_leg_lift_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f leg_position;

    // 角度参数：将时间映射到[0,2π]范围，完成一个完整的摆线周期
    float delta = M_2PI * phase;

    // 使用摆线公式计算腿部轨迹
    leg_position.x = throttle_travel.x * (delta - sinf(delta)) / M_2PI;
    leg_position.y = 0.0f; // 波浪步态专注于前后推进
    leg_position.z = -3 * leg_lift_height * (1.0f - cosf(delta)); // 抬腿高度

    return leg_position;
}

// 波浪步态重心稳定计算函数（支撑阶段）
// 在支撑阶段保持重心稳定，为机器人提供稳定的支撑基础
Vector3f AP_QuadRuped_Wave::calculate_centre_stability_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f offset;

    // 在支撑阶段，重心逐渐回到中心位置
    // 使用三次缓动函数实现平滑的重心稳定运动
    float stability_phase = 3.0f * phase * phase - 2.0f * phase * phase * phase;

    // 根据腿部位置计算重心回中轨迹
    switch (leg_index) {
        case AP_QUADRUPED_LEG_RF: // 右前腿
            offset = Vector3f(20.0f * (1.0f - stability_phase), -30.0f * (1.0f - stability_phase), 0.0f);
            break;
        case AP_QUADRUPED_LEG_LF: // 左前腿
            offset = Vector3f(20.0f * (1.0f - stability_phase), 30.0f * (1.0f - stability_phase), 0.0f);
            break;
        case AP_QUADRUPED_LEG_LB: // 左后腿
            offset = Vector3f(-20.0f * (1.0f - stability_phase), 30.0f * (1.0f - stability_phase), 0.0f);
            break;
        case AP_QUADRUPED_LEG_RB: // 右后腿
            offset = Vector3f(-20.0f * (1.0f - stability_phase), -30.0f * (1.0f - stability_phase), 0.0f);
            break;
    }

    return offset;
}

// 波浪步态腿部支撑计算函数（支撑阶段）
// 使用三次缓动函数实现平滑的支撑相运动，确保推进的稳定性
Vector3f AP_QuadRuped_Wave::calculate_leg_support_wave(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    Vector3f leg_position;

    // 使用三次缓动函数实现平滑的支撑推进运动
    float support_phase = 3.0f * phase * phase - 2.0f * phase * phase * phase;

    // 支撑相腿部轨迹：从最大位移位置平滑回拖到原点
    leg_position.x = throttle_travel.x * (1.0f - support_phase);
    leg_position.y = 0.0f; // 专注于前后推进
    leg_position.z = 0.0f; // 地面接触

    return leg_position;
}

// gait_step本质上是一个离散化的时间变量，将连续的步态运动分解为多个离散的步骤
// 和逆运动学相互约束，逆运动学解算出相应的关节角，再通过轨迹生成生成轨迹
// 末段时间缩放函数：C2 连续，末端 v=a=0
float AP_QuadRuped_Wave::slow_phi(float s, float s0)
{
    if (s <= s0) return s;
    float sigma = (s - s0) / (1.0f - s0); // 0..1
    float w     = sigma
        + 4.0f * powf(sigma, 3.0f)
        - 7.0f * powf(sigma, 4.0f)
        + 3.0f * powf(sigma, 5.0f); // w(0)=0,w'(0)=1; w(1)=1,w'(1)=0
    return s0 + (1.0f - s0) * w;
}

// 三次贝塞尔曲线轨迹生成函数（波浪步态专用版本）
// 针对波浪步态的依次运动特性优化的贝塞尔曲线实现
// 与对角步态版本的主要区别：考虑波浪步态的相位差和运动协调性
Vector3f AP_QuadRuped_Wave::cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3)
{
    // 参数t的范围约束：确保t在[0,1]区间内，防止数值计算错误
    // 这是贝塞尔曲线的数学定义域要求，超出范围会导致轨迹失真
    t = constrain_float(t, 0.0f, 1.0f);

    // 预计算伯恩斯坦基函数，提高计算效率
    // 波浪步态优化：由于需要为四条腿依次计算，预计算更显重要
    const float mt = 1.0f - t;           // (1-t) 项，用于后续基函数计算
    const float mt2 = mt * mt;           // (1-t)²，避免重复乘法运算
    const float mt3 = mt2 * mt;          // (1-t)³，P₀控制点的权重系数
    const float t2 = t * t;              // t²，用于高次项计算
    const float t3 = t2 * t;             // t³，P₃控制点的权重系数
    const float _3mt2t = 3.0f * mt2 * t; // 3(1-t)²t，P₁控制点的权重系数
    const float _3mtt2 = 3.0f * mt * t2; // 3(1-t)t²，P₂控制点的权重系数

    // 三次贝塞尔曲线公式的实现
    // 波浪步态特性：为保证四条腿运动的协调性，使用统一的数学公式
    return p0 * mt3 + p1 * _3mt2t + p2 * _3mtt2 + p3 * t3;
}

// 波浪步态贝塞尔曲线统一轨迹生成器（重新设计版本）
// 将重心移动完全整合到贝塞尔曲线轨迹生成过程中，实现统一的协调控制
// 采用与正弦轨迹相同的时间分配：重心准备阶段(1/6) + 抬腿阶段(1/3) + 支撑阶段(1/2)
void AP_QuadRuped_Wave::generate_bezier_trajectory(uint8_t leg_index)
{
    // 波浪步态相位计算：统一的相位处理逻辑
    int32_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 相位循环处理：确保步数在合理范围内
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位：将离散步数转换为连续的相位值[0,1]
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 用户指令解析：将遥控器输入转换为期望的平面位移向量
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    Vector3f leg_target; // 存储计算得到的腿部目标位置

    // ==================== 相位差别的重心移动策略（贝塞尔版本）====================
    // 采用与正弦轨迹相同的相位差别策略，确保两种轨迹模式的协调性
    const float leg_phase_offset = (float)gait_step_leg_start[leg_index] / (float)gait_step_total;
    const float centre_phase_offset = leg_phase_offset * 0.25f; // 减小偏移幅度，避免过度分散

    // 动态时间分配：根据腿的相位调整重心移动的时间比例
    const float prepare_ratio = 0.167f + 0.033f * sinf(centre_phase_offset * M_2PI); // 1/6 ± 0.033
    const float lift_ratio = 0.333f + 0.025f * cosf(centre_phase_offset * M_2PI);   // 1/3 ± 0.025
    const float support_ratio = 1.0f - prepare_ratio - lift_ratio;                // 剩余时间

    if (p < prepare_ratio) {
        // ==================== 重心准备阶段（动态比例）====================
        const float phase = p / prepare_ratio; // 将[0,prepare_ratio]映射到[0,1]

        // 重心移动轨迹：使用贝塞尔曲线计算重心偏移
        Vector3f centre_offset_calc = calculate_centre_offset_wave_bezier(leg_index, phase, throttle_travel);
        centre_offset = centre_offset_calc;

        // 腿部微调轨迹：使用贝塞尔曲线计算腿部调整
        leg_target = calculate_leg_adjustment_wave_bezier(leg_index, phase, throttle_travel);

    } else if (p < (prepare_ratio + lift_ratio)) {
        // ==================== 抬腿阶段（动态比例）====================
        const float phase = (p - prepare_ratio) / lift_ratio; // 将[prepare_ratio, prepare_ratio+lift_ratio]映射到[0,1]

        // 重心协调轨迹：使用贝塞尔曲线计算重心协调
        Vector3f centre_offset_calc = calculate_centre_coordination_wave_bezier(leg_index, phase, throttle_travel);
        centre_offset = centre_offset_calc;

        // 腿部抬升轨迹：使用贝塞尔曲线计算抬腿运动
        leg_target = calculate_leg_lift_wave_bezier(leg_index, phase, throttle_travel);

    } else {
        // ==================== 支撑阶段（动态比例）====================
        const float phase = (p - prepare_ratio - lift_ratio) / support_ratio; // 将剩余时间映射到[0,1]

        // 重心稳定轨迹：使用贝塞尔曲线计算重心稳定
        Vector3f centre_offset_calc = calculate_centre_stability_wave_bezier(leg_index, phase, throttle_travel);
        centre_offset = centre_offset_calc;

        // 腿部支撑轨迹：使用贝塞尔曲线计算支撑推进
        leg_target = calculate_leg_support_wave_bezier(leg_index, phase, throttle_travel);
    }

    // 设置最终腿部位置
    gait_pos_xyz[leg_index] = leg_target;
}

// ==================== 贝塞尔曲线专用的统一轨迹计算函数 ====================
// 以下函数为贝塞尔曲线轨迹提供专门的计算实现，与正弦轨迹函数形成对应关系
// 主要区别：使用贝塞尔曲线算法替代三角函数，提供更灵活的轨迹形状控制

// 波浪步态重心偏移计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现平滑的重心偏移，为抬腿阶段做准备
Vector3f AP_QuadRuped_Wave::calculate_centre_offset_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的重心偏移计算
    // 控制点设计：创造平滑的重心转移轨迹

    // P0: 起始点 - 无偏移
    Vector3f p0 = Vector3f(0.0f, 0.0f, 0.0f);

    // P1: 第一个控制点 - 开始偏移
    float ctrl_height = bezier_control_height.get() * 5.0f; // 较小的重心高度变化
    Vector3f p1 = Vector3f(10.0f, 10.0f, ctrl_height);

    // P2: 第二个控制点 - 最大偏移
    Vector3f p2 = Vector3f(20.0f, 20.0f, ctrl_height);

    // P3: 终点 - 目标偏移位置
    Vector3f p3 = Vector3f(30.0f, 30.0f, 0.0f);

    // 根据腿的位置调整偏移方向
    switch (leg_index) {
        case AP_QUADRUPED_LEG_RF: // 右前腿
            p1.x = -10.0f; p1.y = -10.0f;
            p2.x = -20.0f; p2.y = -20.0f;
            p3.x = -30.0f; p3.y = -30.0f;
            break;
        case AP_QUADRUPED_LEG_LF: // 左前腿
            p1.y = 10.0f;
            p2.y = 20.0f;
            p3.y = 30.0f;
            break;
        case AP_QUADRUPED_LEG_LB: // 左后腿
            p1.x = 10.0f; p1.y = 10.0f;
            p2.x = 20.0f; p2.y = 20.0f;
            p3.x = 30.0f; p3.y = 30.0f;
            break;
        case AP_QUADRUPED_LEG_RB: // 右后腿
            p1.y = -10.0f;
            p2.y = -20.0f;
            p3.y = -30.0f;
            break;
    }

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase, p0, p1, p2, p3);
}

// 波浪步态腿部调整计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现腿部微调，为抬腿做准备
Vector3f AP_QuadRuped_Wave::calculate_leg_adjustment_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的腿部调整计算
    // 创造平滑的腿部预定位轨迹

    // P0: 起始点 - 原始位置
    Vector3f p0 = Vector3f(0.0f, 0.0f, 0.0f);

    // P1: 第一个控制点 - 开始调整
    Vector3f p1 = Vector3f(5.0f, 5.0f, -2.0f);

    // P2: 第二个控制点 - 最大调整
    Vector3f p2 = Vector3f(10.0f, 10.0f, -5.0f);

    // P3: 终点 - 调整后位置
    Vector3f p3 = Vector3f(15.0f, 15.0f, -8.0f);

    // 根据运动方向调整控制点
    if (throttle_travel.length() > 0.01f) {
        Vector2f direction = throttle_travel.normalized();
        p1.x = direction.x * 5.0f; p1.y = direction.y * 5.0f;
        p2.x = direction.x * 10.0f; p2.y = direction.y * 10.0f;
        p3.x = direction.x * 15.0f; p3.y = direction.y * 15.0f;
    }

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase, p0, p1, p2, p3);
}

// 波浪步态重心协调计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现抬腿过程中的重心协调
Vector3f AP_QuadRuped_Wave::calculate_centre_coordination_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的重心协调计算
    // 创造平滑的重心跟随轨迹

    // P0: 起始点 - 准备阶段的终点
    Vector3f p0 = Vector3f(30.0f, 30.0f, 0.0f);

    // P1: 第一个控制点 - 继续协调
    Vector3f p1 = Vector3f(35.0f, 35.0f, 2.0f);

    // P2: 第二个控制点 - 协调峰值
    Vector3f p2 = Vector3f(40.0f, 40.0f, 3.0f);

    // P3: 终点 - 协调终点
    Vector3f p3 = Vector3f(45.0f, 45.0f, 0.0f);

    // 根据腿的位置调整协调方向
    switch (leg_index) {
        case AP_QUADRUPED_LEG_RF: // 右前腿
            p0.x = -30.0f; p0.y = -30.0f;
            p1.x = -35.0f; p1.y = -35.0f;
            p2.x = -40.0f; p2.y = -40.0f;
            p3.x = -45.0f; p3.y = -45.0f;
            break;
        case AP_QUADRUPED_LEG_LF: // 左前腿
            p0.y = 30.0f;
            p1.y = 35.0f;
            p2.y = 40.0f;
            p3.y = 45.0f;
            break;
        case AP_QUADRUPED_LEG_LB: // 左后腿
            p0.x = 30.0f; p0.y = 30.0f;
            p1.x = 35.0f; p1.y = 35.0f;
            p2.x = 40.0f; p2.y = 40.0f;
            p3.x = 45.0f; p3.y = 45.0f;
            break;
        case AP_QUADRUPED_LEG_RB: // 右后腿
            p0.y = -30.0f;
            p1.y = -35.0f;
            p2.y = -40.0f;
            p3.y = -45.0f;
            break;
    }

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase, p0, p1, p2, p3);
}

// 波浪步态腿部抬升计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现平滑的抬腿轨迹
Vector3f AP_QuadRuped_Wave::calculate_leg_lift_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的抬腿轨迹计算
    // 创造平滑的抬腿和落地轨迹

    // 控制点参数
    float ctrl_height = bezier_control_height.get() * leg_lift_height;
    float ctrl_forward = bezier_control_forward.get() * throttle_travel.length();

    // P0: 起始点 - 地面位置
    Vector3f p0 = Vector3f(0.0f, 0.0f, 0.0f);

    // P1: 第一个控制点 - 抬腿开始
    Vector3f p1 = Vector3f(ctrl_forward * 0.3f, 0.0f, ctrl_height * 0.5f);

    // P2: 第二个控制点 - 抬腿峰值
    Vector3f p2 = Vector3f(ctrl_forward * 0.7f, 0.0f, ctrl_height);

    // P3: 终点 - 落地位置
    Vector3f p3 = Vector3f(throttle_travel.x, throttle_travel.y, 0.0f);

    // 应用运动平滑处理
    float phase_smooth = slow_phi(phase, 0.85f);

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase_smooth, p0, p1, p2, p3);
}

// 波浪步态重心稳定计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现支撑阶段的重心稳定
Vector3f AP_QuadRuped_Wave::calculate_centre_stability_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的重心稳定计算
    // 创造平滑的重心回归轨迹

    // P0: 起始点 - 协调阶段的终点
    Vector3f p0 = Vector3f(45.0f, 45.0f, 0.0f);

    // P1: 第一个控制点 - 开始稳定
    Vector3f p1 = Vector3f(30.0f, 30.0f, 1.0f);

    // P2: 第二个控制点 - 继续稳定
    Vector3f p2 = Vector3f(15.0f, 15.0f, 0.5f);

    // P3: 终点 - 回归原点
    Vector3f p3 = Vector3f(0.0f, 0.0f, 0.0f);

    // 根据腿的位置调整稳定方向
    switch (leg_index) {
        case AP_QUADRUPED_LEG_RF: // 右前腿
            p0.x = -45.0f; p0.y = -45.0f;
            p1.x = -30.0f; p1.y = -30.0f;
            p2.x = -15.0f; p2.y = -15.0f;
            break;
        case AP_QUADRUPED_LEG_LF: // 左前腿
            p0.y = 45.0f;
            p1.y = 30.0f;
            p2.y = 15.0f;
            break;
        case AP_QUADRUPED_LEG_LB: // 左后腿
            p0.x = 45.0f; p0.y = 45.0f;
            p1.x = 30.0f; p1.y = 30.0f;
            p2.x = 15.0f; p2.y = 15.0f;
            break;
        case AP_QUADRUPED_LEG_RB: // 右后腿
            p0.y = -45.0f;
            p1.y = -30.0f;
            p2.y = -15.0f;
            break;
    }

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase, p0, p1, p2, p3);
}

// 波浪步态腿部支撑计算函数（贝塞尔曲线版本）
// 使用贝塞尔曲线实现支撑推进的平滑轨迹
Vector3f AP_QuadRuped_Wave::calculate_leg_support_wave_bezier(uint8_t leg_index, float phase, const Vector2f& throttle_travel)
{
    // 使用贝塞尔曲线的支撑推进计算
    // 创造平滑的推进和回拉轨迹

    // P0: 起始点 - 前支撑位置
    Vector3f p0 = Vector3f(throttle_travel.x, throttle_travel.y, 0.0f);

    // P1: 第一个控制点 - 推进开始
    Vector3f p1 = Vector3f(throttle_travel.x * 0.8f, throttle_travel.y * 0.8f, 0.0f);

    // P2: 第二个控制点 - 推进中段
    Vector3f p2 = Vector3f(throttle_travel.x * 0.3f, throttle_travel.y * 0.3f, 0.0f);

    // P3: 终点 - 后支撑位置
    Vector3f p3 = Vector3f(0.0f, 0.0f, 0.0f);

    // 应用贝塞尔曲线计算
    return cubic_bezier_trajectory(phase, p0, p1, p2, p3);
}