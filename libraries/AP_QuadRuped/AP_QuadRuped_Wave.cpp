#include "AP_QuadRuped_Wave.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        25.0f // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 波浪步态参数表定义
const AP_Param::GroupInfo AP_QuadRuped_Wave::var_info[] = {
    // ==================== 基础步态参数 ====================
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Wave, gait_hz, SPEED_HZ_DEFAULT),
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_Wave, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    // ==================== 轨迹生成模式选择 ====================
    AP_GROUPINFO("TRAJ_MODE", 3, AP_QuadRuped_Wave, trajectory_mode, 0),

    // ==================== 重心偏移参数 ====================
    AP_GROUPINFO("COG_OFFSET", 4, AP_QuadRuped_Wave, centre_offset_ratio, 0.4f),

    // ==================== 贝塞尔曲线控制参数 ====================
    AP_GROUPINFO("BCTRL_H", 5, AP_QuadRuped_Wave, bezier_control_height, 0.3f),
    AP_GROUPINFO("BCTRL_F", 6, AP_QuadRuped_Wave, bezier_control_forward, 0.2f),

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Wave::AP_QuadRuped_Wave(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    // 初始化相位缓存
    for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
        phase_cache[i].angle       = 0.0f;
        phase_cache[i].sin_val     = 0.0f;
        phase_cache[i].cos_val     = 1.0f;
        phase_cache[i].last_update = 0;
        phase_cache[i].valid       = false;
    }
}

// 步态初始化
void AP_QuadRuped_Wave::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_Wave init - Single Leg Swing");

    // 设置每条腿的起始步数 - 波浪步态：90度相位差，确保单腿摆动
    // RF → LF → LB → RB 依次抬起，摆动相占25%，支撑相占75%，保证稳定性
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                           // 右前腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4;         // 左前腿从1/4周期后开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 2;         // 左后腿从1/2周期后开始
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = gait_step_total * 3 / 4;     // 右后腿从3/4周期后开始

    // 设置步态参数 - 适配25%摆动相设计
    gait_travel_divisor = gait_step_total / 2; // 行程除数
    // 采用3让现有yaw与lift计算与1/3占比一致，避免后续比例再额外换算
    gait_lift_divisor   = 3;                   // 抬腿除数：3约等于摆动相占1/3 (33%)
}

// 获取活跃腿索引
uint8_t AP_QuadRuped_Wave::get_active_leg_index()
{
    uint16_t cycle_position = gait_step_now % gait_step_total;

    if (cycle_position < gait_step_total / 4) {
        return AP_QUADRUPED_LEG_RF;
    } else if (cycle_position < gait_step_total / 2) {
        return AP_QUADRUPED_LEG_LF;
    } else if (cycle_position < 3 * gait_step_total / 4) {
        return AP_QUADRUPED_LEG_LB;
    } else {
        return AP_QUADRUPED_LEG_RB;
    }
}

// 更新腿部运动
void AP_QuadRuped_Wave::update_leg()
{
    // 更新步态计数器 - 改进为大步数时的连续增长
    gait_step_now++;

    // 使用大整数范围避免频繁循环，减少相位跳跃
    // 只有当步数超过很大值时才重置，避免边界问题
    if (gait_step_now >= 100000000) { // 使用int32_t接近上限的值
        gait_step_now = 0;
        // 重置时同步调整所有腿的起始步数，保持相位关系
        for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
            gait_step_leg_start[i] = 0;
        }
        // 重新设置波浪步态相位
        gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 4;
        gait_step_leg_start[AP_QUADRUPED_LEG_LB] = gait_step_total / 2;
        gait_step_leg_start[AP_QUADRUPED_LEG_RB] = gait_step_total * 3 / 4;
    }

    // 首先计算当前活跃腿（摆动腿）
    uint8_t swing_leg = get_active_leg_index();

    // 基于支撑多边形计算全局重心偏移
    calculate_support_polygon_centre_offset(swing_leg);

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

// 轨迹生成统一接口函数
// 作为步态控制系统的核心调度器，根据用户参数选择合适的轨迹生成算法
// 这种设计模式实现了算法的可插拔性，便于后续扩展新的轨迹类型
void AP_QuadRuped_Wave::trajectory_generation(uint8_t leg_index)
{
    // 轨迹模式决策：根据参数选择轨迹生成算法
    // trajectory_mode.get() 返回用户设置的轨迹模式值
    // 这种条件判断结构便于后续扩展更多轨迹类型

    if (trajectory_mode.get() == 1) {
        // ==================== 贝塞尔曲线轨迹模式 ====================
        // 适用场景：复杂地形、需要精细控制轨迹形状、特殊运动需求
        // 优势：形状灵活、参数可调、支持复杂轨迹
        // 劣势：计算开销稍大、参数调试复杂
        generate_bezier_trajectory(leg_index);

    } else {
        // ==================== 默认摆线轨迹模式 ====================
        // 适用场景：平坦地面、追求运动稳定性、计算资源受限
        // 优势：计算简单、运动平稳、参数直观
        // 劣势：轨迹形状固定、灵活性有限
        // 默认模式确保系统向后兼容性和基本功能稳定性
        generate_cycloid_trajectory(leg_index);
    }

    // 设计原理说明：
    // 1. 模式切换：通过参数即可实现轨迹算法的实时切换，无需重启系统
    // 2. 接口统一：所有轨迹生成函数都接受相同的leg_index参数，保证接口一致性
    // 3. 性能考虑：条件判断在运行时开销极小，不影响实时性要求
    // 4. 扩展性：可以轻松添加新的else if分支来支持更多轨迹类型
}

void AP_QuadRuped_Wave::get_phase_ratios(uint8_t leg_index, float& prepare_ratio, float& lift_ratio, float& support_ratio) const
{
    // 以研发笔记中的1/6准备、1/3抬腿作为基准，确保时间分配满足设计目标
    const float base_prepare = 1.0f / 6.0f;
    const float base_lift    = 1.0f / 3.0f;

    // 使用可调变量存储计算结果，便于后续根据相位偏移动态调整
    float prepare = base_prepare;
    float lift    = base_lift;

    if (leg_index < AP_QUADRUPED_LEG_ALL && gait_step_total.get() > 0) {
        // 通过腿的起始步数估算相位偏移，保持四条腿的波浪顺序
        const float leg_phase_offset    = (float)gait_step_leg_start[leg_index] / (float)gait_step_total.get();
        // 将腿相位映射为重心相位的一部分，实现“波浪式”重心传递
        const float centre_phase_offset = leg_phase_offset * 0.25f;

        // 小幅正弦调制准备阶段时长，以改善对角腿支撑能力
        prepare += 0.025f * sinf(centre_phase_offset * M_2PI);
        // 余弦调制抬腿阶段时长，从而在不同相位保持摆动时刻的柔性
        lift    += 0.020f * cosf(centre_phase_offset * M_2PI);
    }

    // 夹紧准备阶段时间，避免时间过短导致重心尚未移稳
    prepare = constrain_float(prepare, 0.12f, 0.22f);
    // 夹紧抬腿阶段时间，防止过长影响支撑时间
    lift    = constrain_float(lift,    0.30f, 0.36f);

    // 支撑阶段由剩余时间得到，保证三个阶段总和为1
    float support = 1.0f - prepare - lift;

    if (support < 0.38f) {
        // 支撑时间过短会削弱稳定性，这里提至安全下限
        support = 0.38f;
        const float remain = 1.0f - support;
        const float sum    = prepare + lift;
        if (sum <= 0.0f) {
            // 极端情况下平均分配剩余时间，防止除零
            prepare = remain * 0.5f;
            lift    = remain * 0.5f;
        } else {
            // 正常情况下按原始比例缩放，保持相对关系
            const float scale = remain / sum;
            prepare *= scale;
            lift    *= scale;
        }
    } else if (support > 0.60f) {
        // 支撑阶段也不宜过长，避免动作显得拖沓
        support = 0.60f;
        const float remain = 1.0f - support;
        const float sum    = prepare + lift;
        if (sum <= 0.0f) {
            // 同上，防御性处理
            prepare = remain * 0.5f;
            lift    = remain * 0.5f;
        } else {
            // 按比例缩放，使准备/抬腿保持原有占比
            const float scale = remain / sum;
            prepare *= scale;
            lift    *= scale;
        }
    }

    // 归一化处理，保证三个阶段的权重之和为1
    const float sum = prepare + lift + support;
    if (sum > 0.0f) {
        // 除法前先检查，防止数值异常
        prepare /= sum;
        lift    /= sum;
        support /= sum;
    }

    prepare_ratio = prepare;
    lift_ratio    = lift;
    support_ratio = support;
}

// 偏航轨迹生成
void AP_QuadRuped_Wave::yaw_trajectory_generation(uint8_t leg_index)
{
    int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];
    if (delta_step < 0) delta_step += gait_step_total;

    // 将当前步数转为0-1相位，便于和阶段比例统一处理
    const float p    = (float)delta_step / (float)gait_step_total;
    // yaw_travel 与 gait_lift_divisor 的比值用于保持既有幅度设置
    const float peak = yaw_travel / (float)gait_lift_divisor;

    float prepare_ratio = 0.0f;
    float lift_ratio    = 0.0f;
    float support_ratio = 1.0f;
    get_phase_ratios(leg_index, prepare_ratio, lift_ratio, support_ratio);

    // 在准备阶段前半段禁止偏航，防止未移稳即扭转机体
    const float prepare_mid = prepare_ratio * 0.5f;
    // 抬腿末端用于判断支撑阶段起点
    const float lift_end    = prepare_ratio + lift_ratio;

    if (p < prepare_mid) {
        // 前半准备阶段保持0偏航，保障重心移动不被扰动
        gait_rot_z[leg_index] = 0.0f;
        return;
    }

    if (p < prepare_ratio) {
        // 准备阶段末端采用余弦缓入，避免扭矩突变
        const float denom = fmaxf(prepare_ratio - prepare_mid, 1e-3f);
        const float stage = (p - prepare_mid) / denom;
        gait_rot_z[leg_index] = peak * 0.5f * (1.0f - cosf(constrain_float(stage, 0.0f, 1.0f) * M_PI));
        return;
    }

    if (p < lift_end) {
        // 抬腿阶段保持峰值偏航，便于空中对齐方向
        gait_rot_z[leg_index] = peak;
        return;
    }

    const float support_denom = fmaxf(support_ratio, 1e-3f);
    float support_phase       = (p - lift_end) / support_denom;
    support_phase             = constrain_float(support_phase, 0.0f, 1.0f);
    // 支撑阶段线性回零，使机体逐渐回正
    gait_rot_z[leg_index]     = peak * (1.0f - support_phase);
}

// 摆线轨迹生成器（波浪步态版本）
// 使用经典的摆线（cycloid）曲线生成腿部轨迹，这是四足机器人领域最常用的轨迹算法
// 波浪步态特点：单腿依次运动，需要配合重心偏移保持稳定
void AP_QuadRuped_Wave::generate_cycloid_trajectory(uint8_t leg_index)
{
    // 步态相位计算：与贝塞尔曲线轨迹相同的相位处理逻辑
    // 这种统一设计保证了不同轨迹算法之间的相位一致性
    int32_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 相位循环处理：确保步数在有效范围内，避免整数溢出和相位跳跃
    // 先处理负数再取模，保证数学上的正确性和连续性
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位转换：将离散步数映射到连续的[0,1]区间
    // 这个p值是整个轨迹生成的时间基准，驱动后续所有的数学计算
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 用户指令解析：将遥控器的X/Y输入转换为期望的2D位移向量
    // 这个向量决定了机器人在平面上的运动意图
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    // 临时变量：存储计算得到的腿部目标位置（XY平面和Z轴高度）
    Vector2f leg_xy_target;  // XY平面的目标位置
    float    leg_z_target = 0.0f; // Z轴高度，默认为0（地面）

    // 重心偏移已在update_leg()中统一计算，此处无需重复调用

    float prepare_ratio = 0.0f;
    float lift_ratio    = 0.0f;
    float support_ratio = 1.0f;
    // 通过统一函数获取阶段比例，保证重心、姿态、腿部动作一致
    get_phase_ratios(leg_index, prepare_ratio, lift_ratio, support_ratio);

    const float prepare_end = prepare_ratio;
    const float lift_end    = prepare_ratio + lift_ratio;

    const Vector2f stance_start(-throttle_travel.x, -throttle_travel.y);
    const Vector2f stance_end(throttle_travel.x, throttle_travel.y);

    if (p < prepare_end) {
        // 重心调整阶段：保持腿部着地，允许轻微的顺应调整
        const float settle_phase = (prepare_end > 1e-3f) ? (p / prepare_end) : 0.0f;
        // 余弦插值保证起步加速度平顺，0.1系数限制脚尖滑动幅度
        const float micro_blend  = 0.1f * (1.0f - cosf(constrain_float(settle_phase, 0.0f, 1.0f) * M_PI)) * 0.5f;
        leg_xy_target            = stance_start + (stance_end - stance_start) * micro_blend;
        leg_z_target             = 0.0f;

    } else if (p < lift_end) {
        // 抬腿阶段：按照摆线轨迹抬腿
        // 相位归一化后可复用摆线公式，避免硬编码时间比例
        const float phase      = (p - prepare_end) / fmaxf(lift_ratio, 1e-3f);
        const float phase_slow = slow_phi(constrain_float(phase, 0.0f, 1.0f), 0.80f);
        const float delta      = M_2PI * phase_slow;
        // 按照摆线推导计算XY位移，使起落速度为零
        const float S          = (delta - sinf(delta)) / M_2PI * 2.0f;

        leg_xy_target = throttle_travel * S - throttle_travel;
        // 采用余弦抬升，确保最高点速度为零、落地柔和
        leg_z_target  = -leg_lift_height * (1.0f - cosf(delta));

    } else {
        // 支撑阶段：保持地面接触并提供推进
        const float phase = (p - lift_end) / fmaxf(support_ratio, 1e-3f);
        const float delta = M_2PI * constrain_float(phase, 0.0f, 1.0f);
        // 继续使用摆线使支撑期运动与摆动期镜像，保证连续性
        const float S     = (delta - sinf(delta)) / M_2PI * 2.0f;

        leg_xy_target = -throttle_travel * S + throttle_travel;
        leg_z_target  = 0.0f;
    }

    // 结果输出：将计算得到的3D位置存储到全局数组中
    // 这个位置将被逆运动学解算使用，最终转换为各个关节的角度指令
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 贝塞尔曲线轨迹生成器（波浪步态版本）
// 为单条腿生成完整的步态轨迹，包含摆动相（空中）和支撑相（地面）
// 波浪步态特点：单腿依次运动，需要配合重心偏移保持稳定
void AP_QuadRuped_Wave::generate_bezier_trajectory(uint8_t leg_index)
{
    // 步态相位计算：计算当前腿相对于其起始步数的相位偏移
    // 这种设计允许每条腿有不同的起始时间，实现波浪步态的相位控制
    int32_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

    // 相位循环处理：确保步数在合理范围内，避免整数溢出
    // 使用while循环而不是模运算直接处理负数，保证数学正确性
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    // 模运算实现周期性步态，使步数在[0, gait_step_total)范围内循环
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位：将离散步数转换为连续的相位值[0,1]
    // 这是整个轨迹生成的时间基准，0表示步态开始，1表示步态结束
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 用户指令解析：将遥控器输入转换为期望的平面位移向量
    // throttle_x_travel: 前后方向位移（正值向前，负值向后）
    // throttle_y_travel: 左右方向位移（正值向右，负值向左）
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    Vector3f leg_target; // 存储计算得到的腿部目标位置

    // 重心偏移已在update_leg()中统一计算，此处无需重复调用

    float prepare_ratio = 0.0f;
    float lift_ratio    = 0.0f;
    float support_ratio = 1.0f;
    // 调用共享函数，确保与偏航、重心的阶段划分一致
    get_phase_ratios(leg_index, prepare_ratio, lift_ratio, support_ratio);

    const float prepare_end = prepare_ratio;
    const float lift_end    = prepare_ratio + lift_ratio;

    const Vector3f stance_start(-throttle_travel.x, -throttle_travel.y, 0.0f);
    const Vector3f stance_end(throttle_travel.x, throttle_travel.y, 0.0f);

    // 步态相位判断：先调整重心，再抬腿，最后支撑推行
    if (p < prepare_end) {
        const float settle_phase = (prepare_end > 1e-3f) ? (p / prepare_end) : 0.0f;
        // 以小幅余弦混合限制脚部滑动，避免准备阶段出现大位移
        const float micro_blend  = 0.1f * (1.0f - cosf(constrain_float(settle_phase, 0.0f, 1.0f) * M_PI)) * 0.5f;
        leg_target               = stance_start + (stance_end - stance_start) * micro_blend;

    } else if (p < lift_end) { // 摆动相：腿部离地，在空中移动
        const float phase = (p - prepare_end) / fmaxf(lift_ratio, 1e-3f); // 将抬腿阶段映射到[0,1]

        // ==================== 贝塞尔曲线控制点定义 ====================
        // 四个控制点设计原则：
        // P0和P3保证精确的起止位置，P1和P2控制轨迹形状和运动特性
        // 这种设计实现了"低抬腿、缓落地"的机器人步态要求

        // P0: 起始点 - 后支撑位置，确保腿从地面精确开始
        // 坐标系：机器人中心为原点，X轴向前，Y轴向右，Z轴向上
        // 负值表示相对于机器人中心向后的位置
        Vector3f p0 = Vector3f(-throttle_travel.x, -throttle_travel.y, 0.0f);

        // P1: 第一个控制点 - 抬腿控制点，控制抬腿初期的轨迹
        // 参数化设计：通过可调参数控制轨迹形状
        float ctrl_height = bezier_control_height.get() * leg_lift_height; // 抬腿高度：基于用户设定的抬腿高度进行比例调节
        // 以位移长度调节前向控制点，保证高速时仍能提前摆脚
        float ctrl_forward = bezier_control_forward.get() * throttle_travel.length(); // 前向偏移：基于行程长度进行比例调节
        // P1坐标设计：稍微前移并抬升，创造平滑的抬腿轨迹
        // X坐标：起始位置+30%的前向偏移，避免突然抬腿
        // Y坐标：保持起始Y位置，确保横向稳定性
        // Z坐标：控制抬腿高度，决定越障能力
        Vector3f p1 = Vector3f(-throttle_travel.x + ctrl_forward * 0.3f,
                               -throttle_travel.y,
                               ctrl_height);

        // P2: 第二个控制点 - 空中过渡点，控制落地前的轨迹
        // 对称设计：与P1形成对称的轨迹控制
        // X坐标：目标位置-30%的前向偏移，为落地做准备
        // Y坐标：目标Y位置，保证横向运动准确性
        // Z坐标：保持抬腿高度，确保空中平移距离
        Vector3f p2 = Vector3f(throttle_travel.x - ctrl_forward * 0.3f,
                               throttle_travel.y,
                               ctrl_height);

        // P3: 终点 - 前支撑位置，确保腿精确落地到目标位置
        // 正值表示相对于机器人中心向前的位置
        Vector3f p3 = Vector3f(throttle_travel.x, throttle_travel.y, 0.0f);

        // 运动平滑处理：使用slow_phi函数实现末端减速
        // 0.85f参数表示在85%的摆动相行程开始减速，确保轻柔落地
        // 这种设计可以减少冲击力，保护机械结构，提高运动平稳性
        const float phase_smooth = slow_phi(constrain_float(phase, 0.0f, 1.0f), 0.85f);

        // 轨迹生成：调用贝塞尔曲线函数计算实际位置
        // phase_smooth是经过时间缩放的参数，确保运动学特性符合要求
        leg_target = cubic_bezier_trajectory(phase_smooth, p0, p1, p2, p3);

    } else { // 支撑相：腿部着地，推动机器人前进
        const float phase = (p - lift_end) / fmaxf(support_ratio, 1e-3f); // 将支撑阶段映射到[0,1]
        const float phase_clamped = constrain_float(phase, 0.0f, 1.0f);

        // 支撑相采用线性轨迹设计
        // 线性轨迹的优势：计算简单，确保地面接触的稳定性
        // 从前支撑位置向后移动，实现推动机器人前进的效果

        // X方向：前向位移的线性衰减，从最大值递减到0
        // (1.0f - phase)确保从行程终点平滑返回到起点
        // 使用clamp后可避免超出范围导致回程抖动
        leg_target.x = throttle_travel.x * (1.0f - phase_clamped);

        // Y方向：横向位移的线性衰减，保持侧向运动的协调性
        leg_target.y = throttle_travel.y * (1.0f - phase_clamped);

        // Z方向：强制为0，确保支撑相期间腿部始终与地面接触
        // 这是机器人稳定行走的必要条件
        leg_target.z = 0.0f;
    }

    // 输出结果：将计算得到的腿部目标位置存储到全局数组中
    // 这个数组将被逆运动学解算使用，最终转换为关节角度
    gait_pos_xyz[leg_index] = leg_target;
}

// 三次贝塞尔曲线计算
Vector3f AP_QuadRuped_Wave::cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3)
{
    t = constrain_float(t, 0.0f, 1.0f);

    const float mt     = 1.0f - t;
    const float mt2    = mt * mt;
    const float mt3    = mt2 * mt;
    const float t2     = t * t;
    const float t3     = t2 * t;
    const float _3mt2t = 3.0f * mt2 * t;
    const float _3mtt2 = 3.0f * mt * t2;

    return p0 * mt3 + p1 * _3mt2t + p2 * _3mtt2 + p3 * t3;
}

// 支撑多边形重心计算（改进版：先调重心，后摆腿）
// 提前计算重心偏移，确保在摆动开始前重心已经调整到位
void AP_QuadRuped_Wave::calculate_support_polygon_centre_offset(uint8_t swing_leg)
{
    // 获取用户设定的重心偏移比例（0.0-1.0）
    float offset_ratio = centre_offset_ratio.get();

    // 定义四条腿在机器人坐标系中的理想位置
    Vector3f leg_positions[AP_QUADRUPED_LEG_ALL] = {
        Vector3f(-30.0f, -20.0f, 0.0f), // RF: 右前腿 (后右)
        Vector3f(-30.0f,  20.0f, 0.0f), // LF: 左前腿 (后左)
        Vector3f( 30.0f,  20.0f, 0.0f), // LB: 左后腿 (前左)
        Vector3f( 30.0f, -20.0f, 0.0f)  // RB: 右后腿 (前右)
    };

    // 计算支撑三角形的几何重心（即将支撑的三条腿）
    Vector3f support_centre(0.0f, 0.0f, 0.0f);
    uint8_t support_leg_count = 0;

    // 遍历所有腿，累加支撑腿的位置
    for (uint8_t i = 0; i < AP_QUADRUPED_LEG_ALL; i++) {
        if (i != swing_leg) { // 如果不是摆动腿，则为支撑腿
            support_centre += leg_positions[i];
            support_leg_count++;
        }
    }

    // 计算支撑三角形的几何中心
    if (support_leg_count > 0) {
        support_centre /= support_leg_count;
    }

    // 计算从机器人中心到支撑中心的偏移向量
    Vector3f desired_offset = support_centre - Vector3f(0.0f, 0.0f, 0.0f);

    // 计算当前摆动腿的相位，用于权重计算
    int32_t delta_step = gait_step_now - gait_step_leg_start[swing_leg];
    while (delta_step < 0) delta_step += gait_step_total;
    delta_step = delta_step % gait_step_total.get();
    float phase = (float)delta_step / (float)gait_step_total;

    float prepare_ratio = 0.0f;
    float lift_ratio    = 0.0f;
    float support_ratio = 1.0f;
    // 重用阶段比例，保证重心控制与腿动作同步
    get_phase_ratios(swing_leg, prepare_ratio, lift_ratio, support_ratio);

    const float prepare_end = prepare_ratio;
    const float lift_end    = prepare_ratio + lift_ratio;

    // 改进的重心偏移时序：提前开始重心调整
    float offset_weight = 0.0f;

    if (phase < prepare_end) {
        // 准备阶段用余弦缓启动，实现先移稳再抬腿
        const float prep_phase = (prepare_end > 1e-3f) ? (phase / prepare_end) : 0.0f;
        offset_weight = 0.5f * (1.0f - cosf(constrain_float(prep_phase, 0.0f, 1.0f) * M_PI));

    } else if (phase < lift_end) {
        // 抬腿阶段保持满偏移，最大化支撑多边形稳定性
        offset_weight = 1.0f;

    } else {
        // 支撑阶段按比例回收重心，0.6之前保持支撑稳定
        float support_phase = (phase - lift_end) / fmaxf(support_ratio, 1e-3f);
        support_phase       = constrain_float(support_phase, 0.0f, 1.0f);

        if (support_phase < 0.6f) {
            offset_weight = 1.0f;
        } else {
            // 最后40%使用余弦回中，避免出现突然回摆
            const float return_phase = (support_phase - 0.6f) / 0.4f;
            offset_weight = 0.5f * (1.0f + cosf(constrain_float(return_phase, 0.0f, 1.0f) * M_PI));
        }
    }

    // 应用重心偏移：朝支撑三角形中心偏移
    centre_offset = desired_offset * offset_weight * offset_ratio;

    // 限制最大偏移量，防止过度移动影响稳定性
    const float MAX_OFFSET_X = 30.0f;
    const float MAX_OFFSET_Y = 20.0f;
    const float MAX_OFFSET_Z = 5.0f;

    centre_offset.x = constrain_float(centre_offset.x, -MAX_OFFSET_X, MAX_OFFSET_X);
    centre_offset.y = constrain_float(centre_offset.y, -MAX_OFFSET_Y, MAX_OFFSET_Y);
    centre_offset.z = constrain_float(centre_offset.z, -MAX_OFFSET_Z, MAX_OFFSET_Z);
}

// 主逆运动学
void AP_QuadRuped_Wave::main_inverse_kinematics()
{
    // 执行逆运动学（重心偏移已在轨迹生成中计算）
    AP_QuadRuped_Backend::main_inverse_kinematics();
}

// 主更新函数
void AP_QuadRuped_Wave::update()
{
    main_radio_controller();
    main_inverse_kinematics();
    output_leg_angle();
    send_servo_cmd();
}

// 末端缓动函数
float AP_QuadRuped_Wave::slow_phi(float s, float s0)
{
    if (s <= s0) return s;
    float sigma = (s - s0) / (1.0f - s0);
    float w     = sigma
        + 4.0f * powf(sigma, 3.0f)
        - 7.0f * powf(sigma, 4.0f)
        + 3.0f * powf(sigma, 5.0f);
    return s0 + (1.0f - s0) * w;
}

// 平衡控制器
void AP_QuadRuped_Wave::balance_controller()
{
    const Vector3f& gyro  = _ahrs.get_gyro();
    const Vector3f& accel = _ahrs.get_accel_ef();

    centre_offset.x = constrain_float(gyro.y * 0.1f, -10.0f, 10.0f);
    centre_offset.y = constrain_float(gyro.x * 0.1f, -10.0f, 10.0f);
    centre_offset.z = constrain_float(accel.z * 0.05f, -5.0f, 5.0f);
}

// 相位缓存更新
void AP_QuadRuped_Wave::update_phase_cache(float angle, uint8_t leg_index)
{
    if (leg_index >= AP_QUADRUPED_LEG_ALL) return;

    uint32_t now = AP_HAL::millis();

    if (phase_cache[leg_index].valid && (now - phase_cache[leg_index].last_update < 10) && (fabsf(angle - phase_cache[leg_index].angle) < 0.001f)) {
        return;
    }

    phase_cache[leg_index].angle       = angle;
    phase_cache[leg_index].sin_val     = sinf(angle);
    phase_cache[leg_index].cos_val     = cosf(angle);
    phase_cache[leg_index].last_update = now;
    phase_cache[leg_index].valid       = true;
}

// 获取缓存的sin值
float AP_QuadRuped_Wave::get_cached_sin(float angle, uint8_t leg_index)
{
    update_phase_cache(angle, leg_index);
    return phase_cache[leg_index].sin_val;
}

// 获取缓存的cos值
float AP_QuadRuped_Wave::get_cached_cos(float angle, uint8_t leg_index)
{
    update_phase_cache(angle, leg_index);
    return phase_cache[leg_index].cos_val;
}
