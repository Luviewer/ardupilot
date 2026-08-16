#include "AP_HexRuped_Tripod.h"
#include "AP_HexRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        AP_HEXRUPED_SPEED_HZ_DEFAULT // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 交替三角步态参数。保留既有参数名，避免升级固件后丢失现场标定值。
const AP_Param::GroupInfo AP_HexRuped_Tripod::var_info[] = {
    // @Param: Hz
    // @DisplayName: Tripod gait update rate
    // @Description: Trajectory update rate. Zero freezes the gait phase while servo commands continue at 100 Hz
    // @Units: Hz
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("Hz", 1, AP_HexRuped_Tripod, gait_hz, SPEED_HZ_DEFAULT),

    // @Param: STEP
    // @DisplayName: Tripod gait steps
    // @Description: Number of discrete trajectory points in one complete tripod gait cycle
    // @Range: 8 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STEP", 2, AP_HexRuped_Tripod, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    // @Param: TRAJ_MODE
    // @DisplayName: Tripod trajectory type
    // @Description: Selects the swing-leg trajectory generator
    // @Values: 0:Cycloid,1:Cubic Bezier
    // @User: Standard
    AP_GROUPINFO("TRAJ_MODE", 3, AP_HexRuped_Tripod, trajectory_mode, 0),

    // @Param: BCTRL_H
    // @DisplayName: Tripod Bezier height control
    // @Description: Vertical Bezier control-point scale relative to the configured leg lift height
    // @Range: 0.1 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("BCTRL_H", 4, AP_HexRuped_Tripod, bezier_control_height, 0.3f),

    // @Param: BCTRL_F
    // @DisplayName: Tripod Bezier forward control
    // @Description: Horizontal Bezier control-point scale relative to commanded travel
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("BCTRL_F", 5, AP_HexRuped_Tripod, bezier_control_forward, 0.2f),

    AP_GROUPEND
};

// 构造函数
AP_HexRuped_Tripod::AP_HexRuped_Tripod(AP_HexRuped& frontend, AP_HexRuped::HexRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_HexRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// 步态初始化
void AP_HexRuped_Tripod::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Hexapod tripod gait init");

    // 六足交替三角步态：每组三条腿形成稳定支撑三角形。
    gait_step_leg_start[AP_HEXRUPED_LEG_RF] = 0;
    gait_step_leg_start[AP_HEXRUPED_LEG_RB] = 0;
    gait_step_leg_start[AP_HEXRUPED_LEG_LM] = 0;
    gait_step_leg_start[AP_HEXRUPED_LEG_LF] = gait_step_total / 2;
    gait_step_leg_start[AP_HEXRUPED_LEG_LB] = gait_step_total / 2;
    gait_step_leg_start[AP_HEXRUPED_LEG_RM] = gait_step_total / 2;

    // 设置步态参数
    gait_lift_divisor = 2; // 旋转最大值
}

// 轨迹生成统一接口函数
// 作为步态控制系统的核心调度器，根据用户参数选择合适的轨迹生成算法
// 这种设计模式实现了算法的可插拔性，便于后续扩展新的轨迹类型
void AP_HexRuped_Tripod::trajectory_generation(uint8_t leg_index)
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

// 摆线轨迹生成器（原有实现的重构版本）
// 使用经典的摆线（cycloid）曲线生成腿部轨迹，这是六足机器人领域最常用的轨迹算法
// 摆线轨迹的优势：数学特性优秀，保证起点和终点的速度、加速度连续，减少机械冲击
void AP_HexRuped_Tripod::generate_cycloid_trajectory(uint8_t leg_index)
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
    Vector2f leg_xy_target;       // XY平面的目标位置
    float    leg_z_target = 0.0f; // Z轴高度，默认为0（地面）

    // 步态相位分割：以0.5为界，前半段摆动相，后半段支撑相
    if (p < 0.5f) { // ==================== 摆动相（Swing Phase）====================
        // 摆动相时间映射：将[0,0.5]映射到[0,1]，专门用于空中轨迹计算
        const float phase = p * 2.0f; // 0..1

        // 角度参数：将时间相位转换为角度参数，用于三角函数计算
        // M_2PI * phase_slow 将[0,1]映射到[0,2π]，完成一个完整的摆线周期
        const float delta = M_2PI * phase;

        // ==================== 摆线轨迹数学原理 ====================
        // 摆线公式：S = (δ - sin(δ)) / 2π * 2，其中δ∈[0,2π]
        // 这个公式的物理意义：模拟滚轮边缘一点的运动轨迹
        // 特性：S(0)=0, S(π)=1, S(2π)=2，且起点和终点的速度、加速度都为0

        // 2D摆线轨迹计算：将1D摆线公式推广到2D平面
        // 对X和Y分量同时应用相同的标量变换，保证运动方向的协调性
        const float S = (delta - sinf(delta)) / M_2PI * 2.0f; // 0..2

        // XY平面轨迹：从起始位置到目标位置的摆线运动
        // 公式分解：throttle_travel * S - throttle_travel = throttle_travel * (S - 1)
        // 当S=0时，位置为-throttle_travel（起始位置）
        // 当S=2时，位置为+throttle_travel（目标位置）
        leg_xy_target = throttle_travel * S - throttle_travel; // 原公式在 X 上的 1D 推广到 2D

        // Z轴轨迹：垂直方向的摆线运动，实现抬腿和落地
        // 公式：-0.5 * leg_lift_height * (1 - cos(δ))
        // 特性：
        // - δ=0时，Z=0（地面起始）
        // - δ=π时，Z=-leg_lift_height（最高点）
        // - δ=2π时，Z=0（地面落地）
        // 负号表示向上为负方向（符合机器人坐标系）
        leg_z_target = -0.5f * leg_lift_height * (1.0f - cosf(delta));

    } else { // ==================== 支撑相（Stance Phase）====================
        // 支撑相时间映射：将[0.5,1]映射到[0,1]，用于地面接触期轨迹
        const float phase = (p - 0.5f) * 2.0f; // 0..1
        const float delta = M_2PI * phase;

        // 支撑相摆线计算：使用与摆动相对称的数学公式
        // 这种对称设计保证了步态的连续性和协调性
        const float S = (delta - sinf(delta)) / M_2PI * 2.0f;

        // 支撑相XY轨迹：从目标位置返回起始位置的摆线运动
        // 公式：-throttle_travel * S + throttle_travel = throttle_travel * (1 - S)
        // 当S=0时，位置为+throttle_travel（前支撑位置）
        // 当S=2时，位置为-throttle_travel（后支撑位置）
        // 这种设计实现了腿部推动机器人前进的效果
        leg_xy_target = -throttle_travel * S + throttle_travel; // 同样推广到 2D

        // 支撑相Z轴：强制为0，确保支撑相期间腿部始终与地面接触
        // 这是机器人稳定行走的必要条件，提供持续的地面反作用力
        leg_z_target = 0.0f;
    }

    // 结果输出：将计算得到的3D位置存储到全局数组中
    // 这个位置将被逆运动学解算使用，最终转换为各个关节的角度指令
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 更新腿部运动
void AP_HexRuped_Tripod::update_leg()
{
    // 更新步态计数器 - 改进为大步数时的连续增长
    gait_step_now++;

    // 使用大整数范围避免频繁循环，减少相位跳跃
    // 只有当步数超过很大值时才重置，避免边界问题
    if (gait_step_now >= 100000000) { // 使用int32_t接近上限的值
        gait_step_now = 0;
        // gait_step_total_cached = -1;
        refresh_steps();
    }
    refresh_steps();

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_HEXRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) {
            delta_step += gait_step_total;    // 处理循环计数
        }

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

// 主更新函数，按顺序执行控制流程
void AP_HexRuped_Tripod::update()
{
    // 先更新当前周期的足端轨迹，再逆解并输出，避免关节命令滞后一拍。
    calc_gait_sequence();
    main_inverse_kinematics();
    output_leg_angle();
}

// 三次贝塞尔曲线轨迹生成函数
// 使用四个控制点生成平滑的3D空间轨迹，提供比摆线曲线更灵活的轨迹控制
// 数学原理：B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃，其中t∈[0,1]
// 这种曲线保证C²连续性（位置、速度、加速度都连续），适合机器人运动控制
Vector3f AP_HexRuped_Tripod::cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3)
{
    // 参数t的范围约束：确保t在[0,1]区间内，防止数值计算错误
    // 这是贝塞尔曲线的数学定义域要求，超出范围会导致轨迹失真
    t = constrain_float(t, 0.0f, 1.0f);

    // 预计算伯恩斯坦基函数，提高计算效率
    // 通过预先计算重复使用的项，避免在循环中重复计算，显著提升性能
    const float mt     = 1.0f - t;       // (1-t) 项，用于后续基函数计算
    const float mt2    = mt * mt;        // (1-t)²，避免重复乘法运算
    const float mt3    = mt2 * mt;       // (1-t)³，P₀控制点的权重系数
    const float t2     = t * t;          // t²，用于高次项计算
    const float t3     = t2 * t;         // t³，P₃控制点的权重系数
    const float _3mt2t = 3.0f * mt2 * t; // 3(1-t)²t，P₁控制点的权重系数
    const float _3mtt2 = 3.0f * mt * t2; // 3(1-t)t²，P₂控制点的权重系数

    // 三次贝塞尔曲线公式的实现
    // 公式分解：B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
    // 其中：
    // - P₀: 起始点，权重从1递减到0，保证轨迹起点精确通过P₀
    // - P₁: 第一个控制点，控制轨迹起始方向，影响抬腿初期的运动特性
    // - P₂: 第二个控制点，控制轨迹结束方向，影响落地前的运动特性
    // - P₃: 终点，权重从0递增到1，保证轨迹终点精确通过P₃
    // 向量运算顺序优化：将标量系数放在前面，利用AP_Math库的优化
    return p0 * mt3 + p1 * _3mt2t + p2 * _3mtt2 + p3 * t3;
}

// 贝塞尔曲线轨迹生成器
// 为单条腿生成完整的步态轨迹，包含摆动相（空中）和支撑相（地面）
// 相比摆线轨迹，贝塞尔曲线提供更灵活的轨迹形状控制，特别适合复杂地形
void AP_HexRuped_Tripod::generate_bezier_trajectory(uint8_t leg_index)
{
    // 步态相位计算：计算当前腿相对于其起始步数的相位偏移
    // 这种设计允许每条腿有不同的起始时间，实现交替三角步态的相位控制
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

    // 步态相位判断：以0.5为界，前半段为摆动相，后半段为支撑相
    // 这种设计保证了交替三角步态的协调性：对角腿同时摆动，同时支撑
    if (p < 0.5f) {                   // 摆动相：腿部离地，在空中移动
        const float phase = p * 2.0f; // 将[0,0.5]映射到[0,1]，专门用于摆动相

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
        float ctrl_height  = bezier_control_height.get() * leg_lift_height;           // 抬腿高度：基于用户设定的抬腿高度进行比例调节
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

        // 轨迹生成：调用贝塞尔曲线函数计算实际位置
        // 使用原始相位值，不进行末端减速处理
        leg_target = cubic_bezier_trajectory(phase, p0, p1, p2, p3);

    } else {                                   // 支撑相：腿部着地，推动机器人前进
        const float phase = (p - 0.5f) * 2.0f; // 将[0.5,1]映射到[0,1]，专门用于支撑相

        // 支撑相采用线性轨迹设计
        // 线性轨迹的优势：计算简单，确保地面接触的稳定性
        // 从前支撑位置向后移动，实现推动机器人前进的效果

        // X方向：前向位移的线性衰减，从最大值递减到0
        // (1.0f - phase)确保从行程终点平滑返回到起点
        leg_target.x = throttle_travel.x * (1.0f - phase);

        // Y方向：横向位移的线性衰减，保持侧向运动的协调性
        leg_target.y = throttle_travel.y * (1.0f - phase);

        // Z方向：强制为0，确保支撑相期间腿部始终与地面接触
        // 这是机器人稳定行走的必要条件
        leg_target.z = 0.0f;
    }

    // 输出结果：将计算得到的腿部目标位置存储到全局数组中
    // 这个数组将被逆运动学解算使用，最终转换为关节角度
    gait_pos_xyz[leg_index] = leg_target;
}
