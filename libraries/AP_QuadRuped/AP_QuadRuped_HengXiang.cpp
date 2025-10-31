#include "AP_QuadRuped_HengXiang.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        AP_QUADRUPED_SPEED_HZ_DEFAULT   // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT AP_QUADRUPED_STEP_TOTAL_DEFAULT // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 参数表定义
const AP_Param::GroupInfo AP_QuadRuped_HengXiang::var_info[] = {
    // 步态参数 (1-10)
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_HengXiang, gait_hz, SPEED_HZ_DEFAULT), // 步态频率
    // 步长
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_HengXiang, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数
    // 轨迹模式
    AP_GROUPINFO("TRAJ", 3, AP_QuadRuped_HengXiang, trajectory_mode, 0), // 轨迹生成模式：0=摆线轨迹，1=贝塞尔曲线
    // 贝塞尔曲线参数
    AP_GROUPINFO("BEZ_H", 4, AP_QuadRuped_HengXiang, bezier_control_height, 1.2f),  // 贝塞尔曲线控制点高度系数
    AP_GROUPINFO("BEZ_F", 5, AP_QuadRuped_HengXiang, bezier_control_forward, 0.3f), // 贝塞尔曲线控制点前向偏移系数
    AP_GROUPINFO("COG_X", 7, AP_QuadRuped_HengXiang, centre_offset_ratio_x, 0.4f), // X方向重心偏移比例系数
    AP_GROUPINFO("COG_Y", 8, AP_QuadRuped_HengXiang, centre_offset_ratio_y, 0.1f), // Y方向重心偏移比例系数（较小值）

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_HengXiang::AP_QuadRuped_HengXiang(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

bool AP_QuadRuped_HengXiang::init()
{
    const AP_QuadRuped_SYS_Params& Sys_Param = _frontend.get_sys_params();

    // 初始化腿部起始位置 (Initialize leg starting positions)
    // 每条腿按90度间隔分布 (Each leg is spaced 90 degrees apart)
    // 计算腿部末端执行器在机体坐标系中的位置 (Calculate end effector position in body frame)
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        const float hy = (leg_index == AP_QUADRUPED_LEG_RF || leg_index == AP_QUADRUPED_LEG_RB)
            ? (Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN)
            : -(Sys_Param.FEMUR_LEN + Sys_Param.COXA_LEN);

        endpoint_leg_pos[leg_index] = Vector3f(0.0f, hy, Sys_Param.TIBIA_LEN);
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

// 步态初始化
void AP_QuadRuped_HengXiang::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_HengXiang init");

    const int16_t step_total = gait_step_total.get();

    // 维持12.5%摆动相
    gait_lift_divisor = 8;

    // 设置每条腿的起始步数 - 波浪步态：90度相位差，确保单腿摆动
    gait_step_cog_start[AP_QUADRUPED_LEG_RF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 0 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (0 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_RB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 1 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (1 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_LB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 2 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (2 * 2 + 1), 0, 255));

    gait_step_cog_start[AP_QUADRUPED_LEG_LF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * 3 * 2, 0, 255));
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = static_cast<uint8_t>(constrain_int16(step_total / gait_lift_divisor * (3 * 2 + 1), 0, 255));
}

void AP_QuadRuped_HengXiang::refresh_steps()
{
    const int16_t step_total = gait_step_total.get();
    if (step_total < 0) {
        return;
    }

    if (step_total == gait_step_total_cached) {
        return;
    }

    gait_step_total_cached = step_total;

    gait_init();
}

// 轨迹生成
void AP_QuadRuped_HengXiang::trajectory_generation(uint8_t leg_index)
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
// 使用经典的摆线（cycloid）曲线生成腿部轨迹，这是四足机器人领域最常用的轨迹算法
// 摆线轨迹的优势：数学特性优秀，保证起点和终点的速度、加速度连续，减少机械冲击
void AP_QuadRuped_HengXiang::generate_cycloid_trajectory(uint8_t leg_index)
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

    // 用户指令解析：使用通用接口获取期望的2D位移向量
    // 这个向量决定了机器人在平面上的运动意图
    const Vector2f throttle_travel = get_throttle_travel();

    // 临时变量：存储计算得到的腿部目标位置（XY平面和Z轴高度）
    Vector2f leg_xy_target;       // XY平面的目标位置
    float    leg_z_target = 0.0f; // Z轴高度，默认为0（地面）

    const float swing_ratio = 1.0f / (float)gait_lift_divisor; // 单腿摆动占 1/8 周期

    if (p < swing_ratio) {
        const float phase = constrain_float(p / swing_ratio, 0.0f, 1.0f);
        const float delta = M_2PI * phase;
        const float S     = (delta - sinf(delta)) / M_2PI * 2.0f;

        leg_xy_target = throttle_travel * S - throttle_travel;
        leg_z_target  = -leg_lift_height * (1.0f - cosf(delta));

    } else {
        const float phase = constrain_float((p - swing_ratio) / (1.0f - swing_ratio), 0.0f, 1.0f);
        const float delta = M_2PI * phase;
        const float S     = (delta - sinf(delta)) / M_2PI * 2.0f;

        leg_xy_target = -throttle_travel * S + throttle_travel;
        leg_z_target  = 0.0f;
    }

    // 结果输出：将计算得到的3D位置存储到全局数组中
    // 这个位置将被逆运动学解算使用，最终转换为各个关节的角度指令
    gait_pos_xyz[leg_index] = Vector3f(leg_xy_target, leg_z_target);
}

// 三次贝塞尔曲线轨迹生成函数
// 使用四个控制点生成平滑的3D空间轨迹，提供比摆线曲线更灵活的轨迹控制
// 数学原理：B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃，其中t∈[0,1]
// 这种曲线保证C²连续性（位置、速度、加速度都连续），适合机器人运动控制
Vector3f AP_QuadRuped_HengXiang::cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3)
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

// 通用的位移向量获取方法 - 横向步态实现
Vector2f AP_QuadRuped_HengXiang::get_throttle_travel() const
{
    return Vector2f(0.0f, throttle_y_travel);
}

// 贝塞尔曲线轨迹生成器
// 为单条腿生成完整的步态轨迹，包含摆动相（空中）和支撑相（地面）
// 相比摆线轨迹，贝塞尔曲线提供更灵活的轨迹形状控制，特别适合复杂地形
void AP_QuadRuped_HengXiang::generate_bezier_trajectory(uint8_t leg_index)
{
    // 步态相位计算：计算当前腿相对于其起始步数的相位偏移
    // 这种设计允许每条腿有不同的起始时间，实现对角步态的相位控制
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
    // 用户指令解析：使用通用接口获取期望的平面位移向量
    const Vector2f throttle_travel = get_throttle_travel();
    Vector3f       leg_target; // 存储计算得到的腿部目标位置
    // 步态相位判断：以0.5为界，前半段为摆动相，后半段为支撑相
    // 这种设计保证了对角步态的协调性：对角腿同时摆动，同时支撑
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
        // phase_smooth是经过时间缩放的参数，确保运动学特性符合要求
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

// 生成偏航（旋转）轨迹
void AP_QuadRuped_HengXiang::yaw_trajectory_generation(uint8_t leg_index)
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

Vector3f AP_QuadRuped_HengXiang::leg_inverse_kinematics(Vector3f posxyz)
{
    return AP_QuadRuped_Backend::leg_inverse_kinematics(posxyz);
}

// 主逆运动学计算 - 计算所有腿的关节角度
void AP_QuadRuped_HengXiang::main_inverse_kinematics(void)
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

// 更新腿部运动
void AP_QuadRuped_HengXiang::update_leg()
{
    // 更新步态计数器 - 改进为大步数时的连续增长
    gait_step_now++;
    // 使用大整数范围避免频繁循环，减少相位跳跃
    // 只有当步数超过很大值时才重置，避免边界问题
    if (gait_step_now >= 100000000) { // 使用int32_t接近上限的值
        gait_step_now          = 0;
        gait_step_total_cached = -1;
        refresh_steps();
    }

    refresh_steps();

    // 遍历所有腿，生成轨迹
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
        cog_generation(leg_index);
    }
}

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_HengXiang::update()
{
    // 执行主控制器
    // main_radio_controller();

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
void AP_QuadRuped_HengXiang::balance_controller()
{
    // 简单的平衡控制实现
    // 这里可以根据IMU数据调整重心偏移以保持平衡

    // 获取当前姿态数据
    const Vector3f& gyro  = _ahrs.get_gyro();
    const Vector3f& accel = _ahrs.get_accel_ef();

    // 计算需要的重心补偿（示例实现）
    // 这里可以根据实际的平衡控制算法进行调整
    center_offset.x = constrain_float(gyro.y * 0.1f, -10.0f, 10.0f); // 基于横滚角速度补偿
    center_offset.y = constrain_float(gyro.x * 0.1f, -10.0f, 10.0f); // 基于俯仰角速度补偿

    // 重心高度补偿（基于Z轴加速度）
    center_offset.z = constrain_float(accel.z * 0.05f, -5.0f, 5.0f);
}

void AP_QuadRuped_HengXiang::cog_generation(uint8_t leg_index)
{
    // 步态相位计算：与贝塞尔曲线轨迹相同的相位处理逻辑
    // 这种统一设计保证了不同轨迹算法之间的相位一致性
    int32_t delta_step = gait_step_now - gait_step_cog_start[leg_index];

    // 相位循环处理：确保步数在有效范围内，避免整数溢出和相位跳跃
    // 先处理负数再取模，保证数学上的正确性和连续性
    while (delta_step < 0) {
        delta_step += gait_step_total;
    }
    delta_step = delta_step % gait_step_total.get();

    // 归一化相位转换：将离散步数映射到连续的[0,1]区间
    // 这个p值是整个轨迹生成的时间基准，驱动后续所有的数学计算
    const float p = (float)delta_step / (float)gait_step_total; // 0..1

    // 单腿重心偏移占 1/8 周期
    const float swing_ratio = 1.0f / (float)gait_lift_divisor;

    const float cog_length_x = centre_offset_ratio_x.get();  // X方向重心偏移幅度
    const float cog_length_y = centre_offset_ratio_y.get();  // Y方向重心偏移幅度

    Vector2f cog_xy_target;
    float    phase;

    if (p < swing_ratio) {
        phase = constrain_float(p / swing_ratio, 0.0f, 1.0f);

        if (leg_index == AP_QUADRUPED_LEG_RF) {
            // 椭圆形轨迹：X方向大范围偏移，Y方向小范围偏移
            cog_xy_target.x = -cog_length_x * cosf(M_PI / 2 * (phase - 0.5)); 
            cog_xy_target.y = -cog_length_y * sinf(M_PI / 2 * (phase - 0.5)); 

        } else if (leg_index == AP_QUADRUPED_LEG_RB) {
            cog_xy_target.x = cog_length_x * sinf(M_PI / 2 * (phase - 0.5));  
            cog_xy_target.y = -cog_length_y * cosf(M_PI / 2 * (phase - 0.5)); 

        } else if (leg_index == AP_QUADRUPED_LEG_LB) {
            cog_xy_target.x = cog_length_x * cosf(M_PI / 2 * (phase - 0.5));  
            cog_xy_target.y = cog_length_y * sinf(M_PI / 2 * (phase - 0.5));  

        } else if (leg_index == AP_QUADRUPED_LEG_LF) {
            cog_xy_target.x = -cog_length_x * sinf(M_PI / 2 * (phase - 0.5));
            cog_xy_target.y = cog_length_y * cosf(M_PI / 2 * (phase - 0.5));  
        }
        set_center_offset(cog_xy_target);
    }
}
