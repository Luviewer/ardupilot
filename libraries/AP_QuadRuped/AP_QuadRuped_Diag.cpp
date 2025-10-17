#include "AP_QuadRuped_Diag.h"
#include "AP_QuadRuped.h"
#include <AP_HAL/AP_HAL.h>

#define SPEED_HZ_DEFAULT        25.0f // 默认步态频率（Hz）
#define GAIT_STEP_TOTAL_DEFAULT 24    // 默认步态总步数

// 外部HAL实例
extern const AP_HAL::HAL& hal;

// 对角步态参数表定义
// 使用AP_Param框架实现参数的持久化存储和运行时调整
// 参数前缀QUAD_DIAG_由ArduPilot参数系统自动添加
const AP_Param::GroupInfo AP_QuadRuped_Diag::var_info[] = {
    // ==================== 基础步态参数 ====================

    // 步态频率控制：决定机器人运动的时间分辨率
    // 参数名：QUAD_DIAG_Hz，范围：1-100Hz，默认：25Hz
    // 设计原理：频率越高，运动越平滑，但计算负载越大
    // 25Hz是经验值，在平滑性和计算效率之间取得平衡
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Diag, gait_hz, SPEED_HZ_DEFAULT),

    // 步态总步数：决定一个完整步态周期的离散化精度
    // 参数名：QUAD_DIAG_STEP，范围：8-100步，默认：24步
    // 设计原理：步数越多，轨迹越精确，但内存占用越大
    // 24步对应0.5秒的步态周期（25Hz时），适合四足机器人运动
    AP_GROUPINFO("STEP", 2, AP_QuadRuped_Diag, gait_step_total, GAIT_STEP_TOTAL_DEFAULT),

    // ==================== 轨迹生成模式选择 ====================

    // 轨迹生成算法选择：提供多种轨迹生成方式以适应不同场景
    // 参数名：QUAD_DIAG_TRAJ_MODE，选项：0=摆线轨迹，1=贝塞尔曲线轨迹，默认：0
    // 设计考量：
    // - 摆线轨迹：计算简单，运动平稳，适合平坦地面
    // - 贝塞尔曲线：形状灵活，可调性强，适合复杂地形
    // 默认选择摆线轨迹确保系统稳定性和向后兼容性
    AP_GROUPINFO("TRAJ_MODE", 3, AP_QuadRuped_Diag, trajectory_mode, 0),

    // ==================== 贝塞尔曲线控制参数 ====================
    // 以下参数仅在TRAJ_MODE=1时生效，用于精细调节贝塞尔曲线轨迹形状

    // 贝塞尔曲线控制点高度系数：调节抬腿高度和轨迹弧度
    // 参数名：QUAD_DIAG_BCTRL_H，范围：0.1-2.0，默认：0.3
    // 物理意义：实际抬腿高度 = leg_lift_height * BCTRL_H
    // 设计原理：
    // - 0.3的默认值产生适中的抬腿高度，兼顾越障能力和稳定性
    // - 增大值会提高抬腿高度，增强越障能力但降低稳定性
    // - 减小值会降低抬腿高度，提高稳定性但限制越障能力
    AP_GROUPINFO("BCTRL_H", 4, AP_QuadRuped_Diag, bezier_control_height, 0.3f),

    // 贝塞尔曲线控制点前向偏移系数：调节轨迹的前后延伸程度
    // 参数名：QUAD_DIAG_BCTRL_F，范围：0.0-1.0，默认：0.2
    // 物理意义：前向偏移距离 = throttle_travel.length() * BCTRL_F
    // 设计原理：
    // - 0.2的默认值产生轻微的前向延伸，创造自然的抬腿轨迹
    // - 增大值会延长抬腿距离，适合高速运动但增加冲击风险
    // - 减小值会缩短抬腿距离，适合低速运动但可能限制步长
    // - 0.0产生垂直抬腿，1.0产生最大延伸的轨迹
    AP_GROUPINFO("BCTRL_F", 5, AP_QuadRuped_Diag, bezier_control_forward, 0.2f),

    // 参数表结束标记，AP_Param框架要求的固定格式
    AP_GROUPEND
};

// 构造函数
AP_QuadRuped_Diag::AP_QuadRuped_Diag(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors)
    : AP_QuadRuped_Backend(frontend, state, ahrs, motors)
{
    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// 步态初始化
void AP_QuadRuped_Diag::gait_init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_QuadRuped_Diag step init");

    // 设置每条腿的起始步数
    // 对角步态：左前右后同时抬起，右前左后同时抬起
    gait_step_leg_start[AP_QUADRUPED_LEG_RF] = 0;                   // 右前腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LB] = 0;                   // 左后腿从第0步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_LF] = gait_step_total / 2; // 左前腿从中间步开始
    gait_step_leg_start[AP_QUADRUPED_LEG_RB] = gait_step_total / 2; // 右后腿从中间步开始

    // 设置步态参数
    gait_lift_divisor = 2; // 旋转最大值
}

// 轨迹生成统一接口函数
// 作为步态控制系统的核心调度器，根据用户参数选择合适的轨迹生成算法
// 这种设计模式实现了算法的可插拔性，便于后续扩展新的轨迹类型
void AP_QuadRuped_Diag::trajectory_generation(uint8_t leg_index)
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
void AP_QuadRuped_Diag::generate_cycloid_trajectory(uint8_t leg_index)
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
        // 公式：-leg_lift_height * (1 - cos(δ))
        // 特性：
        // - δ=0时，Z=0（地面起始）
        // - δ=π时，Z=-2*leg_lift_height（最高点）
        // - δ=2π时，Z=0（地面落地）
        // 负号表示向上为负方向（符合机器人坐标系）
        leg_z_target = -leg_lift_height * (1.0f - cosf(delta));

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
void AP_QuadRuped_Diag::update_leg()
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
    for (uint8_t leg_index = 0; leg_index < AP_QUADRUPED_LEG_ALL; leg_index++) {
        int16_t delta_step = gait_step_now - gait_step_leg_start[leg_index];

        if (delta_step < 0) delta_step += gait_step_total; // 处理循环计数

        // 为每条腿生成位置轨迹和旋转轨迹
        trajectory_generation(leg_index);
        yaw_trajectory_generation(leg_index);
    }
}

// 主更新函数，按顺序执行控制流程
void AP_QuadRuped_Diag::update()
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
    //     hengxiang_claw_leg();
    // }

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
void AP_QuadRuped_Diag::balance_controller()
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



// 三次贝塞尔曲线轨迹生成函数
// 使用四个控制点生成平滑的3D空间轨迹，提供比摆线曲线更灵活的轨迹控制
// 数学原理：B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃，其中t∈[0,1]
// 这种曲线保证C²连续性（位置、速度、加速度都连续），适合机器人运动控制
Vector3f AP_QuadRuped_Diag::cubic_bezier_trajectory(float t, const Vector3f& p0, const Vector3f& p1, const Vector3f& p2, const Vector3f& p3)
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
void AP_QuadRuped_Diag::generate_bezier_trajectory(uint8_t leg_index)
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

    // 用户指令解析：将遥控器输入转换为期望的平面位移向量
    // throttle_x_travel: 前后方向位移（正值向前，负值向后）
    // throttle_y_travel: 左右方向位移（正值向右，负值向左）
    const Vector2f throttle_travel(throttle_x_travel, throttle_y_travel);

    Vector3f leg_target; // 存储计算得到的腿部目标位置

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