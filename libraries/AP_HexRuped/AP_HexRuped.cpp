// 六足机器人主控制器实现文件
// 负责管理六足机器人的整体控制逻辑，包括步态切换、遥控器输入处理、硬件接口管理等

#include "AP_HexRuped.h"
#include "AP_HexRuped_Tripod.h"                     // 交替三角步态后端
#include <AP_RCMapper/AP_RCMapper.h>               // 遥控器映射
#include <AP_RangeFinder/AP_RangeFinder_Backend.h> // 测距传感器
#include <RC_Channel/RC_Channel.h>                 // 遥控通道

// 条件编译包含各种步态后端
#if AP_HEXRUPED_WAVE_ENABLE
# include "AP_HexRuped_Wave.h" // 波浪步态后端
#endif

// 统一通过RC_Channel读取输入，使物理接收机和MAVLink RC override遵循同一条控制链路。
uint16_t AP_HexRuped::read_rc_channel_pwm(int8_t channel, uint16_t default_pwm)
{
    if (channel <= 0) {
        return default_pwm;
    }

    const uint16_t pwm = RC_Channels::get_radio_in(uint8_t(channel - 1));
    return pwm == 0 ? default_pwm : pwm;
}

/**
 * @brief 六足机器人参数表定义
 *
 * 这个参数表定义了所有可配置的系统参数，包括：
 * - 基础功能开关和类型设置
 * - 系统级参数
 * - 遥控通道映射参数
 * - 各条腿的独立参数
 * - 各种步态的特定参数
 *
 * 参数通过AP_PARAM系统自动保存到EEPROM中
 */
const AP_Param::GroupInfo AP_HexRuped::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: 六足机器人功能使能
    // @Description: 启用或禁用六足机器人功能
    // @Values: 0:禁用, 1:启用
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_HexRuped, _enabled, 1),

    // @Param: CLASS
    // @DisplayName: 六足机器人类型
    // @Description: 选择六足机器人的硬件类型
    // @Values: 0:普通型, 1:USL_BV2型
    // @User: Standard
    AP_GROUPINFO("CLASS", 3, AP_HexRuped, _hexruped_class, (int8_t)AP_HEXRUPED_NORMAL),

    // @Param: RPTD
    // @DisplayName: 横滚俯仰滤波时间常数
    // @Description: 横滚和俯仰控制的微分项时间常数，用于滤波处理
    // @Units: ms
    // @Range: 100 5000
    // @User: Advanced
    AP_GROUPINFO("RPTD", 4, AP_HexRuped, _roll_pitch_td_r, (float)1000),

    // 系统参数组 (11-20)
    // @Group: SYS_
    AP_SUBGROUPINFO(_sys_params, "SYS_", 11, AP_HexRuped, AP_HexRuped_SYS_Params),

    AP_SUBGROUPINFO(_ctrl_params, "CTL_", 12, AP_HexRuped, AP_HexRuped_CTRL_Params),

    // 通道参数组 (21-30)
    // @Group: CH_
    // @Description: 遥控器通道映射参数，定义各个功能对应的遥控通道
    AP_SUBGROUPINFO(_channel_params, "CH_", 21, AP_HexRuped, AP_HexRuped_CHANNEL_Params),

    // 腿部参数组 (31-40)
    // @Group: RF_
    // @Description: 右前腿(Right Front)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_RF], "RF_", 31, AP_HexRuped, AP_HexRuped_Params),

    // @Group: RB_
    // @Description: 右后腿(Right Back)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_RB], "RB_", 32, AP_HexRuped, AP_HexRuped_Params),

    // @Group: LB_
    // @Description: 左后腿(Left Back)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_LB], "LB_", 33, AP_HexRuped, AP_HexRuped_Params),

    // @Group: LF_
    // @Description: 左前腿(Left Front)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_LF], "LF_", 34, AP_HexRuped, AP_HexRuped_Params),

    // @Group: RM_
    // @Description: 右中腿(Right Middle)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_RM], "RM_", 35, AP_HexRuped, AP_HexRuped_Params),

    // @Group: LM_
    // @Description: 左中腿(Left Middle)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_HEXRUPED_LEG_LM], "LM_", 36, AP_HexRuped, AP_HexRuped_Params),

    // 步态后端参数组 (41-50)
    // @Group: TRI_
    // @Description: 交替三角步态(Tripod)的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_HEXRUPED_GAIT_TRIPOD], "TRI_", 41, AP_HexRuped, backend_var_info[AP_HEXRUPED_GAIT_TRIPOD]),

#if AP_HEXRUPED_WAVE_ENABLE
    // @Group: WAVE_
    // @Description: 波浪步态(Wave/Crawl)的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_HEXRUPED_GAIT_WAVE], "WAVE_", 42, AP_HexRuped, backend_var_info[AP_HEXRUPED_GAIT_WAVE]),
#endif
    AP_GROUPEND
};

// 静态成员变量定义 - 存储各种步态后端的参数表指针
const struct AP_Param::GroupInfo* AP_HexRuped::backend_var_info[AP_HEXRUPED_GAIT_COUNT];

/**
 * @brief 默认构造函数
 *
 * 初始化六足机器人控制器的所有成员变量：
 * - 将硬件接口指针设为nullptr，等待init()函数设置
 * - 初始化所有步态后端的状态数组
 * - 设置初始步态类型为无效值
 * - 加载参数默认值
 */
AP_HexRuped::AP_HexRuped()
    : _ahrs(nullptr)        // 姿态航向参考系统指针初始化
    , _motors(nullptr)      // 电机控制接口指针初始化
    , _rangefinder(nullptr) // 测距传感器接口指针初始化
    , _backend(nullptr)     // 当前活跃步态后端指针初始化
{
    // 初始化所有步态后端的状态结构体
    for (uint8_t i = 0; i < AP_HEXRUPED_GAIT_COUNT; i++) {
        _gait_backends[i]      = nullptr; // 后端实例指针置空
        _state[i].last_time_ms = 0;       // 上次更新时间清零
        _state[i].instance     = i;       // 设置步态实例编号
        _state[i].var_info     = nullptr; // 参数表指针置空
    }

    _gait_last_type = -1; // 设置上次步态类型为无效值，确保首次切换成功

    // 从参数表中加载默认值到成员变量
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief 销毁所有步态后端实例
 *
 * 释放所有已创建的步态后端对象内存，防止内存泄漏。
 * 通常在析构函数或重新初始化时调用。
 */
void AP_HexRuped::destroy_backends()
{
    // 遍历所有步态类型，释放对应的后端实例内存
    for (uint8_t i = 0; i < AP_HEXRUPED_GAIT_COUNT; i++) {
        if (_gait_backends[i] != nullptr) {
            delete _gait_backends[i];    // 释放后端对象内存
            _gait_backends[i] = nullptr; // 指针置空，防止悬空指针
        }
    }
}

/**
 * @brief 初始化六足机器人控制器
 *
 * 设置硬件接口引用，创建所有步态后端实例，初始化滤波器，并设置默认步态。
 * 这是系统启动后必须调用的关键函数。
 *
 * @param ahrs 姿态航向参考系统引用，用于获取机体姿态
 * @param motors 电机控制接口引用，用于控制伺服电机
 * @param rangefinder 测距传感器引用，用于高度检测
 * @return bool 初始化是否成功，true表示至少有一个步态后端创建成功
 */
bool AP_HexRuped::init(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder)
{
    // 保存硬件接口引用
    _ahrs        = &ahrs;        // 姿态传感器接口
    _motors      = &motors;      // 电机控制接口
    _rangefinder = &rangefinder; // 测距传感器接口

    // 创建所有可用的步态后端实例
    create_backends();

    // 初始化横滚和俯仰的微分滤波器
    // 参数：初始值=0, 时间常数=配置值, 初始微分值=0
    _roll_pitch_td[0].init(0.001, _roll_pitch_td_r, 0); // 横滚通道滤波器
    _roll_pitch_td[1].init(0.001, _roll_pitch_td_r, 0); // 俯仰通道滤波器

    // 设置默认步态为交替三角步态（最稳定和常用）
    set_gait_type(AP_HEXRUPED_GAIT_TRIPOD);

    // 检查是否成功创建了至少一个步态后端
    return _backend != nullptr;
}

/**
 * @brief 创建所有可用的步态后端实例
 *
 * 根据编译时的功能开关，创建相应的步态后端对象。
 * 每个后端负责实现特定的步态算法和运动控制逻辑。
 *
 * 创建过程包括：
 * 1. 使用NEW_NOTHROW分配后端对象内存
 * 2. 保存后端的参数表指针
 * 3. 设置步态实例编号
 * 4. 从EEPROM加载该步态的配置参数
 */
void AP_HexRuped::create_backends()
{
    // 创建交替三角步态后端 - Trot步态，最稳定高效
    _gait_backends[AP_HEXRUPED_GAIT_TRIPOD]   = NEW_NOTHROW AP_HexRuped_Tripod(*this, _state[AP_HEXRUPED_GAIT_TRIPOD], *_ahrs, *_motors);
    backend_var_info[AP_HEXRUPED_GAIT_TRIPOD] = _state[AP_HEXRUPED_GAIT_TRIPOD].var_info;
    _state[AP_HEXRUPED_GAIT_TRIPOD].instance  = AP_HEXRUPED_GAIT_TRIPOD;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_HEXRUPED_GAIT_TRIPOD], backend_var_info[AP_HEXRUPED_GAIT_TRIPOD]);

    // 条件编译创建其他步态后端
#if AP_HEXRUPED_WAVE_ENABLE
    // 创建波浪步态后端 - Crawl步态，最稳定但速度慢
    _gait_backends[AP_HEXRUPED_GAIT_WAVE]   = NEW_NOTHROW AP_HexRuped_Wave(*this, _state[AP_HEXRUPED_GAIT_WAVE], *_ahrs, *_motors);
    backend_var_info[AP_HEXRUPED_GAIT_WAVE] = _state[AP_HEXRUPED_GAIT_WAVE].var_info;
    _state[AP_HEXRUPED_GAIT_WAVE].instance  = AP_HEXRUPED_GAIT_WAVE;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_HEXRUPED_GAIT_WAVE], backend_var_info[AP_HEXRUPED_GAIT_WAVE]);
#endif

}

/**
 * @brief 六足机器人主更新循环
 *
 * 这是系统的核心更新函数，以固定频率调用，负责：
 * 1. 检查系统使能状态
 * 2. 按步态频率控制轨迹/逆运动学更新周期
 * 3. 读取和处理遥控器输入
 * 4. 根据当前模式执行相应动作
 * 5. 每次100Hz调度都发送一次最新舵机目标
 *
 * 步态更新频率由当前后端参数决定；DroneCAN发送固定跟随100Hz调度。
 */
void AP_HexRuped::update()
{
    // 检查六足机器人功能是否已启用
    if (!_enabled) {
        return;
    }

    // 检查当前是否有活跃的步态后端
    if (_backend == nullptr) {
        return;
    }

    // 发送自定义MAVLink数据（状态信息）
    send_custom_mavlink_data();

    // 读取并处理遥控器输入数据
    read_radio_input();

    // 保持主更新频率的控制
    _backend->main_radio_controller();

    // 步态计算和舵机命令发送解耦：步态只在参数指定的周期内推进，
    // 但最新目标会在函数末尾由100Hz调度固定重发。
    static constexpr uint16_t scheduler_frequency_hz = 100;
    const uint16_t gait_frequency_hz = _backend->get_gait_frequency_hz();
    _gait_phase_accumulator += gait_frequency_hz;
    const bool gait_update_due = _gait_phase_accumulator >= scheduler_frequency_hz;

    if (gait_update_due) {
        _gait_phase_accumulator -= scheduler_frequency_hz;

        uint16_t down_cm = 0;
        const bool have_down_rf = get_downward_distance_cm(down_cm);
        const uint16_t claw_h_cm = constrain_int16(_sys_params.CLAW_H, 5, 200);

        switch (fly_walk_mode.master_mode) {
        default:
        case Walking_Mode: // 行走模式
            set_gait_type((HexRupedGaitType)fly_walk_mode.walk_mode);
            _backend->update();
            break;

        case Flying_Mode: {
            // 高度门只判断一次: <=收爪高度(默认40cm)或无测距,一律默认爪展开,忽略横爪和合爪通道。
            // 只有测距有效且高于收爪高度,才允许默认爪/横爪,以及 HEX_CH_CLAW 开合。
            const bool allow_claw_air = have_down_rf && (down_cm > claw_h_cm);
            if (!allow_claw_air) {
                _backend->x_sleep_leg();
                break;
            }

            float femur_deg = -75.0f;
            float tibia_deg = 60.0f;
            if (_channel_params.claw_channel != -1) {
                get_claw_joint_angles(femur_deg, tibia_deg);
            }

            if (fly_walk_mode.fly_mode == Fly_Mode_Heng_Claw) {
                _backend->hengxiang_claw_joints(femur_deg, tibia_deg);
            } else {
                _backend->default_claw_joints(femur_deg, tibia_deg);
            }
            break;
        }
        }
    }

    // userhook_FastLoop由Copter调度器以100Hz调用。无论本周期是否推进步态，
    // 都发送一次缓存目标，保证步长/步态频率变化不会改变舵机命令发送频率。
    _backend->send_servo_cmd();
}

bool AP_HexRuped::get_downward_distance_cm(uint16_t &cm) const
{
    if (_rangefinder == nullptr) {
        return false;
    }

    // RNGFNDx_ORIENT = Pitch270 为朝下
    const AP_RangeFinder_Backend *sensor = _rangefinder->find_instance(ROTATION_PITCH_270);
    if (sensor == nullptr || !sensor->has_data()) {
        return false;
    }

    cm = sensor->distance_cm();
    return true;
}

// HEX_CH_CLAW: 1500=默认收起(股-75°/胫60°), 2000=合爪(股+40°/胫0°), 中间线性; <=1500 保持收起
void AP_HexRuped::get_claw_joint_angles(float &femur_deg, float &tibia_deg) const
{
    const float t = constrain_float(_claw_angle / 90.0f, 0.0f, 1.0f);
    femur_deg = -75.0f + (40.0f - (-75.0f)) * t;
    tibia_deg = 60.0f + (0.0f - 60.0f) * t;
}

/**
 * @brief 设置当前步态类型
 *
 * 切换六足机器人的步态模式。步态切换会：
 * 1. 验证步态类型有效性
 * 2. 检查是否需要切换（避免重复切换）
 * 3. 切换到新的步态后端
 * 4. 初始化新步态
 *
 * @param type 要切换到的步态类型
 */
void AP_HexRuped::set_gait_type(HexRupedGaitType type)
{
    // 验证步态类型是否在有效范围内
    if (type >= AP_HEXRUPED_GAIT_COUNT) {
        return;
    }

    // 如果已经是当前步态，则无需切换
    if (_gait_last_type == type) {
        return;
    }

    // 检查目标步态后端是否存在且已初始化
    if (_gait_backends[type]) {
        // 切换到新的步态后端
        _backend = _gait_backends[type];
        // 初始化新步态（重置步态状态参数）
        _backend->init();

        // 更新当前步态类型记录
        _gait_last_type = type;
    }
}

/**
 * @brief 获取指定腿部的参数
 *
 * 返回指定腿部的运动学和限制参数，用于步态计算。
 * 如果腿部索引无效，则默认返回右前腿的参数。
 *
 * @param leg_index 腿部索引（RF, RB, LB, LF, RM, LM）
 * @return const AP_HexRuped_Params& 腿部参数的常量引用
 */
const AP_HexRuped_Params& AP_HexRuped::get_leg_params(uint8_t leg_index) const
{
    // 验证腿部索引是否在有效范围内
    if (leg_index < AP_HEXRUPED_LEG_ALL) {
        return _leg_params[leg_index];
    }
    // 索引无效时，默认返回右前腿的参数
    return _leg_params[AP_HEXRUPED_LEG_RF];
}

/**
 * @brief 读取和处理遥控器输入
 *
 * 这是六足机器人遥控输入处理的核心函数，每周期调用一次，负责：
 * 1. 故障保护检测和安全模式切换
 * 2. 运动控制输入处理（X、Y、Z轴）
 * 3. 姿态平衡控制输入处理（横滚、俯仰）
 * 4. 主工作模式切换（行走/飞行模式）
 * 5. 步态模式选择（Tripod、Wave）
 * 6. 飞行子模式选择（飞行、爪子形态）
 * 7. 爪子角度控制
 *
 * PWM信号规范：
 * - 1000μs: 最小值（完全反向）
 * - 1500μs: 中位值（零输入/停止）
 * - 2000μs: 最大值（完全正向）
 * - 1450-1550μs: 死区范围，视为1500μs处理
 *
 * @note 故障保护状态下会立即停止所有运动并切换到安全的交替三角步态
 */
void AP_HexRuped::read_radio_input()
{
    ////////////////////////////////////////////////////////////////////////////////////
    // 第一部分：遥控器故障保护检测
    ////////////////////////////////////////////////////////////////////////////////////
    // 检查遥控器是否处于故障保护状态（信号丢失或超出范围）
    if (rc().in_rc_failsafe()) {
        // 故障保护响应：立即清零所有运动指令，确保机器人安全停止
        _throttle_xyz        = Vector3f(0, 0, 0); // 清零前进、横向、偏航指令
        _throttle_roll_pitch = Vector2f(0, 0);    // 清零横滚、俯仰平衡指令

        // 切换到最安全的默认模式组合
        set_master_mode(Walking_Mode);             // 设为行走模式（最基础的控制模式）
        set_walk_mode(AP_HEXRUPED_GAIT_TRIPOD); // 设为交替三角步态（最稳定，支撑面最大）
        return;                                    // 直接返回，不处理后续输入
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第二部分：XYZ运动轴输入处理（前进、横向、偏航）
    ////////////////////////////////////////////////////////////////////////////////////
    // 参数使用从1开始的通道号，辅助函数负责转换为RC_Channels的从0开始索引。
    Vector3ui throttle_chan = {
        read_rc_channel_pwm(_channel_params.throttle_x_channel, 1500), // X轴：前进(+)/后退(-)
        read_rc_channel_pwm(_channel_params.throttle_y_channel, 1500), // Y轴：左移(+)/右移(-)
        read_rc_channel_pwm(_channel_params.yaw_channel, 1500)         // Z轴：逆时针(+)/顺时针(-)旋转
    };

    // 死区处理：将接近中位的PWM值（1450-1550μs）强制设为中位值
    // 目的：消除摇杆机械回中误差和电子噪声引起的微小抖动
    for (uint8_t i = 0; i < 3; i++) {
        if (throttle_chan[i] > 1450 && throttle_chan[i] < 1550) {
            throttle_chan[i] = 1500;
        }
    }

    // 将PWM值转换为归一化的控制量（范围：-1.0到+1.0）
    // 转换公式：(当前PWM - 中位PWM) / PWM半行程
    // 示例：2000μs -> (2000-1500)/500 = +1.0, 1000μs -> (1000-1500)/500 = -1.0
    for (uint8_t i = 0; i < 3; i++) {
        _throttle_xyz[i] = ((float)(throttle_chan[i]) - 1500.0f) / 500.0f;
        // 安全限制：确保数值在有效范围内，防止计算错误或异常值
        _throttle_xyz[i] = constrain_float(_throttle_xyz[i], -1.0f, 1.0f);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第三部分：姿态平衡控制输入处理（横滚、俯仰）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取姿态控制通道的PWM值，用于机体平衡调整
    Vector2ui rollpitch_chan = {
        read_rc_channel_pwm(_channel_params.roll_channel, 1500), // 横滚：左倾(+)/右倾(-)
        read_rc_channel_pwm(_channel_params.pitch_channel, 1500) // 俯仰：抬头(+)/低头(-)
    };

    // 同样进行死区处理，避免姿态控制抖动
    for (uint8_t i = 0; i < 2; i++) {
        if (rollpitch_chan[i] > 1450 && rollpitch_chan[i] < 1550) {
            rollpitch_chan[i] = 1500;
        }
    }

    // 姿态控制信号处理：使用微分滤波器进行平滑处理
    // 原因：姿态控制需要更平滑的响应，避免高频噪声影响稳定性
    for (uint8_t i = 0; i < 2; i++) {
        // 动态设置微分滤波器的时间常数（由参数RPTD配置）
        _roll_pitch_td[i].set_r(_roll_pitch_td_r);

        // 通过微分滤波器处理输入值，提供带有微分预测的平滑控制响应
        // 这比简单的低通滤波能提供更好的动态响应特性
        float raw_input         = ((float)(rollpitch_chan[i]) - 1500.0f) / 500.0f;
        _throttle_roll_pitch[i] = _roll_pitch_td[i].update(raw_input);

        // 安全限制：确保姿态控制量在有效范围内
        _throttle_roll_pitch[i] = constrain_float(_throttle_roll_pitch[i], -1.0f, 1.0f);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第四部分：主工作模式切换（行走模式 vs 飞行模式）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取模式开关通道，用于在行走模式和飞行模式之间切换
    const uint16_t mode_value = read_rc_channel_pwm(_channel_params.mode_channel, 1000);

    // 阈值判断：PWM > 1800μs 切换到飞行模式，否则保持行走模式
    if (mode_value > 1800) {
        set_master_mode(Flying_Mode); // 飞行模式：用于特殊姿态和爪子形态
    } else {
        set_master_mode(Walking_Mode); // 行走模式：正常的六足行走控制
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第五部分：步态模式选择（仅在行走模式下有效）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取步态选择通道的PWM值，使用两档开关
    const uint16_t walk_value = read_rc_channel_pwm(_channel_params.walk_mode_channel, 1000);

    // 两档选择：低档为快速Tripod，高档为五腿支撑的Wave。
    set_walk_mode(walk_value > 1500 ? AP_HEXRUPED_GAIT_WAVE : AP_HEXRUPED_GAIT_TRIPOD);

    ////////////////////////////////////////////////////////////////////////////////////
    // 第六部分：飞行子模式选择（仅在飞行模式下有效）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取飞行模式选择通道，用于选择不同的特殊姿态
    const uint16_t flying_value = read_rc_channel_pwm(_channel_params.fly_mode_channel, 1000);

    // 六足只有两种飞行子模式:高档为横向爪,其余为默认爪(按高度收腿/展开)
    if (flying_value > 1500 && flying_value < 2100) {
        set_fly_mode(Fly_Mode_Heng_Claw); // 横向爪子形态：腿部形成横向抓取形状
    } else if (flying_value > 900) {
        set_fly_mode(Fly_Mode_Flying); // 飞行姿态：根据高度自动收起或展开腿部
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第七部分：爪子角度控制（用于爪子形态的角度调节）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取爪子控制通道，用于动态调节爪子开合角度
    const uint16_t claw_value = read_rc_channel_pwm(_channel_params.claw_channel, 1500);

    // HEX_CH_CLAW 线性: 1500=默认收起, 2000=合爪; <=1500 保持收起
    _claw_angle = ((float)claw_value - 1500.0f) / 500.0f * 90.0f;

}

/**
 * @brief 发送自定义MAVLink数据
 *
 * 定期发送六足机器人的状态信息到地面站，主要用于：
 * 1. 实时监控当前步态模式
 * 2. 提供调试和诊断信息
 * 3. 支持地面站显示和日志记录
 *
 * 发送频率限制为2Hz（每500ms一次），避免过多占用通信带宽
 */
void AP_HexRuped::send_custom_mavlink_data()
{
    static uint32_t _last_custom_send_ms;

    // 频率控制：限制发送频率为2Hz，避免通信拥塞
    if (AP_HAL::millis() - _last_custom_send_ms < 500) {
        return;
    }
    _last_custom_send_ms = AP_HAL::millis();

    // 获取当前系统时间（毫秒）
    uint32_t time_boot_ms = AP_HAL::millis();

    // 遍历所有MAVLink通信通道
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        // 检查该通道是否活跃（有设备连接）
        if (!(GCS_MAVLINK::active_channel_mask() & (1U << i))) {
            continue;
        }

        // 获取通道标识符
        mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + i);

        // 检查发送缓冲区是否有足够空间发送该消息
        if (!HAVE_PAYLOAD_SPACE(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT)) {
            continue;
        }

        // 发送当前步态模式信息
        mavlink_msg_named_value_int_send(
            chan,                    // MAVLink通道
            time_boot_ms,            // 时间戳
            "HEX_WALK_",             // 保持现有地面站字段兼容，内容为六足行走模式
            (int32_t)get_walk_mode() // 当前步态模式的整数值
        );
    }
}
