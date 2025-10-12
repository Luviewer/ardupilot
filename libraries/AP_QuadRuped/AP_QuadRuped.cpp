// 四足机器人主控制器实现文件
// 负责管理四足机器人的整体控制逻辑，包括步态切换、遥控器输入处理、硬件接口管理等

#include "AP_QuadRuped.h"
#include "AP_QuadRuped_Diag.h"              // 对角步态后端
#include <AP_RCMapper/AP_RCMapper.h>         // 遥控器映射
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>  // 测距传感器
#include <RC_Channel/RC_Channel.h>          // 遥控通道

// 条件编译包含各种步态后端
#if AP_QUADRUPED_WAVE_ENABLE
# include "AP_QuadRuped_Wave.h"             // 波浪步态后端
#endif
#if AP_QUADRUPED_WAVE_COG_ENABLE
# include "AP_QuadRuped_Wave_COG.h"         // 带重心平移的波浪步态
#endif
#if AP_QUADRUPED_ZongXiang_ENABLE
# include "AP_QuadRuped_ZongXiang.h"        // 纵向工字步态
#endif
#if AP_QuadRuped_HengXiang_ENABLE
# include "AP_QuadRuped_HengXiang.h"        // 横向工字步态
#endif


/**
 * @brief 四足机器人参数表定义
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
const AP_Param::GroupInfo AP_QuadRuped::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: 四足机器人功能使能
    // @Description: 启用或禁用四足机器人功能
    // @Values: 0:禁用, 1:启用
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_QuadRuped, _enabled, 1),

    // @Param: CLASS
    // @DisplayName: 四足机器人类型
    // @Description: 选择四足机器人的硬件类型
    // @Values: 0:普通型, 1:USL_BV2型
    // @User: Standard
    AP_GROUPINFO("CLASS", 3, AP_QuadRuped, _quadruped_class, (int8_t)AP_QUADRUPED_NORMAL),

    // @Param: RPTD
    // @DisplayName: 横滚俯仰滤波时间常数
    // @Description: 横滚和俯仰控制的微分项时间常数，用于滤波处理
    // @Units: ms
    // @Range: 100 5000
    // @User: Advanced
    AP_GROUPINFO("RPTD", 4, AP_QuadRuped, _roll_pitch_td_r, (float)1000),

    // 系统参数组 (11-20)
    // @Group: SYS_
    AP_SUBGROUPINFO(_sys_params, "SYS_", 11, AP_QuadRuped, AP_QuadRuped_SYS_Params),

    // 通道参数组 (21-30)
    // @Group: CH_
    // @Description: 遥控器通道映射参数，定义各个功能对应的遥控通道
    AP_SUBGROUPINFO(_channel_params, "CH_", 21, AP_QuadRuped, AP_QuadRuped_CHANNEL_Params),

    // 腿部参数组 (31-40)
    // @Group: RF_
    // @Description: 右前腿(Right Front)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_RF], "RF_", 31, AP_QuadRuped, AP_QuadRuped_Params),

    // @Group: RB_
    // @Description: 右后腿(Right Back)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_RB], "RB_", 32, AP_QuadRuped, AP_QuadRuped_Params),

    // @Group: LB_
    // @Description: 左后腿(Left Back)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_LB], "LB_", 33, AP_QuadRuped, AP_QuadRuped_Params),

    // @Group: LF_
    // @Description: 左前腿(Left Front)的运动学参数和限制参数
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_LF], "LF_", 34, AP_QuadRuped, AP_QuadRuped_Params),

    // 步态后端参数组 (41-50)
    // @Group: DIAG_
    // @Description: 对角步态(Diagonal/Trot)的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_DIAGONAL], "DIAG_", 41, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL]),

#if AP_QUADRUPED_WAVE_ENABLE
    // @Group: WAVE_
    // @Description: 波浪步态(Wave/Crawl)的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_WAVE], "WAVE_", 42, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_WAVE]),
#endif
#if AP_QUADRUPED_WAVE_COG_ENABLE
    // @Group: WCOG_
    // @Description: 带重心平移的波浪步态(Wave with Center of Gravity)的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_WAVE_COG], "WCOG_", 43, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_WAVE_COG]),
#endif
#if AP_QUADRUPED_ZongXiang_ENABLE
    // @Group: SHU_
    // @Description: 纵向工字步态的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_ZongXiang], "SHU_", 44, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_ZongXiang]),
#endif
#if AP_QuadRuped_HengXiang_ENABLE
    // @Group: HEN_
    // @Description: 横向工字步态的特定参数
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_HengXiang], "HEN_", 45, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_HengXiang]),
#endif

    AP_GROUPEND
};

// 静态成员变量定义 - 存储各种步态后端的参数表指针
const struct AP_Param::GroupInfo* AP_QuadRuped::backend_var_info[AP_QUADRUPED_GAIT_COUNT];

/**
 * @brief 默认构造函数
 *
 * 初始化四足机器人控制器的所有成员变量：
 * - 将硬件接口指针设为nullptr，等待init()函数设置
 * - 初始化所有步态后端的状态数组
 * - 设置初始步态类型为无效值
 * - 加载参数默认值
 */
AP_QuadRuped::AP_QuadRuped()
    : _ahrs(nullptr)         // 姿态航向参考系统指针初始化
    , _motors(nullptr)       // 电机控制接口指针初始化
    , _rangefinder(nullptr)  // 测距传感器接口指针初始化
    , _backend(nullptr)      // 当前活跃步态后端指针初始化
{
    // 初始化所有步态后端的状态结构体
    for (uint8_t i = 0; i < AP_QUADRUPED_GAIT_COUNT; i++) {
        _gait_backends[i]      = nullptr;  // 后端实例指针置空
        _state[i].last_time_ms = 0;        // 上次更新时间清零
        _state[i].instance     = i;        // 设置步态实例编号
        _state[i].var_info     = nullptr;  // 参数表指针置空
    }

    _gait_last_type = -1;  // 设置上次步态类型为无效值，确保首次切换成功

    // 从参数表中加载默认值到成员变量
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief 销毁所有步态后端实例
 *
 * 释放所有已创建的步态后端对象内存，防止内存泄漏。
 * 通常在析构函数或重新初始化时调用。
 */
void AP_QuadRuped::destroy_backends()
{
    // 遍历所有步态类型，释放对应的后端实例内存
    for (uint8_t i = 0; i < AP_QUADRUPED_GAIT_COUNT; i++) {
        if (_gait_backends[i] != nullptr) {
            delete _gait_backends[i];  // 释放后端对象内存
            _gait_backends[i] = nullptr;  // 指针置空，防止悬空指针
        }
    }
}

/**
 * @brief 初始化四足机器人控制器
 *
 * 设置硬件接口引用，创建所有步态后端实例，初始化滤波器，并设置默认步态。
 * 这是系统启动后必须调用的关键函数。
 *
 * @param ahrs 姿态航向参考系统引用，用于获取机体姿态
 * @param motors 电机控制接口引用，用于控制伺服电机
 * @param rangefinder 测距传感器引用，用于高度检测
 * @return bool 初始化是否成功，true表示至少有一个步态后端创建成功
 */
bool AP_QuadRuped::init(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder)
{
    // 保存硬件接口引用
    _ahrs        = &ahrs;        // 姿态传感器接口
    _motors      = &motors;      // 电机控制接口
    _rangefinder = &rangefinder; // 测距传感器接口

    // 创建所有可用的步态后端实例
    create_backends();

    // 初始化横滚和俯仰的微分滤波器
    // 参数：初始值=0, 时间常数=配置值, 初始微分值=0
    _roll_pitch_td[0].init(0.001, _roll_pitch_td_r, 0);  // 横滚通道滤波器
    _roll_pitch_td[1].init(0.001, _roll_pitch_td_r, 0);  // 俯仰通道滤波器

    // 设置默认步态为对角步态（最稳定和常用）
    set_gait_type(AP_QUADRUPED_GAIT_DIAGONAL);

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
void AP_QuadRuped::create_backends()
{
    // 创建对角步态后端 - Trot步态，最稳定高效
    _gait_backends[AP_QUADRUPED_GAIT_DIAGONAL]   = NEW_NOTHROW AP_QuadRuped_Diag(*this, _state[AP_QUADRUPED_GAIT_DIAGONAL], *_ahrs, *_motors);
    backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL] = _state[AP_QUADRUPED_GAIT_DIAGONAL].var_info;
    _state[AP_QUADRUPED_GAIT_DIAGONAL].instance  = AP_QUADRUPED_GAIT_DIAGONAL;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_DIAGONAL], backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL]);

    // 条件编译创建其他步态后端
#if AP_QUADRUPED_WAVE_ENABLE
    // 创建波浪步态后端 - Crawl步态，最稳定但速度慢
    _gait_backends[AP_QUADRUPED_GAIT_WAVE]   = NEW_NOTHROW AP_QuadRuped_Wave(*this, _state[AP_QUADRUPED_GAIT_WAVE], *_ahrs, *_motors);
    backend_var_info[AP_QUADRUPED_GAIT_WAVE] = _state[AP_QUADRUPED_GAIT_WAVE].var_info;
    _state[AP_QUADRUPED_GAIT_WAVE].instance  = AP_QUADRUPED_GAIT_WAVE;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_WAVE], backend_var_info[AP_QUADRUPED_GAIT_WAVE]);
#endif

#if AP_QUADRUPED_WAVE_COG_ENABLE
    // 创建带重心平移的波浪步态 - 在波浪步态基础上增加重心控制
    _gait_backends[AP_QUADRUPED_GAIT_WAVE_COG]   = NEW_NOTHROW AP_QuadRuped_Wave_COG(*this, _state[AP_QUADRUPED_GAIT_WAVE_COG], *_ahrs, *_motors);
    backend_var_info[AP_QUADRUPED_GAIT_WAVE_COG] = _state[AP_QUADRUPED_GAIT_WAVE_COG].var_info;
    _state[AP_QUADRUPED_GAIT_WAVE_COG].instance  = AP_QUADRUPED_GAIT_WAVE_COG;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_WAVE_COG], backend_var_info[AP_QUADRUPED_GAIT_WAVE_COG]);
#endif

    // 创建纵向工字步态后端 - 用于特殊地形通过
#if AP_QUADRUPED_ZongXiang_ENABLE
    _gait_backends[AP_QUADRUPED_GAIT_ZongXiang]   = NEW_NOTHROW AP_QuadRuped_ZongXiang(*this, _state[AP_QUADRUPED_GAIT_ZongXiang], *_ahrs, *_motors);
    backend_var_info[AP_QUADRUPED_GAIT_ZongXiang] = _state[AP_QUADRUPED_GAIT_ZongXiang].var_info;
    _state[AP_QUADRUPED_GAIT_ZongXiang].instance  = AP_QUADRUPED_GAIT_ZongXiang;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_ZongXiang], backend_var_info[AP_QUADRUPED_GAIT_ZongXiang]);
#endif

    // 创建横向工字步态后端 - 用于侧向特殊地形通过
#if AP_QuadRuped_HengXiang_ENABLE
    _gait_backends[AP_QUADRUPED_GAIT_HengXiang]   = NEW_NOTHROW AP_QuadRuped_HengXiang(*this, _state[AP_QUADRUPED_GAIT_HengXiang], *_ahrs, *_motors);
    backend_var_info[AP_QUADRUPED_GAIT_HengXiang] = _state[AP_QUADRUPED_GAIT_HengXiang].var_info;
    _state[AP_QUADRUPED_GAIT_HengXiang].instance  = AP_QUADRUPED_GAIT_HengXiang;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_HengXiang], backend_var_info[AP_QUADRUPED_GAIT_HengXiang]);
#endif
}

/**
 * @brief 四足机器人主更新循环
 *
 * 这是系统的核心更新函数，以固定频率调用，负责：
 * 1. 检查系统使能状态
 * 2. 按步态频率控制更新周期
 * 3. 读取和处理遥控器输入
 * 4. 根据当前模式执行相应动作
 *
 * 更新频率由当前步态的get_Freq()方法决定，通常为50-200Hz
 */
void AP_QuadRuped::update()
{
    // 检查四足机器人功能是否已启用
    if (!_enabled) {
        return;
    }

    // 检查当前是否有活跃的步态后端
    if (_backend == nullptr) {
        return;
    }

    // 发送自定义MAVLink数据（状态信息）
    send_custom_mavlink_data();

    // 频率控制：确保按照当前步态设定的频率更新
    // 计算距离上次更新的时间间隔，如果小于步态周期则跳过本次更新
    if ((AP_HAL::millis() - lasttime) < (1000 / _backend->get_Freq())) {
        return;
    }

    // 更新最后执行时间戳
    lasttime = AP_HAL::millis();

    // 读取并处理遥控器输入数据
    read_radio_input();

    // 根据主模式执行相应的控制逻辑
    switch (fly_walk_mode.master_mode) {
        default:
        case Walking_Mode:  // 行走模式
            // 设置当前选择的步态类型
            set_gait_type((GaitType)fly_walk_mode.walk_mode);
            // 调用当前步态后端的更新函数
            _backend->update();
            break;

        case Flying_Mode:   // 飞行模式（特殊姿态模式）
            switch (fly_walk_mode.fly_mode) {
                default:
                case Fly_Mode_Flying: {
                    // 获取测距传感器数据进行高度检测
                    const AP_RangeFinder_Backend* sensor = _rangefinder->get_backend(0);
                    if (sensor) {
                        // 读取地面距离（厘米）
                        uint16_t sonar_cm = sensor->distance_cm();
                        if (sonar_cm > 40) {
                            // 离地较高时：收起腿部成X形向上姿态
                            _backend->x_up_sleep_leg();
                            break;
                        }
                    }
                    // 离地较低或无传感器时：收起腿部成X形睡眠姿态
                    _backend->x_sleep_leg();
                } break;

                case Fly_Mode_Zhong_Claw:
                    // 纵向爪子模式：腿部形成纵向爪子形状
                    _backend->zhongxiang_claw_leg(get_claw_angle());
                    break;

                case Fly_Mode_Heng_Claw:
                    // 横向爪子模式：腿部形成横向爪子形状
                    _backend->hengxiang_claw_leg(get_claw_angle());
                    break;
            }
            break;
    }
}

/**
 * @brief 设置当前步态类型
 *
 * 切换四足机器人的步态模式。步态切换会：
 * 1. 验证步态类型有效性
 * 2. 检查是否需要切换（避免重复切换）
 * 3. 切换到新的步态后端
 * 4. 初始化新步态
 *
 * @param type 要切换到的步态类型
 */
void AP_QuadRuped::set_gait_type(GaitType type)
{
    // 验证步态类型是否在有效范围内
    if (type >= AP_QUADRUPED_GAIT_COUNT) {
        return;
    }

    // 如果已经是当前步态，则无需切换
    if (_gait_last_type == type)
        return;

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
 * @param leg_index 腿部索引（0-3分别对应RF, RB, LB, LF）
 * @return const AP_QuadRuped_Params& 腿部参数的常量引用
 */
const AP_QuadRuped_Params& AP_QuadRuped::get_leg_params(uint8_t leg_index) const
{
    // 验证腿部索引是否在有效范围内（0-3）
    if (leg_index < AP_QUADRUPED_LEG_ALL) {
        return _leg_params[leg_index];
    }
    // 索引无效时，默认返回右前腿的参数
    return _leg_params[AP_QUADRUPED_LEG_RF];
}

/**
 * @brief 读取和处理遥控器输入
 *
 * 这是四足机器人遥控输入处理的核心函数，每周期调用一次，负责：
 * 1. 故障保护检测和安全模式切换
 * 2. 运动控制输入处理（X、Y、Z轴）
 * 3. 姿态平衡控制输入处理（横滚、俯仰）
 * 4. 主工作模式切换（行走/飞行模式）
 * 5. 步态模式选择（对角、波浪、工字等）
 * 6. 飞行子模式选择（飞行、爪子形态）
 * 7. 爪子角度控制
 *
 * PWM信号规范：
 * - 1000μs: 最小值（完全反向）
 * - 1500μs: 中位值（零输入/停止）
 * - 2000μs: 最大值（完全正向）
 * - 1450-1550μs: 死区范围，视为1500μs处理
 *
 * @note 故障保护状态下会立即停止所有运动并切换到安全的对角步态
 */
void AP_QuadRuped::read_radio_input()
{
    ////////////////////////////////////////////////////////////////////////////////////
    // 第一部分：遥控器故障保护检测
    ////////////////////////////////////////////////////////////////////////////////////
    // 检查遥控器是否处于故障保护状态（信号丢失或超出范围）
    if (rc().in_rc_failsafe()) {
        // 故障保护响应：立即清零所有运动指令，确保机器人安全停止
        _throttle_xyz        = Vector3f(0, 0, 0);     // 清零前进、横向、偏航指令
        _throttle_roll_pitch = Vector2f(0, 0);        // 清零横滚、俯仰平衡指令

        // 切换到最安全的默认模式组合
        set_master_mode(Walking_Mode);                 // 设为行走模式（最基础的控制模式）
        set_walk_mode(AP_QUADRUPED_GAIT_DIAGONAL);    // 设为对角步态（最稳定，支撑面最大）
        return;  // 直接返回，不处理后续输入
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第二部分：XYZ运动轴输入处理（前进、横向、偏航）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取运动控制通道的原始PWM值
    // 注意：通道号-1是因为AP_HAL的rcin使用0基索引，而参数配置使用1基索引
    Vector3ui throttle_chan = {
        hal.rcin->read(_channel_params.throttle_x_channel - 1),  // X轴：前进(+)/后退(-)
        hal.rcin->read(_channel_params.throttle_y_channel - 1),  // Y轴：左移(+)/右移(-)
        hal.rcin->read(_channel_params.yaw_channel - 1)          // Z轴：逆时针(+)/顺时针(-)旋转
    };

    // 为未配置的通道设置默认中位值（1500μs表示零输入）
    if (_channel_params.throttle_x_channel == -1) throttle_chan[0] = 1500;
    if (_channel_params.throttle_y_channel == -1) throttle_chan[1] = 1500;
    if (_channel_params.yaw_channel == -1) throttle_chan[2] = 1500;

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
        hal.rcin->read(_channel_params.roll_channel - 1),   // 横滚：左倾(+)/右倾(-)
        hal.rcin->read(_channel_params.pitch_channel - 1)   // 俯仰：抬头(+)/低头(-)
    };

    // 为未配置的通道设置默认中位值
    if (_channel_params.roll_channel == -1) rollpitch_chan[0] = 1500;
    if (_channel_params.pitch_channel == -1) rollpitch_chan[1] = 1500;

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
        float raw_input = ((float)(rollpitch_chan[i]) - 1500.0f) / 500.0f;
        _throttle_roll_pitch[i] = _roll_pitch_td[i].update(raw_input);

        // 安全限制：确保姿态控制量在有效范围内
        _throttle_roll_pitch[i] = constrain_float(_throttle_roll_pitch[i], -1.0f, 1.0f);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第四部分：主工作模式切换（行走模式 vs 飞行模式）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取模式开关通道，用于在行走模式和飞行模式之间切换
    uint16_t mode_value = hal.rcin->read(_channel_params.mode_channel - 1);
    if (_channel_params.mode_channel == -1) mode_value = 1000;  // 默认为低电平（行走模式）

    // 阈值判断：PWM > 1800μs 切换到飞行模式，否则保持行走模式
    if (mode_value > 1800) {
        set_master_mode(Flying_Mode);   // 飞行模式：用于特殊姿态和爪子形态
    } else {
        set_master_mode(Walking_Mode);  // 行走模式：正常的四足行走控制
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第五部分：步态模式选择（仅在行走模式下有效）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取步态选择通道的PWM值，支持多档位开关
    uint16_t walk_value = hal.rcin->read(_channel_params.walk_mode_channel - 1);
    if (_channel_params.walk_mode_channel == -1) walk_value = 1000;  // 默认最低档位

    // 分段判断步态模式：不同PWM范围对应不同的步态
    // 从高到低排列，优先匹配高PWM值（高档位）
    if (walk_value > 1800 && walk_value < 2100) {
        set_walk_mode(AP_QUADRUPED_GAIT_HengXiang);  // 档位5：横向工字步态
    } else if (walk_value > 1600 && walk_value < 1800) {
        set_walk_mode(AP_QUADRUPED_GAIT_ZongXiang);  // 档位4：纵向工字步态
    } else if (walk_value > 1350 && walk_value <= 1600) {
        // 档位3：根据编译配置选择波浪步态类型
#if AP_QUADRUPED_WAVE_COG_ENABLE
        set_walk_mode(AP_QUADRUPED_GAIT_WAVE_COG);    // 带重心平移的波浪步态
#else
        set_walk_mode(AP_QUADRUPED_GAIT_WAVE);        // 普通波浪步态
#endif
    } else if (walk_value > 900 && walk_value <= 1350) {
        set_walk_mode(AP_QUADRUPED_GAIT_DIAGONAL);    // 档位1：对角步态（默认/最稳定）
    }
    // 注意：PWM < 900μs 或 > 2100μs 被视为无效信号，保持当前模式不变

    ////////////////////////////////////////////////////////////////////////////////////
    // 第六部分：飞行子模式选择（仅在飞行模式下有效）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取飞行模式选择通道，用于选择不同的特殊姿态
    uint16_t flying_value = hal.rcin->read(_channel_params.fly_mode_channel - 1);
    if (_channel_params.fly_mode_channel == -1) flying_value = 1000;  // 默认飞行姿态

    // 飞行子模式判断：同样采用分段PWM范围判断
    if (flying_value > 1800 && flying_value < 2100) {
        set_fly_mode(Fly_Mode_Heng_Claw);    // 横向爪子形态：腿部形成横向抓取形状
    } else if (flying_value > 1400 && flying_value < 1600) {
        set_fly_mode(Fly_Mode_Zhong_Claw);   // 纵向爪子形态：腿部形成纵向抓取形状
    } else if (flying_value > 900 && flying_value < 1100) {
        set_fly_mode(Fly_Mode_Flying);       // 飞行姿态：根据高度自动收起或展开腿部
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // 第七部分：爪子角度控制（用于爪子形态的角度调节）
    ////////////////////////////////////////////////////////////////////////////////////
    // 读取爪子控制通道，用于动态调节爪子开合角度
    uint16_t claw_value = hal.rcin->read(_channel_params.claw_channel - 1);
    if (_channel_params.claw_channel == -1) claw_value = 1500;  // 默认中位角度（0度）

    // 将爪子PWM值转换为角度（范围：-90度到+90度）
    // 1500μs对应0度，1000μs对应-90度，2000μs对应+90度
    _claw_angle = ((float)claw_value - 1500.0f) / 500.0f * 90.0f;

    ////////////////////////////////////////////////////////////////////////////////////
    // 调试代码段（已注释，用于开发时输出遥控器输入数据）
    ////////////////////////////////////////////////////////////////////////////////////
    // static uint32_t lasttime = 0;
    // if (AP_HAL::millis() - lasttime > 1000) {
    //     lasttime = AP_HAL::millis();
    //     // 每秒输出一次当前的遥控器输入值，用于调试和校准
    //     gcs().send_text(MAV_SEVERITY_NOTICE, "_throttle_x, y,z:%f, %f,%f", _throttle_x, _throttle_y, _yaw_rate);
    // }
}

/**
 * @brief 发送自定义MAVLink数据
 *
 * 定期发送四足机器人的状态信息到地面站，主要用于：
 * 1. 实时监控当前步态模式
 * 2. 提供调试和诊断信息
 * 3. 支持地面站显示和日志记录
 *
 * 发送频率限制为2Hz（每500ms一次），避免过多占用通信带宽
 */
void AP_QuadRuped::send_custom_mavlink_data()
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
            chan,                           // MAVLink通道
            time_boot_ms,                   // 时间戳
            "QRD_WALK_",                    // 数据名称标识符（四足机器人行走模式）
            (int32_t)get_walk_mode()        // 当前步态模式的整数值
        );
    }
}
