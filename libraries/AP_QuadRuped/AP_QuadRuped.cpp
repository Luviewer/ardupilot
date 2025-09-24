#include "AP_QuadRuped.h"
#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped_Diag.h"
#include <AP_RCMapper/AP_RCMapper.h>
#include <RC_Channel/RC_Channel.h>

#if AP_QUADRUPED_WAVE_ENABLE
# include "AP_QuadRuped_WAVE_New.h"
#endif
#if AP_QUADRUPED_CRUBE_ENABLE
# include "AP_QuadRuped_Crab.h"
#endif

// 参数定义
const AP_Param::GroupInfo AP_QuadRuped::var_info[] = {
    // 基础参数组 (1-10)
    AP_GROUPINFO("ENABLE", 2, AP_QuadRuped, _enabled, 1),

    AP_GROUPINFO("GTYPE", 1, AP_QuadRuped, _gait_type, (int8_t)GAIT_DIAGONAL),

    // 系统参数组 (11-20)
    AP_SUBGROUPINFO(_sys_params, "SYS_", 11, AP_QuadRuped, AP_QuadRuped_SYS_Params),

    // 通道参数组 (21-30)
    AP_SUBGROUPINFO(_channel_params, "CH_", 21, AP_QuadRuped, AP_QuadRuped_CHANNEL_Params),

    // 腿部参数组 (31-40)
    AP_SUBGROUPINFO(_leg_params[Leg_RF], "RF_", 31, AP_QuadRuped, AP_QuadRuped_Params), // 右前腿
    AP_SUBGROUPINFO(_leg_params[Leg_RB], "RB_", 32, AP_QuadRuped, AP_QuadRuped_Params), // 右后腿
    AP_SUBGROUPINFO(_leg_params[Leg_LB], "LB_", 33, AP_QuadRuped, AP_QuadRuped_Params), // 左后腿
    AP_SUBGROUPINFO(_leg_params[Leg_LF], "LF_", 34, AP_QuadRuped, AP_QuadRuped_Params), // 左前腿

    AP_GROUPEND
};

// 构造函数
AP_QuadRuped::AP_QuadRuped(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder)
    : _ahrs(ahrs)
    , _motors(motors)
    , _rangefinder(rangefinder)
    , _backend(nullptr)
    , _throttle_x(0.0f)
    , _throttle_y(0.0f)
    , _yaw_rate(0.0f)
    , _body_height(0.0f)
{
    // 初始化后端指针数组
    for (uint8_t i = 0; i < GAIT_COUNT; i++) {
        _gait_backends[i] = nullptr;
    }

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
}

// 析构函数
AP_QuadRuped::~AP_QuadRuped()
{
    destroy_backends();
}

// 初始化系统
bool AP_QuadRuped::init()
{
    // 创建后端实例
    create_backends();

    // 设置初始步态
    set_gait_type(get_gait_type());

    return _backend != nullptr;
}

// 主更新循环
void AP_QuadRuped::update()
{
    // 检查是否启用
    if (!_enabled) {
        return;
    }

    // 读取遥控器输入
    read_radio_input();

    switch (_gait_type) {
        default:
        case GAIT_DIAGONAL:
            set_gait_type(GAIT_DIAGONAL);
            break;

#if AP_QUADRUPED_WAVE_ENABLE
        case GAIT_WAVE:
            set_gait_type(GAIT_WAVE);
            break;
#endif
#if AP_QUADRUPED_CRUBE_ENABLE
        case GAIT_CRAB:
            set_gait_type(GAIT_CRAB);
            break;
#endif
    }

    // 调用后端更新
    if (_backend) {
        _backend->update();
    }
}

// 设置步态类型
void AP_QuadRuped::set_gait_type(GaitType type)
{
    if (type >= GAIT_COUNT) {
        return;
    }

    if (_gait_backends[type]) {
        _backend = _gait_backends[type];
        _backend->init();
    }
}

// 设置油门输入
void AP_QuadRuped::set_throttle(float throttle_x, float throttle_y)
{
    _throttle_x = throttle_x;
    _throttle_y = throttle_y;
}

// 设置偏航角速度
void AP_QuadRuped::set_yaw_rate(float yaw_rate)
{
    _yaw_rate = yaw_rate;
}

// 设置机身高度
void AP_QuadRuped::set_body_height(float height)
{
    _body_height = height;
}

// 获取腿部参数
const AP_QuadRuped_Params& AP_QuadRuped::get_leg_params(uint8_t leg_index) const
{
    if (leg_index < LEG_ALL) {
        return _leg_params[leg_index];
    }
    return _leg_params[Leg_RF]; // 默认返回第一条腿的参数
}

// 创建后端实例
void AP_QuadRuped::create_backends()
{
    // 创建对角步态后端
    _gait_backends[GAIT_DIAGONAL] = new AP_QuadRuped_Diag(*this, _ahrs, _motors);
    _gait_backends[GAIT_DIAGONAL]->init();

    // 创建波浪步态后端
#if AP_QUADRUPED_WAVE_ENABLE
    _gait_backends[GAIT_WAVE] = new AP_QuadRuped_WAVE(*this, _ahrs, _motors);
    _gait_backends[GAIT_WAVE]->init();
#endif
    // 创建工字步态后端
#if AP_QUADRUPED_CRUBE_ENABLE
    _gait_backends[GAIT_CRAB] = new AP_QuadRuped_Crab(*this, _ahrs, _motors);
    _gait_backends[GAIT_CRAB]->init();
#endif
}

// 读取遥控器输入
void AP_QuadRuped::read_radio_input()
{
    // 获取遥控器映射
    const RC_Channels& rc_mapper = rc()::get_singleton();

    // 读取各通道输入
    RC_Channel* throttle_x_chan = rc_mapper.rc_channel(_channel_params.throttle_x_channel);
    RC_Channel* throttle_y_chan = rc_mapper.rc_channel(_channel_params.throttle_y_channel);
    RC_Channel* yaw_chan        = rc_mapper.rc_channel(_channel_params.yaw_channel);
    RC_Channel* height_chan     = rc_mapper.rc_channel(_channel_params.height_channel);

    // 获取输入值并归一化
    if (throttle_x_chan) {
        _throttle_x = throttle_x_chan->norm_input();
    }
    if (throttle_y_chan) {
        _throttle_y = throttle_y_chan->norm_input();
    }
    if (yaw_chan) {
        _yaw_rate = yaw_chan->norm_input();
    }
    if (height_chan) {
        _body_height = height_chan->norm_input();
    }

    // 限制油门输入范围
    _throttle_x  = constrain_float(_throttle_x, -1.0f, 1.0f);
    _throttle_y  = constrain_float(_throttle_y, -1.0f, 1.0f);
    _yaw_rate    = constrain_float(_yaw_rate, -1.0f, 1.0f);
    _body_height = constrain_float(_body_height, -1.0f, 1.0f);
}
