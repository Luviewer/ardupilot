#include "AP_QuadRuped.h"
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

    AP_GROUPINFO("GTYPE", 1, AP_QuadRuped, _gait_type, (int8_t)AP_QUADRUPED_GAIT_DIAGONAL),

    // 系统参数组 (11-20)
    AP_SUBGROUPINFO(_sys_params, "SYS_", 11, AP_QuadRuped, AP_QuadRuped_SYS_Params),

    // 通道参数组 (21-30)
    AP_SUBGROUPINFO(_channel_params, "CH_", 21, AP_QuadRuped, AP_QuadRuped_CHANNEL_Params),

    // 腿部参数组 (31-40)
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_RF], "RF_", 31, AP_QuadRuped, AP_QuadRuped_Params), // 右前腿
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_RB], "RB_", 32, AP_QuadRuped, AP_QuadRuped_Params), // 右后腿
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_LB], "LB_", 33, AP_QuadRuped, AP_QuadRuped_Params), // 左后腿
    AP_SUBGROUPINFO(_leg_params[AP_QUADRUPED_LEG_LF], "LF_", 34, AP_QuadRuped, AP_QuadRuped_Params), // 左前腿

    // 步态后端参数组 (41-50)
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_DIAGONAL], "DIAG_", 41, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL]),

#if AP_QUADRUPED_WAVE_ENABLE
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_WAVE], "WAVE_", 42, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_WAVE]),
#endif

#if AP_QUADRUPED_CRUBE_ENABLE
    AP_SUBGROUPVARPTR(_gait_backends[AP_QUADRUPED_GAIT_CRAB], "CRAB_", 43, AP_QuadRuped, backend_var_info[AP_QUADRUPED_GAIT_CRAB]),
#endif

    AP_GROUPEND
};

// 默认构造函数
AP_QuadRuped::AP_QuadRuped()
    : _ahrs(nullptr)
    , _motors(nullptr)
    , _rangefinder(nullptr)
    , _backend(nullptr)
    , _throttle_x(0.0f)
    , _throttle_y(0.0f)
    , _yaw_rate(0.0f)
    , _body_height(0.0f)
{
    // 初始化后端指针数组
    for (uint8_t i = 0; i < AP_QUADRUPED_GAIT_COUNT; i++) {
        _gait_backends[i]      = nullptr;
        _state[i].last_time_ms = 0;
        _state[i].instance     = i;
        _state[i].var_info     = nullptr;
    }

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
}

// 析构函数
AP_QuadRuped::~AP_QuadRuped()
{
    destroy_backends();
}

// 销毁后端实例
void AP_QuadRuped::destroy_backends()
{
    // 目前没有需要销毁的后端实例
    // 未来如果添加动态分配的后端，需要在这里释放内存
}

// 初始化函数 - 设置硬件接口
bool AP_QuadRuped::init(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder)
{
    _ahrs        = &ahrs;
    _motors      = &motors;
    _rangefinder = &rangefinder;

    return init_system();
}

// 初始化系统
bool AP_QuadRuped::init_system()
{
    // 创建后端实例
    create_backends();

    // 设置初始步态
    set_gait_type(get_gait_type());

    return _backend != nullptr;
}

// 创建后端实例
void AP_QuadRuped::create_backends()
{
    // 创建对角步态后端
    _gait_backends[AP_QUADRUPED_GAIT_DIAGONAL] = NEW_NOTHROW AP_QuadRuped_Diag(*this, _state[AP_QUADRUPED_GAIT_DIAGONAL], *_ahrs, *_motors);
    _gait_backends[AP_QUADRUPED_GAIT_DIAGONAL]->init();
    backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL] = _state[AP_QUADRUPED_GAIT_DIAGONAL].var_info;
    _state[AP_QUADRUPED_GAIT_DIAGONAL].instance  = AP_QUADRUPED_GAIT_DIAGONAL;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_DIAGONAL], backend_var_info[AP_QUADRUPED_GAIT_DIAGONAL]);

    // 创建波浪步态后端
#if AP_QUADRUPED_WAVE_ENABLE
    _gait_backends[AP_QUADRUPED_GAIT_WAVE] = new AP_QuadRuped_WAVE(*this, _state[AP_QUADRUPED_GAIT_WAVE], *_ahrs, *_motors);
    _gait_backends[AP_QUADRUPED_GAIT_WAVE]->init();
    backend_var_info[AP_QUADRUPED_GAIT_WAVE] = _state[AP_QUADRUPED_GAIT_WAVE].var_info;
    _state[AP_QUADRUPED_GAIT_WAVE].instance  = AP_QUADRUPED_GAIT_WAVE;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_WAVE], backend_var_info[AP_QUADRUPED_GAIT_WAVE]);
#endif
    // 创建工字步态后端
#if AP_QUADRUPED_CRUBE_ENABLE
    _gait_backends[AP_QUADRUPED_GAIT_CRAB] = new AP_QuadRuped_Crab(*this, _state[AP_QUADRUPED_GAIT_CRAB], *_ahrs, *_motors);
    _gait_backends[AP_QUADRUPED_GAIT_CRAB]->init();
    backend_var_info[AP_QUADRUPED_GAIT_CRAB] = _state[AP_QUADRUPED_GAIT_CRAB].var_info;
    _state[AP_QUADRUPED_GAIT_CRAB].instance  = AP_QUADRUPED_GAIT_CRAB;
    AP_Param::load_object_from_eeprom(_gait_backends[AP_QUADRUPED_GAIT_CRAB], backend_var_info[AP_QUADRUPED_GAIT_CRAB]);
#endif
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
        case AP_QUADRUPED_GAIT_DIAGONAL:
            set_gait_type(AP_QUADRUPED_GAIT_DIAGONAL);
            break;

#if AP_QUADRUPED_WAVE_ENABLE
        case AP_QUADRUPED_GAIT_WAVE:
            set_gait_type(AP_QUADRUPED_GAIT_WAVE);
            break;
#endif
#if AP_QUADRUPED_CRUBE_ENABLE
        case AP_QUADRUPED_GAIT_CRAB:
            set_gait_type(AP_QUADRUPED_GAIT_CRAB);
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
    if (type >= AP_QUADRUPED_GAIT_COUNT) {
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
    if (leg_index < AP_QUADRUPED_LEG_ALL) {
        return _leg_params[leg_index];
    }
    return _leg_params[AP_QUADRUPED_LEG_RF]; // 默认返回第一条腿的参数
}

// 读取遥控器输入
void AP_QuadRuped::read_radio_input()
{
    // 获取遥控器映射
    RC_Channels& rc_mapper = rc();

    // 读取各通道输入
    RC_Channel* throttle_x_chan = rc_mapper.rc_channel(_channel_params.throttle_x_channel);
    RC_Channel* throttle_y_chan = rc_mapper.rc_channel(_channel_params.throttle_y_channel);
    RC_Channel* yaw_chan        = rc_mapper.rc_channel(_channel_params.yaw_channel);

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
    // 身体高度通过参数配置，不通过遥控器通道直接控制
    _body_height = 0.0f; // 使用默认值

    // 限制油门输入范围
    _throttle_x  = constrain_float(_throttle_x, -1.0f, 1.0f);
    _throttle_y  = constrain_float(_throttle_y, -1.0f, 1.0f);
    _yaw_rate    = constrain_float(_yaw_rate, -1.0f, 1.0f);
    _body_height = constrain_float(_body_height, -1.0f, 1.0f);
}

// 静态成员变量定义
const struct AP_Param::GroupInfo* AP_QuadRuped::backend_var_info[AP_QUADRUPED_GAIT_COUNT];
