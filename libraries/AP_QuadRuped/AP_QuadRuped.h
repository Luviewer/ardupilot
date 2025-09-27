#pragma once

#include "AP_QuadRuped_Config.h"
#include "AP_QuadRuped_Defines.h"
#include "AP_QuadRuped_Params.h"
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// 前向声明
class AP_QuadRuped_Backend;
class AP_QuadRuped_Diag;

#if AP_QUADRUPED_WAVE_ENABLE
class AP_QuadRuped_Wave;
#endif
#if AP_QUADRUPED_ZongXiang_ENABLE
class AP_QuadRuped_ZongXiang;
#endif
#if AP_QuadRuped_HengXiang_ENABLE
class AP_QuadRuped_HengXiang;
#endif

class AP_QuadRuped {
    friend class AP_QuadRuped_Backend;
    friend class AP_QuadRuped_Diag;
    friend class AP_QuadRuped_ZongXiang;
    friend class AP_QuadRuped_HengXiang;
    friend class AP_QuadRuped_Wave;

public:
    // 默认构造函数
    AP_QuadRuped();

    // 析构函数
    ~AP_QuadRuped() { destroy_backends(); }

    // 初始化函数 - 设置姿态传感器和电机控制接口
    bool init(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder);

    // 参数表定义 - 用于配置系统参数
    static const struct AP_Param::GroupInfo  var_info[];
    static const struct AP_Param::GroupInfo* backend_var_info[AP_QUADRUPED_GAIT_COUNT];

    void update(); // 主更新循环

    QuadRupedClass get_class() const { return (QuadRupedClass)_quadruped_class.get(); }

    // 步态控制
    void     set_gait_type(GaitType type);
    GaitType get_gait_type() const { return (GaitType)_gait_last_type; }

    // 控制输入接口
    void set_throttle(float throttle_x, float throttle_y) { _throttle_xyz.xy() = { throttle_x, throttle_y }; }

    void set_yaw_rate(float yaw_rate) { _throttle_xyz.z = yaw_rate; }

    // 状态查询接口
    float get_throttle_x() const { return _throttle_xyz.x; }
    float get_throttle_y() const { return _throttle_xyz.y; }
    float get_yaw_rate() const { return _throttle_xyz.z; }

    uint16_t get_mode_channel() { return hal.rcin->read(_channel_params.mode_channel - 1); }
    uint16_t get_fly_mode_channel() { return hal.rcin->read(_channel_params.fly_mode_channel - 1); }
    uint16_t get_walk_mode_channel() { return hal.rcin->read(_channel_params.walk_mode_channel - 1); }

    // 参数访问接口
    uint8_t get_master_mode() { return fly_walk_mode.master_mode; }
    uint8_t get_walk_mode() { return fly_walk_mode.walk_mode; }
    uint8_t get_fly_mode() { return fly_walk_mode.fly_mode; }
    void    set_master_mode(uint8_t value) { fly_walk_mode.master_mode = value; }
    void    set_walk_mode(uint8_t value) { fly_walk_mode.walk_mode = value; }
    void    set_fly_mode(uint8_t value) { fly_walk_mode.fly_mode = value; }

    float get_claw_angle() { return _claw_angle; }

    // 参数访问接口
    const AP_QuadRuped_Params&         get_leg_params(uint8_t leg_index) const;
    const AP_QuadRuped_SYS_Params&     get_sys_params() const { return _sys_params; }
    const AP_QuadRuped_CHANNEL_Params& get_channel_params() const { return _channel_params; }

    // 硬件接口访问
    AP_AHRS_View& get_ahrs() { return *_ahrs; }
    AP_Motors&    get_motors() { return *_motors; }
    RangeFinder&  get_rangefinder() { return *_rangefinder; }

    void send_custom_mavlink_data();

private:
    // 状态结构体定义
    struct QuadRuped_State {
        uint32_t                          last_time_ms;
        uint8_t                           instance;
        const struct AP_Param::GroupInfo* var_info;
    } _state[AP_QUADRUPED_GAIT_COUNT];

    // 硬件接口
    AP_AHRS_View* _ahrs;        // 姿态航向参考系统
    AP_Motors*    _motors;      // 电机控制接口
    RangeFinder*  _rangefinder; // 测距雷达接口

    // 后端管理
    AP_QuadRuped_Backend* _backend;                                // 当前活跃的后端
    AP_QuadRuped_Backend* _gait_backends[AP_QUADRUPED_GAIT_COUNT]; // 所有的步态后端

    // 使能状态
    AP_Int8 _enabled;

    // 上次步态类型
    int8_t _gait_last_type;

    // 四足类型
    AP_Int8 _quadruped_class;

    // 控制输入
    Vector3f _throttle_xyz; // Xyz轴油门输入

    // 抓取角度
    float _claw_angle;

    // 参数组
    AP_QuadRuped_SYS_Params     _sys_params;                       // 系统参数
    AP_QuadRuped_Params         _leg_params[AP_QUADRUPED_LEG_ALL]; // 腿部参数
    AP_QuadRuped_CHANNEL_Params _channel_params;                   // 通道参数

    uint32_t lasttime;

    enum {
        Fly_Mode_Flying = 0,
        Fly_Mode_Zhong_Claw,
        Fly_Mode_Heng_Claw,
        Fly_Mode_Total,
    };

    enum {
        Walking_Mode = 0,
        Flying_Mode  = 1,
    };

    struct {
        uint8_t fly_mode;
        uint8_t walk_mode;
        uint8_t master_mode;
    } fly_walk_mode;

    // 内部辅助函数
    void create_backends();
    void destroy_backends();
    void read_radio_input();
};
