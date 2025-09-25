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

// 前向声明
class AP_QuadRuped_Backend;
class AP_QuadRuped_Diag;

#if AP_QUADRUPED_WAVE_ENABLE
class AP_QuadRuped_WAVE;
#endif
#if AP_QUADRUPED_CRUBE_ENABLE
class AP_QuadRuped_Crab;
#endif

class AP_QuadRuped {
    friend class AP_QuadRuped_Backend;
    friend class AP_QuadRuped_Diag;

public:
    // 默认构造函数
    AP_QuadRuped();

    // 初始化函数 - 设置姿态传感器和电机控制接口
    bool init(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder);

    // Return the number of temperature sensors instances
    // uint8_t num_instances(void) const { return _num_instances; }

    // 析构函数
    ~AP_QuadRuped();

    // 参数表定义 - 用于配置系统参数
    static const struct AP_Param::GroupInfo  var_info[];
    static const struct AP_Param::GroupInfo* backend_var_info[AP_QUADRUPED_GAIT_COUNT];

    void update(); // 主更新循环

    QuadRupedClass get_class() const { return (QuadRupedClass)_quadruped_class.get(); }

    // 步态控制
    void     set_gait_type(GaitType type);
    GaitType get_gait_type() const { return (GaitType)_gait_type.get(); }

    // 控制输入接口
    void set_throttle(float throttle_x, float throttle_y);
    void set_yaw_rate(float yaw_rate);

    // 状态查询接口
    float get_throttle_x() const { return _throttle_xyz.x; }
    float get_throttle_y() const { return _throttle_xyz.y; }
    float get_yaw_rate() const { return _throttle_xyz.z; }

    // 参数访问接口
    const AP_QuadRuped_Params&         get_leg_params(uint8_t leg_index) const;
    const AP_QuadRuped_SYS_Params&     get_sys_params() const { return _sys_params; }
    const AP_QuadRuped_CHANNEL_Params& get_channel_params() const { return _channel_params; }

    // 硬件接口访问
    AP_AHRS_View& get_ahrs() { return *_ahrs; }
    AP_Motors&    get_motors() { return *_motors; }
    RangeFinder&  get_rangefinder() { return *_rangefinder; }

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

    AP_Int8 _gait_type;      // 当前步态类型
    int8_t  _gait_last_type; // 上次步态类型

    AP_Int8 _quadruped_class;

    // 控制输入
    Vector3f _throttle_xyz; // Xyz轴油门输入

    // 参数组
    AP_QuadRuped_SYS_Params     _sys_params;                       // 系统参数
    AP_QuadRuped_Params         _leg_params[AP_QUADRUPED_LEG_ALL]; // 腿部参数
    AP_QuadRuped_CHANNEL_Params _channel_params;                   // 通道参数

    // 内部辅助函数
    void create_backends();
    void destroy_backends();
    void read_radio_input();
};
