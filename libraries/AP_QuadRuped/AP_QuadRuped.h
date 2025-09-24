#pragma once

#include <AP_AHRS/AP_AHRS_View.h> // 姿态航向参考系统
#include <AP_HAL/AP_HAL_Boards.h> // 硬件抽象层
#include <AP_Math/AP_Math.h>      // 数学库
#include <AP_Motors/AP_Motors.h>  // 电机控制
#include <AP_Param/AP_Param.h>    // 参数系统
#include <AP_QuadRuped_Backend.h> // 后端接口
#include <AP_QuadRuped_Config.h>
#include <AP_QuadRuped_Params.h>           // 参数定义
#include <AP_RangeFinder/AP_RangeFinder.h> // 测距传感器
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
public:
    // 构造函数 - 初始化姿态传感器和电机控制接口
    AP_QuadRuped(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder);

    // 析构函数
    ~AP_QuadRuped();

    // 参数表定义 - 用于配置系统参数
    static const struct AP_Param::GroupInfo var_info[];

    // 步态类型枚举
    enum GaitType {
        GAIT_DIAGONAL = 0, // 对角步态（trot步态）
        GAIT_WAVE     = 1, // 波浪步态（crawl步态）
        GAIT_CRAB     = 2, // 工字步态
        GAIT_COUNT,        // 步态总数
    };

    // 腿部索引枚举
    enum {
        Leg_RF = 0, // 右前腿 (Right Front)
        Leg_RB,     // 右后腿 (Right Back)
        Leg_LB,     // 左后腿 (Left Back)
        Leg_LF,     // 左前腿 (Left Front)
        LEG_ALL,    // 腿的总数（4条腿）
    };

    // 主要功能函数
    bool init();   // 初始化系统
    void update(); // 主更新循环

    // 步态控制
    void     set_gait_type(GaitType type);
    GaitType get_gait_type() const { return (GaitType)_gait_type.get(); }

    // 控制输入接口
    void set_throttle(float throttle_x, float throttle_y);
    void set_yaw_rate(float yaw_rate);
    void set_body_height(float height);

    // 状态查询接口
    float get_throttle_x() const { return _throttle_x; }
    float get_throttle_y() const { return _throttle_y; }
    float get_yaw_rate() const { return _yaw_rate; }
    float get_body_height() const { return _body_height; }

    // 参数访问接口
    const AP_QuadRuped_Params&         get_leg_params(uint8_t leg_index) const;
    const AP_QuadRuped_SYS_Params&     get_sys_params() const { return _sys_params; }
    const AP_QuadRuped_CHANNEL_Params& get_channel_params() const { return _channel_params; }

    // 硬件接口访问
    AP_AHRS_View& get_ahrs() { return _ahrs; }
    AP_Motors&    get_motors() { return _motors; }
    RangeFinder&  get_rangefinder() { return _rangefinder; }

private:
    // 硬件接口
    AP_AHRS_View& _ahrs;        // 姿态航向参考系统
    AP_Motors&    _motors;      // 电机控制接口
    RangeFinder&  _rangefinder; // 测距雷达接口

    // 后端管理
    AP_QuadRuped_Backend* _backend;                   // 当前活跃的后端
    AP_QuadRuped_Backend* _gait_backends[GAIT_COUNT]; // 所有的步态后端

    // 主要参数
    AP_Int8 _gait_type; // 当前步态类型
    AP_Int8 _enabled;   // 使能状态

    // 控制输入
    float _throttle_x;  // X轴油门输入
    float _throttle_y;  // Y轴油门输入
    float _yaw_rate;    // 偏航角速度
    float _body_height; // 机身高度

    // 参数组
    AP_QuadRuped_SYS_Params     _sys_params;          // 系统参数
    AP_QuadRuped_Params         _leg_params[LEG_ALL]; // 腿部参数
    AP_QuadRuped_CHANNEL_Params _channel_params;      // 通道参数

    // 内部辅助函数
    void create_backends();
    void read_radio_input();
};