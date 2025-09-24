#pragma once

#include <AP_Baro/AP_Baro.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <AP_OLC/AP_OLC.h>
#include <AP_Param/AP_Param.h>
#include <AP_RPM/AP_RPM_config.h>
#include <GCS_MAVLink/GCS_config.h>
#include <RC_Channel/RC_Channel.h>
#if HAL_GCS_ENABLED
# include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AC_Fence/AC_Fence_config.h>
#include <AP_AHRS/AP_AHRS_View.h>           // 姿态航向参考系统
#include <AP_Motors/AP_MotorsMulticopter.h> // 电机控制
#include <AP_RangeFinder/AP_RangeFinder.h>

class AP_QuadRuped;

class AP_QuadRuped {
public:
    // 构造函数 - 初始化姿态传感器和电机控制接口
    AP_QuadRuped(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder);

    // 空析构函数 - 防止编译器警告
    virtual ~AP_QuadRuped() { }

    // 参数表定义 - 用于配置系统参数
    static const struct AP_Param::GroupInfo var_info[];

    // 步态类型枚举
    enum GaitType {
        GAIT_DIAGONAL = 0, // 对角步态（trot步态）
        GAIT_WAVE     = 1, // 波浪步态（crawl步态）
        GAIT_CRAB     = 2
    };

    // 姿态航向参考系统引用 - 获取机器人姿态信息
    AP_AHRS_View& _ahrs;

    // 电机控制接口引用 - 控制舵机输出
    AP_Motors& _motors;

    // 测距雷达接口
    RangeFinder& _rangefinder;

    // 总更新函数
    void update_all();
};