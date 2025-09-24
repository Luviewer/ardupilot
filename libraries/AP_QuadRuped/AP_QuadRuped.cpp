#include "AP_QuadRuped.h"

const AP_Param::GroupInfo AP_QuadRuped::var_info[] = {

    // 基本运动参数
    AP_GROUPINFO("Hz", 1, AP_QuadRuped_Base, gait_hz, SPEED_HZ_DEFAULT), // 步态频率

    // 遥控通道参数组
    AP_SUBGROUPINFO(channel, "CH_", 2, AP_QuadRuped_Base, AP_QuadRuped_CHANNEL_Params), // 通道配置

    // 系统参数组
    AP_SUBGROUPINFO(Sys_Param, "SYS", 3, AP_QuadRuped_Base, AP_QuadRuped_SYS_Params), // 系统参数

    // 四条腿的参数组
    AP_SUBGROUPINFO(leg_param[Leg_RF], "RF_", 4, AP_QuadRuped_Base, AP_QuadRuped_Params), // 右前腿参数
    AP_SUBGROUPINFO(leg_param[Leg_RB], "RB_", 5, AP_QuadRuped_Base, AP_QuadRuped_Params), // 右后腿参数
    AP_SUBGROUPINFO(leg_param[Leg_LB], "LB_", 6, AP_QuadRuped_Base, AP_QuadRuped_Params), // 左后腿参数
    AP_SUBGROUPINFO(leg_param[Leg_LF], "LF_", 7, AP_QuadRuped_Base, AP_QuadRuped_Params), // 左前腿参数

    // 行程参数
    AP_GROUPINFO("THR_X", 3, AP_QuadRuped_Base, throttle_x_max, MAX_THROTTLE_X_DEFAULT),  // 最大x方向油门行程
    AP_GROUPINFO("THR_Y", 4, AP_QuadRuped_Base, throttle_y_max, MAX_THROTTLE_Y_DEFAULT),  // 最大y方向油门行程
    AP_GROUPINFO("STEP", 5, AP_QuadRuped_Base, gait_step_total, GAIT_STEP_TOTAL_DEFAULT), // 步态总步数

};

// 构造函数 - 初始化四足机器人基类
AP_QuadRuped::AP_QuadRuped(AP_AHRS_View& ahrs, AP_Motors& motors, RangeFinder& rangefinder)
    : _ahrs(ahrs)               // 初始化姿态航向参考系统接口
    , _motors(motors)           // 初始化电机控制接口
    , _rangefinder(rangefinder) //
{
    // 初始化成员变量
    // move_requested = false;          // 移动请求标志，初始为静止
    // max_yaw_rate   = radians(30.0f); // 最大偏航角速度限制为30度/秒

    // 设置参数默认值
    AP_Param::setup_object_defaults(this, var_info);
}