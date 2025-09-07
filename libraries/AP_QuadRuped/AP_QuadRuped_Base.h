#pragma once

// 四足机器人库头文件
#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>                  // PID控制器
#include <AP_AHRS/AP_AHRS_View.h>           // 姿态航向参考系统
#include <AP_HAL/AP_HAL_Boards.h>           // 硬件抽象层
#include <AP_Math/AP_Math.h>                // 数学库
#include <AP_Motors/AP_MotorsMulticopter.h> // 电机控制
#include <AP_Param/AP_Param.h>              // 参数系统
#include <stdio.h>

// 腿部索引枚举
enum {
    Leg_RF = 0, // 右前腿 (Right Front)
    Leg_RB,     // 右后腿 (Right Back)
    Leg_LB,     // 左后腿 (Left Back)
    Leg_LF,     // 左前腿 (Left Front)

    LEG_ALL, // 腿的总数（4条腿）
};

// 四足机器人基类 - 定义了四足机器人的基本功能和接口
class AP_QuadRuped_Base {
public:
    // 构造函数 - 初始化姿态传感器和电机控制接口
    AP_QuadRuped_Base(AP_AHRS_View& ahrs, AP_Motors& motors);

    // 空析构函数 - 防止编译器警告
    virtual ~AP_QuadRuped_Base() { }

    // 步态类型枚举
    enum GaitType {
        GAIT_DIAGONAL = 0, // 对角步态（trot步态）
        GAIT_WAVE     = 1  // 波浪步态（crawl步态）
    };

    // 姿态航向参考系统引用 - 获取机器人姿态信息
    AP_AHRS_View& _ahrs;

    // 电机控制接口引用 - 控制舵机输出
    AP_Motors& _motors;

    // 参数表定义 - 用于配置系统参数
    static const struct AP_Param::GroupInfo var_info[];

    // 纯虚函数 - 子类必须实现
    virtual void gait_init() = 0; // 步态初始化

    // 步态计算相关函数
    virtual void calc_gait_sequence();                             // 计算步态序列
    virtual void trajectory_generation(uint8_t leg_index) { };     // 轨迹生成（由子类实现）
    virtual void yaw_trajectory_generation(uint8_t leg_index) { }; // 偏航轨迹生成（由子类实现）

    // 运动学计算函数
    virtual void     main_inverse_kinematics();                  // 主逆运动学计算
    virtual Vector3f body_forward_kinematics(uint8_t leg_index); // 机体正向运动学
    virtual Vector3f leg_inverse_kinematics(Vector3f posxyz);    // 腿部逆运动学

    // 重心控制函数
    virtual void set_centre_offset(float x, float y, float z); // 设置重心偏移

    // 腿部姿态控制函数
    virtual void reset_leg();              // 重置腿部位置
    virtual void right_sleep_leg();        // 右侧睡眠姿态
    virtual void update_leg() { };         // 更新腿部运动（由子类实现）
    virtual void x_sleep_leg();            // X形睡眠姿态
    virtual void x_up_sleep_leg();         // X形抬升睡眠姿态
    virtual void hengxiang_up_sleep_leg(); // 横向抬升睡眠姿态

    // 控制器函数
    virtual void controller();         // 主控制器 - 处理遥控器输入
    virtual void balance_controller(); // 平衡控制器 - 姿态控制

    // 更新函数（由子类实现具体行为）
    virtual void update() { };

    // 初始化和输出函数
    void init();             // 系统初始化
    void output_leg_angle(); // 输出腿部关节角度
    bool hw_set_servo_cmd(); // 硬件伺服命令设置

    // 辅助函数
    float getFreq() { return gait_hz; } // 获取步态频率

    // 解锁控制函数（已注释）
    // void arm() { armed = true; }      // 解锁电机
    // void disarm() { armed = false; }   // 上锁
    // bool armed() { return armed; }     // 获取解锁状态

protected:
    // 运动控制标志
    bool move_requested; // 移动请求标志 - true表示需要移动，false表示保持静止

    // 解锁状态（已注释）
    // bool armed; // 解锁状态

    // 步态参数
    uint8_t gait_step_leg_start[LEG_ALL]; // 每条腿的步态起始步数
    uint8_t gait_lift_divisor;            // 抬腿除数 - 控制抬腿速度
    uint8_t gait_travel_divisor;          // 行程除数 - 控制前进速度

    // 伺服输出命令
    Vector3ui servo_output_cmd[LEG_ALL]; // 存储每条腿三个关节的PWM值

    // 运动参数
    AP_Float leg_lift_height; // 抬腿高度（mm）- 腿抬起的高度
    AP_Float gait_hz;         // 步态频率（Hz）- 步态更新频率
    AP_Int16 gait_step_total; // 步态总步数 - 一个完整步态周期的步数
    uint16_t gait_step_now;   // 当前步态计数 - 当前步态周期中的步数

    // 腿部位置和角度
    Vector3f endpoint_leg_pos[LEG_ALL];        // 腿部末端初始位置（相对于髋关节）
    Vector3f endpoint_leg_frame[LEG_ALL];      // 腿部在机体框架上的安装位置
    Vector3f endpoint_leg_angle[LEG_ALL];      // 腿部关节角度（度）
    Vector3f endpoint_leg_angle_last[LEG_ALL]; // 腿部上一时刻的关节角度

    // 步态目标位置
    Vector3f gait_pos_xyz[LEG_ALL]; // 步态生成的目标位置（相对于初始位置的偏移）

    // 机体姿态
    Vector3f body_rot_xyz_deg; // 机体旋转角度（横滚、俯仰、偏航）

    // 腿部旋转补偿
    float gait_rot_z[LEG_ALL]; // 每条腿的Z轴旋转补偿（用于转向）

    // 运动控制变量
    float target_yaw;      // 目标偏航角
    float throttle_travel; // 油门行程 - 前进/后退距离
    float z_travel;        // Z轴行程 - 机体升降高度
    float yaw_travel;      // 偏航行程 - 旋转补偿量
    float roll_travel;     // 横滚行程 - 横滚平衡补偿
    float pitch_travel;    // 俯仰行程 - 俯仰平衡补偿

    // 限制和状态
    float max_yaw_rate;     // 最大允许偏航角速度（rad/s）
    bool  first_run = true; // 首次运行标志

    // 重心控制
    Vector3f centre_offset; // 主动控制的重心偏移量（X、Y、Z）
    Vector2f offset_xy;     // 重心平移控制（X、Y平面）

    // 油门参数
    AP_Float throttle_max; // 油门最大值 - 最大前进/后退距离

    // 系统和腿的参数
    AP_QuadRuped_SYS_Params     Sys_Param;          // 系统参数（机身尺寸、腿长等）
    AP_QuadRuped_Params         leg_param[LEG_ALL]; // 每条腿的参数（方向、偏移等）
    AP_QuadRuped_CHANNEL_Params channel;            // 遥控通道参数

    AC_PID roll_pid {
        AC_PID::Defaults {
            .p         = 1.0f,
            .i         = 0.02f,
            .d         = 0.05f,
            .imax      = 1,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID pitch_pid {
        AC_PID::Defaults {
            .p         = 1.0f,
            .i         = 0.02f,
            .d         = 0.1f,
            .imax      = 1,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };

    AC_PID yaw_pid {
        AC_PID::Defaults {
            .p         = 0.5f,
            .i         = 0.01f,
            .d         = 0.05f,
            .imax      = 1,
            .filt_T_hz = 10.0f,
            .filt_E_hz = 10.0f,
            .filt_D_hz = 10.0f,
            .srmax     = 0,
            .srtau     = 1.0 }
    };
};
