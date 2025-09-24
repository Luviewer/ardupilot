#pragma once

#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include "AP_DroneCAN/AP_DroneCAN.h"

// 腿部电机参数定义
#define LEG_MOTOR_MAX_DEG       (120)  // 腿部电机最大角度（度）
#define LEG_MOTOR_MAX_PWM       (500)  // 腿部电机最大PWM值
#define LEG_MOTOR_PWM_MIDDLE    (1500) // 腿部电机中间PWM值（1500μs）

// 默认参数定义
#define LIFT_HEIGHT_DEFAULT     50.0f  // 默认抬腿高度（mm）
#define SPEED_HZ_DEFAULT        25.0f  // 默认步态频率（Hz）
#define MAX_THROTTLE_X_DEFAULT  200.0f // 默认最大油门行程（mm）
#define MAX_THROTTLE_Y_DEFAULT  200.0f // 默认最大油门行程（mm）
#define GAIT_STEP_TOTAL_DEFAULT 24     // 默认步态总步数

#define START_COXA_ANGLE        45 // 起始髋关节角度（度）

// 前向声明
class AP_QuadRuped;

// 后端接口类
class AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Backend(AP_QuadRuped& frontend, AP_AHRS_View& ahrs, AP_Motors& motors)
        : _frontend(frontend)
        , _ahrs(ahrs)
        , _motors(motors)
    {
    }

    // 虚析构函数
    virtual ~AP_QuadRuped_Backend() { }

    // 纯虚函数 - 后端必须实现
    virtual bool init()                                   = 0;
    virtual void update()                                 = 0;
    virtual void gait_init()                              = 0;
    virtual void calc_gait_sequence()                     = 0;
    virtual void trajectory_generation(uint8_t leg_index) = 0;
    virtual bool healthy() const                          = 0;

    // 可选重写的虚函数
    virtual void yaw_trajectory_generation(uint8_t leg_index) { }
    virtual void set_centre_offset(float x, float y, float z) { }
    virtual void update_leg() { }

    // 通用工具函数
    virtual void     reset_leg();
    virtual void     calc_gait_sequence();
    virtual Vector3f leg_inverse_kinematics(Vector3f posxyz);
    virtual Vector3f body_forward_kinematics(uint8_t leg_index);
    virtual void     main_inverse_kinematics(void);
    virtual void     controller();

    virtual void output_leg_angle();
    bool         dronecan_send_servo_cmd();

    // 辅助函数
    void right_sleep_leg();
    void x_sleep_leg();
    void x_up_sleep_leg();
    void hengxiang_up_sleep_leg();
    void zongxiang_up_sleep_leg();

protected:
    // 前端控制器引用
    AP_QuadRuped& _frontend;

    // 硬件接口引用
    AP_AHRS_View& _ahrs;
    AP_Motors&    _motors;

    bool _initialized; // 初始化标志

    // 移动请求标志 - true表示需要移动，false表示保持静止
    bool move_requested;

    // 腿部位置和角度
    Vector3f endpoint_leg_pos[AP_QuadRuped::LEG_ALL];        // 腿部末端初始位置（相对于髋关节）
    Vector3f endpoint_leg_frame[AP_QuadRuped::LEG_ALL];      // 腿部在机体框架上的安装位置
    Vector3f endpoint_leg_angle[AP_QuadRuped::LEG_ALL];      // 腿部关节角度（度）
    Vector3f endpoint_leg_angle_last[AP_QuadRuped::LEG_ALL]; // 腿部上一时刻的关节角度

    // 步态目标位置
    Vector3f gait_pos_xyz[AP_QuadRuped::LEG_ALL]; // 步态生成的目标位置（相对于初始位置的偏移）

    // 通用运动参数
    float _throttle_x;  // X轴油门
    float _throttle_y;  // Y轴油门
    float _yaw_rate;    // 偏航角速度
    float _body_height; // 机身高度

    // 伺服输出命令
    Vector3ui servo_output_cmd[AP_QuadRuped::LEG_ALL]; // 存储每条腿三个关节的PWM值

    // 辅助函数
    void update_control_inputs();
    void normalize_leg_angles();
};