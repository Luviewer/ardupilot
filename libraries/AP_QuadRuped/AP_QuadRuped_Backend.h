#pragma once

#include "AP_DroneCAN/AP_DroneCAN.h"
#include "AP_QuadRuped.h"
#include "AP_QuadRuped_Defines.h"
#include "AP_QuadRuped_Params.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// 腿部电机参数定义
#define LEG_MOTOR_MAX_DEG      (120)  // 腿部电机最大角度（度）
#define LEG_MOTOR_MAX_PWM      (500)  // 腿部电机最大PWM值
#define LEG_MOTOR_PWM_MIDDLE   (1500) // 腿部电机中间PWM值（1500μs）

// 默认参数定义
#define LIFT_HEIGHT_DEFAULT    (50.0f)  // 默认抬腿高度（mm）
#define MAX_THROTTLE_X_DEFAULT (200.0f) // 默认最大油门行程（mm）
#define MAX_THROTTLE_Y_DEFAULT (200.0f) // 默认最大油门行程（mm）

#define START_COXA_ANGLE       (45) // 起始髋关节角度（度）

// 前向声明
class AP_QuadRuped;

// 后端接口类
class AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_Backend(AP_QuadRuped&                  frontend,
                         AP_QuadRuped::QuadRuped_State& state,
                         AP_AHRS_View&                  ahrs,
                         AP_Motors&                     motors);

    // 虚析构函数
    virtual ~AP_QuadRuped_Backend() { }

    // 纯虚函数 - 后端必须实现
    virtual void     update()                                 = 0;
    virtual void     gait_init()                              = 0;
    virtual void     refresh_steps()                          = 0;
    virtual void     trajectory_generation(uint8_t leg_index) = 0;
    virtual uint32_t get_Freq()                               = 0;

    // 可选重写的虚函数
    virtual bool init();
    virtual void yaw_trajectory_generation(uint8_t leg_index) { }
    virtual void update_leg() { }

    // 通用工具函数
    virtual void     reset_leg();
    virtual void     calc_gait_sequence();
    virtual Vector3f leg_inverse_kinematics(Vector3f posxyz);
    virtual Vector3f body_forward_kinematics(uint8_t leg_index);
    virtual void     main_inverse_kinematics(void);
    virtual void     main_radio_controller();

    // 输出函数
    virtual void output_leg_angle();
    virtual bool send_servo_cmd();

    void set_center_offset(Vector2f xy, float z = 0) { center_offset = Vector3f(xy, z); }
    void set_center_offset(float x, float y, float z = 0) { center_offset = Vector3f(x, y, z); }

    // 辅助函数
    void right_sleep_leg();
    void x_sleep_leg();
    void x_up_sleep_leg();
    void hengxiang_claw_leg(float angle_value);
    void zhongxiang_claw_leg(float angle_value);

    // 角度转换函数
    uint16_t radians_to_pwm(float angle_rad);

protected:
    // 前端控制器引用
    AP_QuadRuped&                  _frontend;
    AP_QuadRuped::QuadRuped_State& _state;

    // 硬件接口引用
    AP_AHRS_View& _ahrs;
    AP_Motors&    _motors;

    // 移动请求标志 - true表示需要移动，false表示保持静止
    bool move_requested;

    // 步态参数
    uint8_t gait_step_leg_start[AP_QUADRUPED_LEG_ALL]; // 每条腿的步态起始步数
    uint8_t gait_lift_divisor;                         // 抬腿除数 - 控制抬腿速度
    uint8_t gait_travel_divisor;                       // 行程除数 - 控制前进速度

    // 伺服输出命令
    Vector3ui servo_output_cmd[AP_QUADRUPED_LEG_ALL]; // 存储每条腿三个关节的PWM值

    // 运动参数
    int32_t gait_step_now; // 当前步态计数 - 当前步态周期中的步数

    // 腿部位置和角度
    Vector3f endpoint_leg_pos[AP_QUADRUPED_LEG_ALL];        // 腿部末端初始位置（相对于髋关节）
    Vector3f endpoint_leg_frame[AP_QUADRUPED_LEG_ALL];      // 腿部在机体框架上的安装位置
    Vector3f endpoint_leg_angle[AP_QUADRUPED_LEG_ALL];      // 腿部关节角度（度）
    Vector3f endpoint_leg_angle_last[AP_QUADRUPED_LEG_ALL]; // 腿部上一时刻的关节角度

    // 步态目标位置
    Vector3f gait_pos_xyz[AP_QUADRUPED_LEG_ALL]; // 步态生成的目标位置（相对于初始位置的偏移）

    // 机体姿态
    Vector3f body_rot_xyz_deg; // 机体旋转角度（横滚、俯仰、偏航）

    // 腿部旋转补偿
    float gait_rot_z[AP_QUADRUPED_LEG_ALL]; // 每条腿的Z轴旋转补偿（用于转向）

    // 限制和状态
    float max_yaw_rate;     // 最大允许偏航角速度（rad/s）
    bool  first_run = true; // 首次运行标志

    // 运动控制变量
    float target_yaw;        // 目标偏航角
    float throttle_x_travel; // 油门行程 - 前进/后退距离
    float throttle_y_travel; // 油门行程 - 左/右距离
    float z_travel;          // Z轴行程 - 机体升降高度
    float yaw_travel;        // 偏航行程 - 旋转补偿量
    float roll_travel;       // 横滚行程 - 横滚平衡补偿
    float pitch_travel;      // 俯仰行程 - 俯仰平衡补偿
    float leg_lift_height;   // 抬腿高度（mm）- 腿抬起的高度

    // 重心控制
    Vector3f center_offset; // 主动控制的重心偏移量（X、Y、Z）

    // 缓存步态总长
    int16_t gait_step_total_cached;

    // 每帧把 center_offset 向 target 平滑贴近
    // void com_follow_target();

    //    AC_PID roll_pid {
    //     AC_PID::Defaults {
    //         .p         = 1.0f,
    //         .i         = 0.02f,
    //         .d         = 0.05f,
    //         .imax      = 1,
    //         .filt_T_hz = 10.0f,
    //         .filt_E_hz = 10.0f,
    //         .filt_D_hz = 10.0f,
    //         .srmax     = 0,
    //         .srtau     = 1.0 }
    // };

    // AC_PID pitch_pid {
    //     AC_PID::Defaults {
    //         .p         = 1.0f,
    //         .i         = 0.02f,
    //         .d         = 0.1f,
    //         .imax      = 1,
    //         .filt_T_hz = 10.0f,
    //         .filt_E_hz = 10.0f,
    //         .filt_D_hz = 10.0f,
    //         .srmax     = 0,
    //         .srtau     = 1.0 }
    // };

    // AC_PID yaw_pid {
    //     AC_PID::Defaults {
    //         .p         = 0.5f,
    //         .i         = 0.01f,
    //         .d         = 0.05f,
    //         .imax      = 1,
    //         .filt_T_hz = 10.0f,
    //         .filt_E_hz = 10.0f,
    //         .filt_D_hz = 10.0f,
    //         .srmax     = 0,
    //         .srtau     = 1.0 }
    // };
};