#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// default gains for Plane
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT 0.2f // Soft
# define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN  5.0  // Min lean angle so that vehicle can maintain limited control
#else
// default gains for Copter and Sub
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT 0.15f // Medium
# define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN  10.0  // Min lean angle so that vehicle can maintain limited control
#endif

AC_AttitudeControl* AC_AttitudeControl::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in RTL and Auto flight modes
    // @Units: cdeg/s
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW", 2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: cdeg/s/s
    // @Range: 0 72000
    // @Values: 0:Disabled, 9000:VerySlow, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX", 4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedforward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: cdeg/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: Angle Limit (to maintain altitude) Time Constant
    // @Description: Angle Limit (to maintain altitude) Time Constant
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("ANG_LIM_TC", 16, AC_AttitudeControl, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    // @Param: RATE_R_MAX
    // @DisplayName: Angular Velocity Max for Roll
    // @Description: Maximum angular velocity in roll axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_R_MAX", 17, AC_AttitudeControl, _ang_vel_roll_max, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_P_MAX", 18, AC_AttitudeControl, _ang_vel_pitch_max, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angular Velocity Max for Yaw
    // @Description: Maximum angular velocity in yaw axis
    // @Units: deg/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX", 19, AC_AttitudeControl, _ang_vel_yaw_max, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_GROUPINFO("INPUT_TC", 20, AC_AttitudeControl, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    // @Param: LAND_R_MULT
    // @DisplayName: Landed roll gain multiplier
    // @Description: Roll gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the roll axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_R_MULT", 21, AC_AttitudeControl, _land_roll_mult, 1.0),

    // @Param: LAND_P_MULT
    // @DisplayName: Landed pitch gain multiplier
    // @Description: Pitch gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the pitch axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_P_MULT", 22, AC_AttitudeControl, _land_pitch_mult, 1.0),

    // @Param: LAND_Y_MULT
    // @DisplayName: Landed yaw gain multiplier
    // @Description: Yaw gain multiplier active when landed. A factor of 1.0 means no reduction in gain while landed. Reduce this factor to reduce ground oscitation in the yaw axis.
    // @Range: 0.25 1.0
    // @User: Advanced
    AP_GROUPINFO("LAND_Y_MULT", 23, AC_AttitudeControl, _land_yaw_mult, 1.0),

    AP_GROUPEND
};

constexpr Vector3f AC_AttitudeControl::VECTORF_111;

// get the slew yaw rate limit in deg/s
float AC_AttitudeControl::get_slew_yaw_max_degs() const
{
    if (!is_positive(_ang_vel_yaw_max)) {
        return _slew_yaw * 0.01;
    }
    return MIN(_ang_vel_yaw_max, _slew_yaw * 0.01);
}

// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl::relax_attitude_controllers()
{
    // Initialize the attitude variables to the current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target);
    _attitude_ang_error.initialise();

    // Initialize the angular rate variables to the current rate
    _ang_vel_target = _ahrs.get_gyro();
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);
    _ang_vel_body = _ahrs.get_gyro();

    // Initialize remaining variables
    _thrust_error_angle = 0.0f;

    // Reset the PID filters
    get_rate_roll_pid().reset_filter();
    get_rate_pitch_pid().reset_filter();
    get_rate_yaw_pid().reset_filter();

    // Reset the I terms
    reset_rate_controller_I_terms();
}

void AC_AttitudeControl::reset_rate_controller_I_terms()
{
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
}

// reset rate controller I terms smoothly to zero in 0.5 seconds
void AC_AttitudeControl::reset_rate_controller_I_terms_smoothly()
{
    get_rate_roll_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_pitch_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_yaw_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
}

// Reduce attitude control gains while landed to stop ground resonance
void AC_AttitudeControl::landed_gain_reduction(bool landed)
{
    if (is_positive(_input_tc)) {
        // use 2.0 x tc to match the response time to 86% commanded
        const float spool_step = _dt / (2.0 * _input_tc);
        if (landed) {
            _landed_gain_ratio = MIN(1.0, _landed_gain_ratio + spool_step);
        } else {
            _landed_gain_ratio = MAX(0.0, _landed_gain_ratio - spool_step);
        }
    } else {
        _landed_gain_ratio = landed ? 1.0 : 0.0;
    }
    Vector3f scale_mult = VECTORF_111 * (1.0 - _landed_gain_ratio) + Vector3f(_land_roll_mult, _land_pitch_mult, _land_yaw_mult) * _landed_gain_ratio;
    set_PD_scale_mult(scale_mult);
    set_angle_P_scale_mult(scale_mult);
}

// The attitude controller works around the concept of the desired attitude, target attitude
// and measured attitude. The desired attitude is the attitude input into the attitude controller
// that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
// to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
// directly into the rate controllers. The angular error between the measured attitude and the target attitude is
// fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
// By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
// remain very close together.
//
// All input functions below follow the same procedure
// 1. define the desired attitude or attitude change based on the input variables
// 2. update the target attitude based on the angular velocity target and the time since the last loop
// 3. using the desired attitude and input variables, define the target angular velocity so that it should
//    move the target attitude towards the desired attitude
// 4. if _rate_bf_ff_enabled is not being used then make the target attitude
//    and target angular velocities equal to the desired attitude and desired angular velocities.
// 5. ensure _attitude_target, _euler_angle_target, _euler_rate_target and
//    _ang_vel_target have been defined. This ensures input modes can be changed without discontinuity.
// 6. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
//    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
//    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
//    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.
// 姿态控制器围绕期望姿态、目标姿态和测量姿态的概念工作。
// 期望姿态是指输入到姿态控制器的姿态，表达了高层代码希望飞机移动到的位置。
// 目标姿态通过抖动、加速度和速度限制逐渐移动到期望姿态。
// 目标角速度直接输入到速率控制器中。测量姿态与目标姿态之间的角度误差
// 被输入到角度控制器中，角度控制器的输出在速率控制器的输入处相加。
// 通过将目标角速度直接输入到速率控制器中，测量姿态和目标姿态始终保持非常接近。
//
// 下面所有输入函数都遵循相同的程序
// 1. 根据输入变量定义期望姿态或姿态变化
// 2. 根据角速度目标和自上次循环以来的时间更新目标姿态
// 3. 使用期望姿态和输入变量，定义目标角速度，使其能够将目标姿态移动到期望姿态
// 4. 如果未使用 _rate_bf_ff_enabled，则将目标姿态和目标角速度设置为期望姿态和期望角速度
// 5. 确保 _attitude_target、_euler_angle_target、_euler_rate_target 和
//    _ang_vel_target 已经定义。这确保可以在不出现不连续的情况下更改输入模式。
// 6. 然后运行 attitude_controller_run_quat 将目标角速度传递给速率控制器并将其整合到目标姿态中。
//    任何目标姿态与测量姿态之间的误差首先通过校正推力矢量来纠正，直到目标推力矢量和测量推力矢量之间的角度
//    低于 2*AC_ATTITUDE_THRUST_ERROR_ANGLE。此时，也会校正航向。

// 命令一个四元数姿态，并使用前馈和平滑技术
// attitude_desired_quat: 每个时间步长通过体坐标系角速度的积分进行更新
void AC_AttitudeControl::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body)
{
    // 更新姿态目标
    // 确保目标姿态是最新的，反映了当前的飞行状态
    update_attitude_target();

    // 限制角速度
    // 确保角速度不超过预设的最大值，防止过快的转动导致系统不稳定
    ang_vel_limit(ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    // 计算目标角速度
    // 将期望的姿态四元数和体坐标系角速度相乘，得到目标角速度
    Vector3f ang_vel_target = attitude_desired_quat * ang_vel_body;

    if (_rate_bf_ff_enabled) {
        // 计算姿态误差四元数
        // 通过目标姿态的逆旋转和期望姿态的乘积，得到姿态误差四元数
        Quaternion attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat;

        // 将姿态误差四元数转换为轴角表示
        Vector3f attitude_error_angle;
        attitude_error_quat.to_axis_angle(attitude_error_angle);

        // 当加速度限制和前馈启用时，使用 sqrt 控制器计算欧拉角速度
        // 使欧拉角平滑地停止在输入角度处，并在末尾以指数衰减方式指定 _input_tc 进行减速
        _ang_vel_target.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, ang_vel_target.x, radians(_ang_vel_roll_max), _dt);
        _ang_vel_target.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, ang_vel_target.y, radians(_ang_vel_pitch_max), _dt);
        _ang_vel_target.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, ang_vel_target.z, radians(_ang_vel_yaw_max), _dt);
    } else {
        // 如果未使用前馈，则将目标姿态和目标角速度设置为期望姿态和期望角速度
        // 直接将期望姿态和角速度赋值给目标姿态和角速度
        _attitude_target = attitude_desired_quat;
        _ang_vel_target  = ang_vel_target;
    }

    // 计算目标姿态的欧拉角
    // 将目标姿态四元数转换为欧拉角表示
    _attitude_target.to_euler(_euler_angle_target);

    // 将体坐标系角速度转换为期望姿态的欧拉角导数
    // 计算目标角速度对目标姿态的影响，得到欧拉角的变化率
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);

    // 旋转目标并归一化
    // 计算一个小的旋转增量，用于更新期望姿态
    Quaternion attitude_desired_update;
    attitude_desired_update.from_axis_angle(ang_vel_target * _dt);

    // 更新期望姿态
    attitude_desired_quat = attitude_desired_quat * attitude_desired_update;

    // 归一化更新后的期望姿态四元数
    attitude_desired_quat.normalize();

    // 调用四元数姿态控制器
    // 将目标角速度传递给速率控制器，并整合到目标姿态中
    attitude_controller_run_quat();
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
// 指令欧拉角滚转和俯仰角度以及欧拉角偏航角速度，并带有角速度前馈和平滑处理
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    // 将输入从百分度转换为弧度
    float euler_roll_angle  = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_rate    = radians(euler_yaw_rate_cds * 0.01f);

    // update attitude target
    // 更新姿态目标
    update_attitude_target();

    // calculate the attitude target euler angles
    // 计算姿态目标的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    // 添加滚转角补偿以抵消尾桨推力（对于多旋翼，该值为零）
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        // 将滚转、俯仰和偏航加速度限制转换到欧拉轴
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        // 当启用加速度限制和前馈时，使用平方根控制器计算一个欧拉角角速度，使得欧拉角能以有限的减速平滑停止在输入角度，并在结束时按平滑增益指数衰减。
        _euler_rate_target.x = input_shaping_angle(wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x, _euler_rate_target.x, _dt);
        _euler_rate_target.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y, _euler_rate_target.y, _dt);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        // 当启用偏航加速度限制时，偏航输入塑形器约束偏航轴上的角加速度，使输出率向输入率平滑过渡。
        _euler_rate_target.z = input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        // 将期望姿态的欧拉角导数转换为体坐标系中的角速度向量作为前馈
        euler_rate_to_ang_vel(_attitude_target, _euler_rate_target, _ang_vel_target);
        // Limit the angular velocity
        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        // 将体坐标系中的角速度转换为期望姿态的欧拉角导数
        ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        // 当未启用前馈时，将目标欧拉角输入到目标中，并将前馈率设为零。
        _euler_angle_target.x = euler_roll_angle;
        _euler_angle_target.y = euler_pitch_angle;
        _euler_angle_target.z += euler_yaw_rate * _dt;
        // Compute quaternion target attitude
        // 计算四元数目标姿态
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

        // Set rate feedforward requests to zero
        // 将速率前馈请求设为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // Call quaternion attitude controller
    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 命令一个欧拉角（滚转、俯仰、偏航）姿态，并使用角速度前馈和平滑技术
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // 将公共接口上的角度从百分度转换为弧度
    float euler_roll_angle  = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_angle   = radians(euler_yaw_angle_cd * 0.01f);

    // 更新姿态目标
    // 确保目标姿态是最新的，反映了当前的飞行状态
    update_attitude_target();

    // 计算目标姿态的欧拉角
    // 将目标姿态四元数转换为欧拉角表示
    _attitude_target.to_euler(_euler_angle_target);

    // 添加滚转补偿以抵消直升机尾桨推力（多旋翼将返回零）
    euler_roll_angle += get_roll_trim_rad();

    // 获取最大偏航速率
    const float slew_yaw_max_rads = get_slew_yaw_max_rads();
    if (_rate_bf_ff_enabled) {
        // 将滚转、俯仰和偏航加速度限制转换为欧拉轴上的加速度限制
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // 当加速度限制和前馈启用时，使用 sqrt 控制器计算欧拉角速度
        // 使欧拉角平滑地停止在输入角度处，并在末尾以指数衰减方式指定 _input_tc 进行减速
        _euler_rate_target.x = input_shaping_angle(wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x, _euler_rate_target.x, _dt);
        _euler_rate_target.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y, _euler_rate_target.y, _dt);
        _euler_rate_target.z = input_shaping_angle(wrap_PI(euler_yaw_angle - _euler_angle_target.z), _input_tc, euler_accel.z, _euler_rate_target.z, _dt);
        if (slew_yaw) {
            // 限制偏航角速度
            _euler_rate_target.z = constrain_float(_euler_rate_target.z, -slew_yaw_max_rads, slew_yaw_max_rads);
        }

        // 将期望姿态的欧拉角导数转换为体坐标系角速度向量，用于前馈
        euler_rate_to_ang_vel(_attitude_target, _euler_rate_target, _ang_vel_target);

        // 限制角速度
        // 确保角速度不超过预设的最大值，防止过快的转动导致系统不稳定
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

        // 将体坐标系角速度转换为期望姿态的欧拉角导数
        ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);
    } else {
        // 当未启用前馈时，将目标欧拉角输入到目标中，并将前馈速率设置为零
        _euler_angle_target.x = euler_roll_angle;
        _euler_angle_target.y = euler_pitch_angle;
        if (slew_yaw) {
            // 计算受限制的角度误差
            float angle_error = constrain_float(wrap_PI(euler_yaw_angle - _euler_angle_target.z), -slew_yaw_max_rads * _dt, slew_yaw_max_rads * _dt);
            // 根据受限制的角度误差更新姿态目标
            _euler_angle_target.z = wrap_PI(angle_error + _euler_angle_target.z);
        } else {
            _euler_angle_target.z = euler_yaw_angle;
        }

        // 计算目标姿态四元数
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

        // 将前馈速率请求设置为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 调用四元数姿态控制器
    // 将目标角速度传递给速率控制器，并整合到目标姿态中
    attitude_controller_run_quat();
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_rate  = radians(euler_roll_rate_cds * 0.01f);
    float euler_pitch_rate = radians(euler_pitch_rate_cds * 0.01f);
    float euler_yaw_rate   = radians(euler_yaw_rate_cds * 0.01f);

    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        const Vector3f euler_accel = euler_accel_limit(_attitude_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // When acceleration limiting is enabled, the input shaper constrains angular acceleration, slewing
        // the output rate towards the input rate.
        _euler_rate_target.x = input_shaping_ang_vel(_euler_rate_target.x, euler_roll_rate, euler_accel.x, _dt, _rate_rp_tc);
        _euler_rate_target.y = input_shaping_ang_vel(_euler_rate_target.y, euler_pitch_rate, euler_accel.y, _dt, _rate_rp_tc);
        _euler_rate_target.z = input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target, _euler_rate_target, _ang_vel_target);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        // Pitch angle is restricted to +- 85.0 degrees to avoid gimbal lock discontinuities.
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + euler_roll_rate * _dt);
        _euler_angle_target.y = constrain_float(_euler_angle_target.y + euler_pitch_rate * _dt, radians(-85.0f), radians(85.0f));
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + euler_yaw_rate * _dt);

        // Set rate feedforward requests to zero
        _euler_rate_target.zero();
        _ang_vel_target.zero();

        // Compute quaternion target attitude
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    if (_rate_bf_ff_enabled) {
        // Compute acceleration-limited body frame rates
        // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
        // the output rate towards the input rate.
        _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
        _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
        _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);
    } else {
        // When feedforward is not enabled, the quaternion is calculated and is input into the target and the feedforward rate is zeroed.
        Quaternion attitude_target_update;
        attitude_target_update.from_axis_angle(Vector3f { roll_rate_rads, pitch_rate_rads, yaw_rate_rads } * _dt);
        _attitude_target = _attitude_target * attitude_target_update;
        _attitude_target.normalize();

        // Set rate feedforward requests to zero
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command an angular velocity with angular velocity smoothing using rate loops only with no attitude loop stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // Update the unused targets attitude based on current attitude to condition mode change
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target);
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);
    _ang_vel_body = _ang_vel_target;
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // Update attitude error
    Vector3f attitude_error;
    _attitude_ang_error.to_axis_angle(attitude_error);

    Quaternion attitude_ang_error_update_quat;
    // limit the integrated error angle
    float err_mag = attitude_error.length();
    if (err_mag > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        attitude_error *= AC_ATTITUDE_THRUST_ERROR_ANGLE / err_mag;
        _attitude_ang_error.from_axis_angle(attitude_error);
    }

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    attitude_ang_error_update_quat.from_axis_angle((_ang_vel_target - gyro_latest) * _dt);
    _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;
    _attitude_ang_error.normalize();

    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
    // the output rate towards the input rate.
    _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // Retrieve quaternion body attitude
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // Update the unused targets attitude based on current attitude to condition mode change
    _attitude_target = attitude_body * _attitude_ang_error;
    _attitude_target.normalize();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);

    // Compute the angular velocity target from the integrated rate error
    _attitude_ang_error.to_axis_angle(attitude_error);
    _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);
    _ang_vel_body += _ang_vel_target;
}

// Command an angular step (i.e change) in body frame angle
// Used to command a step in angle without exciting the orthogonal axis during autotune
void AC_AttitudeControl::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd)
{
    // Convert from centidegrees on public interface to radians
    float roll_step_rads  = radians(roll_angle_step_bf_cd * 0.01f);
    float pitch_step_rads = radians(pitch_angle_step_bf_cd * 0.01f);
    float yaw_step_rads   = radians(yaw_angle_step_bf_cd * 0.01f);

    // rotate attitude target by desired step
    Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(Vector3f { roll_step_rads, pitch_step_rads, yaw_step_rads });
    _attitude_target = _attitude_target * attitude_target_update;
    _attitude_target.normalize();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// 命令一个推力向量和航向率
void AC_AttitudeControl::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // 将公共接口上的角度单位从百分度转换为弧度
    float heading_rate = radians(heading_rate_cds * 0.01f);
    if (slew_yaw) {
        // 如果 _angle_vel_yaw_max 为零，表示该设置被禁用
        const float slew_yaw_max_rads = get_slew_yaw_max_rads();
        heading_rate                  = constrain_float(heading_rate, -slew_yaw_max_rads, slew_yaw_max_rads);
    }

    // 更新姿态目标
    update_attitude_target();

    // 计算姿态目标的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 将推力向量转换为四元数姿态
    Quaternion thrust_vec_quat = attitude_from_thrust_vector(thrust_vector, 0.0f);

    // 计算x和y方向的角度误差
    float      thrust_vector_diff_angle;
    Quaternion thrust_vec_correction_quat;
    Vector3f   attitude_error;
    float      returned_thrust_vector_angle;
    thrust_vector_rotation_angles(thrust_vec_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

    if (_rate_bf_ff_enabled) {
        // 当启用航向加速度限制时，航向输入整形器会限制绕航向轴的角加速度，使输出率逐渐接近输入率
        _ang_vel_target.x = input_shaping_angle(attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
        _ang_vel_target.y = input_shaping_angle(attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);

        // 当启用航向加速度限制时，航向输入整形器会限制绕航向轴的角加速度，使输出率逐渐接近输入率
        _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, heading_rate, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
    } else {
        Quaternion yaw_quat;
        yaw_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, heading_rate * _dt });
        _attitude_target = _attitude_target * thrust_vec_correction_quat * yaw_quat;

        // 将速率前馈请求设置为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 将体坐标系下的角速度转换为所需姿态的欧拉角导数
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// Command a thrust vector, heading and heading rate
void AC_AttitudeControl::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // a zero _angle_vel_yaw_max means that setting is disabled
    const float slew_yaw_max_rads = get_slew_yaw_max_rads();

    // Convert from centidegrees on public interface to radians
    float heading_rate  = constrain_float(radians(heading_rate_cds * 0.01f), -slew_yaw_max_rads, slew_yaw_max_rads);
    float heading_angle = radians(heading_angle_cd * 0.01f);

    // update attitude target
    update_attitude_target();

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    // convert thrust vector and heading to a quaternion attitude
    const Quaternion desired_attitude_quat = attitude_from_thrust_vector(thrust_vector, heading_angle);

    if (_rate_bf_ff_enabled) {
        // calculate the angle error in x and y.
        Vector3f   attitude_error;
        float      thrust_vector_diff_angle;
        Quaternion thrust_vec_correction_quat;
        float      returned_thrust_vector_angle;
        thrust_vector_rotation_angles(desired_attitude_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _ang_vel_target.x = input_shaping_angle(attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
        _ang_vel_target.y = input_shaping_angle(attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);
        _ang_vel_target.z = input_shaping_angle(attitude_error.z, _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, heading_rate, slew_yaw_max_rads, _dt);

        // Limit the angular velocity
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), slew_yaw_max_rads);
    } else {
        // set persisted quaternion target attitude
        _attitude_target = desired_attitude_quat;

        // Set rate feedforward requests to zero
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

// Command a thrust vector and heading rate
void AC_AttitudeControl::input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading)
{
    switch (heading.heading_mode) {
        case HeadingMode::Rate_Only:
            input_thrust_vector_rate_heading(thrust_vector, heading.yaw_rate_cds);
            break;
        case HeadingMode::Angle_Only:
            input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, 0.0);
            break;
        case HeadingMode::Angle_And_Rate:
            input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, heading.yaw_rate_cds);
            break;
    }
}

Quaternion AC_AttitudeControl::attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle) const
{
    const Vector3f thrust_vector_up { 0.0f, 0.0f, -1.0f }; // 定义一个向上（地球坐标系中的“上”）的单位向量

    if (is_zero(thrust_vector.length_squared())) { // 检查推力向量长度是否为零
        thrust_vector = thrust_vector_up;          // 如果长度为零，则将推力向量设置为向上向量
    } else {
        thrust_vector.normalize(); // 否则，将推力向量归一化
    }

    // 推力向量与目标向量（向上向量）的叉乘定义了旋转轴
    Vector3f thrust_vec_cross = thrust_vector_up % thrust_vector;

    // 点积用于计算目标推力向量与期望推力向量之间的角度
    const float thrust_vector_angle = acosf(constrain_float(thrust_vector_up * thrust_vector, -1.0f, 1.0f));

    // 归一化推力旋转向量
    const float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_vector_angle)) { // 检查旋转向量长度或角度是否为零
        thrust_vec_cross = thrust_vector_up;                             // 如果为零，则将旋转向量设置为向上向量
    } else {
        thrust_vec_cross /= thrust_vector_length; // 否则，归一化旋转向量
    }

    Quaternion thrust_vec_quat;                                             // 创建一个四元数来表示推力旋转
    thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle); // 根据旋转轴和角度创建四元数

    Quaternion yaw_quat;                                                    // 创建一个四元数来表示偏航旋转
    yaw_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, 1.0f }, heading_angle); // 根据Z轴和偏航角创建四元数

    // 返回两个四元数的乘积，表示最终的姿态
    return thrust_vec_quat * yaw_quat;
}

// 计算体坐标系中的角速度以跟随目标姿态
void AC_AttitudeControl::update_attitude_target()
{
    // 旋转目标并归一化
    Quaternion attitude_target_update;                             // 创建一个新的四元数对象
    attitude_target_update.from_axis_angle(_ang_vel_target * _dt); // 根据角速度和时间间隔计算小角度旋转的四元数
    _attitude_target *= attitude_target_update;                    // 通过四元数乘法更新当前的目标姿态
    _attitude_target.normalize();                                  // 归一化新的目标姿态四元数，确保其模长为 1
}

// 计算体坐标系下的角速度以跟随目标姿态
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // 这表示从NED坐标系到体坐标系的四元数旋转
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // 这个向量表示使用x和y旋转推力向量以及使用z旋转航向的角度误差
    Vector3f attitude_error;
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // 从姿态误差计算体坐标系下的角速度修正
    _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);

    // 确保角速度不超过配置的限制
    ang_vel_limit(_ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    // 从目标坐标系到体坐标系的旋转
    Quaternion rotation_target_to_body = attitude_body.inverse() * _attitude_target;

    // 目标角速度向量在体坐标系下
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target;

    // 修正推力向量并平滑添加前馈和航向输入
    _feedforward_scalar = 1.0f;
    if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE * 2.0f) {
        _ang_vel_body.z = _ahrs.get_gyro().z;
    } else if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        _feedforward_scalar = (1.0f - (_thrust_error_angle - AC_ATTITUDE_THRUST_ERROR_ANGLE) / AC_ATTITUDE_THRUST_ERROR_ANGLE);
        _ang_vel_body.x += ang_vel_body_feedforward.x * _feedforward_scalar;
        _ang_vel_body.y += ang_vel_body_feedforward.y * _feedforward_scalar;
        _ang_vel_body.z += ang_vel_body_feedforward.z;
        _ang_vel_body.z = _ahrs.get_gyro().z * (1.0 - _feedforward_scalar) + _ang_vel_body.z * _feedforward_scalar;
    } else {
        _ang_vel_body += ang_vel_body_feedforward;
    }

    // 记录误差以处理EKF重置
    _attitude_ang_error = attitude_body.inverse() * _attitude_target;
}

// thrust_heading_rotation_angles - 计算两个有序旋转，将attitude_body四元数移动到attitude_target四元数。
// 基于静态输出饱和度限制偏航轴的最大误差。
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const
{
    // 计算推力向量校正四元数
    Quaternion thrust_vector_correction;
    thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_vector_correction, attitude_error, thrust_angle, thrust_error_angle);

    // Todo: 根据输出饱和度和最大误差限制滚转和俯仰误差。

    // 基于当偏航速率为空时会饱和输出的最大值来限制偏航误差
    Quaternion heading_vec_correction_quat;

    // 计算最大偏航加速度
    float heading_accel_max = constrain_float(
        get_accel_yaw_max_radss() / 2.0f, // 最大偏航加速度的一半
        AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, // 最小偏航加速度
        AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS  // 最大偏航加速度
    );

    // 检查偏航PID控制器的比例增益是否不为零
    if (!is_zero(get_rate_yaw_pid().kP())) {
        // 计算最大偏航误差
        float heading_error_max = MIN(
            inv_sqrt_controller(1.0 / get_rate_yaw_pid().kP(), _p_angle_yaw.kP(), heading_accel_max),
            AC_ATTITUDE_YAW_MAX_ERROR_ANGLE
        );

        // 检查偏航角比例因子是否不为零且偏航误差是否超过最大值
        if (!is_zero(_p_angle_yaw.kP()) && fabsf(attitude_error.z) > heading_error_max) {
            // 将偏航误差限制在最大值范围内
            attitude_error.z = constrain_float(wrap_PI(attitude_error.z), -heading_error_max, heading_error_max);

            // 从轴角创建偏航校正四元数
            heading_vec_correction_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, attitude_error.z });

            // 更新目标姿态四元数
            attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;
        }
    }
}

// thrust_vector_rotation_angles - 计算两个有序旋转，将attitude_body四元数移动到attitude_target四元数。
// 第一个旋转校正推力向量，第二个旋转校正航向向量。
void AC_AttitudeControl::thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const
{
    // 推力方向在任何体坐标系中都是 [0,0,-1]，包括体坐标系和目标坐标系。
    const Vector3f thrust_vector_up { 0.0f, 0.0f, -1.0f };

    // attitude_target 和 attitude_body 是从目标/体坐标系到NED坐标系的被动旋转

    // 将 [0,0,-1] 通过 attitude_target 旋转，表示目标推力向量在惯性坐标系中的视图
    Vector3f att_target_thrust_vec = attitude_target * thrust_vector_up; // 目标推力向量

    // 将 [0,0,-1] 通过 attitude_body 旋转，表示当前推力向量在惯性坐标系中的视图
    Vector3f att_body_thrust_vec = attitude_body * thrust_vector_up; // 当前推力向量

    // 使用点积计算当前倾斜角度，用于外部函数
    thrust_angle = acosf(constrain_float(thrust_vector_up * att_body_thrust_vec, -1.0f, 1.0f));

    // 使用目标推力向量和当前推力向量的叉积定义旋转向量
    Vector3f thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;

    // 使用点积计算目标推力向量和当前推力向量之间的角度
    thrust_error_angle = acosf(constrain_float(att_body_thrust_vec * att_target_thrust_vec, -1.0f, 1.0f));

    // 归一化推力旋转向量
    float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_error_angle)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction 是相对于体坐标系定义的，但其轴 `thrust_vec_cross` 是在惯性坐标系中计算的。
    // 首先通过 attitude_body 的逆旋转将其转换回体坐标系
    thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
    thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle);

    // 计算 x 和 y 方向的角度误差
    Vector3f rotation;
    thrust_vector_correction.to_axis_angle(rotation);
    attitude_error.x = rotation.x;
    attitude_error.y = rotation.y;

    // 计算在推力向量旋转后仍需进行的旋转，转换到体坐标系后的 heading_vector_correction
    Quaternion heading_vec_correction_quat = thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

    // 计算 z 方向的角度误差（此时 x 和 y 应该为零）
    heading_vec_correction_quat.to_axis_angle(rotation);
    attitude_error.z = rotation.z;
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
float AC_AttitudeControl::input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt)
{
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
    desired_ang_vel += sqrt_controller(error_angle, 1.0f / MAX(input_tc, 0.01f), accel_max, dt);
    if (is_positive(max_ang_vel)) {
        desired_ang_vel = constrain_float(desired_ang_vel, -max_ang_vel, max_ang_vel);
    }

    // Acceleration is limited directly to smooth the beginning of the curve.
    return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt, 0.0f);
}

// Shapes the velocity request based on a rate time constant. The angular acceleration and deceleration is limited.
float AC_AttitudeControl::input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc)
{
    if (is_positive(input_tc)) {
        // Calculate the acceleration to smoothly achieve rate. Jerk is not limited.
        float error_rate        = desired_ang_vel - target_ang_vel;
        float desired_ang_accel = sqrt_controller(error_rate, 1.0f / MAX(input_tc, 0.01f), 0.0f, dt);
        desired_ang_vel         = target_ang_vel + desired_ang_accel * dt;
    }
    // Acceleration is limited directly to smooth the beginning of the curve.
    if (is_positive(accel_max)) {
        float delta_ang_vel = accel_max * dt;
        return constrain_float(desired_ang_vel, target_ang_vel - delta_ang_vel, target_ang_vel + delta_ang_vel);
    } else {
        return desired_ang_vel;
    }
}

// calculates the expected angular velocity correction from an angle error based on the AC_AttitudeControl settings.
// This function can be used to predict the delay associated with angle requests.
void AC_AttitudeControl::input_shaping_rate_predictor(const Vector2f& error_angle, Vector2f& target_ang_vel, float dt) const
{
    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        target_ang_vel.x = input_shaping_angle(wrap_PI(error_angle.x), _input_tc, get_accel_roll_max_radss(), target_ang_vel.x, dt);
        target_ang_vel.y = input_shaping_angle(wrap_PI(error_angle.y), _input_tc, get_accel_pitch_max_radss(), target_ang_vel.y, dt);
    } else {
        const float angleP_roll  = _p_angle_roll.kP() * _angle_P_scale.x;
        const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
        target_ang_vel.x         = angleP_roll * wrap_PI(error_angle.x);
        target_ang_vel.y         = angleP_pitch * wrap_PI(error_angle.y);
    }
    // Limit the angular velocity correction
    Vector3f ang_vel(target_ang_vel.x, target_ang_vel.y, 0.0f);
    ang_vel_limit(ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), 0.0f);

    target_ang_vel.x = ang_vel.x;
    target_ang_vel.y = ang_vel.y;
}

// limits angular velocity
void AC_AttitudeControl::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const
{
    if (is_zero(ang_vel_roll_max) || is_zero(ang_vel_pitch_max)) {
        if (!is_zero(ang_vel_roll_max)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
        }
        if (!is_zero(ang_vel_pitch_max)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
        }
    } else {
        Vector2f thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max, euler_rad.y / ang_vel_pitch_max);
        float    thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
        }
    }
    if (!is_zero(ang_vel_yaw_max)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
    }
}

// translates body frame acceleration limits to the euler axis
Vector3f AC_AttitudeControl::euler_accel_limit(const Quaternion& att, const Vector3f& euler_accel)
{
    if (!is_positive(euler_accel.x) || !is_positive(euler_accel.y) || !is_positive(euler_accel.z)) {
        return Vector3f { euler_accel };
    }

    const float phi   = att.get_euler_roll();
    const float theta = att.get_euler_pitch();

    const float sin_phi   = constrain_float(fabsf(sinf(phi)), 0.1f, 1.0f);
    const float cos_phi   = constrain_float(fabsf(cosf(phi)), 0.1f, 1.0f);
    const float sin_theta = constrain_float(fabsf(sinf(theta)), 0.1f, 1.0f);
    const float cos_theta = constrain_float(fabsf(cosf(theta)), 0.1f, 1.0f);

    return Vector3f {
        euler_accel.x,
        MIN(euler_accel.y / cos_phi, euler_accel.z / sin_phi),
        MIN(MIN(euler_accel.x / sin_theta, euler_accel.y / (sin_phi * cos_theta)), euler_accel.z / (cos_phi * cos_theta))
    };
}

// Sets attitude target to vehicle attitude and sets all rates to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void AC_AttitudeControl::reset_target_and_rate(bool reset_rate)
{
    // move attitude target to current attitude
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target);

    if (reset_rate) {
        _ang_vel_target.zero();
        _euler_rate_target.zero();
    }
}

// Sets yaw target to vehicle heading and sets yaw rate to zero
// If reset_rate is false rates are not reset to allow the rate controllers to run
void AC_AttitudeControl::reset_yaw_target_and_rate(bool reset_rate)
{
    // move attitude target to current heading
    float      yaw_shift = _ahrs.yaw - _euler_angle_target.z;
    Quaternion _attitude_target_update;
    _attitude_target_update.from_axis_angle(Vector3f { 0.0f, 0.0f, yaw_shift });
    _attitude_target = _attitude_target_update * _attitude_target;

    if (reset_rate) {
        // set yaw rate to zero
        _euler_rate_target.z = 0.0f;

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target, _euler_rate_target, _ang_vel_target);
    }
}

// Shifts the target attitude to maintain the current error in the event of an EKF reset
void AC_AttitudeControl::inertial_frame_reset()
{
    // Retrieve quaternion body attitude
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // Recalculate the target quaternion
    _attitude_target = attitude_body * _attitude_ang_error;

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);
}

// Convert a 321-intrinsic euler angle derivative to an angular velocity vector
void AC_AttitudeControl::euler_rate_to_ang_vel(const Quaternion& att, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi   = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi   = sinf(phi);
    const float cos_phi   = cosf(phi);

    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
// Returns false if the vehicle is pitched 90 degrees up or down
bool AC_AttitudeControl::ang_vel_to_euler_rate(const Quaternion& att, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    const float theta = att.get_euler_pitch();
    const float phi   = att.get_euler_roll();

    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);
    const float sin_phi   = sinf(phi);
    const float cos_phi   = cosf(phi);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

// 使用姿态误差更新目标角速度
Vector3f AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f& attitude_error_rot_vec_rad)
{
    // 定义一个向量来存储目标角速度
    Vector3f rate_target_ang_vel;

    // 计算滚转角速度需求
    // 获取滚转角的比例因子
    const float angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
    // 检查是否启用平方根控制器并且滚转加速度限制不为零
    if (_use_sqrt_controller && !is_zero(get_accel_roll_max_radss())) {
        // 使用平方根控制器计算滚转角速度
        rate_target_ang_vel.x = sqrt_controller(
            attitude_error_rot_vec_rad.x,                  // 滚转角误差
            angleP_roll,                                   // 滚转角比例因子
            constrain_float(                               // 限制滚转加速度
                get_accel_roll_max_radss() / 2.0f,         // 最大滚转加速度的一半
                AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, // 最小滚转加速度
                AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS  // 最大滚转加速度
                ),
            _dt // 时间步长
        );
    } else {
        // 直接使用比例因子计算滚转角速度
        rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
    }

    // 计算俯仰角速度需求
    // 获取俯仰角的比例因子
    const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
    // 检查是否启用平方根控制器并且俯仰加速度限制不为零
    if (_use_sqrt_controller && !is_zero(get_accel_pitch_max_radss())) {
        // 使用平方根控制器计算俯仰角速度
        rate_target_ang_vel.y = sqrt_controller(
            attitude_error_rot_vec_rad.y,                  // 俯仰角误差
            angleP_pitch,                                  // 俯仰角比例因子
            constrain_float(                               // 限制俯仰加速度
                get_accel_pitch_max_radss() / 2.0f,        // 最大俯仰加速度的一半
                AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, // 最小俯仰加速度
                AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS  // 最大俯仰加速度
                ),
            _dt // 时间步长
        );
    } else {
        // 直接使用比例因子计算俯仰角速度
        rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
    }

    // 计算航向角速度需求
    // 获取航向角的比例因子
    const float angleP_yaw = _p_angle_yaw.kP() * _angle_P_scale.z;
    // 检查是否启用平方根控制器并且航向加速度限制不为零
    if (_use_sqrt_controller && !is_zero(get_accel_yaw_max_radss())) {
        // 使用平方根控制器计算航向角速度
        rate_target_ang_vel.z = sqrt_controller(
            attitude_error_rot_vec_rad.z,                 // 航向角误差
            angleP_yaw,                                   // 航向角比例因子
            constrain_float(                              // 限制航向加速度
                get_accel_yaw_max_radss() / 2.0f,         // 最大航向加速度的一半
                AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, // 最小航向加速度
                AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS  // 最大航向加速度
                ),
            _dt // 时间步长
        );
    } else {
        // 直接使用比例因子计算航向角速度
        rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
    }

    // 重置角度P比例因子，保存使用的值
    // 保存当前使用的角度P比例因子
    _angle_P_scale_used = _angle_P_scale;
    // 重置角度P比例因子为1.0
    _angle_P_scale = VECTORF_111;

    // 返回计算得到的目标角速度向量
    return rate_target_ang_vel;
}

// Enable or disable body-frame feed forward
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        _accel_roll_max.set(0.0f);
        _accel_pitch_max.set(0.0f);
        _accel_yaw_max.set(0.0f);
    }
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl::get_althold_lean_angle_max_cd() const
{
    // convert to centi-degrees for public interface
    return MAX(ToDeg(_althold_lean_angle_max), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.0f;
}

// Return roll rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_roll_pid().get_filt_E_alpha(dt_average), get_rate_roll_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_roll_pid().kD()) / _dt + get_rate_roll_pid().kP());
    if (is_positive(_ang_vel_roll_max)) {
        rate_max = MIN(rate_max, get_ang_vel_roll_max_rads());
    }
    return rate_max;
}

// Return pitch rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_pitch_pid().get_filt_E_alpha(dt_average), get_rate_pitch_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_pitch_pid().kD()) / _dt + get_rate_pitch_pid().kP());
    if (is_positive(_ang_vel_pitch_max)) {
        rate_max = MIN(rate_max, get_ang_vel_pitch_max_rads());
    }
    return rate_max;
}

// Return yaw rate step size in centidegrees/s that results in maximum output after 4 time steps
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_yaw_pid().get_filt_E_alpha(dt_average), get_rate_yaw_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // todo: When a thrust_max is available we should replace 0.5f with 0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_yaw_pid().kD()) / _dt + get_rate_yaw_pid().kP());
    if (is_positive(_ang_vel_yaw_max)) {
        rate_max = MIN(rate_max, get_ang_vel_yaw_max_rads());
    }
    return rate_max;
}

bool AC_AttitudeControl::pre_arm_checks(const char*   param_prefix,
                                        char*         failure_msg,
                                        const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char* pid_name;
        AC_P&       p;
    } ps[] = {
        { "ANG_PIT", get_angle_pitch_p() },
        { "ANG_RLL", get_angle_roll_p() },
        { "ANG_YAW", get_angle_yaw_p() }
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // validate AC_PID members:
    const struct {
        const char* pid_name;
        AC_PID&     pid;
    } pids[] = {
        { "RAT_RLL", get_rate_roll_pid() },
        { "RAT_PIT", get_rate_pitch_pid() },
        { "RAT_YAW", get_rate_yaw_pid() },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(pids); i++) {
        // if the PID has a positive FF then we just ensure kP and
        // kI aren't negative
        AC_PID&     pid      = pids[i].pid;
        const char* pid_name = pids[i].pid_name;
        if (is_positive(pid.ff())) {
            // kP and kI must be non-negative:
            if (is_negative(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be >= 0", param_prefix, pid_name);
                return false;
            }
            if (is_negative(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be >= 0", param_prefix, pid_name);
                return false;
            }
        } else {
            // kP and kI must be positive:
            if (!is_positive(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, pid_name);
                return false;
            }
            if (!is_positive(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be > 0", param_prefix, pid_name);
                return false;
            }
        }
        // never allow a negative D term (but zero is allowed)
        if (is_negative(pid.kD())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_D must be >= 0", param_prefix, pid_name);
            return false;
        }
    }
    return true;
}

/*
  get the slew rate for roll, pitch and yaw, for oscillation
  detection in lua scripts
*/
void AC_AttitudeControl::get_rpy_srate(float& roll_srate, float& pitch_srate, float& yaw_srate)
{
    roll_srate  = get_rate_roll_pid().get_pid_info().slew_rate;
    pitch_srate = get_rate_pitch_pid().get_pid_info().slew_rate;
    yaw_srate   = get_rate_yaw_pid().get_pid_info().slew_rate;
}
