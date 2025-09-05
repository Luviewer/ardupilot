#include "Copter.h"

/*************************************************************
 *  姿态速率控制器和时序控制
 ****************************************************************/

/*
  从主线程运行时更新速率控制器（正常操作模式）
*/
void Copter::run_rate_controller_main()
{
    // 设置姿态和位置控制器的循环时间
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt(last_loop_time_s);
    attitude_control->set_dt(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt(last_loop_time_s);
        // 如果不使用速率线程，则只运行速率控制器
        attitude_control->rate_controller_run();
    }
    // 重置系统识别和其他临时输入
    attitude_control->rate_controller_target_reset();
}

/*************************************************************
 *  油门控制
 ****************************************************************/

// 更新悬停所需的估计油门值（如果需要）
// 以100Hz频率调用
void Copter::update_throttle_hover()
{
    // 如果未解锁、已着陆或处于待机状态，则退出
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // 在手动油门模式或漂移模式下不更新
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // 在爬升或下降时不更新
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // 获取油门输出
    float throttle = motors->get_throttle();

    // 如果处于水平悬停状态，则计算平均油门值。考虑直升机悬停横滚修正
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // 是否可以自动设置时间常数
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate - 将飞行员的油门输入转换为爬升率(cm/s)
// 底部没有任何死区
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // 油门故障安全检查
    if (failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // 允许在油门解锁后减小油门，以及
        // 在接近地面时缓慢下降
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // 确保油门值合理
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // 确保死区值合理
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // 检查油门是否在死区上方、下方或死区内
    if (throttle_control < deadband_bottom) {
        // 在死区下方
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // 在死区上方
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // 必须在死区内
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - 一个在最小和中等油门之间的油门值，不应导致起飞
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// set_accel_throttle_I_from_pilot_throttle - 平滑从飞行员控制油门到自动驾驶油门的过渡
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // 获取发送到姿态控制器的最后一个油门输入
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // 将飞行员油门和悬停油门之间的差值转移到加速度计I项
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// 将向量从车辆视角旋转到北东坐标系
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// 如果PILOT_SPEED_DN值非零，则返回该值；如果为零，则返回PILOT_SPEED_UP值。
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    } else {
        return abs(g2.pilot_speed_dn);
    }
}
