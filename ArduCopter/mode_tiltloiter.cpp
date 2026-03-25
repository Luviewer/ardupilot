#include "Copter.h"

#include <AP_InertialSensor/AP_InertialSensor.h>

/*
 * TiltLoiter 飞行模式
 *
 * 有杆量时：直接用 pos_control 速度接口控制 NED 速度。
 * 松杆时：切回 loiter_nav 处理刹车和定点悬停（复用 LOIT_BRK_ACC / LOIT_BRK_JRK 参数）。
 *
 * 坐标变换：
 *   虚拟 IMU 系（AHRS 看到的"水平"系）→ NED 系需要绕 Y 轴旋转 pitch_off_deg。
 *
 *   pitch 杆 → 虚拟系前向速度 v_pitch  （推杆前进为正）
 *   油门杆   → 虚拟系垂直速率 v_throttle（推杆向上为正）
 *   roll 杆  → 虚拟系右向速度 v_roll    （不受 pitch_off_deg 影响）
 *
 *   用 Ry(-pitch_off_deg) 将虚拟系向量变换回 NED：
 *     v_north =  v_pitch * cos(tilt) - v_throttle * sin(tilt)
 *     v_down  = -v_pitch * sin(tilt) - v_throttle * cos(tilt)
 *
 *   pitch_off_deg > 0 = 机头朝上。tilt=0 时退化为标准行为。
 */

bool ModeTiltLoiter::init(bool ignore_checks)
{
    // 垂直速度/加速度限制
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // 初始化 D 控制器
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // 初始化 loiter_nav（刹车/定点用）
    loiter_nav->init_target();

    _pilot_has_vel_cmd = false;

    return true;
}

void ModeTiltLoiter::run()
{
    // 更新垂直速度/加速度限制
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // SIMPLE 模式变换
    update_simple_mode();

    //==========================================================================
    // 从遥控杆获取虚拟 IMU 系中的期望速度
    //==========================================================================
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    const float angle_max_rad = MAX(loiter_nav->get_angle_max_rad(), 0.01f);
    const float speed_max_ne = loiter_nav->get_speed_max_NE_ms();

    const float pitch_norm = -target_pitch_rad / angle_max_rad;
    const float roll_norm  =  target_roll_rad  / angle_max_rad;

    const float v_pitch_ms = pitch_norm * speed_max_ne;
    const float v_roll_ms  = roll_norm  * speed_max_ne;

    const float v_throttle_ms = get_pilot_desired_climb_rate_ms();
    const float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    //==========================================================================
    // 绕 Y 轴旋转 pitch_off_deg：虚拟 IMU 系 → NED 系
    //==========================================================================
    const float pitch_off_rad = radians(AP::ins().get_imu_pitch_rot_deg());
    const float cos_tilt = cosf(pitch_off_rad);
    const float sin_tilt = sinf(pitch_off_rad);

    // Ry(-pitch_off_deg) 将虚拟 IMU 系变换回真实 NED 系
    //   v_forward_ned =  v_pitch * cos(tilt) - v_throttle * sin(tilt)
    //   v_down_ned    = -v_pitch * sin(tilt) - v_throttle * cos(tilt)
    const float v_forward_ned_ms =  v_pitch_ms * cos_tilt - v_throttle_ms * sin_tilt;
    const float v_down_ned_ms    = -v_pitch_ms * sin_tilt - v_throttle_ms * cos_tilt;
    const float v_right_ned_ms   =  v_roll_ms;

    // body → NED（偏航旋转）
    const float yaw_rad = ahrs.get_yaw();
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    Vector2f vel_ne_ms;
    vel_ne_ms.x = v_forward_ned_ms * cos_yaw - v_right_ned_ms * sin_yaw;
    vel_ne_ms.y = v_forward_ned_ms * sin_yaw + v_right_ned_ms * cos_yaw;

    float vel_d_ms = v_down_ned_ms;

    //==========================================================================
    // 判断是否有杆量产生了 NE 方向速度指令
    // pitch/roll 杆直接产生 NE 速度，油门杆在 tilt≠0 时也会通过 sin(tilt) 产生 NE 分量
    //==========================================================================
    const bool has_ne_vel_cmd = !vel_ne_ms.is_zero();

    //==========================================================================
    // 高度状态机
    //==========================================================================
    float target_climb_rate_ms = -vel_d_ms;
    AltHoldModeState alt_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    switch (alt_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->D_relax_controller(0.0f);
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Takeoff:
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g2.pilot_takeoff_alt_m, 0.0, 10.0));
        }
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        loiter_nav->update();
        break;

    case AltHoldModeState::Flying:
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif

        //==================================================================
        // 水平控制核心：有杆量 → 速度控制，松杆 → loiter_nav 刹车定点
        //==================================================================
        if (has_ne_vel_cmd) {
            // 有 NE 速度指令：直接送 NED 速度到 pos_control
            Vector2f accel_ne_zero;
            pos_control->input_vel_accel_NE_m(vel_ne_ms, accel_ne_zero, false);
            pos_control->NE_stop_pos_stabilisation();
        } else {
            if (_pilot_has_vel_cmd) {
                // 刚松杆：用当前位置/速度初始化 loiter_nav，开始刹车
                loiter_nav->init_target();
            }
            // 松杆后：loiter_nav 处理刹车 + 定点保持
            loiter_nav->set_pilot_desired_acceleration_rad(0.0f, 0.0f);
            if (copter.ap.land_complete_maybe) {
                loiter_nav->soften_for_landing();
            }
            loiter_nav->update();
        }

        _pilot_has_vel_cmd = has_ne_vel_cmd;
        break;
    }

    //==========================================================================
    // 垂直控制
    //   有杆量时：NE 和 D 都用 vel_accel 接口，动态响应一致，轨迹笔直
    //   松杆时：用爬升率 → 位置目标（标准高度保持行为）
    //==========================================================================
    const bool has_any_vel_cmd = has_ne_vel_cmd || !is_zero(vel_d_ms);

    if (alt_state == AltHoldModeState::Flying) {
        if (has_any_vel_cmd) {
            pos_control->input_vel_accel_D_m(vel_d_ms, 0.0f, false);
        } else {
            pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        }
    }

    // 运行控制器
    pos_control->D_update_controller();

    if (has_ne_vel_cmd && alt_state == AltHoldModeState::Flying) {
        pos_control->NE_update_controller();
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), target_yaw_rate_rads, false);
    } else {
        attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    }
}

float ModeTiltLoiter::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeTiltLoiter::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}
