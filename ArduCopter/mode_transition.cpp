#include "Copter.h"

// transition_land_init - initialise land controller
bool ModeTransition::init(bool ignore_checks)
{
    // 调试用
    gcs().send_text(MAV_SEVERITY_NOTICE, "######### #################### ########");
    gcs().send_text(MAV_SEVERITY_NOTICE, "######### INTO TRANSITION MODE ########");
    gcs().send_text(MAV_SEVERITY_NOTICE, "######### #################### ########");

    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();

    // 设置水平速度以及加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化水平位置控制器
    if (control_position && !pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // 设置垂直速度以及加速度限制
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化高度控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    transition_start_time = millis();
    transition_pause = false;

    // 重置标志位，表示飞行员在降落过程中尚未进行滚转或俯仰操作
    copter.ap.land_repo_active = false;

    // 设置标志位，表示精准降落尚未被激活
    copter.ap.prec_land_active = false;

    // 初始化偏航角，设置偏航模式为保持当前偏航角
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // 初始化成功，返回true
    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeTransition::run()
{

    if (hal.rcin->read(CH_8) < 1500) {
        transition_mode = SubMode::TakeOff;
    } else {
        transition_mode = SubMode::Land;
    }

    switch (transition_mode) {

    case SubMode::TakeOff:
        // run takeoff controller
        transition_takeoff_run();
        gcs().send_text(MAV_SEVERITY_NOTICE, "********* *********************** *********");
        gcs().send_text(MAV_SEVERITY_NOTICE, "********* INTO TRANSITION TAKEOFF *********");
        gcs().send_text(MAV_SEVERITY_NOTICE, "********* *********************** *********");
        break;

    case SubMode::Land:
        // run land controller
        if (control_position) {
            transition_land_gps_run();
        } else {
            transition_land_nogps_run();
            gcs().send_text(MAV_SEVERITY_NOTICE, "********* *********************** *********");
            gcs().send_text(MAV_SEVERITY_NOTICE, "********* INTO TRANSITION LAND ************");
            gcs().send_text(MAV_SEVERITY_NOTICE, "********* *********************** *********");
        }
        break;
    }
}

// takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void ModeTransition::transition_takeoff_run()
{
    auto_takeoff_run();
    if (auto_takeoff_complete && !transition_takeoff_complete) {
        transition_takeoff_complete = true;
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeTransition::transition_land_gps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::TRANSITION);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (transition_pause && millis()-transition_start_time >= LAND_WITH_DELAY_MS) {
            transition_pause = false;
        }

        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, auto_yaw.get_heading().yaw_rate_cds);

        // run normal landing or precision landing (if enabled)
        pos_control->land_at_climb_rate_cm(10.0f, false);
        pos_control->update_z_controller();
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeTransition::transition_land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }

    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (transition_pause && millis()-transition_start_time >= LAND_WITH_DELAY_MS) {
            transition_pause = false;
        }

        land_run_vertical_control(transition_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, auto_yaw.get_heading().yaw_rate_cds);
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_transition_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::TRANSITION, reason);
    mode_land.set_land_pause(true);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::transition_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::TRANSITION &&
            mode_land.controlling_position());
}