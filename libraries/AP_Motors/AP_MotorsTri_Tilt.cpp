/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   共轴 Y6B 倾转三旋翼（5DOF 控制）

   电机布局：
   - 电机 0-1：前右共轴一对（上 CW、下 CCW）
   - 电机 2-3：后置共轴一对（上 CW、下 CCW）
   - 电机 4-5：前左共轴一对（上 CW、下 CCW）

   控制分配基于 MATLAB 推导：
   静态矩阵 F_alloc 将 5DOF [Fx, Fz, Mx, My, Mz] 映射到 6 个中间量
   [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)]
*/

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

# include <AP_HAL/AP_HAL.h>
# include <AP_Math/AP_Math.h>
# include <AP_Vehicle/AP_Vehicle_Type.h>
# include <GCS_MAVLink/GCS.h>

# include "AP_MotorsTri_Tilt.h"
# include <AP_AHRS/AP_AHRS_View.h>
# include <AP_InertialSensor/AP_InertialSensor.h>
# define AP_MOTORS_TRI_TILT_USE_113E6D0_ALLOC

extern const AP_HAL::HAL& hal;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 宏定义：限制警告提示（宏定义在头文件中）
////////////////////////////////////////////////////////////////////////////////////////////////////////////
# if AP_MOTORS_TRI_TILT_ENABLE_LIMIT_WARNINGS
// 警告发送间隔（毫秒）
#  define LIMIT_WARN_INTERVAL_MS 500

// 发送限制警告的辅助宏（只在状态从false变为true时发送，且0.5秒内只发送一次）
#  define SEND_LIMIT_WARNING(type, format, ...)                                    \
      do {                                                                         \
          if (!_limit_warn_state.type##_last) {                                    \
              const uint32_t now_ms = AP_HAL::millis();                            \
              if (now_ms - _limit_warn_state.type##_ms > LIMIT_WARN_INTERVAL_MS) { \
                  GCS_SEND_TEXT(MAV_SEVERITY_INFO, format, ##__VA_ARGS__);         \
                  _limit_warn_state.type##_ms = now_ms;                            \
              }                                                                    \
          }                                                                        \
          _limit_warn_state.type##_last = true;                                    \
      } while (0)

// 发送多个限制的警告（用于RPY同时受限）
#  define SEND_MULTI_LIMIT_WARNING(type, format, ...)                              \
      do {                                                                         \
          if (!_limit_warn_state.type##_last) {                                    \
              const uint32_t now_ms = AP_HAL::millis();                            \
              if (now_ms - _limit_warn_state.type##_ms > LIMIT_WARN_INTERVAL_MS) { \
                  GCS_SEND_TEXT(MAV_SEVERITY_INFO, format, ##__VA_ARGS__);         \
                  _limit_warn_state.type##_ms = now_ms;                            \
                  /* 同时更新相关的单个限制时间戳和状态，避免重复提示 */           \
                  _limit_warn_state.roll_ms    = now_ms;                           \
                  _limit_warn_state.pitch_ms   = now_ms;                           \
                  _limit_warn_state.yaw_ms     = now_ms;                           \
                  _limit_warn_state.roll_last  = true;                             \
                  _limit_warn_state.pitch_last = true;                             \
                  _limit_warn_state.yaw_last   = true;                             \
              }                                                                    \
          }                                                                        \
          _limit_warn_state.type##_last = true;                                    \
      } while (0)

// 重置限制状态（当限制解除时调用）
#  define RESET_LIMIT_STATE(type)                \
      do {                                       \
          _limit_warn_state.type##_last = false; \
      } while (0)
# else
#  define SEND_LIMIT_WARNING(type, format, ...) \
      do {                                      \
      } while (0)
#  define SEND_MULTI_LIMIT_WARNING(type, format, ...) \
      do {                                            \
      } while (0)
#  define RESET_LIMIT_STATE(type) \
      do {                        \
      } while (0)
# endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 参数
////////////////////////////////////////////////////////////////////////////////////////////////////////////
const AP_Param::GroupInfo AP_MotorsTri_Tilt::var_info[] = {
    // 先链入父类 Multicopter 参数（MOT_YAW_HEADROOM、MOT_THST_EXPO 等），再接本类自定义参数
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: TRI_TILT_ANG_MAX
    // @DisplayName: 最大倾转角
    // @Description: 转子允许的最大倾转角（度）
    // @Range: 0 135
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("TILT_ANG_MAX", 4, AP_MotorsTri_Tilt, _servo_angle_max, 135.0f),

    // @Param: TRI_TILT_YAW_FAC
    // @DisplayName: 偏航力矩因子
    // @Description: 共轴对差分推力产生偏航力矩的比例因子
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_FAC", 8, AP_MotorsTri_Tilt, _yaw_torque_factor, 1.0f),

    // @Param: TRI_TILT_SVO_FR_REV
    // @DisplayName: 前右倾转舵机反向
    // @Description: 设为 1 以反向前右倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FR_REV", 11, AP_MotorsTri_Tilt, _tilt_servo_fr_rev, 0),

    // @Param: TRI_TILT_SVO_REAR_REV
    // @DisplayName: 后置倾转舵机反向
    // @Description: 设为 1 以反向后置倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_REAR_REV", 12, AP_MotorsTri_Tilt, _tilt_servo_rear_rev, 0),

    // @Param: TRI_TILT_SVO_FL_REV
    // @DisplayName: 前左倾转舵机反向
    // @Description: 设为 1 以反向前左倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_FL_REV", 13, AP_MotorsTri_Tilt, _tilt_servo_fl_rev, 0),

    // @Param: TRI_TILT_PIT_OFF_MAX
    // @DisplayName: 最大俯仰姿态偏置（RC7）
    // @Description: RC7 指令的最大俯仰姿态偏置（度）（1500->0，1000->-max，2000->+max）
    // @Range: 0 45
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PIT_OFF_MAX", 14, AP_MotorsTri_Tilt, _tilt_pitch_off_max_deg, 20.0f),

    // @Param: TILT_EN
    // @DisplayName: Tilt enable
    // @Description: Tilt enable
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO("TILT_EN", 15, AP_MotorsTri_Tilt, _tilt_enable, 1),

    // @Param: FORWARD_FACTOR
    // @DisplayName: Forward factor
    // @Description: Forward factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("FORW_FACT", 16, AP_MotorsTri_Tilt, _forward_factor, 1),

    // @Param: SVO_FR_OFF
    // @DisplayName: Front-right tilt servo offset
    // @Description: Front-right tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_FR_OFF", 17, AP_MotorsTri_Tilt, _svo_fr_offset, 0),

    // @Param: SVO_REAR_OFF
    // @DisplayName: Rear tilt servo offset
    // @Description: Rear tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_REAR_OFF", 18, AP_MotorsTri_Tilt, _svo_rear_offset, 0),

    // @Param: SVO_FL_OFF
    // @DisplayName: Front-left tilt servo offset
    // @Description: Front-left tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_FL_OFF", 19, AP_MotorsTri_Tilt, _svo_fl_offset, 0),

    // @Param: ANTI_YAW_FAC
    // @DisplayName: Anti-yaw factor
    // @Description: Anti-yaw factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("ANTI_YAW_FAC", 20, AP_MotorsTri_Tilt, _anti_yaw_factor, 1),

    // @Param: PITCH_FAC
    // @DisplayName: Pitch factor
    // @Description: Pitch factor for bicopter tilt control
    // @Range: 1.0~2.0
    // @User: Advanced
    AP_GROUPINFO("BI_PIT_FAC", 21, AP_MotorsTri_Tilt, _bicopter_pitch_P_factor, 0.0f),

    // @Param: LAT_FACT
    // @DisplayName: Lateral factor
    // @Description: Lateral factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("LAT_FACT", 22, AP_MotorsTri_Tilt, _lateral_factor, 1),

    // @Param: LAT_FACT
    // @DisplayName: Lateral factor
    // @Description: Lateral factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("LAT_ENABLE", 23, AP_MotorsTri_Tilt, _lateral_enable, 1),


    AP_GROUPEND
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 启用 6 电机的共轴 Y6B 配置
    add_motor_num(AP_MOTORS_MOT_1); // 前右上
    add_motor_num(AP_MOTORS_MOT_2); // 前右下
    add_motor_num(AP_MOTORS_MOT_3); // 后置上
    add_motor_num(AP_MOTORS_MOT_4); // 后置下
    add_motor_num(AP_MOTORS_MOT_5); // 前左上
    add_motor_num(AP_MOTORS_MOT_6); // 前左下

    // 设置电机更新频率
    set_update_rate(_speed_hz);

    // 标记电机启用（用于校准）
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // 设置默认电机与舵机映射（可通过 SERVOx_FUNCTION 覆盖）
    // 映射：a1->前右，a2->后置，a3->前左
    const bool ok_fr   = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, AP_MOTORS_TRI_TILT_SERVO_FR);
    const bool ok_rear = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRear, AP_MOTORS_TRI_TILT_SERVO_REAR);
    const bool ok_fl   = SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, AP_MOTORS_TRI_TILT_SERVO_FL);
    // 设置倾转舵机角度范围（厘度）
    // 若 _servo_angle_max 为 0，则视为“无软件限幅”，这里设置为较宽范围。
    // const float   ang_max_deg    = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
    // const int16_t servo_range_cd = int16_t(constrain_float(ang_max_deg, 0.0f, float(AP_MOTORS_TRI_TILT_ANGLE_MAX)) * 100.0f);
    const float    ang_max_deg    = 270.0f / 2.0f;
    const uint16_t servo_range_cd = uint16_t(ang_max_deg * 100.0f);

    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, servo_range_cd);

    // 设置倾转舵机默认 PWM 范围（硬件：500~2500us）
    // 仅修改默认值；用户设置的 SERVOx_MIN/MAX 不会被覆盖。
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRight, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorRear, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_tiltMotorLeft, 500, 2500);

    // 检查舵机是否已分配（默认或用户映射）
    _servos_assigned = (ok_fr || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight))
        && (ok_rear || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear))
        && (ok_fl || SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));

    // 配置电机与分配矩阵
    setup_motors(frame_class, frame_type);

    _mav_type = MAV_TYPE_COAXIAL;

    // 记录初始化成功
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI && _servos_assigned);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置机架类别与类型
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 机架变化时重新初始化
    if (frame_class != _active_frame_class || frame_type != _active_frame_type) {
        _active_frame_class = frame_class;
        _active_frame_type  = frame_type;
        setup_motors(frame_class, frame_type);
    }

    set_initialised_ok((frame_class == MOTOR_FRAME_TRI) && _servos_assigned);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置电机更新频率
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::set_update_rate(uint16_t speed_hz)
{
    // 记录请求的频率
    _speed_hz = speed_hz;

    // 为全部 6 个电机设置更新频率
    uint32_t mask = 1U << AP_MOTORS_MOT_1
        | 1U << AP_MOTORS_MOT_2
        | 1U << AP_MOTORS_MOT_3
        | 1U << AP_MOTORS_MOT_4
        | 1U << AP_MOTORS_MOT_5
        | 1U << AP_MOTORS_MOT_6;

    rc_set_freq(mask, _speed_hz);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 配置电机 - 设置静态分配矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 注意：
    // 不要在此处调用 AP_MotorsMatrix::remove_motor()。
    // 该后端通过 rc_write() 直接驱动电机输出，并使用 motor_enabled[]
    // 实现起转状态逻辑。remove_motor() 会清空 motor_enabled[]，导致输出停止
    // （用户反馈电机输出卡在 0）。
    //
    // 电机通道默认值在 init() 里通过 add_motor_num() 设置。
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;

    // 矩阵清零
    memset(_thrust, 0, sizeof(_thrust));
    memset(_tilt_angle_rad, 0, sizeof(_tilt_angle_rad));
    memset(_rpy_out, 0, sizeof(_rpy_out));

    _frame_class_string = "TRI_TILT";
    _frame_type_string  = "Coaxial-Y6B";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取电机掩码
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t AP_MotorsTri_Tilt::get_motor_mask()
{
    uint32_t mask = AP_MotorsMatrix::get_motor_mask();

    // 加入倾转舵机输出
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRight, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorRear, chan)) {
        mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_tiltMotorLeft, chan)) {
        mask |= 1U << chan;
    }

    return mask;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 执行解锁检查
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AP_MotorsTri_Tilt::arming_checks(size_t buflen, char* buffer) const
{
    // 检查舵机是否已分配
    if (!_servos_assigned) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Servos not assigned");
        return false;
    }

    // 检查角度限制：
    // _servo_angle_max == 0 表示“无软件限幅”（仍受 SERVOx_MIN/MAX 约束）。
    // 否则强制一个最小值，避免范围过小导致过度缩放/饱和。
    const float ang_max = _servo_angle_max;
    if ((!is_zero(ang_max) && ang_max < AP_MOTORS_TRI_TILT_ANGLE_MIN) || ang_max < 0.0f || ang_max > AP_MOTORS_TRI_TILT_ANGLE_MAX) {
        hal.util->snprintf(buffer, buflen, "TRI_TILT: Invalid angle limits");
        return false;
    }

    return true;
}

void AP_MotorsTri_Tilt::servoOutput(enum TiltIndex servo_index, float svo_out_cd)
{
    if (servo_index == FR) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, (float)_tilt_servo_fr_rev.get() * svo_out_cd + (float)_svo_fr_offset.get());
    } else if (servo_index == REAR) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, (float)_tilt_servo_rear_rev.get() * svo_out_cd + (float)_svo_rear_offset.get());
    } else if (servo_index == FL) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, (float)_tilt_servo_fl_rev.get() * svo_out_cd + (float)_svo_fl_offset.get());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_to_motors - 发送电机与舵机指令
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    int16_t fr_out_cd   = 0;
    int16_t fl_out_cd   = 0;
    int16_t rear_out_cd = 0;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // 电机输出最小值
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }

        case SpoolState::GROUND_IDLE: {
            // 解锁但未起飞时给电机输出
            float spin_up = actuator_spin_up_to_ground_idle();
            for (uint8_t i = 0; i < 6; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], spin_up);
                }
            }
            break;
        }

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            for (uint8_t i = 0; i < 6; i++) {
                set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust[i]));
            }

            // 输出倾转舵机角度（厘度）
            // 若 _servo_angle_max == 0，则无软件限幅（仍受 SERVOx_MIN/MAX 限制）。
            // const float lim_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
            // fr_out_cd           = int16_t(constrain_float(degrees(_tilt_angle_rad[0]), -lim_deg, lim_deg) * 100);
            // fl_out_cd           = int16_t(constrain_float(degrees(_tilt_angle_rad[2]), -lim_deg, lim_deg) * 100);
            // rear_out_cd         = int16_t(constrain_float(degrees(_tilt_angle_rad[1]), -lim_deg, lim_deg) * 100);
            break;
    }

    for (uint8_t i = 0; i < 6; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }

    const float lim_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_TRI_TILT_ANGLE_MAX);
    fr_out_cd           = int16_t(constrain_float(degrees(_tilt_angle_rad[0]), -lim_deg, lim_deg) * 100);
    fl_out_cd           = int16_t(constrain_float(degrees(_tilt_angle_rad[2]), -lim_deg, lim_deg) * 100);
    rear_out_cd         = int16_t(constrain_float(degrees(_tilt_angle_rad[1]), -lim_deg, lim_deg) * 100);

# if 0
    (void)fr_out_cd;
    (void)fl_out_cd;
    (void)rear_out_cd;
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * 9000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * 0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_servo_rear_rev.get() * 9000);
# else
    // if (get_tilt_enable() == 1) {
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * fr_out_cd);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * fl_out_cd);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_servo_rear_rev.get() * rear_out_cd);
    // } else {
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * 0);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * 0);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_servo_rear_rev.get() * 0);
    // }
    // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * fr_out_cd);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * fl_out_cd);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_servo_rear_rev.get() * rear_out_cd);

    servoOutput(FR, fr_out_cd);
    servoOutput(FL, fl_out_cd);
    servoOutput(REAR, rear_out_cd);
# endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 推力补偿
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::thrust_compensation(void)
{
    // 调用父类推力补偿
    AP_MotorsMatrix::thrust_compensation();
}

// sets the roll and pitch offset, this rotates the thrust vector in body frame
// these are typically set such that the throttle thrust vector is earth frame up
void AP_MotorsTri_Tilt::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset  = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

float AP_MotorsTri_Tilt::get_bicopter_pitch_P_factor()
{
    float ahrs_pitch_abs = constrain_value(fabsf(AP::ins().get_imu_pitch_rot_deg() / 90.0f), 0.0f, 1.0f);
    return constrain_value(_bicopter_pitch_P_factor.get() * ahrs_pitch_abs, -0.1f, 0.1f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_armed_stabilizing - 主要控制分配
// 实现流程：5DOF 输入 -> 静态矩阵 -> 6 个中间量 -> 推力 + 倾转角
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::output_armed_stabilizing()
{
    float roll_thrust;     // roll thrust input value, +/- 1.0
    float pitch_thrust;    // pitch thrust input value, +/- 1.0
    float yaw_thrust;      // yaw thrust input value, +/- 1.0
    float throttle_thrust; // throttle thrust input value, 0.0 - 1.0
    float forward_thrust;  // forward thrust input value, +/- 1.0
    float lateral_thrust;  // lateral thrust input value, +/- 1.0

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第1层：基础补偿 - 电压与高度补偿增益
    /////////////////////////////////////////////////////////////////////////////////////////////////
    const float compensation_gain = thr_lin.get_compensation_gain();

    roll_thrust     = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust    = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust      = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第2层：Throttle 补偿和限制
    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Throttle 平均最大值补偿
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // Throttle 最大值限制
    const float throttle_thrust_max = _throttle_thrust_max * compensation_gain;

    // Throttle 下限检查
    if (throttle_thrust <= 0.0f) {
        throttle_thrust      = 0.0f;
        limit.throttle_lower = true;
        SEND_LIMIT_WARNING(throttle_lower,
                           "Throttle limited: lower bound (req: %.2f)",
                           get_throttle() * compensation_gain);
    }

    // Throttle 上限检查
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust      = throttle_thrust_max;
        limit.throttle_upper = true;
        SEND_LIMIT_WARNING(throttle_upper,
                           "Throttle limited: upper bound (req: %.2f, max: %.2f)",
                           get_throttle() * compensation_gain,
                           throttle_thrust_max);
    }

    // 确保 throttle_avg_max 在合理范围内
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // 计算提供最大 RPY 控制范围的最佳油门
    float throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // 互补控制融合权重
    float ahrs_pitch_abs  = fabsf(AP::ins().get_imu_pitch_rot_deg() / 90.0f);
    float sign_ahrs_pitch = AP::ins().get_imu_pitch_rot_deg() > 0.0f ? 1.0f : -1.0f;
    ahrs_pitch_abs        = constrain_float(ahrs_pitch_abs, -1.0f, 1.0f);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第3层：三旋翼基础控制分配
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 初始化倾转角度
    if (get_tilt_enable()) {
        _tilt_angle_rad[FR] = _tilt_angle_rad[REAR] = _tilt_angle_rad[FL] = -radians(AP::ins().get_imu_pitch_rot_deg());
    } else {
        _tilt_angle_rad[FR] = _tilt_angle_rad[REAR] = _tilt_angle_rad[FL] = 0;
    }

    // 三旋翼 RPY 控制分配（不含 throttle）
    _thrust_right_tricopter = roll_thrust * -0.5f + pitch_thrust * 0.5f;
    _thrust_left_tricopter  = roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_rear_tricopter  = -0.5f * pitch_thrust;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第4层：双旋翼控制分配
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 双旋翼推力分配（不含 throttle）
    _thrust_right_bicopter = -roll_thrust * 0.5f;
    _thrust_left_bicopter  = roll_thrust * 0.5f;
    _thrust_rear_bicopter  = 0.0f;

    // 双旋翼倾转控制
    _tilt_right_bicopter = pitch_thrust * 0.5f * sign_ahrs_pitch;
    _tilt_left_bicopter  = pitch_thrust * 0.5f * sign_ahrs_pitch;
    _tilt_rear_bicopter  = -pitch_thrust * 0.5f * sign_ahrs_pitch;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第6层：Yaw 限制（不按电机余量收紧，仅用 MOT_YAW_HEADROOM 保底，避免与后续缩放耦合导致首次 yaw 窜高）
    /////////////////////////////////////////////////////////////////////////////////////////////////
    float yaw_allowed = 1.0f;

    // 应用 yaw headroom 保底
    // float yaw_allowed_min = (float)_yaw_headroom * 0.001f;
    // yaw_allowed           = MAX(yaw_allowed, yaw_allowed_min);

    // 限制yaw_thrust（参考 AP_MotorsMatrix 第327-331行）
    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw  = true;
# if AP_MOTORS_TRI_TILT_ENABLE_LIMIT_WARNINGS
        const float yaw_requested = (_yaw_in + _yaw_in_ff) * compensation_gain;
        SEND_LIMIT_WARNING(yaw,
                           "Yaw limited: %.0f%% (req: %.2f, allowed: %.2f)",
                           yaw_allowed * 100.0f,
                           yaw_requested,
                           yaw_allowed);
# endif
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第7层：计算 RPY 组合输出范围（用于缩放补偿）
    /////////////////////////////////////////////////////////////////////////////////////////////////
    float rpy_low  = 1.0f;  // 最低推力值
    float rpy_high = -1.0f; // 最高推力值

    // Yaw 共轴差分：正 yaw 时前右/后臂 上+yaw 下-yaw；前左 上-yaw 下+yaw（与 Mz 符号一致）
    // 前右（0,1）：上 CW 下 CCW
    float thrust_right_mixed = _thrust_right_tricopter * (1.0f - ahrs_pitch_abs) + _thrust_right_bicopter * ahrs_pitch_abs;
    _rpy_out[FR_UP]          = thrust_right_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[FR_DOWN]        = thrust_right_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;

    // 后（2,3）：上 CW 下 CCW
    float thrust_rear_mixed = _thrust_rear_tricopter * (1.0f - ahrs_pitch_abs) + _thrust_rear_bicopter * ahrs_pitch_abs;
    _rpy_out[REAR_UP]       = thrust_rear_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[REAR_DOWN]     = thrust_rear_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;

    // 前左（4,5）：上 CW 下 CCW，yaw 符号与前右/后相反
    float thrust_left_mixed = _thrust_left_tricopter * (1.0f - ahrs_pitch_abs) + _thrust_left_bicopter * ahrs_pitch_abs;
    _rpy_out[FL_UP]         = thrust_left_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[FL_DOWN]       = thrust_left_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;


    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第12层：侧向力控制
    /////////////////////////////////////////////////////////////////////////////////////////////////
    if (_lateral_enable == 1) {
        lateral_thrust = _lateral_in;

        // 侧向力混合
        _rpy_out[FR_UP] += lateral_thrust * _lateral_factor;
        _rpy_out[FR_DOWN] -= lateral_thrust * _lateral_factor;
        
        _rpy_out[FL_UP] -= lateral_thrust * _lateral_factor;
        _rpy_out[FL_DOWN] += lateral_thrust * _lateral_factor;
    }

    // 找出最高和最低 RPY 输出
    for (uint8_t i = 0; i < MotorIndex_COUNT; i++) {
        if (_rpy_out[i] < rpy_low) {
            rpy_low = _rpy_out[i];
        }
        if (_rpy_out[i] > rpy_high) {
            rpy_high = _rpy_out[i];
        }
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第8层：RPY 缩放补偿
    /////////////////////////////////////////////////////////////////////////////////////////////////
    float rpy_scale = 1.0f;

    // 如果 RPY 范围超出 1.0，需要缩放
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }

    // 如果下限会导致负值，也需要缩放
    if (throttle_avg_max + rpy_low < 0.0f) {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // 应用缩放
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;

    // 应用缩放到所有 RPY 输出
    for (uint8_t i = 0; i < MotorIndex_COUNT; i++) {
        _rpy_out[i] *= rpy_scale;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第9层：Throttle 调整补偿（避免首次打 yaw 时基座高于飞行员油门导致窜高）
    /////////////////////////////////////////////////////////////////////////////////////////////////
    throttle_thrust_best_rpy = -rpy_low;
    float thr_adj            = throttle_thrust - throttle_thrust_best_rpy;

    if (rpy_scale < 1.0f) {
        // RPY 占满了全部范围
        limit.roll  = true;
        limit.pitch = true;
        limit.yaw   = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;

        // RPY同时受限的特殊提示
        SEND_MULTI_LIMIT_WARNING(rpy_all,
                                 "RPY saturated: scale=%.2f (R:%.2f P:%.2f Y:%.2f)",
                                 rpy_scale,
                                 roll_thrust,
                                 pitch_thrust,
                                 yaw_thrust);
    } else if (thr_adj < 0.0f) {
        // 基座需求 (-rpy_low) 高于飞行员油门：若简单将 thr_adj 置 0，总推力会高于指令→窜高。
        // 改为以飞行员油门为总推力，缩放 RPY 使最低电机刚好≥0，总推力不超指令。
        if ((-rpy_low) > 1e-6f) {
            const float scale2 = MIN(1.0f, throttle_thrust / (-rpy_low));
            for (uint8_t i = 0; i < MotorIndex_COUNT; i++) {
                _rpy_out[i] *= scale2;
            }
        }
        thr_adj = throttle_thrust - throttle_thrust_best_rpy; // 保持 base+thr_adj = throttle_thrust
    } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
        // Throttle 不能提升到期望值
        thr_adj              = 1.0f - (throttle_thrust_best_rpy + rpy_high);
        limit.throttle_upper = true;
        SEND_LIMIT_WARNING(throttle_upper,
                           "Throttle limited: RPY range (req: %.2f, adj: %.2f)",
                           throttle_thrust,
                           thr_adj);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第10层：最终推力输出
    /////////////////////////////////////////////////////////////////////////////////////////////////
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;

    for (uint8_t i = 0; i < MotorIndex_COUNT; i++) {
        _thrust[i] = throttle_thrust_best_plus_adj + _rpy_out[i];
        // 安全约束（正常情况下不应该触发）
        _thrust[i] = constrain_float(_thrust[i], 0.0f, 1.0f);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第11层：倾转角度混合
    /////////////////////////////////////////////////////////////////////////////////////////////////
    float tilt_right_mixed = _tilt_right_bicopter * ahrs_pitch_abs;
    float tilt_rear_mixed  = _tilt_rear_bicopter * ahrs_pitch_abs;
    float tilt_left_mixed  = _tilt_left_bicopter * ahrs_pitch_abs;
    if (get_tilt_enable() != 1) {
        tilt_right_mixed = tilt_rear_mixed = tilt_left_mixed = 0;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第5层：前向力控制
    /////////////////////////////////////////////////////////////////////////////////////////////////
    if (get_tilt_enable()) {
        forward_thrust = _forward_in * _forward_factor;
    } else {
        forward_thrust = 0;
    }

    // 倾转角度混合（Yaw 控制：FR 和 FL 差速倾转，REAR 不参与）
    _tilt_angle_rad[FR] += tilt_right_mixed - forward_thrust * 0.5f + yaw_thrust * 0.5f * _yaw_torque_factor;
    _tilt_angle_rad[FL] += tilt_left_mixed - forward_thrust * 0.5f - yaw_thrust * 0.5f * _yaw_torque_factor;
    _tilt_angle_rad[REAR] += tilt_rear_mixed - forward_thrust * 0.5f;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第12层：记录输出用于谐波陷波滤波器
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // compensation_gain 不会为零
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

# if AP_MOTORS_TRI_TILT_ENABLE_LIMIT_WARNINGS
    // 根据limit标志重置状态（如果限制解除，状态会被重置，下次触发时才能再次发送）
    if (!limit.throttle_lower) {
        RESET_LIMIT_STATE(throttle_lower);
    }
    if (!limit.throttle_upper) {
        RESET_LIMIT_STATE(throttle_upper);
    }
    if (!limit.yaw) {
        RESET_LIMIT_STATE(yaw);
    }
    if (!limit.roll) {
        RESET_LIMIT_STATE(roll);
    }
    if (!limit.pitch) {
        RESET_LIMIT_STATE(pitch);
    }
    // 注意：rpy_all需要检查roll、pitch、yaw都解除
    if (!limit.roll && !limit.pitch && !limit.yaw) {
        RESET_LIMIT_STATE(rpy_all);
    }
# endif

    // CMCU-06A provides measured body-X contact force for impedance mode, so
    // the motor-model estimate below is intentionally disabled.
    // const float F_fr   = (_thrust[FR_UP]   + _thrust[FR_DOWN]);
    // const float F_rear = (_thrust[REAR_UP] + _thrust[REAR_DOWN]);
    // const float F_fl   = (_thrust[FL_UP]   + _thrust[FL_DOWN]);
    //
    // const float F_sum = F_fr + F_rear + F_fl;
    //
    // float Fx_model = F_fr * -sinf(_tilt_angle_rad[FR]) + F_rear * -sinf(_tilt_angle_rad[REAR]) + F_fl * -sinf(_tilt_angle_rad[FL]);
    // _est_body_x_thrust_ratio = Fx_model / MAX(F_sum,  1.0e-4f);

    // static int count = 0;
    // count++;
    // if (count % 400 == 0) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "estimated force: %.2f", _est_body_x_thrust_ratio);
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 输出测试序列
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsTri_Tilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    float _tilt_out_cd;

    // 电机测试（motor_seq 1-6）
    if (motor_seq >= 1 && motor_seq <= 6) {
        // 将测试序号映射到实际电机号
        uint8_t motor_num;
        switch (motor_seq) {
            case 1:
                motor_num = AP_MOTORS_MOT_1;
                break;
            case 2:
                motor_num = AP_MOTORS_MOT_2;
                break;
            case 3:
                motor_num = AP_MOTORS_MOT_3;
                break;
            case 4:
                motor_num = AP_MOTORS_MOT_4;
                break;
            case 5:
                motor_num = AP_MOTORS_MOT_5;
                break;
            case 6:
                motor_num = AP_MOTORS_MOT_6;
                break;
            default:
                return;
        }

        // 输出 PWM 到电机
        if (motor_enabled[motor_num]) {
            rc_write(motor_num, pwm);
        }
        return;
    }

    // 舵机测试（motor_seq 7-9）
    switch (motor_seq) {
        case 7:
            // 前右倾转舵机测试
            _tilt_out_cd = int16_t(constrain_float((pwm - 1500) / 500.0f * 90.0f, -90.0f, 90.0f) * 100);

            // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * _tilt_out_cd);
            // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * _tilt_out_cd);
            servoOutput(FR, _tilt_out_cd);
            servoOutput(FL, _tilt_out_cd);
            // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_servo_fr_rev.get() * 9000);
            // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_servo_fl_rev.get() * 9000);
            break;

        case 8:
            _tilt_out_cd = int16_t(constrain_float((pwm - 1500) / 500.0f * 90.0f, -90.0f, 90.0f) * 100);

            // 后倾转舵机测试
            // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_servo_rear_rev.get() * _tilt_out_cd);
            servoOutput(REAR, _tilt_out_cd);
            break;

        default:
            // 无效的测试序号
            return;
    }
}

#endif // AP_MOTORS_TRI_TILT_ENABLED
