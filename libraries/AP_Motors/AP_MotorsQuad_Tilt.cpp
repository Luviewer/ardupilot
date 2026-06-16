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
   共轴X型四臂八桨（每臂上CCW、下CW）

   电机布局（俰视）：
        前(+X)
    LF       RF
    LR       RR
        后(-X)

   臂编号顺序：RF→RR→LR→LF
   - MOT1/MOT2：右前上（CCW）/下（CW）
   - MOT3/MOT4：右后上（CCW）/下（CW）
   - MOT5/MOT6：左后上（CCW）/下（CW）
   - MOT7/MOT8：左前上（CCW）/下（CW）

   偏航：共轴对上下差速（正偏航增大所有CCW上桨、减小所有CW下桨）
   前飞/侧飞：全臂倾转
*/

#include "AP_Motors_config.h"

#if AP_MOTORS_QUAD_TILT_ENABLED

# include <AP_HAL/AP_HAL.h>
# include <AP_Math/AP_Math.h>
# include <AP_Vehicle/AP_Vehicle_Type.h>
# include <GCS_MAVLink/GCS.h>

# include "AP_MotorsQuad_Tilt.h"
# include <AP_AHRS/AP_AHRS_View.h>
# include <AP_InertialSensor/AP_InertialSensor.h>
# define AP_MOTORS_QUAD_TILT_USE_113E6D0_ALLOC

extern const AP_HAL::HAL& hal;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 宏定义：限制警告提示（宏定义在头文件中）
////////////////////////////////////////////////////////////////////////////////////////////////////////////
# if AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS
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
const AP_Param::GroupInfo AP_MotorsQuad_Tilt::var_info[] = {
    // 先链入父类 Multicopter 参数（MOT_YAW_HEADROOM、MOT_THST_EXPO 等），再接本类自定义参数
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: QUAD_TILT_ANG_MAX
    // @DisplayName: 最大倾转角
    // @Description: 转子允许的最大倾转角（度）
    // @Range: 0 135
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("TILT_ANG_MAX", 4, AP_MotorsQuad_Tilt, _servo_angle_max, 135.0f),

    // @Param: QUAD_TILT_YAW_FAC
    // @DisplayName: 偏航力矩因子
    // @Description: 共轴对差分推力产生偏航力矩的比例因子
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("TILT_YAW_FAC", 8, AP_MotorsQuad_Tilt, _yaw_torque_factor, 1.0f),

    // @Param: QUAD_TILT_SVO_FR_REV
    // @DisplayName: 右前倾转舵机反向
    // @Description: 设为 1 以反向右前倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_RF_REV", 11, AP_MotorsQuad_Tilt, _tilt_servo_rf_rev, 0),

    // @Param: QUAD_TILT_SVO_REAR_REV
    // @DisplayName: 右后倾转舵机反向
    // @Description: 设为 1 以反向右后倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_RR_REV", 12, AP_MotorsQuad_Tilt, _tilt_servo_rr_rev, 0),

    // @Param: QUAD_TILT_SVO_LR_REV
    // @DisplayName: 左后倾转舵机反向
    // @Description: 设为 1 以反向左后倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_LR_REV", 24, AP_MotorsQuad_Tilt, _tilt_servo_lr_rev, 0),

    // @Param: QUAD_TILT_SVO_FL_REV
    // @DisplayName: 左前倾转舵机反向
    // @Description: 设为 1 以反向左前倾转舵机方向
    // @Values: 0:Normal, 1:Reversed
    // @User: Advanced
    AP_GROUPINFO("SVO_LF_REV", 13, AP_MotorsQuad_Tilt, _tilt_servo_lf_rev, 0),

    // @Param: SVO_FR_OFF
    // @DisplayName: Front-right tilt servo offset
    // @Description: Front-right tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_RF_OFF", 17, AP_MotorsQuad_Tilt, _svo_rf_offset, 0),

    // @Param: SVO_REAR_OFF
    // @DisplayName: Rear tilt servo offset
    // @Description: Rear tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_RR_OFF", 18, AP_MotorsQuad_Tilt, _svo_rr_offset, 0),

    // @Param: SVO_FL_OFF
    // @DisplayName: Front-left tilt servo offset
    // @Description: Front-left tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_LF_OFF", 19, AP_MotorsQuad_Tilt, _svo_lf_offset, 0),

    // @Param: SVO_LR_OFF
    // @DisplayName: Rear-left tilt servo offset
    // @Description: Rear-left tilt servo offset
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("SVO_LR_OFF", 25, AP_MotorsQuad_Tilt, _svo_lr_offset, 0),

    // @Param: QUAD_TILT_PIT_OFF_MAX
    // @DisplayName: 最大俯仰姿态偏置（RC7）
    // @Description: RC7 指令的最大俯仰姿态偏置（度）（1500->0，1000->-max，2000->+max）
    // @Range: 0 45
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("PIT_OFF_MAX", 14, AP_MotorsQuad_Tilt, _tilt_pitch_off_max_deg, 20.0f),

    // @Param: TILT_EN
    // @DisplayName: Tilt enable
    // @Description: Tilt enable
    // @Values: 0:Disable, 1:Enable
    // @User: Advanced
    AP_GROUPINFO("TILT_EN", 15, AP_MotorsQuad_Tilt, _tilt_enable, 1),

    // @Param: FORWARD_FACTOR
    // @DisplayName: Forward factor
    // @Description: Forward factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("FORW_FACT", 16, AP_MotorsQuad_Tilt, _forward_factor, 1),

    // @Param: ANTI_YAW_FAC
    // @DisplayName: Anti-yaw factor
    // @Description: Anti-yaw factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("ANTI_YAW_FAC", 20, AP_MotorsQuad_Tilt, _anti_yaw_factor, 1),

    // @Param: PITCH_FAC
    // @DisplayName: Pitch factor
    // @Description: Pitch factor for bicopter tilt control
    // @Range: 1.0~2.0
    // @User: Advanced
    AP_GROUPINFO("BI_PIT_FAC", 21, AP_MotorsQuad_Tilt, _bicopter_pitch_P_factor, 0.0f),

    // @Param: LAT_FACT
    // @DisplayName: Lateral factor
    // @Description: Lateral factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("LAT_FACT", 22, AP_MotorsQuad_Tilt, _lateral_factor, 1),

    // @Param: LAT_FACT
    // @DisplayName: Lateral factor
    // @Description: Lateral factor
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("LAT_ENABLE", 23, AP_MotorsQuad_Tilt, _lateral_enable, 1),


    AP_GROUPEND
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 启用 8 电机的共轴 X 型配置
    add_motor_num(AP_MOTORS_MOT_1); // 右前上（CCW）
    add_motor_num(AP_MOTORS_MOT_2); // 右前下（CW）
    add_motor_num(AP_MOTORS_MOT_3); // 右后上（CCW）
    add_motor_num(AP_MOTORS_MOT_4); // 右后下（CW）
    add_motor_num(AP_MOTORS_MOT_5); // 左后上（CCW）
    add_motor_num(AP_MOTORS_MOT_6); // 左后下（CW）
    add_motor_num(AP_MOTORS_MOT_7); // 左前上（CCW）
    add_motor_num(AP_MOTORS_MOT_8); // 左前下（CW）

    // 设置电机更新频率
    set_update_rate(_speed_hz);

    // 标记电机启用（用于校准）
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_5] = true;
    motor_enabled[AP_MOTORS_MOT_6] = true;
    motor_enabled[AP_MOTORS_MOT_7] = true;
    motor_enabled[AP_MOTORS_MOT_8] = true;

    // 设置默认电机与舵机映射（可通过 SERVOx_FUNCTION 覆盖）
    // 映射：RF->右前，RR->右后，LR->左后，LF->左前
    const bool ok_rf = SRV_Channels::set_aux_channel_default(SRV_Channel::k_actuator1,    AP_MOTORS_QUAD_TILT_SERVO_RF);
    const bool ok_rr = SRV_Channels::set_aux_channel_default(SRV_Channel::k_actuator2,     AP_MOTORS_QUAD_TILT_SERVO_RR);
    const bool ok_lr = SRV_Channels::set_aux_channel_default(SRV_Channel::k_actuator3, AP_MOTORS_QUAD_TILT_SERVO_LR);
    const bool ok_lf = SRV_Channels::set_aux_channel_default(SRV_Channel::k_actuator4,     AP_MOTORS_QUAD_TILT_SERVO_LF);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_actuator5, CH_13);

    const float    ang_max_deg    = 270.0f / 2.0f;
    const uint16_t servo_range_cd = uint16_t(ang_max_deg * 100.0f);

    SRV_Channels::set_angle(SRV_Channel::k_actuator1,   servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_actuator2,   servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_actuator3,   servo_range_cd);
    SRV_Channels::set_angle(SRV_Channel::k_actuator4,   servo_range_cd);

    SRV_Channels::set_angle(SRV_Channel::k_actuator5,   servo_range_cd);

    // 设置倾转舵机默认 PWM 范围（硬件：500~2500us）
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_actuator1, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_actuator2, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_actuator3, 500, 2500);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_actuator4, 500, 2500);

    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_actuator5, 500, 2500);

    // 检查舵机是否已分配（默认或用户映射）
    _servos_assigned = (ok_rf || SRV_Channels::function_assigned(SRV_Channel::k_actuator1))
        && (ok_rr || SRV_Channels::function_assigned(SRV_Channel::k_actuator2))
        && (ok_lr || SRV_Channels::function_assigned(SRV_Channel::k_actuator3))
        && (ok_lf || SRV_Channels::function_assigned(SRV_Channel::k_actuator4));

    // 配置电机与分配矩阵
    setup_motors(frame_class, frame_type);

    _mav_type = MAV_TYPE_COAXIAL;

    // 记录初始化成功
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI && _servos_assigned);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 设置机架类别与类型
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
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
void AP_MotorsQuad_Tilt::set_update_rate(uint16_t speed_hz)
{
    // 记录请求的频率
    _speed_hz = speed_hz;

    // 为全部 8 个电机设置更新频率
    uint32_t mask = 1U << AP_MOTORS_MOT_1
        | 1U << AP_MOTORS_MOT_2
        | 1U << AP_MOTORS_MOT_3
        | 1U << AP_MOTORS_MOT_4
        | 1U << AP_MOTORS_MOT_5
        | 1U << AP_MOTORS_MOT_6
        | 1U << AP_MOTORS_MOT_7
        | 1U << AP_MOTORS_MOT_8;

    rc_set_freq(mask, _speed_hz);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 配置电机 - 设置静态分配矩阵
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
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
    motor_enabled[AP_MOTORS_MOT_7] = true;
    motor_enabled[AP_MOTORS_MOT_8] = true;

    // 矩阵清零
    memset(_thrust, 0, sizeof(_thrust));
    memset(_tilt_angle_rad, 0, sizeof(_tilt_angle_rad));
    memset(_rpy_out, 0, sizeof(_rpy_out));

    _frame_class_string = "QUAD_TILT";
    _frame_type_string  = "Coaxial-X8";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 获取电机掩码
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t AP_MotorsQuad_Tilt::get_motor_mask()
{
    // 只返回电机 ESC 输出通道。倾转舵机没有 ESC 遥测，不能并入此掩码，
    // 否则升空 RPM 检查 (motors_takeoff_check) 会永远等不到舵机通道的 ESC 转速。
    // 舵机的安全开关行为由 BRD_SAFETY_MASK 单独控制。
    return AP_MotorsMatrix::get_motor_mask();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 执行解锁检查
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AP_MotorsQuad_Tilt::arming_checks(size_t buflen, char* buffer) const
{
    // 检查舵机是否已分配
    if (!_servos_assigned) {
        hal.util->snprintf(buffer, buflen, "QUAD_TILT: Servos not assigned");
        return false;
    }

    // 检查角度限制：
    // _servo_angle_max == 0 表示“无软件限幅”（仍受 SERVOx_MIN/MAX 约束）。
    // 否则强制一个最小值，避免范围过小导致过度缩放/饱和。
    const float ang_max = _servo_angle_max;
    if ((!is_zero(ang_max) && ang_max < AP_MOTORS_QUAD_TILT_ANGLE_MIN) || ang_max < 0.0f || ang_max > AP_MOTORS_QUAD_TILT_ANGLE_MAX) {
        hal.util->snprintf(buffer, buflen, "QUAD_TILT: Invalid angle limits");
        return false;
    }

    return true;
}

void AP_MotorsQuad_Tilt::servoOutput(enum TiltIndex servo_index, float svo_out_cd)
{
    if (servo_index == RF) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_actuator1,    (float)_tilt_servo_rf_rev.get() * svo_out_cd + (float)_svo_rf_offset.get());
    } else if (servo_index == RR) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_actuator2,     (float)_tilt_servo_rr_rev.get() * svo_out_cd + (float)_svo_rr_offset.get());
    } else if (servo_index == LR) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_actuator3, (float)_tilt_servo_lr_rev.get() * svo_out_cd + (float)_svo_lr_offset.get());
    } else if (servo_index == LF) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_actuator4,     (float)_tilt_servo_lf_rev.get() * svo_out_cd + (float)_svo_lf_offset.get());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_to_motors - 发送电机与舵机指令
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_to_motors - 发送电机与舵机指令
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // 电机输出最小値
            for (uint8_t i = 0; i < 8; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }

        case SpoolState::GROUND_IDLE: {
            // 解锁但未起飞时给电机输出
            float spin_up = actuator_spin_up_to_ground_idle();
            for (uint8_t i = 0; i < 8; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], spin_up);
                }
            }
            break;
        }

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            for (uint8_t i = 0; i < 8; i++) {
                set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust[i]));
            }
            break;
    }

    for (uint8_t i = 0; i < 8; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }

    const float lim_deg = (_servo_angle_max > 0.0f) ? float(_servo_angle_max) : float(AP_MOTORS_QUAD_TILT_ANGLE_MAX);
    const int16_t rf_out_cd = int16_t(constrain_float(degrees(_tilt_angle_rad[RF]), -lim_deg, lim_deg) * 100);
    const int16_t rr_out_cd = int16_t(constrain_float(degrees(_tilt_angle_rad[RR]), -lim_deg, lim_deg) * 100);
    const int16_t lr_out_cd = int16_t(constrain_float(degrees(_tilt_angle_rad[LR]), -lim_deg, lim_deg) * 100);
    const int16_t lf_out_cd = int16_t(constrain_float(degrees(_tilt_angle_rad[LF]), -lim_deg, lim_deg) * 100);

    servoOutput(RF, rf_out_cd);
    servoOutput(RR, rr_out_cd);
    servoOutput(LR, lr_out_cd);
    servoOutput(LF, lf_out_cd);

    SRV_Channels::set_output_scaled(SRV_Channel::k_actuator5, (float)AP::ins().get_imu_pitch_rot_deg()*100);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 推力补偿
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::thrust_compensation(void)
{
    // 调用父类推力补偿
    AP_MotorsMatrix::thrust_compensation();
}

// sets the roll and pitch offset, this rotates the thrust vector in body frame
// these are typically set such that the throttle thrust vector is earth frame up
void AP_MotorsQuad_Tilt::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset  = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

float AP_MotorsQuad_Tilt::get_bicopter_pitch_P_factor()
{
    float ahrs_pitch_abs = constrain_value(fabsf(AP::ins().get_imu_pitch_rot_deg() / 90.0f), 0.0f, 1.0f);
    return constrain_value(_bicopter_pitch_P_factor.get() * ahrs_pitch_abs, -0.1f, 0.1f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// output_armed_stabilizing - 主要控制分配
// X型四臂共轴8桨：RPY通道分配 + 倾转角控制前飞/侧飞
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::output_armed_stabilizing()
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
    // 第3层：X型四臂 RPY 控制分配
    //
    // 机架布局（俯视）：
    //        前(+X)
    //    LF       RF
    //    LR       RR
    //        后(-X)
    //
    // Roll：正Roll=右侧下沉 -> 右臂推力减，左臂增
    //   RF/RR: -roll * 0.5, LR/LF: +roll * 0.5
    // Pitch：正Pitch=机头抬起 -> 前臂增，后臂减
    //   RF/LF: +pitch * 0.5, RR/LR: -pitch * 0.5
    // Yaw：上桨均为CCW，下桨均为CW
    //   正偏航(机头右转)=增大CCW扭矩=上桨全部增大，下桨全部减小
    //   所有UP: +yaw * 0.5, 所有DOWN: -yaw * 0.5
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 初始化倾转角度
    if (get_tilt_enable()) {
        _tilt_angle_rad[RF] = _tilt_angle_rad[RR] = _tilt_angle_rad[LR] = _tilt_angle_rad[LF] = -radians(AP::ins().get_imu_pitch_rot_deg());
    } else {
        _tilt_angle_rad[RF] = _tilt_angle_rad[RR] = _tilt_angle_rad[LR] = _tilt_angle_rad[LF] = 0;
    }

    // 4旋翼 RPY 控制分配（不含 throttle）
    _thrust_tricopter[RF] = roll_thrust * -0.5f + pitch_thrust *  0.5f;
    _thrust_tricopter[RR] = roll_thrust * -0.5f + pitch_thrust * -0.5f;
    _thrust_tricopter[LR] = roll_thrust *  0.5f + pitch_thrust * -0.5f;
    _thrust_tricopter[LF] = roll_thrust *  0.5f + pitch_thrust *  0.5f;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第4层：双旋翼控制分配
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 双旋翼推力分配（不含 throttle）
    _thrust_bicopter[RF] = -roll_thrust * 0.5f;
    _thrust_bicopter[RR] = -roll_thrust * 0.5f;
    _thrust_bicopter[LR] = roll_thrust * 0.5f;
    _thrust_bicopter[LF] = roll_thrust * 0.5f;

    // 双旋翼倾转控制
    _tilt_bicopter[RF] = pitch_thrust * 0.5f * sign_ahrs_pitch;
    _tilt_bicopter[RR] = -pitch_thrust * 0.5f * sign_ahrs_pitch;
    _tilt_bicopter[LR] = -pitch_thrust * 0.5f * sign_ahrs_pitch;
    _tilt_bicopter[LF] = pitch_thrust * 0.5f * sign_ahrs_pitch;

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
# if AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS
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
    // 右前（0,1）：上 CW 下 CCW
    float thrust_right_mixed = _thrust_tricopter[RF] * (1.0f - ahrs_pitch_abs) + _thrust_bicopter[RF] * ahrs_pitch_abs;
    _rpy_out[RF_UP]          = thrust_right_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[RF_DOWN]        = thrust_right_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;

    // 右后（2,3）：上 CW 下 CCW
    float thrust_rear_mixed = _thrust_tricopter[RR] * (1.0f - ahrs_pitch_abs) + _thrust_bicopter[RR] * ahrs_pitch_abs;
    _rpy_out[RR_UP]       = thrust_rear_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[RR_DOWN]     = thrust_rear_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;

    // 左后（4,5）：上 CW 下 CCW，yaw 符号与前右/后相反
    float thrust_front_mixed = _thrust_tricopter[LR] * (1.0f - ahrs_pitch_abs) + _thrust_bicopter[LR] * ahrs_pitch_abs;
    _rpy_out[LR_UP]         = thrust_front_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[LR_DOWN]       = thrust_front_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;

    // 左前（6,7）：上 CW 下 CCW，yaw 符号与前右/后相反
    float thrust_left_mixed = _thrust_tricopter[LF] * (1.0f - ahrs_pitch_abs) + _thrust_bicopter[LF] * ahrs_pitch_abs;
    _rpy_out[LF_UP]         = thrust_left_mixed - yaw_thrust * 0.5f * _anti_yaw_factor;
    _rpy_out[LF_DOWN]       = thrust_left_mixed + yaw_thrust * 0.5f * _anti_yaw_factor;


    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第12层：侧向力控制
    /////////////////////////////////////////////////////////////////////////////////////////////////
    if (_lateral_enable == 1) {
        lateral_thrust = _lateral_in;

        // 侧向力混合
        _rpy_out[RF_UP] += lateral_thrust * _lateral_factor;
        _rpy_out[RF_DOWN] += lateral_thrust * _lateral_factor;

        _rpy_out[RR_UP] += lateral_thrust * _lateral_factor;
        _rpy_out[RR_DOWN] -= lateral_thrust * _lateral_factor;

        _rpy_out[LR_UP] -= lateral_thrust * _lateral_factor;
        _rpy_out[LR_DOWN] += lateral_thrust * _lateral_factor;

        _rpy_out[LF_UP] -= lateral_thrust * _lateral_factor;
        _rpy_out[LF_DOWN] += lateral_thrust * _lateral_factor;
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
    tilt_mixed[RF] = _tilt_bicopter[RF] * ahrs_pitch_abs;
    tilt_mixed[RR] = _tilt_bicopter[RR] * ahrs_pitch_abs;
    tilt_mixed[LR] = _tilt_bicopter[LR] * ahrs_pitch_abs;
    tilt_mixed[LF] = _tilt_bicopter[LF] * ahrs_pitch_abs;
    if (get_tilt_enable() != 1) {
        tilt_mixed[RF] = tilt_mixed[RR] = tilt_mixed[LR] = tilt_mixed[LF] = 0;
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
    _tilt_angle_rad[RF] += tilt_mixed[RF] - forward_thrust * 0.5f + yaw_thrust * 0.5f * _yaw_torque_factor;
    _tilt_angle_rad[RR] += tilt_mixed[RR] - forward_thrust * 0.5f + yaw_thrust * 0.5f * _yaw_torque_factor;
    _tilt_angle_rad[LR] += tilt_mixed[LR] - forward_thrust * 0.5f - yaw_thrust * 0.5f * _yaw_torque_factor;
    _tilt_angle_rad[LF] += tilt_mixed[LF] - forward_thrust * 0.5f - yaw_thrust * 0.5f * _yaw_torque_factor;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 第12层：记录输出用于谐波陷波滤波器
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // compensation_gain 不会为零
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

# if AP_MOTORS_QUAD_TILT_ENABLE_LIMIT_WARNINGS
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

    const float F_rf = (_thrust[RF_UP]   + _thrust[RF_DOWN]);
    const float F_rr = (_thrust[RR_UP]   + _thrust[RR_DOWN]);
    const float F_lr = (_thrust[LR_UP]   + _thrust[LR_DOWN]);
    const float F_lf = (_thrust[LF_UP]   + _thrust[LF_DOWN]);

    const float F_sum = F_rf + F_rr + F_lr + F_lf;

    float Fx_model = F_rf * -sinf(_tilt_angle_rad[RF]) + F_rr * -sinf(_tilt_angle_rad[RR]) + F_lr * -sinf(_tilt_angle_rad[LR]) + F_lf * -sinf(_tilt_angle_rad[LF]);
    _est_body_x_thrust_ratio = Fx_model / MAX(F_sum,  1.0e-4f);

    // static int count = 0;
    // count++;
    // if (count % 400 == 0) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "estimated force: %.2f", _est_body_x_thrust_ratio);
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 输出测试序列
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AP_MotorsQuad_Tilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    float tilt_out_cd;

    // 电机测试（motor_seq 1-8）
    if (motor_seq >= 1 && motor_seq <= 8) {
        uint8_t motor_num;
        switch (motor_seq) {
            case 1:  motor_num = AP_MOTORS_MOT_1; break; // 右前上
            case 2:  motor_num = AP_MOTORS_MOT_2; break; // 右前下
            case 3:  motor_num = AP_MOTORS_MOT_3; break; // 右后上
            case 4:  motor_num = AP_MOTORS_MOT_4; break; // 右后下
            case 5:  motor_num = AP_MOTORS_MOT_5; break; // 左后上
            case 6:  motor_num = AP_MOTORS_MOT_6; break; // 左后下
            case 7:  motor_num = AP_MOTORS_MOT_7; break; // 左前上
            case 8:  motor_num = AP_MOTORS_MOT_8; break; // 左前下
            default: return;
        }
        if (motor_enabled[motor_num]) {
            rc_write(motor_num, pwm);
        }
        // return;
    }

    // 舵机测试（motor_seq 9-12）
    tilt_out_cd = int16_t(constrain_float((pwm - 1500) / 500.0f * 90.0f, -90.0f, 90.0f) * 100);
    switch (motor_seq) {
        case 5: servoOutput(RF, tilt_out_cd); break; // 右前倾转
        case 6: servoOutput(RR, tilt_out_cd); break; // 右后倾转
        case 7: servoOutput(LR, tilt_out_cd); break; // 左后倾转
        case 8: servoOutput(LF, tilt_out_cd); break; // 左前倾转
        default: return;
    }
}

#endif // AP_MOTORS_QUAD_TILT_ENABLED
