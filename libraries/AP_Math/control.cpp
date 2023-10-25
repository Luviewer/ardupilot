/*
 * control.cpp
 * Copyright (C) Leonard Hall 2020
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  this module provides common controller functions
 */
#include "AP_Math.h"
#include "vector2.h"
#include "vector3.h"
#include <AP_InternalError/AP_InternalError.h>

// control default definitions
#define CORNER_ACCELERATION_RATIO 1.0 / safe_sqrt(2.0) // acceleration reduction to enable zero overshoot corners

// update_vel_accel - single axis projection of velocity, vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// vel_error - specifies the direction of the velocity error used in limit handling.
void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error)
{
    float delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the direction of limit
    // unless adding delta_vel will reduce vel towards zero
    if (is_positive(delta_vel * limit) && is_positive(vel_error * limit)) {
        if (is_negative(vel * limit)) {
            delta_vel = constrain_float(delta_vel, -fabsf(vel), fabsf(vel));
        } else {
            delta_vel = 0.0;
        }
    }
    vel += delta_vel;
}

// update_pos_vel_accel - single axis projection of position and velocity forward in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    float delta_pos = vel * dt + accel * 0.5f * sq(dt);
    // do not add delta_pos if it will increase the velocity error in the direction of limit
    if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
        delta_pos = 0.0;
    }
    pos += delta_pos;

    update_vel_accel(vel, accel, dt, limit, vel_error);
}

// update_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& vel_error)
{
    // increase velocity by acceleration * dt if it does not increase error when limited.
    // unless adding delta_vel will reduce the magnitude of vel
    Vector2f delta_vel = accel * dt;
    if (!limit.is_zero() && !delta_vel.is_zero()) {
        // check if delta_vel will increase the velocity error in the direction of limit
        if (is_positive(delta_vel * limit) && is_positive(vel_error * limit) && !is_negative(vel * limit)) {
            delta_vel.zero();
        }
    }
    vel += delta_vel;
}

// update_pos_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& pos_error, const Vector2f& vel_error)
{
    // move position and velocity forward by dt.
    Vector2f delta_pos = vel * dt + accel * 0.5f * sq(dt);

    if (!is_zero(limit.length_squared())) {
        // zero delta_pos if it will increase the velocity error in the direction of limit
        if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
            delta_pos.zero();
        }
    }

    pos += delta_pos.topostype();

    update_vel_accel_xy(vel, accel, dt, limit, vel_error);
}

/* shape_accel calculates a jerk limited path from the current acceleration to an input acceleration.
 The function takes the current acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    maximum jerk - jerk_max (must be positive).
 The function alters the variable accel to follow a jerk limited kinematic path to accel_input.
*/
void shape_accel(float accel_input, float& accel,
                 float jerk_max, float dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        float accel_delta = accel_input - accel;
        accel_delta       = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel += accel_delta;
    }
}

// 2D version
void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
                    float jerk_max, float dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        Vector2f accel_delta = accel_input - accel;
        accel_delta.limit_length(jerk_max * dt);
        accel = accel + accel_delta;
    }
}

void shape_accel_xy(const Vector3f& accel_input, Vector3f& accel,
                    float jerk_max, float dt)
{
    const Vector2f accel_input_2f { accel_input.x, accel_input.y };
    Vector2f       accel_2f { accel.x, accel.y };

    shape_accel_xy(accel_input_2f, accel_2f, jerk_max, dt);
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

/* shape_vel_accel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    minimum acceleration - accel_min (must be negative),
    maximum acceleration - accel_max (must be positive),
    maximum jerk - jerk_max (must be positive).
 The function alters the variable accel to follow a jerk limited kinematic path to vel_input and accel_input.
 The correction acceleration is limited from accel_min to accel_max. If limit_total is true the target acceleration is limited from accel_min to accel_max.
*/
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_min, accel_max and jerk_max.
    if (!is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // velocity error to be corrected
    float vel_error = vel_input - vel;

    // Calculate time constants and limits to ensure stable operation
    // The direction of acceleration limit is the same as the velocity error.
    // This is because the velocity error is negative when slowing down while
    // closing a positive position error.
    float KPa;
    if (is_positive(vel_error)) {
        KPa = jerk_max / accel_max;
    } else {
        KPa = jerk_max / (-accel_min);
    }

    // acceleration to correct velocity
    float accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // constrain correction acceleration from accel_min to accel_max
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_input;

    // constrain total acceleration from accel_min to accel_max
    if (limit_total_accel) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt);
}

// 2D version
void shape_vel_accel_xy(const Vector2f& vel_input, const Vector2f& accel_input,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max and jerk_max.
    if (!is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = jerk_max / accel_max;

    // velocity error to be corrected
    const Vector2f vel_error = vel_input - vel;

    // acceleration to correct velocity
    Vector2f accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // limit correction acceleration to accel_max
    if (vel_input.is_zero()) {
        accel_target.limit_length(accel_max);
    } else {
        // calculate acceleration in the direction of and perpendicular to the velocity input
        const Vector2f vel_input_unit = vel_input.normalized();
        float          accel_dir      = vel_input_unit * accel_target;
        Vector2f       accel_cross    = accel_target - (vel_input_unit * accel_dir);

        // ensure 1/sqrt(2) of maximum acceleration is available to correct cross component
        // relative to vel_input
        if (sq(accel_dir) <= accel_cross.length_squared()) {
            // accel_target can be simply limited in magnitude
            accel_target.limit_length(accel_max);
        } else {
            // limiting the length of the vector will reduce the lateral acceleration below 1/sqrt(2)
            // limit the lateral acceleration to 1/sqrt(2) and retain as much of the remaining
            // acceleration as possible.
            accel_cross.limit_length(CORNER_ACCELERATION_RATIO * accel_max);
            float accel_max_dir = safe_sqrt(sq(accel_max) - accel_cross.length_squared());
            accel_dir           = constrain_float(accel_dir, -accel_max_dir, accel_max_dir);
            accel_target        = accel_cross + vel_input_unit * accel_dir;
        }
    }

    accel_target += accel_input;

    // limit total acceleration to accel_max
    if (limit_total_accel) {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

/* shape_pos_vel_accel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    minimum velocity - vel_min (must not be positive),
    maximum velocity - vel_max (must not be negative),
    minimum acceleration - accel_min (must be negative),
    maximum acceleration - accel_max (must be positive),
    maximum jerk - jerk_max (must be positive).
 The function alters the variable accel to follow a jerk limited kinematic path to pos_input, vel_input and accel_input.
 The correction velocity is limited to vel_max to vel_min. If limit_total is true the target velocity is limited to vel_max to vel_min.
 The correction acceleration is limited from accel_min to accel_max. If limit_total is true the target acceleration is limited from accel_min to accel_max.
*/
void shape_pos_vel_accel(postype_t pos_input, float vel_input, float accel_input,
                         postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total)
{
    // sanity check vel_min, vel_max, accel_min, accel_max and jerk_max.
    if (is_positive(vel_min) || is_negative(vel_max) || !is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // position error to be corrected
    float pos_error = pos_input - pos;

    // Calculate time constants and limits to ensure stable operation
    // The negative acceleration limit is used here because the square root controller
    // manages the approach to the setpoint. Therefore the acceleration is in the opposite
    // direction to the position error.
    float accel_tc_max;
    float KPv;
    if (is_positive(pos_error)) {
        accel_tc_max = -0.5 * accel_min;
        KPv          = 0.5 * jerk_max / (-accel_min);
    } else {
        accel_tc_max = 0.5 * accel_max;
        KPv          = 0.5 * jerk_max / accel_max;
    }

    // velocity to correct position
    float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity between vel_min and vel_max
    if (is_negative(vel_min) || is_positive(vel_max)) {
        vel_target = constrain_float(vel_target, vel_min, vel_max);
    }

    // velocity correction with input velocity
    vel_target += vel_input;

    // limit velocity between vel_min and vel_max
    if (limit_total) {
        vel_target = constrain_float(vel_target, vel_min, vel_max);
    }

    shape_vel_accel(vel_target, accel_input, vel, accel, accel_min, accel_max, jerk_max, dt, limit_total);
}

// 2D version
void shape_pos_vel_accel_xy(const Vector2p& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total)
{
    // sanity check vel_max, accel_max and jerk_max.
    if (is_negative(vel_max) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = 0.5 * jerk_max / accel_max;
    // reduce breaking acceleration to support cornering without overshooting the stopping point
    const float accel_tc_max = 0.5 * accel_max;

    // position error to be corrected
    Vector2f pos_error = (pos_input - pos).tofloat();

    // velocity to correct position
    Vector2f vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity to vel_max
    if (is_positive(vel_max)) {
        vel_target.limit_length(vel_max);
    }

    // velocity correction with input velocity
    vel_target = vel_target + vel_input;

    // limit velocity to vel_max
    if (limit_total) {
        vel_target.limit_length(vel_max);
    }

    shape_vel_accel_xy(vel_target, accel_input, vel, accel, accel_max, jerk_max, dt, limit_total);
}

/* limit_accel_xy limits the acceleration to prioritise acceleration perpendicular to the provided velocity vector.
 Input parameters are:
    vel is the velocity vector used to define the direction acceleration limit is biased in.
    accel is the acceleration vector to be limited.
    accel_max is the maximum length of the acceleration vector after being limited.
 Returns true when accel vector has been limited.
*/
bool limit_accel_xy(const Vector2f& vel, Vector2f& accel, float accel_max)
{
    // check accel_max is defined
    if (!is_positive(accel_max)) {
        return false;
    }
    // limit acceleration to accel_max while prioritizing cross track acceleration
    if (accel.length_squared() > sq(accel_max)) {
        if (vel.is_zero()) {
            // We do not have a direction of travel so do a simple vector length limit
            accel.limit_length(accel_max);
        } else {
            // calculate acceleration in the direction of and perpendicular to the velocity input
            const Vector2f vel_input_unit = vel.normalized();
            // acceleration in the direction of travel
            float accel_dir = vel_input_unit * accel;
            // cross track acceleration
            Vector2f accel_cross = accel - (vel_input_unit * accel_dir);
            if (accel_cross.limit_length(accel_max)) {
                accel_dir = 0.0;
            } else {
                float accel_max_dir = safe_sqrt(sq(accel_max) - accel_cross.length_squared());
                accel_dir           = constrain_float(accel_dir, -accel_max_dir, accel_max_dir);
            }
            accel = accel_cross + vel_input_unit * accel_dir;
        }
        return true;
    }
    return false;
}

// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
float sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both the P and second order limit have been defined.
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            correction_rate = error * p;
        }
    }
    if (is_positive(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt)
{
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f {};
    }

    const float correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

// inv_sqrt_controller calculates the inverse of the sqrt controller.
// This function calculates the input (aka error) to the sqrt_controller required to achieve a given output.
float inv_sqrt_controller(float output, float p, float D_max)
{
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0 * D_max);
    }
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function.
    const float linear_velocity = D_max / p;

    if (fabsf(output) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return output / p;
    }

    const float linear_dist   = D_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(output) / (2.0 * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

// stopping_distance calculates the stopping distance for the square root controller based deceleration path.
float stopping_distance(float velocity, float p, float accel_max)
{
    return inv_sqrt_controller(velocity, p, accel_max);
}

// kinematic_limit calculates the maximum acceleration or velocity in a given direction.
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if (is_zero(direction.length_squared()) || is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg)) {
        return 0.0;
    }

    max_xy    = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    direction.normalize();
    const float xy_length = Vector2f { direction.x, direction.y }.length();

    if (is_zero(xy_length)) {
        return is_positive(direction.z) ? max_z_pos : max_z_neg;
    }

    if (is_zero(direction.z)) {
        return max_xy;
    }

    const float slope = direction.z / xy_length;
    if (is_positive(slope)) {
        if (fabsf(slope) < max_z_pos / max_xy) {
            return max_xy / xy_length;
        }
        return fabsf(max_z_pos / direction.z);
    }

    if (fabsf(slope) < max_z_neg / max_xy) {
        return max_xy / xy_length;
    }
    return fabsf(max_z_neg / direction.z);
}

// 计算输入值的"曲线响应"（expo function）。
// "曲线响应"是为了改变输入信号的响应特性，通常用于调整飞行器的控制性能，使其更加灵敏或平稳
// input_expo 计算输入值的曲线响应。
// 输入值必须在 -1 到 1 的范围内。
// 曲线响应的程度应小于 1.0，但限制在小于 0.95。
float input_expo(float input, float expo)
{
    // 将输入值限制在 -1 到 1 的范围内
    input = constrain_float(input, -1.0, 1.0);

    // 如果曲线响应的程度小于 0.95
    if (expo < 0.95) {
        // 计算曲线响应后的输出值
        return (1 - expo) * input / (1 - expo * fabsf(input));
    }

    // 如果曲线响应的程度不小于 0.95，直接返回原始输入值
    return input;
}

// angle_to_accel 将最大倾斜角度（以度为单位）转换为最大加速度限制（以 m/s² 为单位）
float angle_to_accel(float angle_deg)
{
    // 使用标准重力加速度（以 m/s² 为单位）和 tan 函数将角度转换为加速度
    return GRAVITY_MSS * tanf(radians(angle_deg));
}

// accel_to_angle 将最大加速度（以 m/s² 为单位）转换为最大倾斜角度（以度为单位）
float accel_to_angle(float accel)
{
    // 使用 atan 和 degrees 函数将加速度转换为角度
    return degrees(atanf((accel / GRAVITY_MSS)));
}

// 这段代码执行了从标准化滚动和俯仰遥杆输入到滚动和俯仰的欧拉角命令的转换，同时考虑了最大倾斜角度和可调的角度限制。
// 这对于将飞行员的输入转化为飞行器的控制命令非常重要，以确保飞行器以安全和合适的方式响应飞行员的操作。
// rc_input_to_roll_pitch - 将飞行员的标准化滚动或俯仰遥杆输入转换为滚动和俯仰的欧拉角命令
// roll_in_unit 和 pitch_in_unit - 是标准化的滚动和俯仰遥杆输入
// angle_max_deg - 允许的最大倾斜角度，相对于飞行器的Z轴
// angle_limit_deg - 提供减小最大输出倾斜角至小于angle_max_deg的能力
// 返回滚动和俯仰角度，单位为度
void rc_input_to_roll_pitch(float roll_in_unit, float pitch_in_unit, float angle_max_deg, float angle_limit_deg, float& roll_out_deg, float& pitch_out_deg)
{
    // 限制angle_max_deg不超过85度
    angle_max_deg = MIN(angle_max_deg, 85.0);

    // 将遥杆输入转换为弧度（radians）表示
    float rc_2_rad = radians(angle_max_deg);

    // 获取滚动和俯仰遥杆位置并将它们转换为标准化的水平推力
    Vector2f thrust;
    thrust.x = -tanf(rc_2_rad * pitch_in_unit);
    thrust.y = tanf(rc_2_rad * roll_in_unit);

    // 基于角度限制计算水平推力的极限
    angle_limit_deg    = constrain_float(angle_limit_deg, 10.0f, angle_max_deg);
    float thrust_limit = tanf(radians(angle_limit_deg));

    // 应用水平推力限制
    thrust.limit_length(thrust_limit);

    // 从角推力矢量转换为欧拉角度
    float pitch_rad = -atanf(thrust.x);
    float roll_rad  = atanf(cosf(pitch_rad) * thrust.y);

    // 转换为度数表示
    roll_out_deg  = degrees(roll_rad);
    pitch_out_deg = degrees(pitch_rad);
}