/*
   天蝎座三旋翼:等腰三角形,三个旋翼各带一个独立倾转舵机。

   安装序号(俯视,前右 / 后 / 前左;代码数组 [0]/[1]/[2] 同此顺序):
     1 前右: Motor1 + k_tiltMotorRight
     2 后:   Motor2 + k_tiltMotorRear
     3 前左: Motor3 + k_tiltMotorLeft
   实际引脚由 SERVOx_FUNCTION 映射,代码不写死通道。

   旋翼转向(俯视,推力向下;前两个同向、后反向):
     前右 CCW   后 CW   前左 CCW

   舵机正旋向(输出正角度时旋翼倾倒的方向,与机械安装一致):
     前右:往后偏(后偏右)   后:向左倒   前左:往前偏(前偏右)
   这三个正方向构成右偏航力偶,偏航按 (+1, +1, +1) 分配。

   电机推力混控沿用 AP_MotorsTri,横滚/俯仰完全由电机差速实现;
   倾转舵机只做偏航,偏航量直接当舵量,满偏对应 MOT_SC_SV_ANG,
   不做 sin/asin 映射。平移靠机体倾斜(普通多旋翼逻辑),不用 6DoF。
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_MotorsScorpio.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsScorpio::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // 参数编号 1~5 和 8~20 由早期天蝎座实现保留占用,不要复用
    // 参数编号 6 由已删除的 SC_F_ANG 保留:倾转舵机只做偏航后,
    // 前倾转轴安装角不再参与混控
    // 参数编号 7 由已删除的 SC_R_ANG 保留:等腰三角形构型下
    // 后旋翼在中轴线上,倾转方向固定为纯横向,无需参数

    // @Param: SC_FR_REV
    // @DisplayName: Scorpio front-right tilt servo reverse
    // @Description: Reverse the direction of the front-right tilt servo, applied to both flight output and motor test
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    // 前右倾转舵机反相:反转前右倾转舵机的方向,飞行输出和电机测试同时生效(0 不反向,1 反向)
    AP_GROUPINFO("SC_FR_REV", 21, AP_MotorsScorpio, _fr_tilt_reverse, 0),

    // @Param: SC_RR_REV
    // @DisplayName: Scorpio rear tilt servo reverse
    // @Description: Reverse the direction of the rear tilt servo, applied to both flight output and motor test
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    // 后倾转舵机反相:反转后倾转舵机的方向,飞行输出和电机测试同时生效(0 不反向,1 反向)
    AP_GROUPINFO("SC_RR_REV", 22, AP_MotorsScorpio, _rr_tilt_reverse, 0),

    // @Param: SC_FL_REV
    // @DisplayName: Scorpio front-left tilt servo reverse
    // @Description: Reverse the direction of the front-left tilt servo, applied to both flight output and motor test
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    // 前左倾转舵机反相:反转前左倾转舵机的方向,飞行输出和电机测试同时生效(0 不反向,1 反向)
    AP_GROUPINFO("SC_FL_REV", 23, AP_MotorsScorpio, _fl_tilt_reverse, 0),

    // @Param: SC_R_THST
    // @DisplayName: Scorpio rear thrust ratio
    // @Description: Hover thrust of the rear motor divided by hover thrust of each front motor. Pre-loads the collective throttle split so the vehicle lifts off balanced instead of waiting for the pitch integrator. Measure from a hover log, 1.0 disables the compensation
    // @Range: 0.5 3.0
    // @User: Standard
    // 后旋翼推力比:后电机悬停推力除以单个前电机悬停推力。用于预置集体油门的分配比例,
    // 使起飞瞬间即接近配平,不必等俯仰积分器慢慢建立。从悬停日志实测得到,设 1.0 关闭该补偿
    AP_GROUPINFO("SC_R_THST", 24, AP_MotorsScorpio, _rear_thrust_ratio, 1.55f),

    // @Param: SC_SV_ANG
    // @DisplayName: Scorpio tilt servo max angle
    // @Description: Maximum tilt angle of each rotor when the mixer asks for full deflection. Maps scaled output +/- this angle to SERVOx_MIN/MAX
    // @Units: deg
    // @Range: 5 90
    // @User: Standard
    // 天蝎座倾转舵机最大角度:混控满偏时旋翼的最大倾转角,±此角对应 SERVOx_MIN/MAX
    AP_GROUPINFO("SC_SV_ANG", 25, AP_MotorsScorpio, _servo_angle_max_deg, 30.0f),

    AP_GROUPEND
};

// 声明三个倾转舵机的角度行程(使用天蝎座自己的 MOT_SC_SV_ANG)。
// 不设默认通道,不写死引脚。
void AP_MotorsScorpio::setup_tilt_servos()
{
    // SRV_Channel angle unit is centidegrees, so convert deg * 100 (30 deg -> 3000)
    // 舵机库角度单位是百分之一度,度要乘 100(30° → 3000),后面 set_output_scaled 同样 *100
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, _servo_angle_max_deg*100);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, _servo_angle_max_deg*100);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, _servo_angle_max_deg*100);

    // 后/前悬停推力比 → 集体油门分配因子,份额最大的一路归一化到 1,
    // 保证任何油门下单电机推力不超上限
    const float ratio = constrain_float(_rear_thrust_ratio, 0.5f, 3.0f);
    if (ratio >= 1.0f) {
        _throttle_factor_rear = 1.0f;
        _throttle_factor_front = 1.0f / ratio;
    } else {
        _throttle_factor_front = 1.0f;
        _throttle_factor_rear = ratio;
    }
}

// init
// 初始化
void AP_MotorsScorpio::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);

    // set update rate for the 3 motors (but not the tilt servos)
    // 设置三个电机的更新频率(不含倾转舵机)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    // 置位 motor_enabled 标志,使电调可以像其他机架一样校准
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;

    setup_tilt_servos();

    _mav_type = MAV_TYPE_TRICOPTER;

    // record successful initialisation if what we setup was the desired frame_class
    // 机架类别匹配时才记录初始化成功
    set_initialised_ok(frame_class == MOTOR_FRAME_SCORPIO);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
// 设置机架类别(四/六旋翼等)与类型(X、+ 等),本机架忽略类型
void AP_MotorsScorpio::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(frame_class == MOTOR_FRAME_SCORPIO);
}

// set update rate to motors - a value in hertz
// 设置电机更新频率,单位 Hz
void AP_MotorsScorpio::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    // 记录请求的频率
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the tilt servos)
    // 设置三个电机的更新频率(不含倾转舵机)
    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsScorpio::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            // 向电机输出最小值；倾转舵机保留当前混控角度，未解锁时也可检查和预置舵机
            _actuator[AP_MOTORS_MOT_1] = 0.0f;
            _actuator[AP_MOTORS_MOT_2] = 0.0f;
            _actuator[AP_MOTORS_MOT_3] = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            // 已解锁但未起飞时输出地面怠速；倾转舵机继续响应混控输入
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_3], actuator_spin_up_to_ground_idle());
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            // 按推力请求设置电机输出
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], thr_lin.thrust_to_actuator(_thrust_rear));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_3], thr_lin.thrust_to_actuator(_thrust_left));
            break;
    }

    rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
    rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[AP_MOTORS_MOT_2]));
    rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[AP_MOTORS_MOT_3]));
    // 反相参数:1 时输出角度取反
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, degrees(_servo_angle[0])*100 * (_fr_tilt_reverse ? -1.0f : 1.0f));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, degrees(_servo_angle[1])*100 * (_rr_tilt_reverse ? -1.0f : 1.0f));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, degrees(_servo_angle[2])*100 * (_fl_tilt_reverse ? -1.0f : 1.0f));
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
// 返回电机/舵机占用的输出通道位掩码(1 表示占用)
//  可用于避免其他 PWM 输出(如普通舵机)与之冲突
uint32_t AP_MotorsScorpio::get_motor_mask()
{
    uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2) |
                          (1U << AP_MOTORS_MOT_3);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    // 合并父类的掩码
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// 向电机发送指令,包含新的缩放稳定性补丁
void AP_MotorsScorpio::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
                                        // 横滚推力输入,±1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
                                        // 俯仰推力输入,±1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
                                        // 偏航推力输入,±1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
                                        // 油门推力输入,0.0~1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
                                        // 油门推力平均上限,0.0~1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
                                        // 在不爬升的前提下给横滚/俯仰/偏航留出最大余量的油门
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
                                        // 缩放横滚/俯仰/偏航,使其落在电机输出范围内
    float   rpy_low = 0.0f;             // lowest motor value
                                        // 电机中的最低值
    float   rpy_high = 0.0f;            // highest motor value
                                        // 电机中的最高值
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
                                        // 飞手期望油门与 throttle_thrust_best_rpy 之间的差值

    setup_tilt_servos();

    // apply voltage and air pressure compensation
    // 施加电压与气压补偿
    const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // tilt servos do yaw only: command is already +/- 1.0 servo demand
    // 倾转舵机只做偏航:偏航量直接当舵量,三个舵机同向满偏对应 MOT_SC_SV_ANG
    yaw_thrust = _yaw_in + _yaw_in_ff;
    const float servo_angle_max = radians(_servo_angle_max_deg);

    if (fabsf(yaw_thrust) > 1.0f) {
        limit.yaw = true;
    }
    const float yaw_angle = constrain_float(yaw_thrust, -1.0f, 1.0f) * servo_angle_max;
    _tilt_in[0] = _tilt_in[1] = _tilt_in[2] = yaw_thrust;
    _servo_angle[0] = _servo_angle[1] = _servo_angle[2] = yaw_angle;

    // 三个旋翼都会倾转,垂直推力上限取三者中最小的 cos
    const float thrust_max = MIN(cosf(_servo_angle[0]), MIN(cosf(_servo_angle[1]), cosf(_servo_angle[2])));

    // sanity check throttle is above zero and below current limited throttle
    // 检查油门在零以上、且不超过限流后的油门上限
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // clamp throttle_avg_max into [pilot throttle, current throttle limit]
    // this is the highest collective the mixer may use to make room for roll/pitch:
    // never below what the pilot asked, never above the voltage/current-limited max
    // 把油门平均上限夹到 [飞手当前油门, 当前限流油门] 之间
    // 混控为横滚/俯仰留余量时最多只能抬到这个值:
    // 不能低于飞手指令,也不能超过限流后的油门上限
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // isosceles triangle: two front motors share pitch, rear takes the opposite of their sum
    // so a pitch command is a pure moment (net force 0). each front is +0.5, rear is -1.0
    // 等腰三角形:两个前旋翼分担俯仰,后旋翼取它们之和的反号,俯仰指令才是纯力矩(净推力为 0)
    // 每个前旋翼 +0.5,后旋翼 -1.0,正好是前面单个的 2 倍
    _thrust_right = roll_thrust * -0.5f + pitch_thrust * 0.5f;
    _thrust_left = roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_rear = pitch_thrust * -1.0f;

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors
    // 计算各电机的横滚/俯仰分量后,
    // 取三个电机中的最小值和最大值存入 rpy_low 和 rpy_high
    rpy_low = MIN(_thrust_right, MIN(_thrust_left, _thrust_rear));
    rpy_high = MAX(_thrust_right, MAX(_thrust_left, _thrust_rear));

    // pick a collective so the roll/pitch band sits in the middle of [0, thrust_max]:
    //   best = midpoint(available) - midpoint(rpy) = 0.5*thrust_max - (rpy_low+rpy_high)/2
    // then cap it by throttle_avg_max so we do not raise throttle just to make room
    // 选一个集体油门,让横滚/俯仰差动落在 [0, thrust_max] 的正中间:
    //   best = 可用区间中点 - 差动区间中点 = 0.5*thrust_max - (rpy_low+rpy_high)/2
    // 再和 throttle_avg_max 取小,避免为了留余量把油门抬得过高
    throttle_thrust_best_rpy = MIN(0.5f * thrust_max - (rpy_low + rpy_high) / 2.0, throttle_avg_max);

    // if the most-negative motor (rpy_low < 0) would go below 0 at this collective,
    // shrink all roll/pitch by rpy_scale so that motor lands on 0. rpy_low==0 means
    // no motor is asking to decrease, so keep full roll/pitch
    // 若最负的那路(rpy_low < 0)叠上这个集体油门会小于 0,
    // 就把全部横滚/俯仰按 rpy_scale 缩小,让那路刚好到 0。rpy_low==0 表示
    // 没有电机要减推力,横滚/俯仰保持 1.0
    if (is_zero(rpy_low)) {
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy / rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    // 计算电机能多接近期望油门
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // Full range is being used by roll, pitch, and yaw.
        // 全部输出范围已被横滚/俯仰/偏航占满
        limit.roll = true;
        limit.pitch = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy + rpy_low)) {
            // Throttle can't be reduced to desired value
            // 油门无法降到期望值
            thr_adj = -(throttle_thrust_best_rpy + rpy_low);
        } else if (thr_adj > thrust_max - (throttle_thrust_best_rpy + rpy_high)) {
            // Throttle can't be increased to desired value
            // 油门无法升到期望值
            thr_adj = thrust_max - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    // determine throttle thrust for harmonic notch
    // 确定用于谐波陷波器的油门推力
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    // compensation_gain can never be zero
    // compensation_gain 不可能为零
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    // 为每个电机叠加缩放后的横滚/俯仰、受限的偏航以及油门
    // 集体油门按悬停推力比分配(后旋翼份额更大),离地瞬间推力分布即接近
    // 配平,不必等俯仰积分器慢慢建立,避免起飞后倒退
    _thrust_right = throttle_thrust_best_plus_adj * _throttle_factor_front + rpy_scale * _thrust_right;
    _thrust_left = throttle_thrust_best_plus_adj * _throttle_factor_front + rpy_scale * _thrust_left;
    _thrust_rear = throttle_thrust_best_plus_adj * _throttle_factor_rear + rpy_scale * _thrust_rear;

    // scale thrust to account for servo angle
    // 按舵机倾转角放大推力,补偿垂直分量损失
    _thrust_right = _thrust_right / cosf(_servo_angle[0]);
    _thrust_rear = _thrust_rear / cosf(_servo_angle[1]);
    _thrust_left = _thrust_left / cosf(_servo_angle[2]);

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    // 把所有输出约束到 0.0~1.0
    // 正常情况下这几行不应产生任何作用;跑测试代码时应将其注释掉以便暴露问题
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
}

// motor test 时向倾转舵机输出 PWM;反相时绕该通道的 trim 镜像,
// 保证 test 的方向与飞行输出(角度取反)一致
void AP_MotorsScorpio::output_test_tilt(SRV_Channel::Aux_servo_function_t function, bool reversed, int16_t pwm)
{
    if (reversed) {
        const SRV_Channel *ch = SRV_Channels::get_channel_for(function);
        if (ch != nullptr) {
            pwm = 2 * (int16_t)ch->get_trim() - pwm;
        }
    }
    SRV_Channels::set_output_pwm(function, pwm);
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
// 以指定 PWM 值驱动某个电机
//  motor_seq 是电机序号,从 1 到机架电机总数
//  pwm 为实际输出的 PWM 值,通常在 1000~2000 范围内
void AP_MotorsScorpio::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    // 输出到电机和舵机
    switch (motor_seq) {
    case 1:
        // front right motor
        // 前右电机
        rc_write(AP_MOTORS_MOT_1, pwm);
        break;
    case 2:
        // all three tilt servos together to check directions in one go
        // 三个倾转舵机同时输出,便于一次校验方向
        output_test_tilt(SRV_Channel::k_tiltMotorRight, _fr_tilt_reverse, pwm);
        output_test_tilt(SRV_Channel::k_tiltMotorRear, _rr_tilt_reverse, pwm);
        output_test_tilt(SRV_Channel::k_tiltMotorLeft, _fl_tilt_reverse, pwm);

        break;
    case 3:
        // back motor
        // 后电机
        rc_write(AP_MOTORS_MOT_2, pwm);
        break;
    case 4:
        // back tilt servo
        // 后倾转舵机
        output_test_tilt(SRV_Channel::k_tiltMotorRear, _rr_tilt_reverse, pwm);
        break;
    case 5:
        // front left motor
        // 前左电机
        rc_write(AP_MOTORS_MOT_3, pwm);
        break;
    case 6:
        // front left tilt servo
        // 前左倾转舵机
        output_test_tilt(SRV_Channel::k_tiltMotorLeft, _fl_tilt_reverse, pwm);
        break;
    default:
        // do nothing
        // 什么也不做
        break;
    }
}

/*
  call vehicle supplied thrust compensation if set. This allows for
  vehicle specific thrust compensation for motor arrangements such as
  the forward motors tilting
  若设置了载具提供的推力补偿回调则调用它。用于针对特定电机布局
  (例如前电机可倾转)做载具级的推力补偿
*/
void AP_MotorsScorpio::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        // convert 3 thrust values into an array indexed by motor number
        // 把三个推力值放进按电机编号索引的数组
        float thrust[3] { _thrust_right, _thrust_rear, _thrust_left };

        // apply vehicle supplied compensation function
        // 调用载具提供的补偿函数
        _thrust_compensation_callback(thrust, 3);

        // extract compensated thrust values
        // 取回补偿后的推力值
        _thrust_right = thrust[0];
        _thrust_rear = thrust[1];
        _thrust_left = thrust[2];
    }
}

float AP_MotorsScorpio::get_roll_factor(uint8_t i)
{
    float ret = 0.0f;

    switch (i) {
        // right motor
        // 右电机
        case AP_MOTORS_MOT_1:
            ret = -1.0f;
            break;
            // left motor
            // 左电机
        case AP_MOTORS_MOT_3:
            ret = 1.0f;
            break;
    }

    return ret;
}

// Run arming checks
// 执行解锁前检查
bool AP_MotorsScorpio::arming_checks(size_t buflen, char *buffer) const
{
    // Check for tilt servos
    // 检查倾转舵机功能是否已分配
    if (!SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight) ||
        !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear) ||
        !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft)) {
        hal.util->snprintf(buffer, buflen, "no SERVOx_FUNCTION set to TiltMotorRight/Rear/Left");
        return false;
    }

    // run base class checks
    // 执行基类检查
    return AP_MotorsMulticopter::arming_checks(buflen, buffer);
}
