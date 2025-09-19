#include "Copter.h"
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

// 在编程和嵌入式系统中，钩子（Hook） 是一种机制，允许开发者在系统默认流程中插入自定义代码，从而扩展或修改原有功能。
// 钩子在 ArduPilot 中的作用
// 在 ArduPilot（如 Copter 代码）中，钩子用于扩展飞行控制器的功能，例如：
// 初始化自定义硬件（如传感器、舵机驱动）。
// 添加高频控制逻辑（如 100Hz 的平衡控制）。
// 响应遥控器开关指令（如切换飞行模式）。
#ifdef USERHOOK_INIT
void Copter::userhook_init() // 如果启用了用户初始化钩子
{
    // put your initialisation code here
    // this will be called once at start-up
    // 初始化代码（仅在启动时调用一次）
    qrupd->init();
}
#endif

#ifdef USERHOOK_FASTLOOP // 如果启用了 100Hz 高速循环钩子
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here   // 100Hz 代码（每 10ms 执行一次）
    const AP_RangeFinder_Backend* sensor = rangefinder.get_backend(0);

    static uint32_t lasttime         = 0; // 记录上一次执行时间（用于频率控制）
    static uint32_t balance_lasttime = 0; // 平衡控制器的上一次执行时间

    // 检查是否到达逆运动学计算的执行时间（基于 qrupd.getFreq() 返回的频率）
    if ((AP_HAL::millis() - lasttime) > (1000 / qrupd->getFreq())) {
        lasttime = AP_HAL::millis(); // 更新最后执行时间

        // 检查遥控器通道 6（CH_6）的值是否大于 1500（通常表示开关激活）并且没有解锁
        if (hal.rcin->read(CH_6) > 1800 && !motors->armed()) {
            qrupd->update();
        } else if (hal.rcin->read(CH_6) > 1400 && hal.rcin->read(CH_6) < 1600) {
            // 检测测距cm
            uint16_t sonar_cm = sensor->distance_cm();
            if (sonar_cm > 30) {
                qrupd->x_up_sleep_leg();
            } else {
                qrupd->x_sleep_leg();
            }
        } else if (hal.rcin->read(CH_6) > 900 && hal.rcin->read(CH_6) < 1200) {
            qrupd->hengxiang_up_sleep_leg();
        }
        qrupd->hw_set_servo_cmd(); // 发送舵机控制命令
    }

    // 平衡控制器（固定 100Hz 运行）
    if ((AP_HAL::millis() - balance_lasttime) > (1000 / 100)) {
        balance_lasttime = AP_HAL::millis();

        // 如果通道 6 激活，运行平衡控制器
        // if (hal.rcin->read(CH_6) > 1500) {
        //     qrupd->balance_controller();
        // } else {
        // }
    }
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here      // 50Hz 代码（每 20ms 执行一次）
}
#endif

#ifdef USERHOOK_MEDIUMLOOP // 如果启用了 10Hz 循环钩子
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP // 如果启用了 3.3Hz 循环钩子
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP // 如果启用了 1Hz 循环钩子
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH // 如果启用了辅助开关钩子
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)    // 辅助开关 1 的回调（遥控器通道选项设为 47 时触发）
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
