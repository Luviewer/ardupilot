#pragma once

#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped_Diag.h"
#include <AP_Param/AP_Param.h>

// 连续步态控制器
class AP_QuadRuped_ContinuousGait : public AP_QuadRuped_Backend {
public:
    // 构造函数
    AP_QuadRuped_ContinuousGait(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state,
                                AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_ContinuousGait() { }

    // 参数表
    static const struct AP_Param::GroupInfo var_info[];

    uint32_t get_Freq() override { return gait_hz.get(); }

    // 后端接口实现
    void update() override;
    void update_leg() override;

    void gait_init() override;
    void trajectory_generation(uint8_t leg_index) override;
    void yaw_trajectory_generation(uint8_t leg_index) override;

private:
    // 参数定义
    AP_Float gait_hz;

    uint32_t lasttime;

    // 连续时间变量
    uint32_t _current_time;              // 当前连续时间(毫秒)
    uint32_t _init_time;                 // 初始化时间(毫秒)

    // 连续相位参数
    float _leg_phase_offset[4];        // 每条腿的时间偏移
    float _leg_start_time[4];          // 每条腿的开始时间
    bool _leg_active[4];               // 腿部激活状态

    // 连续步态时序参数
    float _stance_duration;            // 支撑相持续时间
    float _transfer_duration;          // 摆动相持续时间

    // 连续相位计算 - 计算指定腿在步态周期中的归一化相位
    float calculate_continuous_phase(uint8_t leg_index);

    // 轨迹完成检测 - 连续模式中可能不需要，但保留以兼容接口
    bool is_leg_trajectory_complete(uint8_t leg_index);

    // 启动下一条腿的轨迹 - 连续模式中不需要，但保留以兼容接口
    void start_next_leg_trajectory(uint8_t completed_leg_index);
};