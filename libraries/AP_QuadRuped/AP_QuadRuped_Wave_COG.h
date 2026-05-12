#pragma once

#include "AC_TD/AC_TD.h"
#include "AP_QuadRuped_Params.h"
#include "AP_QuadRuped_Wave.h"
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>

// 前向声明
class AP_QuadRuped;

// 对角步态后端实现
class AP_QuadRuped_Wave_COG : public AP_QuadRuped_Wave {
public:
    // 构造函数
    AP_QuadRuped_Wave_COG(AP_QuadRuped& frontend, AP_QuadRuped::QuadRuped_State& state, AP_AHRS_View& ahrs, AP_Motors& motors);

    // 析构函数
    virtual ~AP_QuadRuped_Wave_COG() { }

    // 参数表定义
    static const struct AP_Param::GroupInfo var_info[];

    uint8_t gait_step_cog_start[AP_QUADRUPED_LEG_ALL]; // 每条腿的步态起始步数

    void update_leg() override;
    void update() override;

    void gait_init() override;

private:
    void cog_generation(uint8_t leg_index); // 重心偏移生成器

    // 重心平滑过渡相关
    bool cog_last_move_requested = false;   // 上一帧的移动请求状态
    bool cog_transitioning = false;         // 是否正在过渡中
    Vector2f cog_transition_start;          // 过渡起始位置
    Vector2f cog_transition_end;            // 过渡目标位置
    Vector2f cog_last_output;               // 上一帧实际输出的重心偏移
    uint32_t cog_transition_start_ms = 0;   // 过渡开始时间戳

    AP_Float centre_offset_ratio;
    AP_Float cog_transition_time;   // 重心过渡时间（秒）
};
