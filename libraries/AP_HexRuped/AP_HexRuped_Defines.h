#pragma once

// 腿部索引枚举
enum AP_HexRuped_LegIndex {
    AP_HEXRUPED_LEG_RF = 0, // 右前腿 (Right Front)
    AP_HEXRUPED_LEG_RB,     // 右后腿 (Right Back)
    AP_HEXRUPED_LEG_LB,     // 左后腿 (Left Back)
    AP_HEXRUPED_LEG_LF,     // 左前腿 (Left Front)
    // 中腿追加在原四腿索引之后，以保持已有参数和舵机映射兼容。
    AP_HEXRUPED_LEG_RM,     // 右中腿 (Right Middle)
    AP_HEXRUPED_LEG_LM,     // 左中腿 (Left Middle)
    AP_HEXRUPED_LEG_ALL,    // 腿的总数（6条腿）
};

#define AP_HEXRUPED_JOINTS_PER_LEG 3
#define AP_HEXRUPED_SERVO_COUNT    (AP_HEXRUPED_LEG_ALL * AP_HEXRUPED_JOINTS_PER_LEG)

// 步态类型枚举
enum AP_HexRuped_GaitType {
    AP_HEXRUPED_GAIT_TRIPOD     = 0, // 六足交替三角步态
    AP_HEXRUPED_GAIT_WAVE       = 1, // 波浪步态（crawl步态）
    AP_HEXRUPED_GAIT_COUNT,          // 步态总数
};

// 蜘蛛类型枚举
enum AP_HexRuped_Class {
    AP_HEXRUPED_NORMAL  = 0, // 普通模型
    AP_HEXRUPED_USL_BV2 = 1, // USL_BV2
    AP_HEXRUPED_CLASS_COUNT,
};

using HexRupedGaitType = AP_HexRuped_GaitType;
using HexRupedLegIndex = AP_HexRuped_LegIndex;
using HexRupedClass = AP_HexRuped_Class;

// 统一的常量定义
#define AP_HEXRUPED_SPEED_HZ_DEFAULT     25.0f  // 默认步态频率（Hz）
#define AP_HEXRUPED_STEP_TOTAL_DEFAULT   24     // 默认步态总步数
#define AP_HEXRUPED_START_COXA_ANGLE     45.0f  // 起始髋关节角度
