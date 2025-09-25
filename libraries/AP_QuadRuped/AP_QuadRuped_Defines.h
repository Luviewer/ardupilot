#pragma once

// 腿部索引枚举
enum AP_QuadRuped_LegIndex {
    AP_QUADRUPED_LEG_RF = 0, // 右前腿 (Right Front)
    AP_QUADRUPED_LEG_RB,     // 右后腿 (Right Back)
    AP_QUADRUPED_LEG_LB,     // 左后腿 (Left Back)
    AP_QUADRUPED_LEG_LF,     // 左前腿 (Left Front)
    AP_QUADRUPED_LEG_ALL,    // 腿的总数（4条腿）
};

// 步态类型枚举
enum AP_QuadRuped_GaitType {
    AP_QUADRUPED_GAIT_DIAGONAL = 0, // 对角步态（trot步态）
    AP_QUADRUPED_GAIT_WAVE     = 1, // 波浪步态（crawl步态）
    AP_QUADRUPED_GAIT_CRAB     = 2, // 工字步态
    AP_QUADRUPED_GAIT_COUNT,        // 步态总数
};

// 蜘蛛类型枚举
enum AP_QuadRuped_CLASS {
    AP_QUADRUPED_NORMAL  = 0, // 普通模型
    AP_QUADRUPED_USL_BV2 = 1, // USL_BV2
    AP_QuadRuped_CLASS_COUNT, // 步态总数
};

// 使用 AP_QuadRuped_Defines.h 中定义的枚举类型
typedef AP_QuadRuped_GaitType GaitType;
typedef AP_QuadRuped_LegIndex LegIndex;
typedef AP_QuadRuped_CLASS    QuadRupedClass;
