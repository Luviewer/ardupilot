#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class AC_TD {
public:
    AC_TD()
    {
        v1 = 0;
        v2 = 0;
        r  = 10;
        h  = 0.01;
        // AP_Param::setup_object_defaults(this, var_info);
    }
    // Parameter block
    // static const struct AP_Param::GroupInfo var_info[];

    float update(float in);
    void  init(float _h, float _r);

private:
    float fhan(float x1, float x2);
    float sign(float x);

    float v1; // 跟踪信号
    float v2; // 微分信号

    float r; // 速度因子
    float h; // 滤波因子/步长
};
