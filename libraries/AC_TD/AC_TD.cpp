#include "AC_TD.h"

// const AP_Param::GroupInfo AC_TD::var_info[] = {
//     AP_GROUPINFO("r", 1, AC_TD, r, 10),
//     AP_GROUPINFO("h", 2, AC_TD, h, 0.01),

//     AP_GROUPEND
// };

// 符号函数
float AC_TD::sign(float x)
{
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// 最速控制综合函数
float AC_TD::fhan(float x1, float x2)
{
    float d  = r * h;
    float d0 = h * d;
    float y  = x1 + h * x2;
    float a0 = sqrtf(d * d + 8 * r * fabsf(y));
    float a;

    if (fabsf(y) > d0) {
        a = x2 + (a0 - d) / 2 * sign(y);
    } else {
        a = x2 + y / h;
    }

    if (fabsf(a) > d) {
        return -r * sign(a);
    } else {
        return -r * a / d;
    }
}

// 更新跟踪微分器状态
float AC_TD::update(float in)
{
    float f = fhan(v1 - in, v2);

    float v2_old = v2;
    v2 += h * f;      // 更新微分信号
    v1 += h * v2_old; // 更新跟踪信号

    return v1;
}

void AC_TD::init(float _h, float _r, float _v1)
{
    h  = _h;
    r  = _r;
    v1 = _v1;
}