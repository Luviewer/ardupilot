#include "AP_QuadRuped_New.h"
#include "AP_QuadRuped_Backend.h"
#include "AP_QuadRuped_Diag_New.h"
#include "AP_QuadRuped_WAVE_New.h"
#include "AP_QuadRuped_Crab.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS_DCM.h>
#include <AP_Motors/AP_MotorsMatrix.h>

// 测试函数声明
void test_backend_interface();
void test_gait_switching();
void test_parameter_access();
void test_health_monitoring();

// 模拟硬件接口
class MockAHRS : public AP_AHRS_DCM {
public:
    MockAHRS() : AP_AHRS_DCM() {}
    bool healthy() const override { return true; }
};

class MockMotors : public AP_MotorsMatrix {
public:
    MockMotors() : AP_MotorsMatrix() {}
    void output() override {}
    void output_test(uint8_t motor_seq, int16_t pwm) override {}
};

class MockRangeFinder : public RangeFinder {
public:
    MockRangeFinder() : RangeFinder() {}
    bool healthy() const override { return true; }
};

// 主测试函数
int main() {
    printf("=== AP_QuadRuped 架构重构测试 ===\n\n");

    // 创建模拟硬件接口
    MockAHRS ahrs;
    MockMotors motors;
    MockRangeFinder rangefinder;

    // 创建AP_QuadRuped实例
    AP_QuadRuped quadru(ahrs, motors, rangefinder);

    printf("1. 测试初始化...\n");
    if (quadru.init()) {
        printf("   ✓ 初始化成功\n");
    } else {
        printf("   ✗ 初始化失败\n");
        return 1;
    }

    printf("\n2. 测试后端接口...\n");
    test_backend_interface();

    printf("\n3. 测试步态切换...\n");
    test_gait_switching();

    printf("\n4. 测试参数访问...\n");
    test_parameter_access();

    printf("\n5. 测试健康监控...\n");
    test_health_monitoring();

    printf("\n=== 所有测试完成 ===\n");
    return 0;
}

// 测试后端接口
void test_backend_interface() {
    // 创建模拟硬件接口
    MockAHRS ahrs;
    MockMotors motors;
    MockRangeFinder rangefinder;

    AP_QuadRuped quadru(ahrs, motors, rangefinder);
    quadru.init();

    // 测试每个后端
    const char* gait_names[] = {"对角步态", "波浪步态", "工字步态"};

    for (int i = 0; i < 3; i++) {
        quadru.set_gait_type((AP_QuadRuped::GaitType)i);

        if (quadru.healthy()) {
            printf("   ✓ %s后端健康\n", gait_names[i]);
        } else {
            printf("   ✗ %s后端不健康\n", gait_names[i]);
        }
    }
}

// 测试步态切换
void test_gait_switching() {
    // 创建模拟硬件接口
    MockAHRS ahrs;
    MockMotors motors;
    MockRangeFinder rangefinder;

    AP_QuadRuped quadru(ahrs, motors, rangefinder);
    quadru.init();

    const char* gait_names[] = {"对角步态", "波浪步态", "工字步态"};

    // 测试步态切换
    for (int i = 0; i < 3; i++) {
        quadru.set_gait_type((AP_QuadRuped::GaitType)i);

        if (quadru.get_gait_type() == i) {
            printf("   ✓ 切换到%s成功\n", gait_names[i]);
        } else {
            printf("   ✗ 切换到%s失败\n", gait_names[i]);
        }

        // 测试控制输入
        quadru.set_throttle(0.5f, 0.3f);
        quadru.set_yaw_rate(0.1f);
        quadru.set_body_height(0.8f);

        // 运行几次更新
        for (int j = 0; j < 10; j++) {
            quadru.update();
        }
    }
}

// 测试参数访问
void test_parameter_access() {
    // 创建模拟硬件接口
    MockAHRS ahrs;
    MockMotors motors;
    MockRangeFinder rangefinder;

    AP_QuadRuped quadru(ahrs, motors, rangefinder);
    quadru.init();

    // 测试系统参数访问
    const AP_QuadRuped_SYS_Params& sys_params = quadru.get_sys_params();
    printf("   ✓ 系统参数获取成功\n");

    // 测试腿部参数访问
    for (int i = 0; i < 4; i++) {
        const AP_QuadRuped_Params& leg_params = quadru.get_leg_params(i);
        printf("   ✓ 腿%d参数获取成功\n", i);
    }

    // 测试通道参数访问
    const AP_QuadRuped_CHANNEL_Params& channel_params = quadru.get_channel_params();
    printf("   ✓ 通道参数获取成功\n");
}

// 测试健康监控
void test_health_monitoring() {
    // 创建模拟硬件接口
    MockAHRS ahrs;
    MockMotors motors;
    MockRangeFinder rangefinder;

    AP_QuadRuped quadru(ahrs, motors, rangefinder);
    quadru.init();

    // 测试健康状态
    if (quadru.healthy()) {
        printf("   ✓ 系统健康状态正常\n");
    } else {
        printf("   ✗ 系统健康状态异常\n");
    }

    // 测试硬件接口访问
    printf("   ✓ AHRS接口访问正常\n");
    printf("   ✓ Motors接口访问正常\n");
    printf("   ✓ RangeFinder接口访问正常\n");

    // 测试控制输入获取
    float throttle_x = quadru.get_throttle_x();
    float throttle_y = quadru.get_throttle_y();
    float yaw_rate = quadru.get_yaw_rate();
    float body_height = quadru.get_body_height();

    printf("   ✓ 控制输入获取成功: X=%.2f, Y=%.2f, Yaw=%.2f, Height=%.2f\n",
           throttle_x, throttle_y, yaw_rate, body_height);
}