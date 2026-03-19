/*
 * ArduPilot + RT-Thread HAL - HAL class and driver instances.
 * run() 顺序：若 RT-Thread 未在 board 启动中初始化则 rt_hw_board_init /
 * rt_system_timer_init / rt_system_scheduler_init；创建主线程（入口 _main_loop_entry：
 * setup() 后 while(1) loop()）；最后 rt_system_scheduler_start()。
 * 需链接 librtthread；依赖符号：rt_hw_board_init, rt_system_timer_init,
 * rt_system_scheduler_init, rt_system_scheduler_start, rt_thread_create 等。
 */

#include <AP_HAL/AP_HAL.h>
#include "HAL_RTT_Class.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include <rtthread.h>
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#include "SPIDeviceManager.h"
#include "I2CDeviceManager.h"
#include <AP_HAL/OpticalFlow.h>
#include <AP_HAL/Flash.h>
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
#include <AP_HAL/SIMState.h>
#endif

extern "C" void rt_hw_board_init(void);

/* Minimal stubs for OpticalFlow and Flash (not in task list) */
namespace RTT
{
class OpticalFlowStub : public AP_HAL::OpticalFlow
{
public:
    void init() override {}
    bool read(Data_Frame& frame) override { (void)frame; return false; }
    void push_gyro(float gyro_x, float gyro_y, float dt) override { (void)gyro_x; (void)gyro_y; (void)dt; }
    void push_gyro_bias(float gyro_bias_x, float gyro_bias_y) override { (void)gyro_bias_x; (void)gyro_bias_y; }
};
class FlashStub : public AP_HAL::Flash
{
public:
    uint32_t getpageaddr(uint32_t page) override { (void)page; return 0; }
    uint32_t getpagesize(uint32_t page) override { (void)page; return 0; }
    uint32_t getnumpages() override { return 0; }
    bool erasepage(uint32_t page) override { (void)page; return false; }
    bool write(uint32_t addr, const void *buf, uint32_t count) override { (void)addr; (void)buf; (void)count; return false; }
    void keep_unlocked(bool set) override { (void)set; }
    bool ispageerased(uint32_t page) override { (void)page; return true; }
};
}

static RTT::UARTDriver cons(0);
static RTT::UARTDriver serial1Driver(1);
static RTT::UARTDriver serial2Driver(2);
static RTT::UARTDriver serial3Driver(3);
static RTT::UARTDriver serial4Driver(4);
static RTT::UARTDriver serial5Driver(5);
static RTT::UARTDriver serial6Driver(6);
static RTT::UARTDriver serial7Driver(7);
static RTT::UARTDriver serial8Driver(8);
static RTT::UARTDriver serial9Driver(9);

static RTT::I2CDeviceManager i2cDeviceManager;
static RTT::SPIDeviceManager spiDeviceManager;
static RTT::AnalogIn analogIn;
static RTT::Storage storageDriver;
static RTT::GPIO gpioDriver;
static RTT::RCInput rcinDriver;
static RTT::RCOutput rcoutDriver;
static RTT::Scheduler schedulerInstance;
static RTT::Util utilInstance;
static RTT::OpticalFlowStub opticalFlowDriver;
static RTT::FlashStub flashDriver;
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
static AP_HAL::SIMState xsimstate;
#endif

extern const AP_HAL::HAL& hal;

HAL_RTT::HAL_RTT() :
    AP_HAL::HAL(
        &cons,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        &serial4Driver,
        &serial5Driver,
        &serial6Driver,
        &serial7Driver,
        &serial8Driver,
        &serial9Driver,
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        &xsimstate,
#endif
        nullptr
    )
{}

struct main_loop_arg {
    RTT::Scheduler* sched;
    AP_HAL::HAL::Callbacks* callbacks;
};

static void _main_loop_entry(void* arg)
{
    main_loop_arg* a = (main_loop_arg*)arg;
    a->sched->set_main_thread_id(rt_thread_self());
    a->callbacks->setup();
    a->sched->set_system_initialized();
    for (;;) {
        a->callbacks->loop();
    }
}

void HAL_RTT::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    (void)argc;
    (void)argv;

    rt_hw_board_init();
    /* ~1.5s delay before USB init so host sees a clean connect (ChibiOS: usbDisconnectBus + 1.5s + usbConnectBus) */
    for (volatile uint32_t i = 0; i < 100000000U; i++) {
        (void)i;
    }
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_init();  /* CherryUSB CDC, 其他 INIT_COMPONENT_EXPORT */
#endif
    rt_system_timer_init();
    rt_system_scheduler_init();

    ((RTT::Scheduler*)scheduler)->set_callbacks(callbacks);
    scheduler->init();

    static main_loop_arg s_arg;
    s_arg.sched = (RTT::Scheduler*)scheduler;
    s_arg.callbacks = callbacks;

    rt_thread_t main_thread = rt_thread_create(
        "ap_main",
        _main_loop_entry,
        &s_arg,
        4096,
        RT_THREAD_PRIORITY_MAX - 2,
        20);
    if (main_thread == nullptr) {
        AP_HAL::panic("RTT: failed to create main thread");
    }
    rt_thread_startup(main_thread);

    rt_system_scheduler_start();
}

void AP_HAL::init()
{
}

static HAL_RTT hal_rtt;

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_rtt;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_rtt;
}
