/*
 * ArduPilot + RT-Thread HAL - I2CDevice implementation
 * Uses rt_i2c_bus_device_find, rt_i2c_transfer from RTT drivers.
 */

#include "I2CDevice.h"
#include <AP_HAL/AP_HAL.h>
#include <rtthread.h>
#include <drivers/dev_i2c.h>

using namespace RTT;

#ifndef HAL_RTT_I2C_BUS_NAMES
#define HAL_RTT_I2C_BUS_NAMES "i2c1", "i2c2", "i2c3", "i2c4"
#endif

static const char *const _i2c_bus_names[] = { HAL_RTT_I2C_BUS_NAMES };
#define HAL_RTT_I2C_BUS_COUNT (sizeof(_i2c_bus_names) / sizeof(_i2c_bus_names[0]))

I2CDevice::I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock,
                   bool use_smbus, uint32_t timeout_ms)
    : AP_HAL::I2CDevice()
    , _bus(nullptr)
    , _address(address)
    , _bus_clock(bus_clock)
    , _timeout_ms(timeout_ms)
    , _split(false)
{
    set_device_bus(bus);
    set_device_address(address);
    if (bus < HAL_RTT_I2C_BUS_COUNT) {
        _bus = rt_i2c_bus_device_find(_i2c_bus_names[bus]);
    }
}

I2CDevice::~I2CDevice()
{
}

bool I2CDevice::set_speed(AP_HAL::Device::Speed)
{
    if (_bus == nullptr) return false;
    return true;
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                        uint8_t *recv, uint32_t recv_len)
{
    if (_bus == nullptr) return false;
    if (!_sem.take_nonblocking()) return false;
    rt_bool_t ok = RT_TRUE;
    rt_tick_t tick = rt_tick_from_millisecond(_timeout_ms > 0 ? _timeout_ms : 4);
    if (rt_i2c_bus_lock(_bus, tick) != RT_EOK) {
        _sem.give();
        return false;
    }
    if (send_len > 0 && send != nullptr) {
        rt_ssize_t n = rt_i2c_master_send(_bus, _address, RT_I2C_WR, send, send_len);
        if (n != (rt_ssize_t)send_len) ok = RT_FALSE;
    }
    if (ok && recv_len > 0 && recv != nullptr) {
        rt_ssize_t n = rt_i2c_master_recv(_bus, _address, RT_I2C_RD, recv, recv_len);
        if (n != (rt_ssize_t)recv_len) ok = RT_FALSE;
    }
    rt_i2c_bus_unlock(_bus);
    _sem.give();
    return ok == RT_TRUE;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    (void)times;
    return read_registers(first_reg, recv, recv_len);
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &_sem;
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb)
{
    (void)period_usec;
    return nullptr;
}

bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle, uint32_t)
{
    return false;
}
