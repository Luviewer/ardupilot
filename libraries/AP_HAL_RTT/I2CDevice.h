/*
 * ArduPilot + RT-Thread HAL - I2CDevice
 * Wraps RT-Thread I2C bus for AP_HAL::I2CDevice interface.
 */

#pragma once

#include <AP_HAL/I2CDevice.h>
#include "Semaphores.h"
#include "HAL_RTT_Namespace.h"

struct rt_i2c_bus_device;

namespace RTT
{

class I2CDevice : public AP_HAL::I2CDevice
{
public:
    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock,
              bool use_smbus, uint32_t timeout_ms);
    ~I2CDevice();

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                uint32_t recv_len, uint8_t times) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;
    void set_split_transfers(bool set) override { _split = set; }

private:
    struct rt_i2c_bus_device *_bus;
    uint8_t _address;
    uint32_t _bus_clock;
    uint32_t _timeout_ms;
    bool _split;
    Semaphore _sem;
};

} // namespace RTT
