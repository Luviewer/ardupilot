/*
 * ArduPilot + RT-Thread HAL - DeviceBus stub (Phase 0)
 * Placeholder for SPI/I2C bus abstraction; actual implementation
 * will use RT-Thread device and mutex.
 */

#pragma once

#include <stdint.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Device.h>
#include "Semaphores.h"
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class DeviceBus
{
public:
    DeviceBus(uint8_t thread_priority);
    struct DeviceBus *next;
    Semaphore semaphore;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *hal_device);
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);

private:
    uint8_t _thread_priority;
};

} // namespace RTT
