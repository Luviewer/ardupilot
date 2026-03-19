/*
 * ArduPilot + RT-Thread HAL - DeviceBus stub implementation (Phase 0)
 */

#include "DeviceBus.h"

namespace RTT
{

DeviceBus::DeviceBus(uint8_t thread_priority)
    : next(nullptr), _thread_priority(thread_priority)
{
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *hal_device)
{
    (void)period_usec;
    (void)cb;
    (void)hal_device;
    return nullptr;
}

bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    (void)h;
    (void)period_usec;
    return false;
}

} // namespace RTT
