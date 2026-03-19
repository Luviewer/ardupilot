/*
 * ArduPilot + RT-Thread HAL - I2CDeviceManager implementation
 */

#include "I2CDeviceManager.h"
#include "I2CDevice.h"

namespace RTT
{

AP_HAL::I2CDevice *I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                                   uint32_t bus_clock,
                                                   bool use_smbus,
                                                   uint32_t timeout_ms)
{
    return NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms);
}

} // namespace RTT
