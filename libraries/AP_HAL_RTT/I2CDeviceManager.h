/*
 * ArduPilot + RT-Thread HAL - I2CDeviceManager stub (Phase 0)
 */

#pragma once

#include <AP_HAL/I2CDevice.h>
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class I2CDeviceManager : public AP_HAL::I2CDeviceManager
{
public:
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                     uint32_t bus_clock = 400000,
                                     bool use_smbus = false,
                                     uint32_t timeout_ms = 4) override;
};

} // namespace RTT
