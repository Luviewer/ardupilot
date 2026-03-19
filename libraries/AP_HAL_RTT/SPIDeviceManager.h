/*
 * ArduPilot + RT-Thread HAL - SPIDeviceManager stub (Phase 0)
 */

#pragma once

#include <AP_HAL/SPIDevice.h>
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
    uint8_t get_count() override;
    const char *get_device_name(uint8_t idx) override;
};

} // namespace RTT
