/*
 * ArduPilot + RT-Thread HAL - SPIDevice
 * Wraps RT-Thread SPI device for AP_HAL::SPIDevice interface.
 */

#pragma once

#include <AP_HAL/SPIDevice.h>
#include "Semaphores.h"
#include "HAL_RTT_Namespace.h"

struct rt_spi_device;

namespace RTT
{

class SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(const char *name, uint8_t bus_id);
    ~SPIDevice();

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;
    bool set_chip_select(bool set) override;

private:
    struct rt_spi_device *_dev;
    uint8_t _bus_id;
    Semaphore _sem;
    bool _cs_held = false;
};

} // namespace RTT
