/*
 * ArduPilot + RT-Thread HAL - SPIDeviceManager implementation
 */

#include "SPIDeviceManager.h"
#include "SPIDevice.h"
#include <cstring>

namespace RTT
{

static const struct { const char *ap_name; const char *rtt_name; uint8_t bus_id; } _spi_table[] = {
    {"icm42688", "spi10", 1},
    {"bmi055_g", "spi11", 1},
    {"bmi055_a", "spi12", 1},
    {"ramtron", "spi20", 2},
};
#define _SPI_TABLE_COUNT (sizeof(_spi_table) / sizeof(_spi_table[0]))

AP_HAL::SPIDevice *SPIDeviceManager::get_device_ptr(const char *name)
{
    for (size_t i = 0; i < _SPI_TABLE_COUNT; i++) {
        if (strcmp(name, _spi_table[i].ap_name) == 0) {
            return NEW_NOTHROW SPIDevice(_spi_table[i].rtt_name, _spi_table[i].bus_id);
        }
    }
    return nullptr;
}

uint8_t SPIDeviceManager::get_count()
{
    return _SPI_TABLE_COUNT;
}

const char *SPIDeviceManager::get_device_name(uint8_t idx)
{
    if (idx >= _SPI_TABLE_COUNT) return nullptr;
    return _spi_table[idx].ap_name;
}

} // namespace RTT
