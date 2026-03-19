/*
 * ArduPilot + RT-Thread HAL - GPIO stub (Phase 0)
 */

#pragma once

#include <AP_HAL/GPIO.h>
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class DigitalSource : public AP_HAL::DigitalSource
{
public:
    void mode(uint8_t output) override;
    uint8_t read() override;
    void write(uint8_t value) override;
    void toggle() override;
};

class GPIO : public AP_HAL::GPIO
{
public:
    void init() override;
    void pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void write(uint8_t pin, uint8_t value) override;
    void toggle(uint8_t pin) override;
    AP_HAL::DigitalSource* channel(uint16_t n) override;
    bool usb_connected() override;
};

} // namespace RTT
