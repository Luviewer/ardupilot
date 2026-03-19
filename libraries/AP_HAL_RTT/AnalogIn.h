/*
 * ArduPilot + RT-Thread HAL - AnalogIn
 * Pixhawk6C-Mini: PC5(BATT_VOLTAGE), PC4(BATT_CURRENT), etc.
 * Uses RTT ADC device when available, else returns 0V (stub).
 */

#pragma once

#include <AP_HAL/AnalogIn.h>
#include "HAL_RTT_Namespace.h"
#include "Semaphores.h"

#define RTT_ANALOG_MAX_CHANNELS 8

namespace RTT
{

class AnalogSource : public AP_HAL::AnalogSource
{
public:
    AnalogSource() : _pin(-1), _value(0.0f), _latest_value(0.0f) {}
    AnalogSource(int16_t pin) : _pin(pin), _value(0.0f), _latest_value(0.0f) {}
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override WARN_IF_UNUSED;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    int16_t _pin;
    float _value;
    float _latest_value;

    float _read_raw();
};

class AnalogIn : public AP_HAL::AnalogIn
{
public:
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage() override;

private:
    AnalogSource _sources[RTT_ANALOG_MAX_CHANNELS];
    float _board_voltage = 5.0f;
    bool _initialized = false;
};

} // namespace RTT
