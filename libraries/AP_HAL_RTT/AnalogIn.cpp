/*
 * ArduPilot + RT-Thread HAL - AnalogIn
 * Pixhawk6C-Mini: PC5(BATT_VOLTAGE), PC4(BATT_CURRENT).
 * When RT_USING_ADC and adc1 device exist, reads via rt_adc_read; else returns 0V.
 */

#include "AnalogIn.h"
#include <AP_HAL/AP_HAL.h>
#include <rtthread.h>
#include <stddef.h>

#if defined(RT_USING_ADC)
extern "C" {
#include <drivers/adc.h>
}
#endif

#define VOLTAGE_SCALING (3.3f / 4095.0f)

// Pixhawk6C ADC channel map: pin_index -> (adc_dev_name, rt channel)
// Ch0: BATT_VOLTAGE PC5 -> ADC1 CH19, Ch1: BATT_CURRENT PC4 -> ADC1 CH14
static const struct { const char *dev; uint8_t ch; } _adc_map[RTT_ANALOG_MAX_CHANNELS] = {
    {"adc1", 19},   // 0: PC5 BATT_VOLTAGE
    {"adc1", 14},   // 1: PC4 BATT_CURRENT
    {"adc1", 14},   // 2: spare
    {"adc1", 14},   // 3
    {"adc1", 14},   // 4
    {"adc1", 14},   // 5
    {"adc1", 14},   // 6
    {"adc1", 14},   // 7
};

namespace RTT
{

float AnalogSource::_read_raw()
{
#if defined(RT_USING_ADC)
    if (_pin < 0 || _pin >= RTT_ANALOG_MAX_CHANNELS) return 0.0f;
    rt_device_t dev = rt_device_find(_adc_map[_pin].dev);
    if (dev == nullptr) return 0.0f;
    rt_uint32_t val = rt_adc_read((struct rt_adc_device *)dev, _adc_map[_pin].ch);
    return (float)(val & 0xFFF);  // 12-bit
#else
    (void)_pin;
    return 0.0f;
#endif
}

float AnalogSource::read_average()
{
    _value = _read_raw() * VOLTAGE_SCALING;
    _latest_value = _value;
    return _value;
}

float AnalogSource::read_latest()
{
    return _latest_value;
}

bool AnalogSource::set_pin(uint8_t p)
{
    if (p < RTT_ANALOG_MAX_CHANNELS) {
        _pin = (int16_t)p;
        return true;
    }
    return false;
}

float AnalogSource::voltage_average()
{
    read_average();
    return _value;
}

float AnalogSource::voltage_latest()
{
    return _latest_value;
}

float AnalogSource::voltage_average_ratiometric()
{
    return voltage_average();
}

void AnalogIn::init()
{
    if (_initialized) return;
    for (int i = 0; i < RTT_ANALOG_MAX_CHANNELS; i++) {
        IGNORE_RETURN(_sources[i].set_pin(i));
    }
#if defined(RT_USING_ADC)
    rt_device_t dev = rt_device_find("adc1");
    if (dev != nullptr) {
        rt_adc_enable((struct rt_adc_device *)dev, 19);
        rt_adc_enable((struct rt_adc_device *)dev, 14);
    }
#endif
    _initialized = true;
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n)
{
    init();
    if (n < 0 || n >= RTT_ANALOG_MAX_CHANNELS) return nullptr;
    return &_sources[n];
}

float AnalogIn::board_voltage()
{
    init();
#if defined(HAL_BATT_VOLT_PIN) && defined(HAL_BATT_VOLT_SCALE)
    AP_HAL::AnalogSource *v = channel(HAL_BATT_VOLT_PIN);
    if (v != nullptr) {
        float vv = v->voltage_average() * (float)HAL_BATT_VOLT_SCALE;
        if (vv > 0.1f) {
            _board_voltage = vv;
        }
    }
#else
    AP_HAL::AnalogSource *v = channel(0);
    if (v != nullptr) {
        float vv = v->voltage_average();
        if (vv > 0.1f) {
            _board_voltage = vv;
        }
    }
#endif
    return _board_voltage;
}

} // namespace RTT
