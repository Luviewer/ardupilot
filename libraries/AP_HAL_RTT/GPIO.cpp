/*
 * ArduPilot + RT-Thread HAL - GPIO stub implementation (Phase 0)
 */

#include "GPIO.h"

namespace RTT
{

void DigitalSource::mode(uint8_t output) { (void)output; }
uint8_t DigitalSource::read() { return 0; }
void DigitalSource::write(uint8_t value) { (void)value; }
void DigitalSource::toggle() {}

void GPIO::init() {}
void GPIO::pinMode(uint8_t pin, uint8_t output) { (void)pin; (void)output; }
uint8_t GPIO::read(uint8_t pin) { (void)pin; return 0; }
void GPIO::write(uint8_t pin, uint8_t value) { (void)pin; (void)value; }
void GPIO::toggle(uint8_t pin) { (void)pin; }
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) { (void)n; return nullptr; }
bool GPIO::usb_connected() { return false; }

} // namespace RTT
