/*
 * ArduPilot + RT-Thread HAL - RCOutput stub (Phase 0)
 */

#pragma once

#include <AP_HAL/RCOutput.h>
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class RCOutput : public AP_HAL::RCOutput
{
public:
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;
    void enable_ch(uint8_t chan) override;
    void disable_ch(uint8_t chan) override;
    void write(uint8_t chan, uint16_t period_us) override;
    void cork() override;
    void push() override;
    uint16_t read(uint8_t chan) override;
    void read(uint16_t* period_us, uint8_t len) override;
};

} // namespace RTT
