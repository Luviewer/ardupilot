/*
 * ArduPilot + RT-Thread HAL - RCInput stub (Phase 0)
 */

#pragma once

#include <AP_HAL/RCInput.h>
#include "HAL_RTT_Namespace.h"

namespace RTT
{

class RCInput : public AP_HAL::RCInput
{
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
};

} // namespace RTT
