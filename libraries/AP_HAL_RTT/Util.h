/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RTT_Namespace.h"

class RTT::Util : public AP_HAL::Util
{
public:
    uint32_t available_memory() override;
    void set_hw_rtc(uint64_t time_utc_usec) override;
    uint64_t get_hw_rtc() const override;
    void thread_info(ExpandingString& str) override;

    /** Used by system.cpp for AP_HAL::millis() / micros64() (RTT port). */
    uint32_t get_millis() const;
    uint64_t get_micros64() const;
};
