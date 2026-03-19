/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include "HAL_RTT_Namespace.h"

class HAL_RTT : public AP_HAL::HAL
{
public:
    HAL_RTT();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
