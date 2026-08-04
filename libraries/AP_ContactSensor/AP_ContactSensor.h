#pragma once

#include <AP_Math/AP_Math.h>

// Common contact sensor interface. The wrench API leaves room for a tool-tip
// six-axis force/torque sensor without coupling flight modes to a driver.
class AP_ContactSensor
{
public:
    struct ForceSample {
        float tool_force_n;
        int32_t raw_value;
        uint32_t timestamp_ms;
        uint32_t sequence;
    };

    struct WrenchSample {
        Vector3f force_n;
        Vector3f torque_nm;
        uint32_t timestamp_ms;
        uint32_t sequence;
    };

    virtual ~AP_ContactSensor() = default;

    // A returned historical sample may be stale; callers must also check healthy().
    virtual bool get_force_sample(ForceSample &sample) const = 0;
    virtual bool healthy() const = 0;
    virtual bool tare() = 0;
    virtual bool tare_complete() const = 0;
    virtual bool get_wrench_sample(WrenchSample &sample) const
    {
        return false;
    }
};
