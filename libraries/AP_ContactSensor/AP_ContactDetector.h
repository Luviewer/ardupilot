#pragma once

#include <stdint.h>

// New-sample-driven hysteresis detector.  Flight loops may run faster than the
// sensor; callers must invoke update only once for each new sensor sample.
class AP_ContactDetector
{
public:
    enum class Event : uint8_t {
        NONE,
        CONTACT,
        RELEASE,
    };

    void configure(float contact_on_n, float contact_off_n, uint8_t samples)
    {
        _contact_on_n = contact_on_n;
        _contact_off_n = contact_off_n < contact_on_n ? contact_off_n : contact_on_n;
        _samples = samples > 0 ? samples : 1;
    }

    void reset(bool contact = false)
    {
        _contact = contact;
        _on_count = 0;
        _off_count = 0;
    }

    Event update(float force_n)
    {
        if (!_contact) {
            _off_count = 0;
            _on_count = force_n >= _contact_on_n ? uint8_t(_on_count + 1U) : 0U;
            if (_on_count >= _samples) {
                _contact = true;
                _on_count = 0;
                return Event::CONTACT;
            }
        } else {
            _on_count = 0;
            _off_count = force_n < _contact_off_n ? uint8_t(_off_count + 1U) : 0U;
            if (_off_count >= _samples) {
                _contact = false;
                _off_count = 0;
                return Event::RELEASE;
            }
        }
        return Event::NONE;
    }

    bool contact() const
    {
        return _contact;
    }
    bool contact_candidate() const
    {
        return !_contact && _on_count > 0;
    }

private:
    float _contact_on_n = 0.5f;
    float _contact_off_n = 0.25f;
    uint8_t _samples = 3;
    uint8_t _on_count = 0;
    uint8_t _off_count = 0;
    bool _contact = false;
};
