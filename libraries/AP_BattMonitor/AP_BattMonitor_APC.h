#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

class AP_BattMonitor_APC : public AP_BattMonitor_Analog
{
public:
    /// Constructor
    AP_BattMonitor_APC(AP_BattMonitor &mon,
                       AP_BattMonitor::BattMonitor_State &mon_state,
                       AP_BattMonitor_Params &params);

    bool has_consumed_energy() const override { return false; }

    bool has_current() const override { return false; }

    void init(void) override {}

    /// Read the battery voltage.
    void read() override;

private:
  
    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t pulse_width_us;
        uint32_t pulse_count1;
    } irq_state;

    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);


    int8_t last_pin = -1;
    uint32_t pulse_count2;
};