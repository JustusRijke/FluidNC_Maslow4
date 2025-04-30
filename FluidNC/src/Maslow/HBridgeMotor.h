/*
Driver for TI DRV8876 H-Bridge Motor Driver
Latched in PWM Control Mode
Datasheet: https://www.ti.com/lit/ds/symlink/drv8876.pdf

TODO: Split into driver and hal classes
*/
#pragma once

#include "../Configuration/Configurable.h"

class HBridgeMotor : public Configuration::Configurable {
public:
    HBridgeMotor() = default;

    bool init();

    void stop() { stop(false); }
    void stop(bool brake);
    void forward(uint32_t duty);
    void reverse(uint32_t duty);

private:
    // Configuration
    Pin         _current_sense_pin;
    Pin         _fwd_pin;
    Pin         _rev_pin;
    uint32_t    _frequency = 4000;

    uint32_t _max_duty;

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
