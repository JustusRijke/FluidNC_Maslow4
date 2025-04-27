/*
Driver for TI DRV8876 H-Bridge Motor Driver
Latched in PWM Control Mode
Datasheet: https://www.ti.com/lit/ds/symlink/drv8876.pdf

TODO: Split into driver and hal classes
*/
#pragma once

#include "../../Configuration/Configurable.h"

class HBridgeMotor : public Configuration::Configurable {
public:
    HBridgeMotor() = default;

    bool init();


private:
    // Configuration
    Pin         _current_sense_pin;
    Pin         _fwd_pin;
    Pin         _rev_pin;
    uint32_t    _frequency = 16000;

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
