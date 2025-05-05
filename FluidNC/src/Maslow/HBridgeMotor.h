/*
Driver for TI DRV8876 H-Bridge Motor Driver
Latched in PWM Control Mode
Datasheet: https://www.ti.com/lit/ds/symlink/drv8876.pdf

TODO: Split into driver and hal classes
*/
#pragma once

#include "../Configuration/Configurable.h"
#include "utils/HierarchicalLog.hpp"

class HBridgeMotor : public Configuration::Configurable, public HierarchicalLog {
public:
    HBridgeMotor() = default;

    bool init();
    void update();

    void set_speed(float speed);
    void stop() { set_speed(0); };

    float get_current();

    // Status flags
    bool overcurrent_warning = false;
    bool overcurrent_error   = false;

private:
    // Configuration
    Pin      _fwd_pin;
    Pin      _rev_pin;
    uint32_t _frequency = 4000;  // Hz
    Pin      _current_sense_pin;
    uint32_t _current_sense_resistor        = 1500;  // Ohm
    float    _overcurrent_warning_threshold = 1.0;   // Amperes
    float    _overcurrent_error_threshold   = 1.5;   // Amperes

    static constexpr float FLOAT_NEAR_ZERO = 0.01f;  // Filter near-zero float values

    uint32_t _max_duty;

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
