/*
Driver for TI DRV8876 H-Bridge Motor Driver
Latched in PWM Control Mode
Datasheet: https://www.ti.com/lit/ds/symlink/drv8876.pdf

TODO: Split into driver and hal classes
*/
#pragma once

#include "../Configuration/Configurable.h"
#include "utils/RollingAverage.hpp"

class HBridgeMotor : public Configuration::Configurable {
public:
    HBridgeMotor() = default;

    bool init(uint8_t cycle_time);
    void update();

    void  set_torque(float torque);
    float get_torque();
    void  stop(bool coast = false);

    float get_current();

    // Status flags
    // IMPORTANT: Caller is responsible for taking action (e.g. stop motor)
    bool overcurrent_warning = false;
    bool overcurrent_error   = false;

private:
    // Configuration
    Pin      _fwd_pin;
    Pin      _rev_pin;
    uint32_t _frequency = 4000;  // Hz
    Pin      _current_sense_pin;
    uint32_t _current_sense_resistor        = 1500;  // Ohm
    float    _overcurrent_warning_threshold = 0.9;   // Amperes
    float    _overcurrent_error_threshold   = 1.4;   // Amperes
    bool     _reverse                       = false;  // Reverse motor direction

    static constexpr float FLOAT_NEAR_ZERO = std::numeric_limits<float>::epsilon();  // Filter near-zero float values

    uint32_t _max_duty = 0;
    float    _torque_act = 0.0f;  // Calculated actual torque
    float    _torque_set = 0.0f;  // Target torque setpoint
    float    _current   = 0.0f;

    uint8_t _cycle_time = 0;  // [ms] Time between calls to update()

    RollingAverage<50> _rolling_average_current;  // 50 samples x 10ms cycles = 500ms rolling average filter for current measurement

    void update_pwm_outputs();  // Update PWM outputs based on the calculated actual torque

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
