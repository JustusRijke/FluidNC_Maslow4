#include "HBridgeMotor.h"

#include <Arduino.h>
#include <algorithm>

bool HBridgeMotor::init(uint8_t cycle_time) {
    _cycle_time = cycle_time;

    if (_fwd_pin.undefined() || _rev_pin.undefined() || _current_sense_pin.undefined()) {
        p_log_config_error("Missing pin configurations");
        return false;
    }

    if (_fwd_pin.capabilities().has(Pin::Capabilities::PWM)) {
        _fwd_pin.setAttr(Pin::Attr::PWM, _frequency);
    } else {
        p_log_config_error(_fwd_pin.name() << " cannot be used as PWM output");
        return false;
    }

    if (_rev_pin.capabilities().has(Pin::Capabilities::PWM)) {
        _rev_pin.setAttr(Pin::Attr::PWM, _frequency);
    } else {
        p_log_config_error(_rev_pin.name() << " cannot be used as PWM output");
        return false;
    }

    if (!_current_sense_pin.capabilities().has(Pin::Capabilities::ADC)) {
        p_log_config_error(_current_sense_pin.name() << " cannot be used as analog input");
        return false;
    }

    _max_duty = _fwd_pin.maxDuty();

    p_log_debug("Initialized (FWD/IN1:" << _fwd_pin.name() << ", REV/IN2:" << _rev_pin.name() << ", IPROPI:" << _current_sense_pin.name()
                                        << ")");
    return true;
}

// Called periodically to check current limits
void HBridgeMotor::update() {
    // Read the current sense pin and convert to Amperes:
    // Convert IPROPI voltage (V) to motor current (I_OUT in Amperes)
    // Read the IPROPI pin voltage directly in millivolts (ESP32 specific)
    // Note: ESP32 ADCs are non-linear, the return value will be an estimate.
    // The x1.6 factor is emperical (based on actual measurements)
    // Store the rolling average to filter out noise.
    float sample = (float)analogReadMilliVolts(_current_sense_pin.index()) * 1.6f / _current_sense_resistor;
    _current     = _rolling_average_current.update(sample);

    // Check for overcurrent conditions
    overcurrent_error   = (_current >= _overcurrent_error_threshold);
    overcurrent_warning = (_current >= _overcurrent_warning_threshold);

    // Adjust torque until it reaches the setpoint
    // Ramp up slowly, ramp down immediately
    if (fabs(_torque_set - _torque_act) > FLOAT_NEAR_ZERO) {
        if (_torque_act * _torque_set < 0) {  // Changing direction: brake to zero
            _torque_act = 0.0f;
        } else if (fabs(_torque_set) < fabs(_torque_act)) {
            // Brake/reduce torque
            _torque_act = _torque_set;
        } else {
            // Ramp up torque
            const float STEP =
                0.1f;  // 0.01 sec (cycletime) / 0.1 step = 0.1s to 100% TODO: make this configurable, taking cycle time into account
            float delta = _torque_set - _torque_act;
            _torque_act += std::copysign(std::min(fabs(delta), STEP), delta);
        }
        update_pwm_outputs();
    } else  // Reached the setpoint
        _torque_act = _torque_set;
}

// Torque: -1.0 (full reverse) to 1.0 (full forward). 0.0 is stop.
void HBridgeMotor::set_torque(float torque) {
    _torque_set = std::clamp(torque, -1.0f, 1.0f);
}

// Return the actual torque (-1.0...1.0)
float HBridgeMotor::get_torque() {
    return _torque_act;
}

void HBridgeMotor::stop(bool coast) {
    _torque_act = 0.0f;
    _torque_set = 0.0f;
    if (coast) {
        _fwd_pin.setDuty(0);
        _rev_pin.setDuty(0);
    } else {  // brake
        _fwd_pin.setDuty(_max_duty);
        _rev_pin.setDuty(_max_duty);
    }
};

void HBridgeMotor::update_pwm_outputs() {
    // Set the duty cycle based on the torque
    uint32_t duty = static_cast<uint32_t>(abs(_torque_act) * _max_duty);

    float directed_torque = _reverse ? -_torque_act : _torque_act;
    if (directed_torque < -FLOAT_NEAR_ZERO) {
        _rev_pin.setDuty(duty);
        _fwd_pin.setDuty(0);
    } else if (directed_torque > FLOAT_NEAR_ZERO) {
        _fwd_pin.setDuty(duty);
        _rev_pin.setDuty(0);
    } else {  // NaN or near zero: brake
        _fwd_pin.setDuty(_max_duty);
        _rev_pin.setDuty(_max_duty);
    }
}

// Returns the current in Amperes
float HBridgeMotor::get_current() {
    return _current;
}

void HBridgeMotor::group(Configuration::HandlerBase& handler) {
    handler.item("fwd_pin", _fwd_pin);
    handler.item("rev_pin", _rev_pin);
    handler.item("frequency", _frequency, 1000, 100000);
    handler.item("current_sense_pin", _current_sense_pin);
    handler.item("current_sense_resistor", _current_sense_resistor, 1, 100000);
    handler.item("overcurrent_warning", _overcurrent_warning_threshold, 0.1f, 100.0f);
    handler.item("overcurrent_error", _overcurrent_error_threshold, 0.1f, 100.0f);
    handler.item("reverse", _reverse);
}