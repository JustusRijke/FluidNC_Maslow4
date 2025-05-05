#include "HBridgeMotor.h"

#include <Arduino.h>
#include <algorithm>

bool HBridgeMotor::init() {
    if (!_fwd_pin.defined() || !_rev_pin.defined() || !_current_sense_pin.defined()) {
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

    p_log_info("Initialized (FWD/IN1:" << _fwd_pin.name() << ", REV/IN2:" << _rev_pin.name() << ", IPROPI:" << _current_sense_pin.name()
                                       << ")");
    return true;
}

// Called periodically to check current limits
void HBridgeMotor::update() {
    float current       = get_current();
    overcurrent_error   = (current >= _overcurrent_error_threshold);
    overcurrent_warning = (current >= _overcurrent_warning_threshold);
}

// Speed: -1.0 (full reverse) to 1.0 (full forward). 0.0 is stop.
void HBridgeMotor::set_speed(float speed) {
    speed = std::clamp(speed, -1.0f, 1.0f);

    // Set the duty cycle based on the speed
    uint32_t duty = static_cast<uint32_t>(abs(speed) * _max_duty);

    if (speed < -FLOAT_NEAR_ZERO) {
        _rev_pin.setDuty(duty);
        _fwd_pin.setDuty(0);
    } else if (speed > FLOAT_NEAR_ZERO) {
        _fwd_pin.setDuty(duty);
        _rev_pin.setDuty(0);
    } else {  // NaN or near zero: stop
        _fwd_pin.setDuty(0);
        _rev_pin.setDuty(0);
    }
}

// Returns the current in Amperes
float HBridgeMotor::get_current() {
    // Convert IPROPI voltage (V) to motor current (I_OUT in Amperes)
    // Read the IPROPI pin voltage directly in millivolts (ESP32 specific)
    // Note: ESP32 ADCs are non-linear, the return value will be an estimate.
    // Filter the value to avoid noise by using a low-pass filter (capacitor),
    // or by averaging multiple readings - this is not implemented here.
    return (float)analogReadMilliVolts(_current_sense_pin.index()) / _current_sense_resistor;
}

void HBridgeMotor::group(Configuration::HandlerBase& handler) {
    handler.item("fwd_pin", _fwd_pin);
    handler.item("rev_pin", _rev_pin);
    handler.item("frequency", _frequency, 1000, 100000);
    handler.item("current_sense_pin", _current_sense_pin);
    handler.item("current_sense_resistor", _current_sense_resistor, 1, 100000);
    handler.item("overcurrent_warning", _overcurrent_warning_threshold, 0.1f, 100.0f);
    handler.item("overcurrent_error", _overcurrent_error_threshold, 0.1f, 100.0f);
}