#include "HBridgeMotor.h"
#include "Arduino.h"

bool HBridgeMotor::init() {
    if (!_fwd_pin.defined() || !_rev_pin.defined()|| !_current_sense_pin.defined()) {
        log_error("H-Bridge Motor: missing pin configurations");
        return false;
    }

    if (_fwd_pin.capabilities().has(Pin::Capabilities::PWM)) {
        // _fwd_pin.setAttr(Pin::Attr::PWM, _frequency);
        log_info("Pin attr: "<<_fwd_pin.getAttr())
        ledcSetup(0, 16000, 10);  // configure PWM functionalities...this uses timer 0 (channel, freq, resolution)
        ledcAttachPin(_fwd_pin.index(), 0);
    } else
    {
        log_error("H-Bridge Motor: " << _fwd_pin.name() << " cannot be used as PWM output");
        return false;
    }

    if (_rev_pin.capabilities().has(Pin::Capabilities::PWM)) {
        _rev_pin.setAttr(Pin::Attr::PWM, _frequency);
    } else   
    {    
        log_error("H-Bridge Motor: " << _rev_pin.name() << " cannot be used as PWM output");
        return false;
    }

    log_info("H-Bridge Motor: using " << _fwd_pin.name() << " (FWD/IN1), " << _rev_pin.name() << " (REV/IN2), " << _current_sense_pin.name() << " (IPROPI), PWM=" << _frequency << " Hz");

    return true;
}

void HBridgeMotor::group(Configuration::HandlerBase& handler) {
    handler.item("fwd_pin", _fwd_pin);
    handler.item("rev_pin", _rev_pin);
    handler.item("current_sense_pin", _current_sense_pin);
    handler.item("frequency", _frequency, 1000, 100000);
}