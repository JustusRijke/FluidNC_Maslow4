#include "HBridgeMotor.h"

bool HBridgeMotor::init() {
    if (!_fwd_pin.defined() || !_rev_pin.defined()|| !_current_sense_pin.defined()) {
        log_error("H-Bridge Motor: missing pin configurations");
        return false;
    }

    if (_fwd_pin.capabilities().has(Pin::Capabilities::PWM)) {
        _fwd_pin.setAttr(Pin::Attr::PWM, _frequency);
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

    _max_duty = _fwd_pin.maxDuty();

    log_info("H-Bridge Motor: using " << _fwd_pin.name() << " (FWD/IN1), " << _rev_pin.name() << " (REV/IN2), " << _current_sense_pin.name() << " (IPROPI), PWM=" << _frequency << " Hz, max.duty " << _max_duty);

    return true;
}


void HBridgeMotor::stop(bool brake) {    
    if (brake) {
        _fwd_pin.setDuty(_max_duty);
        _rev_pin.setDuty(_max_duty);
    } else {
        _fwd_pin.setDuty(0);
        _rev_pin.setDuty(0);
    }
}

void HBridgeMotor::forward(uint32_t duty) {
    _fwd_pin.setDuty(duty);
    _rev_pin.setDuty(0);
   }
void HBridgeMotor::reverse(uint32_t duty) {
    _fwd_pin.setDuty(0);
    _rev_pin.setDuty(duty);
}

void HBridgeMotor::group(Configuration::HandlerBase& handler) {
    handler.item("fwd_pin", _fwd_pin);
    handler.item("rev_pin", _rev_pin);
    handler.item("current_sense_pin", _current_sense_pin);
    handler.item("frequency", _frequency, 1000, 100000);
}