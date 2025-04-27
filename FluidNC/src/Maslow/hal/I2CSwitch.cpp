/*
Wrapper for the SparkFun I2C Mux library used for the TCA9546A I2C switch, 
which is used to access the I2C port for each of the 4 belt position encoders.

Its function is to 
- provide access to the I2C switch
- abstract away the hardware implementation details
*/

#include "I2CSwitch.h"

#include "../../NutsBolts.h"

#include <Arduino.h>

bool I2CSwitch::init() {
    if (!_sda_pin.defined() || !_scl_pin.defined()) {
        log_error("I2C Switch: pins not defined");
        return false;
    }

    log_info("I2C Switch: using " << _scl_pin.name() << " (SCL) and " << _sda_pin.name() << " (SDA), " << _frequency << " Hz, address "
                                  << to_hex(_address));

    // FluidNC has no ESP32S3 I2C support, fall back to the Arduino Wire library.
    if (Wire.begin(_sda_pin.index(), _scl_pin.index(), _frequency)) {
        _i2c_mux.begin(_address, Wire);
        return true;
    }

    log_error("I2C Switch: failed to initialize");
    return false;
}

void I2CSwitch::select_port(uint8_t port) {
    _i2c_mux.setPort(port);
}

void I2CSwitch::group(Configuration::HandlerBase& handler) {
    handler.item("scl_pin", _scl_pin);
    handler.item("sda_pin", _sda_pin);
    handler.item("frequency", _frequency, 100000, 1000000);
    handler.item("address", _address);
}