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
    if (!_sda.defined() || !_scl.defined()) {
        log_error("I2C Switch: pins not defined");
        return false;
    }

    log_info("I2C Switch: using pins " << _scl.index() << " (SCL) and " << _sda.index() << " (SDA), " << _frequency << " Hz, address "
                                       << to_hex(_address));

    // FluidNC has no ESP32S3 I2C support, fall back to the Arduino Wire library.
    if (Wire.begin(_sda.index(), _scl.index(), _frequency)) {
        _i2c_mux.begin(_address, Wire);  // TODO: returns false even though the device is connected. Investigate.
        return true;
    }

    return false;
}

void I2CSwitch::select_port(uint8_t port) {
    _i2c_mux.setPort(port);
}

void I2CSwitch::group(Configuration::HandlerBase& handler) {
    handler.item("scl_pin", _scl);
    handler.item("sda_pin", _sda);
    handler.item("frequency", _frequency, 100000, 1000000);
    handler.item("address", _address);
}