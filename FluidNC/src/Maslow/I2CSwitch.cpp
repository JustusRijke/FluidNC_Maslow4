#include "I2CSwitch.h"

#include "../NutsBolts.h"

#include <Arduino.h>

bool I2CSwitch::init() {
    if (!_sda_pin.defined() || !_scl_pin.defined()) {
        p_log_config_error("Pins not defined");
        return false;
    }

    // FluidNC has no ESP32S3 I2C support, fall back to the Arduino Wire library.
    if (Wire.begin(_sda_pin.index(), _scl_pin.index(), _frequency)) {
        _i2c_mux = new TCA9548(_address, &Wire);
        if (!_i2c_mux->isConnected()) {
            p_log_config_error("I2C switch not found");
            return false;
        }
    } else {
        p_log_config_error("I2C bus not initialized");
        return false;
    }

    p_log_info("Initialized (" << _scl_pin.name() << " (SCL), " << _sda_pin.name() << " (SDA), " << _frequency << "Hz, address "
                                  << to_hex(_address) << ").");
    return true;
}

void I2CSwitch::select_port(uint8_t port) {
    _i2c_mux->selectChannel(port);
}

void I2CSwitch::group(Configuration::HandlerBase& handler) {
    handler.item("scl_pin", _scl_pin);
    handler.item("sda_pin", _sda_pin);
    handler.item("frequency", _frequency, 100000, 1000000);
    handler.item("address", _address);
}