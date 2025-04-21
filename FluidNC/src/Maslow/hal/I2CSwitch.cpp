/*
Wrapper for the SparkFun I2C Mux library used for the TCA9546A I2C switch, 
which is used to access the I2C port for each of the 4 belt position encoders.

Its function is to 
- provide a single point of access to the I2C switch by using a singleton pattern
- abstract away the hardware implementation details
*/

#include "I2CSwitch.h"

#include <Arduino.h>

// Private namespace for local constants (prefered over #define)
namespace {
    constexpr uint32_t I2C_FREQUENCY = 400000;  // I2C bus frequency
    constexpr uint8_t  TCAADDR       = 0x70;    // I2C address of TCA9546A switch
}

bool I2CSwitch::init() {
    if (!_sda.defined() || !_scl.defined()) {
        log_error("I2C Switch: pins not defined");
        return false;
    }

    log_info("I2C Switch: using pins " << _scl.index() << " (SCL) and " << _sda.index() << " (SDA)");

    // FluidNC has no ESP32S3 I2C support, fall back to the Arduino Wire library.
    if (Wire.begin(_sda.index(), _scl.index(), I2C_FREQUENCY)) {
        _i2c_mux.begin(TCAADDR, Wire);  // TODO: returns false even though the device is connected. Investigate.
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
    // handler.item("delay_ms", _delay_ms, 0, 10000);
}