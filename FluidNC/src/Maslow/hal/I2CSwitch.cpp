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
    constexpr int      MCU_PIN_SDA   = 5;       // I2C SDA pin
    constexpr int      MCU_PIN_SCL   = 4;       // I2C SCL pin
    constexpr uint32_t I2C_FREQUENCY = 400000;  // I2C bus frequency
    constexpr uint8_t  TCAADDR       = 0x70;    // I2C address of TCA9546A switch
}

// Returns the single instance of I2CSwitch.
I2CSwitch& I2CSwitch::instance() {
    static I2CSwitch instance;  // Guaranteed to be created once (C++11+ thread-safe)
    return instance;
}

// Private constructor to enforce singleton.
I2CSwitch::I2CSwitch() {}

void I2CSwitch::select_port(uint8_t port) {
    _i2c_mux.setPort(port);
}

bool I2CSwitch::init() {
    // FluidNC has no ESP32S3 I2C support, fall back to the Arduino Wire library.
    if (Wire.begin(MCU_PIN_SDA, MCU_PIN_SCL, I2C_FREQUENCY)) {
        _i2c_mux.begin(TCAADDR, Wire);  // TODO: returns false even though the device is connected. Investigate.
        return true;
    }
    return false;
}
