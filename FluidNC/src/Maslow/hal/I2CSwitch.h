/*
Wrapper for the SparkFun I2C Mux library used for the TCA9546A I2C switch, 
which is used to access the I2C port for each of the 4 belt position encoders.

Its function is to 
- provide a single point of access to the I2C switch by using a singleton pattern
- abstract away the hardware implementation details
*/

#pragma once

#include "../../Configuration/Configurable.h"
#include "../drivers/SparkFun_I2C_Mux_Arduino_Library.h"

#include <cstdint>

class I2CSwitch : public Configuration::Configurable {
public:
    Pin _sda;
    Pin _scl;

    I2CSwitch() = default;

    bool init();
    void select_port(uint8_t port);

private:
    QWIICMUX _i2c_mux;  // TCA9546A I2C switch hardware driver

    // Configuration handlers.
    void group(Configuration::HandlerBase& handler) override;

    ~I2CSwitch() = default;
};
