/*
Wrapper for the TCA9546A I2C switch, which is used to access the I2C port for each of the 4 belt position encoders.

Its function is to 
- provide access to the I2C switch
- build a bridge between the hardware and the FluidNC machine configuration architecture
*/
#pragma once

#include "../Configuration/Configurable.h"
#include "TCA9548.h"
#include <cstdint>

class I2CSwitch : public Configuration::Configurable {
public:
    I2CSwitch() = default;

    // Configuration
    Pin      _sda_pin;
    Pin      _scl_pin;
    uint8_t  _address   = 0x70;    // I2C address of TCA9546A switch
    uint32_t _frequency = 400000;  // I2C bus frequency

    bool init();
    void select_port(uint8_t port);

private:
    TCA9548* _i2c_mux = nullptr;  // TCA9546A I2C switch hardware driver

    // Configuration handlers
    void group(Configuration::HandlerBase& handler) override;
};
