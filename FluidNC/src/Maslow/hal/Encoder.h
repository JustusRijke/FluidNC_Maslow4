/*
Wrapper for the Arduino library for AS5600 magnetic rotation meters used for the belt position encoders.

Its function is to 
- abstract away the hardware implementation details
- provide only the necessary functionality for the Maslow class to interact with the encoders
*/

#pragma once

#include "../../Configuration/Configurable.h"
#include "../drivers/AS5600.h"
#include "I2CSwitch.h"

#include <cstdint>

class Encoder : public Configuration::Configurable {
public:
    Encoder() = default;

    bool init(I2CSwitch* i2c_switch);

    uint16_t get_position();

private:
    AS5600  _rotation_meter;

    I2CSwitch* _i2c_switch = nullptr;
    uint8_t    _port       = 0;  // I2C port for the encoder

    // Configuration handlers
    void group(Configuration::HandlerBase& handler) override;
};
