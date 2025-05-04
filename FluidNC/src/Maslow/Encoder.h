/*
Wrapper for the AS5600 magnetic rotation meters used for the belt position encoders.

Its function is to 
- provide access to the encoders via the I2C switch
- build a bridge between the hardware and the FluidNC machine configuration architecture
*/

#pragma once

#include "../Configuration/Configurable.h"
#include "AS5600.h"
#include "I2CSwitch.h"
#include "utils/HierarchicalLog.hpp"

#include <cstdint>

class Encoder : public Configuration::Configurable, public HierarchicalLog {
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
