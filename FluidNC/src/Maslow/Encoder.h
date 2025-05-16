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

#include <cstdint>

class Encoder : public Configuration::Configurable {
public:
    Encoder() = default;

    bool init(I2CSwitch* i2c_switch, uint8_t cycle_time);
    void update();

    void  set_mm_per_revolution(float mm_per_revolution);
    float get_mm_per_revolution();
    float get_position();
    float get_velocity();
    void  set_position(float position);

private:
    // Components
    AS5600     _rotation_meter;        // Encoder hardware driver
    I2CSwitch* _i2c_switch = nullptr;  // I2C switch to which the encoder is connected

    // Configuration
    uint8_t    _port       = 0;  // I2C port for the rotation meter
    float      _mm_per_revolution = 44.0f;  // mm of movement per revolution of the encoder gear
    bool       _invert            = false;  // Invert the direction of the encoder

    int32_t _revolutions       = 0;     // Cumulative position of the rotation meter (4096=1 full revolution)
    int32_t _revolutions_prev  = 0;     // Position of the rotation meter at the last update
    float _revolutions_to_mm = 0.0f;  // helper variable to avoid division in get_position()

    uint8_t _cycle_time = 0;  // Time in ms per call to update(), used to determine velocity

    // Configuration handlers
    void group(Configuration::HandlerBase& handler) override;
};
