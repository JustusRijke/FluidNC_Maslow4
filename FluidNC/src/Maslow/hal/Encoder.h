/*
Wrapper for the Arduino library for AS5600 magnetic rotation meters used for the belt position encoders.

Its function is to 
- abstract away the hardware implementation details
- provide only the necessary functionality for the Maslow class to interact with the encoders
*/

#pragma once

#include "../drivers/AS5600.h"

#include <cstdint>

class Encoder {
public:
    explicit Encoder(uint8_t port);
    bool     is_connected();
    uint16_t get_position();

private:
    uint8_t _port;
    AS5600  _rotation_meter;

    void select_i2c_port() const;
};
