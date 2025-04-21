#include "Encoder.h"

#include "I2CSwitch.h"

Encoder::Encoder(uint8_t port) : _port(port) {};

inline void Encoder::select_i2c_port() const {
    I2CSwitch::instance().select_port(_port);
}

bool Encoder::is_connected() {
    select_i2c_port();
    return (_rotation_meter.begin());
}

uint16_t Encoder::get_position() {
    select_i2c_port();
    _rotation_meter.begin();
    return (_rotation_meter.rawAngle());
}
