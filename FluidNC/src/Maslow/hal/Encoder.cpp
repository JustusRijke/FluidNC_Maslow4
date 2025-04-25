#include "Encoder.h"

#include "../../Machine/MachineConfig.h"

Encoder::Encoder(uint8_t port) : _port(port) {};

inline void Encoder::select_i2c_port() const {
    _i2c_switch->select_port(_port);
}

bool Encoder::is_connected() {
    if (_i2c_switch == nullptr) {
        log_error("Encoder: I2C switch not defined");
        return false;
    }
    select_i2c_port();
    return (_rotation_meter.begin());
}

uint16_t Encoder::get_position() {
    select_i2c_port();
    //_rotation_meter.begin();
    return (_rotation_meter.rawAngle());
}
