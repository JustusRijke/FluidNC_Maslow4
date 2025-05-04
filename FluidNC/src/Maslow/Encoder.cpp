#include "Encoder.h"

#include "../Machine/MachineConfig.h"

bool Encoder::init(I2CSwitch* i2c_switch) {
    _i2c_switch = i2c_switch;

    if (_i2c_switch == nullptr) {
        p_log_config_error("I2C switch not defined");
        return false;
    }

    _i2c_switch->select_port(_port);
    if (!_rotation_meter.begin()) {
        p_log_config_error("Not found at port " << _port);
        return false;
    }

    if (!_rotation_meter.detectMagnet()) {
        p_log_config_error("Magnet not detected");
        return false;
    }

    p_log_info("Initialized (port " << _port << ", angle: " << _rotation_meter.rawAngle() << ").");
    return true;
}

uint16_t Encoder::get_position() {
    _i2c_switch->select_port(_port);
    return (_rotation_meter.rawAngle());
}

void Encoder::group(Configuration::HandlerBase& handler) {
    handler.item("port", _port, 0, 3);
}