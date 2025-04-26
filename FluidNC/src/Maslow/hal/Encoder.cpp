#include "Encoder.h"

#include "../../Machine/MachineConfig.h"

bool Encoder::init(I2CSwitch* i2c_switch) {
    _i2c_switch = i2c_switch;

    if (_i2c_switch == nullptr) {
        log_error("I2C switch not defined");
        return false;
    }

    _i2c_switch->select_port(_port);
    if (!_rotation_meter.begin()) {
        log_error("Encoder not found at port " << _port);
        return false;
    }

    if (!_rotation_meter.detectMagnet()) {
        log_error("Magnet not detected");
    }

    log_info("Encoder at port " << _port << " initialized. Current angle: " << _rotation_meter.rawAngle());

    return true;
}

uint16_t Encoder::get_position() {
    _i2c_switch->select_port(_port);
    return (_rotation_meter.rawAngle());
}

void Encoder::group(Configuration::HandlerBase& handler) {
    handler.item("port", _port, 0, 3);
}