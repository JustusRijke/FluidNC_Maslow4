#include "Encoder.h"

bool Encoder::init(I2CSwitch* i2c_switch, uint8_t cycle_time) {
    _i2c_switch = i2c_switch;
    _cycle_time = cycle_time;

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

    if (_rotation_meter.magnetTooStrong()) {
        p_log_warn("Magnet too strong");
    }

    if (_rotation_meter.magnetTooWeak()) {
        p_log_warn("Magnet too weak");
    }

    // TODO: encoder has tuning options (filters etc), useful?

    set_mm_per_revolution(_mm_per_revolution);

    p_log_info("Initialized (port:" << _port << ", raw angle:" << _rotation_meter.rawAngle() << ")");
    return true;
}

void Encoder::update() {
    _i2c_switch->select_port(_port);
    _revolutions_prev = _revolutions;
    _revolutions = _rotation_meter.getCumulativePosition(true);
}

// Set mm per revolution of the encoder gear
void Encoder::set_mm_per_revolution(float mm_per_revolution) {
    // Invert _revolutions_to_mm to reverse direction without altering other logic
    _mm_per_revolution = _invert ? -mm_per_revolution : mm_per_revolution;
    _revolutions_to_mm = _mm_per_revolution / 4096.0f;
}

// Get mm per revolution of the encoder gear
float Encoder::get_mm_per_revolution() {
    return _invert ? -_mm_per_revolution : _mm_per_revolution;
    ;
}

// Return position of the encoder in mm
float Encoder::get_position() {
    return static_cast<double>(_revolutions) * _revolutions_to_mm;
}

// Set the encoder position in mm
void Encoder::set_position(float position = 0.0f) {
    _rotation_meter.resetCumulativePosition(static_cast<int32_t>(position / _revolutions_to_mm));
    update();
}

// Get velocity in mm/s
float Encoder::get_velocity() {
    return (static_cast<float>(_revolutions - _revolutions_prev) * _revolutions_to_mm) / (_cycle_time / 1000.0f);
}

void Encoder::group(Configuration::HandlerBase& handler) {
    handler.item("port", _port, 0, 3);
    handler.item("mm_per_revolution", _mm_per_revolution, 0.0f, 10000.0f);
    handler.item("invert", _invert);
}