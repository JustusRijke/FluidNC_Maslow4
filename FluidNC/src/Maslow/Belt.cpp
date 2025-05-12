// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Belt.h"

#include "../Logging.h"
#include "../Machine/MachineConfig.h"

// Belt initialization logic.
bool Belt::init(I2CSwitch* i2c_switch, uint8_t cycle_time) {
    if (_encoder == nullptr) {
        p_log_config_error("Encoder not defined");
        return false;
    }

    if (_motor == nullptr) {
        p_log_config_error("Motor not defined");
        return false;
    }

    // Initialize the encoder with the I2C switch (dependency injection)
    if (!_encoder->init(i2c_switch))
        return false;

    if (!_motor->init())
        return false;

    _sm.ms_per_cycle = cycle_time;
    _sm.state        = eState::Entrypoint;

    p_log_info("Initialized.");
    return true;
}

// The main Belt state machine loop, called cyclically.
void Belt::update() {
    _motor->update();
    _prev_position = _encoder->get_position();
    _encoder->update();

    if (_max_direction_errors > 0) {
        // Check if direction of encoder and motor match up.
        // If not, configuration or hardware could be wrong,
        // e.g., floating pin issue on Maslow PCB 4.0.

        //TODO: check if encoder is moving at all (only after motor has been moving for a while)
        //TODO: Add "velocity" readback on encoder class
        //TODO: what if direction of motor changes suddenly (inertia)
        bool  extending = (_prev_position < _encoder->get_position());
        float speed     = _motor->get_speed();
        if ((extending && speed < 0.0f) || (!extending && speed > 0.0f)) {
            _direction_errors++;
            if (_direction_errors > _max_direction_errors) {
                p_log_error("Encoder and motor direction mismatch");
                _direction_errors = 0;  // Avoid re-triggering the error
                _sm.state         = eState::Error;
            }
        } else {
            // Motor stopped or direction OK, reset counter
            _direction_errors = 0;
        }
    }

    _sm.update();

    switch (_sm.state) {
        case eState::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = eState::WaitForCommand;
            break;

        case eState::WaitForCommand:
            break;

        case eState::Retract:
            if (_sm.state_changed) {
                _motor->set_speed(-_retract_speed);  // Set motor speed to retract
            }
            if (_motor->overcurrent_error) {
                // If the motor is in overcurrent error, stop the motor
                _motor->stop();
                p_log_error("Motor overcurrent error");
                _status   = BeltStatus::COMPLETED_ERROR;
                _sm.state = eState::Error;
            }
            if (_motor->get_current() > _retract_current) {
                // If the motor current exceeds the threshold, stop the motor
                _motor->stop();
                _encoder->set_position(0.0f);
                _status   = BeltStatus::COMPLETED_SUCCESS;
                _sm.state = eState::WaitForCommand;
            }
            // TODO: timeout for retracting?
            break;

        case eState::Error:
            // Only way to get out of this state is calling reset()
            if (_sm.state_changed) {
                _motor->stop();
            }
            break;

        case eState::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            if (_sm.state_changed) {
                p_log_fatal("Entered undefined state");
            }
            break;

        default:
            _sm.state = eState::Undefined;
            break;
    }
}

void Belt::retract() {
    if (_sm.state == eState::WaitForCommand) {
        _sm.state = eState::Retract;
        _status   = BeltStatus::BUSY;
    } else if (_sm.state == eState::Retract) {
        p_log_warn("Already retracting");
    } else {
        _status = BeltStatus::COMPLETED_ERROR;
        p_log_error("Cannot retract while in state " << static_cast<uint16_t>(_sm.state));
    }
}

// Hard reset: stop motor and return to an idle state
void Belt::reset() {
    _motor->stop();
    _sm.state = eState::WaitForCommand;
    _status   = BeltStatus::IDLE;
}

void Belt::group(Configuration::HandlerBase& handler) {
    handler.section("encoder", _encoder);
    handler.section("motor", _motor);

    handler.item("retract_speed", _retract_speed, 0.01f, 1.0f);
    handler.item("retract_current", _retract_current, 0.01f, 100.0f);
    handler.item("max_direction_errors", _max_direction_errors, 0, 100);
}