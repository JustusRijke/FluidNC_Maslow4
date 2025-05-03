// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Maslow.h"

#include "../Logging.h"

// TODO: Remove after debugging
extern TaskHandle_t maslowTaskHandle;

// Constructor for the Maslow class. Initializes the state machine.
Maslow::Maslow() : _sm(_state_names) {
    _sm.ms_per_cycle = _cycle_time;
    _sm.state        = eState::Entrypoint;
}

// Maslow initialization logic.
bool Maslow::init() {
    if (_i2c_switch == nullptr) {
        log_error("Missing config: i2c_switch");
        return false;
    }

    if (!_i2c_switch->init())
        return false;

    for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
        if (_belts[i] == nullptr) {
            log_error("Missing config: belt " << BELT_NAMES[i]);
            return false;
        }
        if (!_belts[i]->init(BELT_NAMES[i], _i2c_switch)) {
            log_error("Failed to initialize belt " << BELT_NAMES[i]);
            return false;
        }
    }

    return true;
}

// The main Maslow state machine loop, called cyclically.
void Maslow::cycle() {
    // Cycle time measurement
    _cycle_stats.track_cycles();

    // Update the state machine, so we can check state changes and time spent in state.
    _sm.update();

    switch (_sm.state) {
        case eState::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = eState::Report;
            break;

        case eState::Report:
            if ((_sm.state_changed) || (_sm.time_in_state() > 5000)) {
                _sm.reset_time_in_state();

                // log_info("Raw angle: " << position);

                // Log Maslow task stack size for debugging
                UBaseType_t stackHWM_Words = uxTaskGetStackHighWaterMark(maslowTaskHandle);
                log_info("Maslow task stack High Water Mark (HWM): " << stackHWM_Words << " bytes free");
            }
            break;

        case eState::Test:
            if (_sm.state_changed){
                _belts[static_cast<size_t>(eBelt::TopLeft)]->retract();}

            if ((_sm.time_in_state() > 500)  && (_sm.time_in_state() < 510)){
                _belts[static_cast<size_t>(eBelt::TopLeft)]->extent();
            }

            if (_sm.time_in_state() > 900) {
                _belts[static_cast<size_t>(eBelt::TopLeft)]->stop();
                log_info("Test state timeout, moving to Report state");
                _sm.state = eState::Report;
            }
            break;

        case eState::FatalError:
            // TODO - Add code to handle fatal error, e.g. stop motors, turn off power, etc.
            break;

        case eState::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            log_error("Entered undefined state");
            _sm.state = eState::FatalError;
            break;

        default:
            _sm.state = eState::Undefined;
            break;
    }
}

// External function to request a state change.
void Maslow::request_state_change(eState new_state) {
    // Add conditional logic here if needed, for example
    // check if the new state is valid or if the current state allows the transition.
    _sm.state = new_state;
}

void Maslow::group(Configuration::HandlerBase& handler) {
    handler.item("cycle_time", _cycle_time, 1, 100);

    handler.section("i2c_switch", _i2c_switch);

    for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
        handler.section(BELT_NAMES[i], _belts[i]);
    }
}