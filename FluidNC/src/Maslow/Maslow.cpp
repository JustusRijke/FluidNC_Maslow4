// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Maslow.h"

#include "../Logging.h"

// TODO: Remove after debugging
extern TaskHandle_t maslowTaskHandle;

// Private namespace for local constants (prefered over #define)
namespace {
    constexpr uint32_t MS_PER_CYCLE = 5;     // [ms] Expected time between consecutive calls to cycle()
    constexpr uint32_t REPORT_DELAY = 5000;  // [ms] Time between reporting stuff to the console
}

// Constructor for the Maslow class. Initializes the state machine.
Maslow::Maslow() : _sm(MS_PER_CYCLE) {
    _sm.state = State::Entrypoint;
}

// Maslow initialization logic.
_Bool Maslow::init() {
    if (_i2c_switch == nullptr) {
        log_error("Missing config: i2c_switch");
        return false;
    }

    if (!_i2c_switch->init())
        return false;

    for (size_t i = 0; i < _encoders.size(); ++i) {
        if (!_encoders[i].is_connected()) {
            log_error("Encoder initialization failed");
            return false;
        }
    }

    // //Check for the presence of the magnet
    // if (!encoder.detectMagnet()) {
    //     log_warn("Magnet not detected");
    // }

    return true;
}

// The main Maslow state machine loop, called cyclically.
void Maslow::cycle() {
    // Cycle time measurement
    //_cycle_stats.track_cycles();

    // For debugging: combine all positions so changes are easy to see in the terminal
    position = 0;
    for (size_t i = 0; i < _encoders.size(); ++i) {
        position += _encoders[i].get_position();
    }

    // Update the state machine, so we can check state changes and time spent in state.
    _sm.update();

    switch (_sm.state) {
        case State::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = State::Report;
            break;

        case State::Report:
            log_state_change("Entered state 'Report'");
            if ((_sm.state_changed) || (_sm.time_in_state() > REPORT_DELAY)) {
                _sm.reset_time_in_state();

                log_info("Raw angle: " << position);

                // Log Maslow task stack size for debugging
                UBaseType_t stackHWM_Words = uxTaskGetStackHighWaterMark(maslowTaskHandle);
                log_info("Maslow task stack High Water Mark (HWM): " << stackHWM_Words << " bytes free");
            }
            break;

        case State::Test:
            log_state_change("Entered state 'Test'");
            if (_sm.time_in_state() > 3000) {
                log_info("Test state timeout, moving to Report state");
                _sm.state = State::Report;
            }
            break;

        case State::FatalError:
            // TODO - Add code to handle fatal error, e.g. stop motors, turn off power, etc.
            break;

        case State::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            log_error("Entered undefined state");
            _sm.state = State::FatalError;
            break;

        default:
            _sm.state = State::Undefined;
            break;
    }
}

// Helper function to log state changes. Only logs once per state change.
inline void Maslow::log_state_change(const char* msg) {
    if (_sm.state_changed) {
        log_info(std::string("Maslow: ") + msg);
    }
}

// External function to request a state change.
void Maslow::request_state_change(State new_state) {
    // Add conditional logic here if needed, for example
    // check if the new state is valid or if the current state allows the transition.
    _sm.state = new_state;
}

void Maslow::group(Configuration::HandlerBase& handler) {
    handler.section("i2c_switch", _i2c_switch);
}