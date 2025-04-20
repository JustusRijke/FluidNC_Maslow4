// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Maslow.h"
#include "../Logging.h"
#include "hal/I2CSwitch.h"

// TODO: Remove after debugging
extern TaskHandle_t maslowTaskHandle;

// Private namespace for local constants (prefered over #define)
namespace {
    constexpr uint32_t MS_PER_CYCLE = 5;  // Expected time between consecutive calls to cycle()
}

// Returns the single instance of Maslow.
Maslow& Maslow::instance() {
    static Maslow instance;  // Guaranteed to be created once (C++11+ thread-safe)
    return instance;
}

// Private constructor to enforce singleton.
Maslow::Maslow() : _sm(MS_PER_CYCLE) {
    _sm.state = State::Entrypoint;
}

// Maslow initialization logic.
void Maslow::init() {
    if (!I2CSwitch::instance().init()) {
        log_error("I2C bus initialization failed");
        return;
    }

    for (size_t i = 0; i < _encoders.size(); ++i) {
        if (!_encoders[i].is_connected()) {
            log_error("Encoder initialization failed");
            return;
        }
    }

    // //Check for the presence of the magnet
    // if (!encoder.detectMagnet()) {
    //     log_warn("Magnet not detected");
    // }
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
            if ((_sm.state_changed) || (_sm.time_in_state() > 1000)) {
                _sm.reset_time_in_state();

                log_info("Raw angle: " << position);

                // Log Maslow task stack size for debugging
                UBaseType_t stackHWM_Words = uxTaskGetStackHighWaterMark(maslowTaskHandle);
                log_info("Maslow task stack High Water Mark (HWM): " << stackHWM_Words << " bytes free");
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