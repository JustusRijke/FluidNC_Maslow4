// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Maslow.h"

#include "../Logging.h"
#include "../System.h"

// TODO: Remove after debugging
extern TaskHandle_t maslowTaskHandle;

// Constructor for the Maslow class. Initializes the state machine.
Maslow::Maslow() : _sm(_state_names, [this](const std::string& msg) { _log_state_change(msg); }) {
    _sm.ms_per_cycle = cycle_time;
    _sm.state        = eState::Entrypoint;
}

// Maslow initialization logic.
bool Maslow::init() {
    if (_i2c_switch == nullptr) {
        p_log_config_error("I2C Switch not defined");
        return false;
    }

    if (!_i2c_switch->init())
        return false;

    for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
        if (_belts[i] == nullptr) {
            p_log_config_error("Missing config: belt " << BELT_NAMES[i]);
            return false;
        }
        if (!_belts[i]->init(_i2c_switch, cycle_time)) {
            return false;
        }
    }

    if (_fan_pin.undefined()) {
        p_log_config_error("Missing fan pin configuration");
        return false;
    } else
        _fan_pin.setAttr(Pin::Attr::Output);

    p_log_debug("Initialized.");
    return true;
}

// The main Maslow state machine loop, called cyclically.
void Maslow::update() {
    // Cycle time measurement
    _cycle_stats.track_cycles(cycle_time * 1250);  // Warn when cycle time is exceeded by 25%

    // Activate fan if any belt requests it
    // TODO: check if Z axis is active
    bool activate_fan = false;
    for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
        _belts[i]->update();
        activate_fan |= _belts[i]->request_fan;
    }
    _fan_pin.synchronousWrite(activate_fan);

    // Reporting
    if (report_HWM) {
        // Log Maslow task stack size for debugging
        UBaseType_t stackHWM_Words = uxTaskGetStackHighWaterMark(maslowTaskHandle);
        p_log_info("Maslow task stack High Water Mark (HWM): " << stackHWM_Words << " bytes free");
        report_HWM = false;
    }

    // Update the state machine, so we can check state changes and time spent in state.
    _sm.update();

    switch (_sm.state) {
        case eState::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = eState::WaitForCommand;
            break;

        case eState::WaitForCommand:
            if (_sm.state_changed) {
                // Reset all commands
                cmd_reset = false;
                cmd_test  = false;
            }

            // Handle all commands in order of priority.
            if (cmd_reset)
                _sm.state = eState::Reset;
            else if (cmd_test)
                _sm.state = eState::Test;
            else {
                // No command given -> wait for system state changes.

                // Check if the system is in a state that requires action.
                // System state descriptions (from Types.h):
                //  WaitForCommand: The system is idle and waiting for a command.
                //  Alarm: In alarm state. Locks out all g-code processes. Allows settings access.
                //  CheckMode: G-code check mode. Locks out planner and motion only.
                //  Homing: Performing homing cycle
                //  Cycle: Cycle is running or motions are being executed.
                //  Hold: Active feed hold
                //  Jog: Jogging mode
                //  SafetyDoor: Safety door is ajar. Feed holds and de-energizes system
                //  Sleep: Sleep state
                //  ConfigAlarm: You can't do anything but fix your config file
                //  Critical: You can't do anything but reset with CTRL-x or the reset button
                switch (sys.state) {
                    case State::Idle:
                    case State::Alarm:
                    case State::CheckMode:
                    case State::Homing:
                    case State::Cycle:
                    case State::Hold:
                    case State::SafetyDoor:
                    case State::Sleep:
                    case State::ConfigAlarm:
                    case State::Critical:
                        // Do nothing.
                        break;

                    case State::Jog:
                        _sm.state = eState::Jog;
                        break;

                    default:
                        p_log_fatal("Unexpected system state: " << static_cast<size_t>(sys.state));
                        _sm.state = eState::FatalError;
                        break;
                }
            }
            break;

        case eState::Jog:
            if ((_sm.state_changed) || (_sm.time_in_state() > 500)) {
                _sm.reset_time_in_state();
                p_log_info("Target mpos x: " << steps_to_mpos(get_axis_motor_steps(X_AXIS), X_AXIS));
            }

            if (sys.state != State::Jog) {
                _sm.state = eState::WaitForCommand;
            }
            break;

        case eState::Test:
            if (_sm.state_changed) {
                // p_log_info("P=" << _belts[static_cast<size_t>(eBelt::TopLeft)]->_encoder->get_position_mm(44.0f));
            }

            // if ((_sm.time_in_state() > 250) && (_sm.time_in_state() < 260)) {
            //     // p_log_info("A=" << _belts[static_cast<size_t>(eBelt::TopLeft)]->_motor->get_current());
            // }

            // if ((_sm.time_in_state() > 1000) && (_sm.time_in_state() < 1010)) {
            //     // p_log_info("P=" << _belts[static_cast<size_t>(eBelt::TopLeft)]->_encoder->get_position_mm(44.0f));
            //     _belts[static_cast<size_t>(eBelt::TopLeft)]->_motor->set_speed(0.8f);
            // }

            // // if ((_sm.time_in_state() > 750) && (_sm.time_in_state() < 770)) {
            // // p_log_info("A=" << _belts[static_cast<size_t>(eBelt::TopLeft)]->_motor->get_current());
            // // }

            if (_sm.time_in_state() > 2000) {
                // Safeguard
                // p_log_info("A=" << _belts[static_cast<size_t>(eBelt::TopLeft)]->_motor->get_current());
                _belts[static_cast<size_t>(eBelt::TopLeft)]->cmd_reset = true;
                _sm.state                                              = eState::WaitForCommand;
            }
            break;

        case eState::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            if (_sm.state_changed) {
                p_log_fatal("Entered undefined state");
            }
            break;

        case eState::Reset:
            if (_sm.state_changed) {
                // Reset belt errors
                for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
                    _belts[i]->cmd_reset = true;
                }
            } else
                // Make sure to have at least one update cycle to allow the components to reset their state.
                _sm.state = eState::WaitForCommand;
            break;

        case eState::FatalError:
            // We're done...
            break;

        default:
            _sm.state = eState::Undefined;
            break;
    }
}

// Log state changes (called by StateMachine)
void Maslow::_log_state_change(const std::string& state_name) {
    p_log_info("State changed to " << state_name)
}

void Maslow::group(Configuration::HandlerBase& handler) {
    // Components
    handler.section("i2c_switch", _i2c_switch);
    for (size_t i = 0; i < NUMBER_OF_BELTS; ++i) {
        handler.section(BELT_NAMES[i], _belts[i]);
    }
    handler.item("fan_pin", _fan_pin);

    // Configuration
    handler.item("cycle_time", cycle_time);

    // Reporting
    handler.item("report_HWM", report_HWM);

    // Commands
    handler.item("cmd_reset", cmd_reset);
    handler.item("cmd_test", cmd_test);
}