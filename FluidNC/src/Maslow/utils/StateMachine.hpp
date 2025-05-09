/**
 * Copyright (c) 2025 -	Justus Rijke
 * Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
 *
 * A simple state machine for uint16_t-based enums, with callback to a custom logging function to log state changes.
 * 
 * Example Usage:
 * 
 * #include "StateMachine.hpp"
 * 
 * // Example logging function
 * void my_logger(const std::string& state_name) {
 *   log_info("State machine changed state to: " << state_name);
 * }
 * 
 * // This is a simple example of a state machine with three states: Delay, DoSomething, and Undefined.
 * enum class State : uint16_t { Undefined, Delay, DoSomething, _ENUM_SIZE };
 * const std::array<std::string, static_cast<size_t>(State::_ENUM_SIZE)> state_names = {
 *   "Undefined state",
 *     "", // Empty string means a state change to State::Delay will not be logged
 *     "Doing something"
 * };
 * StateMachine<State> _statemachine(state_names, my_logger);
 * 
 * void cycle() {
 *     _statemachine.update();
 * 
 *     switch (_statemachine.state) {
 *         case State::Delay:
 *             if (_statemachine.time_in_state() >= 1000) {  // 1 second, if ms_per_cycle = 1
 *                 _statemachine.state = State::DoSomething;
 *             }
 *             break;
 *         case State::DoSomething:
 *             if (_statemachine.state_changed) {
 *                 // Do something once
 *             }
 *             // Do something every cycle
 *             break;
 *         case State::Undefined:
 *             // Fatal error, should never reach here
 *             break;
 *         default:
 *             _statemachine.state = State::Undefined;
 *             break;
 *     }
 * }
 * @endcode
 */

#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>

// A simple state machine for uint16_t-based enums
template <typename StateT>
class StateMachine {
    static_assert(std::is_enum_v<StateT>, "StateT must be an enum!");
    static_assert(std::is_same_v<std::underlying_type_t<StateT>, uint16_t>, "StateT must have uint16_t as its underlying type!");

private:
    static constexpr size_t                 _number_of_states = static_cast<size_t>(StateT::_ENUM_SIZE);
    std::function<void(const std::string&)> _log_callback;  // Callable for logging

public:
    StateT state;
    bool   state_changed;

    StateMachine() {}  // Default constructor, without state name logging

    StateMachine(const std::array<std::string, _number_of_states>& state_names, std::function<void(const std::string&)> log_callback) :
        state(static_cast<StateT>(0)), state_changed(false), _cycles(0), _state_prev(static_cast<StateT>(0)), _state_names(state_names),
        _log_callback(log_callback) {
        if (_state_names.size() < _number_of_states) {
            throw std::runtime_error("State names array does not have enough elements");
        }
    }

    uint16_t ms_per_cycle = 1;  // [ms] Time per cycle, defaults to 1ms. Adjust to match update() call frequency.

    // Update the state machine
    void update() {
        state_changed = (state != _state_prev);
        if (state_changed) {
            // Log state change if a state name is provided and state_names array is initialized
            if (!_state_names.empty() && !_state_names[static_cast<size_t>(state)].empty()) {
                _log_callback(_state_names[static_cast<size_t>(state)]);
            }
            _cycles     = 0;
            _state_prev = state;
        } else {
            if (_cycles < UINT32_MAX)
                _cycles++;
        }
    }

    // Return the time in the current state in milliseconds
    uint32_t time_in_state() const {
        // Prevent overflow
        uint64_t result = static_cast<uint64_t>(_cycles) * ms_per_cycle;
        return result > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(result);
    }

    // Reset the time in the current state
    void reset_time_in_state() { _cycles = 0; }

private:
    uint32_t                                   _cycles;
    StateT                                     _state_prev;
    std::array<std::string, _number_of_states> _state_names;
};
