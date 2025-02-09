/**
 * Copyright (c) 2025 -	Justus Rijke
 * Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
 *
 * A simple state machine for uint16_t-based enums.
 * Example Usage:
 * 
 * @code
 * #include "StateMachine.hpp"
 *
 * enum class State : uint16_t { Undefined, Delay, DoSomething };
 *
 * class FooBar {
 * public:
 *     void cycle() {
 *         _statemachine.update();
 *
 *         switch (_statemachine.state) {
 *             case State::Delay:
 *                 if (_statemachine.time_in_state() >= 1000) {  // 1 second
 *                      _statemachine.state = State::DoSomething;
 *                 }
 *                 break;
 *             case State::DoSomething:
 *                 if (_statemachine.state_changed) {
 *                    log("Entered DoSomething state");
 *                 }
 *                 // Do something
 *                 break;
 *             case State::Undefined:
 *                 // Fatal error, should never reach here
 *                 break;
 *             default:
 *                 _statemachine.state = State::Undefined;
 *                 break;
 *         }
 *     }
 *
 * private:
 *     StateMachine<State> _statemachine{100}; // 100ms per cycle
 * };
 * @endcode
 */

#pragma once
#include <cstdint>
#include <type_traits>

// A simple state machine for uint16_t-based enums
template <typename StateT>
class StateMachine {
    static_assert(std::is_enum_v<StateT>, "StateT must be an enum!");
    static_assert(std::is_same_v<std::underlying_type_t<StateT>, uint16_t>, "StateT must have uint16_t as its underlying type!");

public:
    StateT state;
    bool   state_changed;

    explicit StateMachine(uint32_t ms_per_cycle) :
        state(static_cast<StateT>(0)), state_changed(false), _cycles(0), _ms_per_cycle(ms_per_cycle), _state_prev(static_cast<StateT>(0)) {}

    // Update the state machine
    void update() {
        state_changed = (state != _state_prev);
        if (state_changed) {
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
        uint64_t result = static_cast<uint64_t>(_cycles) * _ms_per_cycle;
        return result > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(result);
    }

    // Reset the time in the current state
    void reset_time_in_state() { _cycles = 0; }

private:
    uint32_t _cycles;
    uint32_t _ms_per_cycle;
    StateT   _state_prev;
};
