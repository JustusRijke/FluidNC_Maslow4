// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "hal/Encoder.h"
#include "utils/CycleStats.hpp"
#include "utils/StateMachine.hpp"

#include <array>

class Maslow {
public:
    enum class State : uint16_t { Undefined, Entrypoint, Report, Test, FatalError };

    static Maslow& instance();  // Get the singleton instance

    void init();   // Called once to perform initialization logic
    void cycle();  // Called periodically to perform state machine logic

    // Delete copy constructor and assignment operator
    Maslow(const Maslow&)            = delete;
    Maslow& operator=(const Maslow&) = delete;

    void request_state_change(State new_state);

private:
    Maslow();  // Private constructor (singleton restriction)

    StateMachine<State> _sm;
    CycleStats          _cycle_stats { 500 };  // Report every 0.5 seconds
    inline void         log_state_change(const char* msg);

    static constexpr int              NUM_ENCODERS = 4;
    std::array<Encoder, NUM_ENCODERS> _encoders    = { Encoder(0), Encoder(1), Encoder(2), Encoder(3) };
    std::string                       encoderPositions;
    uint64_t                          position = 0;
};
