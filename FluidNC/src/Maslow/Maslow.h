// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "hal/Encoder.h"
#include "hal/I2CSwitch.h"
#include "utils/CycleStats.hpp"
#include "utils/StateMachine.hpp"

#include <array>

class Maslow : public Configuration::Configurable {
public:
    Maslow();

    // Confguration
    uint8_t _cycle_time = 5;  // [ms] Maslow task cycle time

    // Components
    I2CSwitch* _i2c_switch = nullptr;  // I2C switch for the encoders

    // States
    enum class State : uint16_t { Undefined, Entrypoint, Report, Test, FatalError };

    bool init();   // Called once to perform initialization logic
    void cycle();  // Called periodically to perform state machine logic

    void request_state_change(State new_state);

private:
    StateMachine<State> _sm;
    CycleStats          _cycle_stats { 10000 };  // Report every 10 seconds
    inline void         log_state_change(const char* msg);

    static constexpr int              NUM_ENCODERS = 4;
    std::array<Encoder, NUM_ENCODERS> _encoders    = { Encoder(0), Encoder(1), Encoder(2), Encoder(3) };
    std::string                       encoderPositions;
    uint64_t                          position = 0;

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
