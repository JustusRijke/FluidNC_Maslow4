// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "Belt.h"
#include "Encoder.h"
#include "I2CSwitch.h"
#include "utils/CycleStats.hpp"
#include "utils/StateMachine.hpp"
#include "utils/HierarchicalLog.hpp"

#include <array>

class Maslow : public Configuration::Configurable, public HierarchicalLog {
public:
    Maslow();

    // Configuration
    uint8_t _cycle_time = 5;  // [ms] Maslow task cycle time

    // Components
    I2CSwitch* _i2c_switch = nullptr;  // I2C switch for the encoders

    static constexpr int  NUMBER_OF_BELTS                = 4;
    static constexpr char BELT_NAMES[NUMBER_OF_BELTS][3] = { "TL", "TR", "BL", "BR" };
    enum class eBelt { TopLeft, TopRight, BottomLeft, BottomRight };
    Belt* _belts[NUMBER_OF_BELTS] = { nullptr, nullptr, nullptr, nullptr };

    // States
    enum class eState : uint16_t { 
        Undefined, 
        Entrypoint,
        Report, 
        Test, 
        FatalError,
        _ENUM_SIZE
    };
    const std::array<std::string, static_cast<size_t>(eState::_ENUM_SIZE)> _state_names = {
        "Undefined",
        "Entrypoint",
        "Report",
        "Test",
        "FatalError"
    };

    bool init();   // Called once to perform initialization logic
    void cycle();  // Called periodically to perform state machine logic

    void request_state_change(eState new_state);

private:
    // State machine
    StateMachine<eState> _sm;
    CycleStats          _cycle_stats { 10000 };  // Report every 10 seconds
    void                _log_state_change(const std::string& state_name);

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
