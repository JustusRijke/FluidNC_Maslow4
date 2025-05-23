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

#include <array>

class Maslow : public Configuration::Configurable {
public:
    Maslow();

    bool init();    // Called once to perform initialization logic
    void update();  // Called periodically to perform state machine logic

    // Configuration
    uint8_t cycle_time = 10;  // [ms] Maslow task cycle time (used by Main.cpp to setup the Maslow task)

    // Reporting
    bool report_HWM = false;  // Report high water mark of the task stack

    // Commands
    bool cmd_reset = false;
    bool cmd_test  = false;  // Test command for debugging

private:
    static constexpr int  NUMBER_OF_BELTS                = 4;
    static constexpr char BELT_NAMES[NUMBER_OF_BELTS][3] = { "TL", "TR", "BL", "BR" };
    enum class eBelt { TopLeft, TopRight, BottomLeft, BottomRight };

    // Components
    Belt*      _belts[NUMBER_OF_BELTS] = { nullptr, nullptr, nullptr, nullptr };
    I2CSwitch* _i2c_switch             = nullptr;  // I2C switch for the belt encoders
    Pin        _fan_pin;

    // State machine
    enum class eState : uint16_t { Undefined, Entrypoint, WaitForCommand, Jog, Test, Reset, FatalError, _ENUM_SIZE };
    const std::array<std::string, static_cast<size_t>(eState::_ENUM_SIZE)> _state_names = { "Undefined", "Entrypoint", "WaitForCommand",
                                                                                            "Jog",       "Test",       "Reset",
                                                                                            "FatalError" };
    StateMachine<eState>                                                   _sm;

    CycleStats           _cycle_stats { 5000 };  // Analyse cycle times every 2 seconds
    void                 _log_state_change(const std::string& state_name);

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
