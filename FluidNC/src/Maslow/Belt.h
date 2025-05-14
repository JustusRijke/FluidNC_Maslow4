// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "Encoder.h"
#include "HBridgeMotor.h"
#include "utils/StateMachine.hpp"

class Belt : public Configuration::Configurable {
public:
    Belt() = default;

    // Configuration
    float    _retract_speed        = 0.6f;  // [0.0-1.0] Speed for retracting the belt
    float    _retract_current      = 0.5f;  // [A] Current threshold for retracting the belt
    uint32_t _max_direction_errors = 0;     // Maximum number of direction errors before stopping the motor. 0 to disable check.

    // Commands
    bool cmd_retract = false;
    bool cmd_reset   = false;

    // Components
    Encoder*      _encoder = nullptr;
    HBridgeMotor* _motor   = nullptr;

    bool init(I2CSwitch* i2c_switch, uint8_t cycle_time);
    void update();

private:
    enum class eState : uint16_t { Undefined, Entrypoint, WaitForCommand, Retract, Reset, Error, _ENUM_SIZE };
    StateMachine<eState> _sm;

    float    _prev_position    = 0.0f;  // Previous position of the encoder
    uint16_t _direction_errors = 0;     // Number of direction errors detected

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
