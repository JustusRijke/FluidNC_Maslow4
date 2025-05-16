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

    bool init(I2CSwitch* i2c_switch, uint8_t cycle_time);
    void update();

    // Reporting
    bool report_status = false;  // General status report

    // Commands
    bool cmd_retract = false;  // Retract the belt and set position to zero
    bool cmd_extend  = false;  // Extend the belt
    bool cmd_reset   = false;  // Reset belt errors

private:
    // Components
    Encoder*      _encoder = nullptr;
    HBridgeMotor* _motor   = nullptr;

    // Configuration
    float    _retract_speed        = 0.6f;  // [0.0-1.0] Speed for retracting the belt
    float    _extend_speed         = 1.0f;  // [0.0-1.0] Speed for retracting the belt
    float    _retract_current      = 0.5f;  // [A] Current threshold for retracting the belt
    uint32_t _max_direction_errors = 1;     // Max direction errors allowed before stopping the motor. 0 disables the check.
    // Max movement errors allowed before stopping the motor. 0 disables the check. Higher values give the motor more time to move the belt.
    uint32_t _max_movement_errors = 10;

    enum class eState : uint16_t { Undefined, Entrypoint, WaitForCommand, Retract, Extend, Reset, Error, _ENUM_SIZE };
    StateMachine<eState> _sm;

    float    _prev_position    = 0.0f;  // Previous position of the encoder
    uint16_t _direction_errors = 0;     // Number of direction errors detected
    uint16_t _movement_errors  = 0;     // Number of motion detection errors detected

    unsigned long             _timestamp_last_warning = 0;    // Last warning time (to avoid spamming the log)
    constexpr static uint32_t WARNING_INTERVAL        = 500;  // [ms] Interval for showing warnings

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
