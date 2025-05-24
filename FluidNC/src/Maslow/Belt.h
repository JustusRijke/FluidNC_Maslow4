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
    float get_position();

    // Reporting
    bool report_status = false;  // General status report

    // Commands
    bool cmd_retract = false;  // Retract the belt and set position to zero (home)
    bool cmd_extend  = false;  // Extend the belt
    bool cmd_reset   = false;  // Reset to initial state (stop motor, reset errors, etc.)

    bool  cmd_move_to_target = false;  // Move to target position
    float target_pos         = 0.0f;  // [mm] Target position to move the belt to. TODO: remove from configuration, only used during testing

    bool request_fan = false;  // Request to turn on the fan for cooling the motor

private:
    // Components
    Encoder*      _encoder = nullptr;
    HBridgeMotor* _motor   = nullptr;

    // Configuration
    float    _retract_torque       = 1.0f;  // [0.0-1.0] Torque for retracting the belt
    float    _extend_torque        = 1.0f;  // [0.0-1.0] Torque for retracting the belt
    float    _minimum_torque       = 0.3f;  // [0.0-1.0] Minimum torque for belt to move
    float    _retract_current      = 0.6f;  // [A] Current threshold for retracting the belt
    uint32_t _max_direction_errors = 10;    // Max direction errors allowed before stopping the motor. 0 disables the check.
    // Max movement errors allowed before stopping the motor. 0 disables the check. Higher values give the motor more time to move the belt.
    uint32_t _max_movement_errors = 50;
    float    _gain                = 0.1f;  // Gain (P of a PID) for the motor torque control (output=P*(setpoint-current_value)
    float    _hysteresis          = 0.1f;  // [mm] Target reached if the position is within this range.

    enum class eState : uint16_t {
        Undefined,
        Entrypoint,
        WaitForCommand,
        Retract,
        StartExtend,
        Extending,
        PauseExtend,
        MoveToTarget,
        Reset,
        Error,
        _ENUM_SIZE
    };
    StateMachine<eState> _sm;

    uint16_t _direction_errors = 0;     // Number of direction errors detected
    uint16_t _movement_errors  = 0;     // Number of motion detection errors detected

    float _extend_length = 1000.0f;  // [mm] Distance to extend the belt
    float _position      = 0.0f;     // Current position of the belt
    float _last_position = 0.0f;     // Last position of the belt

    bool _homed = false;  // False if the position of the belt is unclear (never retracted, after power cycle, etc.)

    unsigned long             _timestamp_last_warning = 0;    // Last warning time (to avoid spamming the log)
    constexpr static uint32_t WARNING_INTERVAL        = 400;  // [ms] Interval for showing warnings

    static constexpr float FLOAT_NEAR_ZERO = std::numeric_limits<float>::epsilon();  // Filter near-zero float values

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
