// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "Encoder.h"
#include "HBridgeMotor.h"
#include "utils/StateMachine.hpp"
#include <QuickPID.h>

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
    float _retract_duty    = 1.0f;  // [0.0-1.0] PWM duty cycle for retracting the belt
    float _extend_duty     = 1.0f;  // [0.0-1.0] PWM duty cycle for retracting the belt
    float _minimum_duty    = 0.3f;  // [0.0-1.0] Minimum PWM duty cycle for belt to move (dead zone)
    float _retract_current = 0.9f;  // [A] Current threshold for retracting the belt

    uint32_t _max_direction_errors = 10;    // Max direction errors allowed before stopping the motor. 0 disables the check.
    uint32_t _max_stall_errors     = 20;    // Max stall errors allowed before stopping the motor. 0 disables the check.
    float    _min_stall_duty       = 0.6f;  // Minimum PWM duty cycle to consider the belt moving during stall detection

    float    _hysteresis          = 0.1f;  // [mm] Target reached if the position is within this range.
    float    _Kp                  = 0.020f;  // Proportional gain: Error response gain
    float    _Ki                  = 0.000f;  // Integral: Eliminates steady-state error
    float    _Kd                  = 0.001f;  // Derative gain: Damps oscillations/overshoot

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

    uint16_t               _direction_errors = 0;      // Number of direction errors detected
    constexpr static float DIR_ERROR_MIN_VEL = 10.0f;  // [mm/s] Min. velocity to consider the belt moving
    uint16_t               _stall_errors     = 0;      // Number of stall errors detected
    constexpr static float MOV_ERROR_MIN_VEL = 0.01f;  // [mm/s] Min. velocity to consider the belt stalled

    float _extend_length = 1000.0f;  // [mm] Distance to extend the belt
    float _position      = 0.0f;     // Current position of the belt
    float _last_position = 0.0f;     // Last position of the belt
    bool _homed = false;  // False if the position of the belt is unclear (never retracted, after power cycle, etc.)

    // PID controller
    float    _pid_output;
    QuickPID _PID = QuickPID(&_position, &_pid_output, &target_pos);

    unsigned long             _timestamp_last_warning = 0;    // Last warning time (to avoid spamming the log)
    constexpr static uint32_t WARNING_INTERVAL        = 400;  // [ms] Interval for showing warnings

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
