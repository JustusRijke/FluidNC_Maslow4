// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Belt.h"

#include "../Logging.h"
#include "../Machine/MachineConfig.h"

// Belt initialization logic.
bool Belt::init(I2CSwitch* i2c_switch, uint8_t cycle_time) {
    if (_encoder == nullptr) {
        p_log_config_error("Encoder not defined");
        return false;
    }

    if (_motor == nullptr) {
        p_log_config_error("Motor not defined");
        return false;
    }

    // Initialize the encoder with the I2C switch (dependency injection)
    if (!_encoder->init(i2c_switch, cycle_time))
        return false;

    if (!_motor->init(cycle_time))
        return false;

    _sm.ms_per_cycle = cycle_time;
    _sm.state        = eState::Entrypoint;

    _PID.SetMode(QuickPID::Control::automatic);
    // _PID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);

    p_log_info("Initialized.");
    return true;
}

// The main Belt state machine loop, called cyclically.
void Belt::update() {
    _motor->update();
    _encoder->update();
    _PID.Compute();

    _position              = _encoder->get_position();
    float encoder_velocity = _encoder->get_velocity();
    float motor_duty       = _motor->get_duty();

    bool _errors_during_extend = false;

    // Check if direction of encoder and motor match up.
    // If not, configuration or hardware could be wrong,
    // e.g., floating pin issue on Maslow PCB 4.0.
    if (_max_direction_errors > 0) {
        if (((encoder_velocity > DIR_ERROR_MIN_VEL) && motor_duty < -_minimum_duty) ||
            ((encoder_velocity < -DIR_ERROR_MIN_VEL) && motor_duty > _minimum_duty)) {
            _direction_errors++;
            if (_direction_errors > _max_direction_errors) {
                if ((_sm.state == eState::StartExtend) || (_sm.state == eState::Extending)) {
                    // Suppress the error when we are extending the belt
                    p_log_warn("Encoder and motor direction mismatch");
                    _errors_during_extend = true;
                } else {
                    p_log_error("Encoder and motor direction mismatch");
                    _sm.state = eState::Error;
                }
                _direction_errors = 0;  // Avoid re-triggering the error
            }
        } else {
            // Motor stopped or direction OK, reset counter
            _direction_errors = 0;
        }
    }

    // Check if belt is stalled
    if (_max_stall_errors > 0) {
        if ((fabs(encoder_velocity) < MOV_ERROR_MIN_VEL) && (fabs(motor_duty) > _min_stall_duty)) {
            _stall_errors++;
            if (_stall_errors > _max_stall_errors) {
                if ((_sm.state == eState::StartExtend) || (_sm.state == eState::Extending)) {
                    // Suppress the error when we are extending the belt
                    p_log_warn("Motor is active, but belt is not moving");
                    _errors_during_extend = true;
                } else {
                    p_log_error("Motor is active, but belt is not moving");
                    _sm.state = eState::Error;
                }
                _stall_errors = 0;  // Avoid re-triggering the error
            }
        } else {
            // Belt movement detected, reset counter
            _stall_errors = 0;
        }
    }

    // Log motor overcurrent warning (ignore when retracting)
    if ((_motor->overcurrent_warning) && (_sm.state != eState::Retract)) {
        unsigned long now = millis();
        // Avoid spamming the log
        if (now - _timestamp_last_warning > WARNING_INTERVAL) {
            p_log_warn("Motor overcurrent warning");
            _timestamp_last_warning = now;
        }
    }

    // Handle motor overcurrent error
    if ((_motor->overcurrent_error) && (_sm.state != eState::Error)) {
        p_log_error("Motor overcurrent error");
        _sm.state = eState::Error;
    }

    // Reporting
    if (report_status) {
        p_log_info("State=" << static_cast<uint16_t>(_sm.state) << ", V=" << encoder_velocity << "mm/s, T=" << motor_duty * 100
                            << "%, I=" << _motor->get_current() << "A, Ptarget=" << target_pos << "mm, Pact=" << _position
                            << "mm, lag=" << target_pos - _position << "mm");
        report_status = false;
    }

    // Reset command overrules all states
    if (cmd_reset)
        _sm.state = eState::Reset;

    // State machine
    _sm.update();

    switch (_sm.state) {
        case eState::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = eState::WaitForCommand;
            break;

        case eState::WaitForCommand:
            if (_sm.state_changed) {
                // Reset all commands except cmd_reset
                cmd_retract = false;
                cmd_extend  = false;
                cmd_move_to_target = false;
            }
            // Handle all commands in order of priority.
            if (cmd_retract)
                _sm.state = eState::Retract;
            else if (cmd_extend)
                _sm.state = eState::StartExtend;
            else if (cmd_move_to_target)
                _sm.state = eState::MoveToTarget;

            break;

        case eState::Retract:
            if (_sm.state_changed) {
                _motor->set_duty(-_retract_duty);
            }

            // If the motor current exceeds the threshold, stop the motor
            if (_motor->get_current() > _retract_current) {
                _motor->stop();
                _sm.reset_time_in_state();  // Reset time in state, so we can wait for the motor to stop
            }

            if (_motor->stopped() && (_sm.time_in_state() > 500)) {
                _encoder->set_position(0.0f);
                _homed    = true;
                _sm.state = eState::WaitForCommand;
            }
            break;

        case eState::StartExtend:
            // Extend procedure:
            // First, extend the belt a little bit (to get it unstuck from its retracted position)
            // Then, as long as the encoder position is changing, keep extending the belt.
            // Pause extend procedure when detecting errors (suppress Error state using `_errors_during_extend`).

            if (_sm.state_changed) {
                if (!_homed) {
                    p_log_warn("Belt position unknown, retract first");
                    _sm.state = eState::WaitForCommand;
                } else
                    // Extend the belt a little bit (to get it unstuck from its retracted position)
                    _motor->set_duty(_extend_duty);
            }

            if (_position >= _extend_length) {
                // If the belt is extended enough, stop the procedure
                _motor->stop();
                _sm.state = eState::WaitForCommand;
            } else if (_sm.time_in_state() > 500) {
                _last_position = _position;
                _sm.state      = eState::Extending;
            }

            break;

        case eState::Extending:
            // Extend the belt as lang as the encoder is reporting movement,
            // pause on errors (which will occurr when the user is not pulling the belt)
            if (_position >= _extend_length) {
                // If the belt is extended enough, stop the procedure
                _motor->stop();
                _sm.state = eState::WaitForCommand;
            } else if (_errors_during_extend) {
                // Error detected, pause for a bit
                _sm.state = eState::PauseExtend;
            } else if (_position > _last_position) {
                // Encoder is reporting movement, keep extending the belt
                _last_position = _position;
                _motor->set_duty(_extend_duty);
            } else {
                _motor->stop();
            }

            break;

        case eState::PauseExtend:
            // Pause extending the belt if there are errors, to give user time to respond (e.g., pull the belt)
            if (_sm.state_changed) {
                _motor->stop();  // Brake
            }
            if (_sm.time_in_state() > 1000) {
                _motor->stop(true);         // Coast, so belt can be pulled out
                _last_position = _position;  // If not, lot of force is required to move the belt past the last known position
                _sm.state = eState::Extending;
            }
            break;

        case eState::MoveToTarget:
            if (_sm.state_changed) {
                // TODO: uncomment this code before release
                // if (!_homed) {
                //     p_log_warn("Belt position unknown, retract first");
                //     _sm.state = eState::WaitForCommand;
                // } else
                {
                    // Restart the PID controller
                    _PID.SetTunings(_Kp, _Ki, _Kd);
                    // "Clamp" limits, to avoid motor dead zone
                    _PID.SetOutputLimits(-1.0f + _minimum_duty, 1.0f - _minimum_duty);
                    _PID.Reset();
                }
            } else if (!cmd_move_to_target) {
                // We're done
                _motor->stop();
                _sm.state = eState::WaitForCommand;
            } else {
                // Drive the motor. Apply minimum PWM duty cycle to PID output
                if (_pid_output < 0.0f)
                    _motor->set_duty(_pid_output - _minimum_duty);
                else
                    _motor->set_duty(_pid_output + _minimum_duty);
            }
            break;

        case eState::Reset:
            // Hard reset: stop motor and return to an idle state
            cmd_reset = false;
            _motor->stop();
            _sm.state = eState::WaitForCommand;
            break;

        case eState::Error:
            // Only way to get out of this state is a reset command
            if (_sm.state_changed)
                _motor->stop();
            break;

        case eState::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            if (_sm.state_changed) {
                p_log_fatal("Entered undefined state");
            }
            break;

        default:
            _sm.state = eState::Undefined;
            break;
    }

    request_fan = (!_motor->stopped());
}

float Belt::get_position() {
    return _position;
}

void Belt::group(Configuration::HandlerBase& handler) {
    // Components
    handler.section("encoder", _encoder);
    handler.section("motor", _motor);

    // Configuration
    handler.item("retract_duty", _retract_duty, 0.01f, 1.0f);
    handler.item("retract_current", _retract_current, 0.01f, 100.0f);
    handler.item("minimum_duty", _minimum_duty, 0.0f, 1.0f);
    handler.item("extend_duty", _extend_duty, 0.01f, 1.0f);
    handler.item("max_direction_errors", _max_direction_errors, 0, 100);
    handler.item("max_stall_errors", _max_stall_errors, 0, 1000);
    handler.item("min_stall_duty", _min_stall_duty, 0.01f, 1.0f);
    handler.item("target_pos", target_pos, 0.0f, 10000.0f);
    handler.item("Kp", _Kp, 0.001f, 100.0f);
    handler.item("Ki", _Ki, 0.0f, 100.0f);
    handler.item("Kd", _Kd, 0.0f, 100.0f);

    // Reports
    handler.item("report_status", report_status);

    // Commands
    handler.item("cmd_retract", cmd_retract);
    handler.item("cmd_extend", cmd_extend);
    handler.item("cmd_reset", cmd_reset);
    handler.item("cmd_move_to_target", cmd_move_to_target);
}