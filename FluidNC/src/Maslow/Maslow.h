#pragma once

#include "StateMachine.hpp"

class Maslow {
public:
    enum class State : uint16_t { Undefined, Entrypoint, InitI2C };

    static Maslow& instance();  // Get the singleton instance

    void init();   // Called once to perform initialization logic
    void cycle();  // Called periodically to perform state machine logic

    // Delete copy/move constructors to enforce singleton behavior
    Maslow(const Maslow&)            = delete;
    Maslow& operator=(const Maslow&) = delete;
    Maslow(Maslow&&)                 = delete;
    Maslow& operator=(Maslow&&)      = delete;

private:
    Maslow();  // Private constructor (singleton restriction)

    StateMachine<State> _sm;

    inline void log_state_change(const char* msg);
};
