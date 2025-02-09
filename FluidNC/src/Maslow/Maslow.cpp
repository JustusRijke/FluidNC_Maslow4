#include "Maslow.h"
#include "../Logging.h"

// Private namespace for local constants (prefered over #define)
namespace {
    constexpr uint32_t MS_PER_CYCLE = 50;  // Expected time between consecutive calls to cycle()
}

// Returns the single instance of Maslow.
Maslow& Maslow::instance() {
    static Maslow instance;  // Guaranteed to be created once (C++11+ thread-safe)
    return instance;
}

// Private constructor to enforce singleton.
Maslow::Maslow() : _sm(MS_PER_CYCLE) {
    _sm.state = State::Entrypoint;
}

// Maslow initialization logic.
void Maslow::init() {
    log_info("Maslow::init() called");
}

// The main Maslow state machine loop, called cyclically.
void Maslow::cycle() {
    // Update the state machine, so we can check state changes and time spent in state.
    _sm.update();

    switch (_sm.state) {
        case State::Entrypoint:
            // Entry point logic: do nothing but move to an initial state.
            _sm.state = State::InitI2C;
            break;
        case State::InitI2C:
            log_state_change("Initialize the I2C bus and the TCA9546A switch");
            // TODO
            break;
        case State::Undefined:
            // Oops, we should never end up here. Fatal programming error.
            break;
        default:
            _sm.state = State::Undefined;
            break;
    }
}

inline void Maslow::log_state_change(const char* msg) {
    if (_sm.state_changed) {
        log_info(std::string("Maslow: ") + msg);
    }
}