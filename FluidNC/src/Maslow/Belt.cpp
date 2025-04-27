// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Belt.h"

#include "../Logging.h"
#include "../Machine/MachineConfig.h"

// Belt initialization logic.
bool Belt::init(const char* name, I2CSwitch* i2c_switch) {
    _name = name;

    if (_encoder == nullptr) {
        log_error("Missing config: encoder " << name);
        return false;
    }

    if (_motor == nullptr) {
        log_error("Missing config: motor " << name);
        return false;
    }

    // Initialize the encoder with the I2C switch (dependency injection)
    if (!_encoder->init(i2c_switch))
        return false;

    if (!_motor->init())
        return false;

    log_info("Belt " << name << " initialized");
    return true;
}

// The main Belt state machine loop, called cyclically.
void Belt::cycle() {}

void Belt::group(Configuration::HandlerBase& handler) {
    handler.section("encoder", _encoder);
    handler.section("motor", _motor);
}