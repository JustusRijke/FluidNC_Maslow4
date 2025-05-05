// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "Belt.h"

#include "../Logging.h"
#include "../Machine/MachineConfig.h"

// Belt initialization logic.
bool Belt::init(I2CSwitch* i2c_switch) {
    if (_encoder == nullptr) {
        p_log_config_error("Missing config: encoder");
        return false;
    }

    if (_motor == nullptr) {
        p_log_config_error("Missing config: motor");
        return false;
    }

    // Initialize the encoder with the I2C switch (dependency injection)
    _encoder->initName("Encoder", this);
    if (!_encoder->init(i2c_switch))
        return false;

    _motor->initName("Motor", this);
    if (!_motor->init())
        return false;

    p_log_info("Initialized.");
    return true;
}

// The main Belt state machine loop, called cyclically.
void Belt::cycle() {
    _motor->update();
}

void Belt::group(Configuration::HandlerBase& handler) {
    handler.section("encoder", _encoder);
    handler.section("motor", _motor);
}