// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "hal/Encoder.h"
#include "hal/HBridgeMotor.h"

class Belt : public Configuration::Configurable {
public:
    Belt() = default;

    // Components
    Encoder* _encoder = nullptr;
    HBridgeMotor* _motor   = nullptr;

    bool init(const char* name, I2CSwitch* i2c_switch);
    void cycle();

private:
    // StateMachine<State> _sm;
    const char* _name = nullptr;  // Identifier for the belt, e.g. "TL", "TR", etc.

    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
