// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../Configuration/Configurable.h"
#include "Encoder.h"
#include "HBridgeMotor.h"
#include "utils/HierarchicalLog.hpp"

class Belt : public Configuration::Configurable, public HierarchicalLog {
public:
    Belt() = default;

    // Components
    Encoder*      _encoder = nullptr;
    HBridgeMotor* _motor   = nullptr;

    bool init(I2CSwitch* i2c_switch);
    void update();

private:
    // Configuration handler
    void group(Configuration::HandlerBase& handler) override;
};
