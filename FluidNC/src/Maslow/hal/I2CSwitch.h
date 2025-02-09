/*
Wrapper for the SparkFun I2C Mux library used for the TCA9546A I2C switch, 
which is used to access the I2C port for each of the 4 belt position encoders.

Its function is to 
- provide a single point of access to the I2C switch by using a singleton pattern
- abstract away the hardware implementation details
*/

#pragma once

#include <cstdint>
#include "../drivers/SparkFun_I2C_Mux_Arduino_Library.h"

class I2CSwitch {
public:
    static I2CSwitch& instance();  // Get the singleton instance

    void select_port(uint8_t port);
    bool init();

    // Delete copy constructor and assignment operator
    I2CSwitch(const I2CSwitch&)            = delete;
    I2CSwitch& operator=(const I2CSwitch&) = delete;

private:
    I2CSwitch();  // Private constructor (singleton restriction)

    QWIICMUX _i2c_mux;  // TCA9546A I2C switch hardware driver
};
