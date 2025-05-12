// Copyright (c) 2025 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#pragma once

#include "../../Logging.h"

#include <Arduino.h>  // Include Arduino for micros()

/**
 * @brief A class for tracking and reporting cycle statistics.
 *
 * This class measures the time between consecutive calls to its `track_cycles()` method,
 * calculates the minimum, maximum, and mean cycle times, and reports these statistics
 * via `log_info()` after a specified time interval.
 *
 * @example
 * CycleStats cycle_stats(5000); // Report every 5000 milliseconds
 *
 * void loop() {
 *   //... your code...
 *   cycle_stats.track_cycles();
 *   //... your code...
 * }
 */
class CycleStats {
public:
    /**
     * @brief Constructs a CycleStats object.
     *
     * @param report_interval The time interval between reporting statistics, in milliseconds.
     */
    explicit CycleStats(uint32_t report_interval) : _report_interval(report_interval) {}

    /**
     * @brief Tracks the cycle time and reports statistics.
     *
     * This method should be called in each cycle of your application. It measures the time
     * since the last call, updates the cycle statistics, and reports the statistics via
     * `log_info()` after a specified time interval.
     */
    void track_cycles(uint32_t expected_cycle_time, bool verbose = false) {
        // Cycle time measurement
        uint32_t now        = micros();
        if (_last_micros == 0)
            _last_micros = now;
        uint32_t cycle_time = now - _last_micros;
        _last_micros        = now;

        // Cycle statistics
        _min_cycle_time = (_last_micros > 0) ? min(_min_cycle_time, cycle_time) : cycle_time;
        _max_cycle_time = max(_max_cycle_time, cycle_time);
        _total_cycle_time += cycle_time;
        _cycle_counter++;

        // Reporting
        if (_total_cycle_time >= _report_interval * 1000) {
            uint32_t mean_cycle_time = _total_cycle_time / _cycle_counter;
            if (verbose)
                log_info("Cycle stats: min=" << _min_cycle_time << "us, max=" << _max_cycle_time << "us, mean=" << mean_cycle_time << "us");
            if (_max_cycle_time > expected_cycle_time)
                log_warn("Cycle time exceeded: " << _max_cycle_time << "us (max: " << expected_cycle_time << "us)");
            _min_cycle_time   = UINT32_MAX;
            _max_cycle_time   = 0;
            _total_cycle_time = 0;
            _cycle_counter    = 0;
        }
    }

private:
    uint32_t       _last_micros      = 0;
    uint32_t       _min_cycle_time   = UINT32_MAX;
    uint32_t       _max_cycle_time   = 0;
    uint64_t       _total_cycle_time = 0;
    uint32_t       _cycle_counter    = 0;
    const uint32_t _report_interval;
};