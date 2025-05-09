#pragma once
#include <array>
#include <cstddef>

/**
 * @brief Rolling average filter keeping the last <NumberOfSamples> float values.
 *
 * ### Example
 * ```cpp
 * RollingAverage<8> filt;      // keeps the last 8 samples
 * float avg = filt.update(v);  // feed a new sample each cycle and return the average
 * ```
 *
 * ⚠️ Average is biased toward `0.0f` until the history is full.
 */
template <std::size_t NumberOfSamples>
class RollingAverage {
    static_assert(NumberOfSamples > 0, "Number of samples must be > 0");

public:
    constexpr RollingAverage() noexcept : _sum { 0.0f }, _index { 0 }, _used { 0 } { _buf.fill(0.0f); }

    // Add one sample, obtain the current average.
    constexpr float update(float sample) noexcept {
        _sum -= _buf[_index];   // discard oldest
        _buf[_index] = sample;  // store newest
        _sum += sample;         // accumulate

        _index = (_index + 1u) % NumberOfSamples;
        if (_used < NumberOfSamples)
            ++_used;
        return _sum * _invSamples;  // multiply is cheaper than divide
    }

private:
    static constexpr float _invSamples = 1.0f / static_cast<float>(NumberOfSamples);

    std::array<float, NumberOfSamples> _buf;
    float                              _sum;
    std::size_t                        _index;
    std::size_t                        _used;
};
