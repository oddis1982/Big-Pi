#pragma once
/*
  =============================================================================
  EarlyReflections.h — Big Pi Early Reflection Generator (heavily annotated)
  =============================================================================
*/

#include <algorithm>
#include <cmath>

#include "dsp/common/Dsp.h"

class EarlyReflections {
public:
    struct Params {
        float level = 0.30f;      // 0..1: how loud early reflections are
        float size = 0.55f;       // scales delay times (bigger room = longer early delays)
        float dampHz = 9000.0f;   // LP cutoff for bright/dark ER
        float width = 1.0f;       // stereo width multiplier
    };

    EarlyReflections() = default;

    // Allocate buffers and initialize smoothers.
    void prepare(float sampleRate);

    // Flush delay/filter state.
    void reset();

    // Update target parameters (smoothing happens in process).
    void setParams(const Params& p);

    // Process block of stereo input -> stereo ER output.
    void processBlock(const float* inL, const float* inR,
        float* outL, float* outR,
        int n);

private:
    float sr = 48000.0f;

    Params target{};

    // Delay lines for left/right ER
    dsp::DelayLine delayL{};
    dsp::DelayLine delayR{};

    // Damping filters (one-pole low-pass)
    dsp::OnePoleLP dampL{};
    dsp::OnePoleLP dampR{};

    // Parameter smoothers
    dsp::SmoothValue levelSm{};
    dsp::SmoothValue sizeSm{};
    dsp::SmoothValue dampSm{};
    dsp::SmoothValue widthSm{};

    // A simple, fixed tap pattern (ms) and gains.
    // These are short times designed to feel like early room reflections.
    static constexpr int kNumTaps = 6;

    static constexpr float kTapTimesMs[kNumTaps] = { 7.0f, 11.0f, 17.0f, 23.0f, 31.0f, 41.0f };
    static constexpr float kTapGains[kNumTaps] = { 0.70f, 0.60f, 0.50f, 0.40f, 0.35f, 0.30f };
};
