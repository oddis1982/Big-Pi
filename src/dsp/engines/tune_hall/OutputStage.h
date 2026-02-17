#pragma once
/*
  =============================================================================
  OutputStage.h — Big Pi Final Wet Signal Processing
  =============================================================================
*/

#include "dsp/common/Dsp.h"

class OutputStage {
public:
    struct Params {
        float hpHz = 20.0f;     // rumble removal
        float lowShelfHz = 200.0f;
        float lowGainDb = 0.0f;

        float highShelfHz = 8000.0f;
        float highGainDb = 0.0f;

        float width = 1.0f;   // stereo width multiplier
        float drive = 0.0f;   // saturation drive
        float level = 1.0f;   // output level scaling
    };

    OutputStage() = default;

    void prepare(float sampleRate);
    void reset();
    void setParams(const Params& p);

    // In-place processing of wet buffers
    void processBlock(float* wetL, float* wetR, int n);

private:
    float sr = 48000.0f;

    Params target{};

    // Filters
    dsp::Biquad hpL{}, hpR{};
    dsp::Biquad lowL{}, lowR{};
    dsp::Biquad highL{}, highR{};

    // Smoothers
    dsp::SmoothValue widthSm{};
    dsp::SmoothValue driveSm{};
    dsp::SmoothValue levelSm{};

    bool prepared = false;

    // Updates filter coefficients based on current target params.
    // Called from prepare() and setParams().
    void updateFilters();
};
