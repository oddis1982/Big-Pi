#pragma once
/*
  =============================================================================
  OutputStage.h — Big Pi Final Wet Signal Processing
  =============================================================================

  The OutputStage processes the already-built wet signal
  (early reflections + late tail) before it is mixed with dry.

  Why we need this stage:
  -----------------------
  High-end reverbs rarely output the raw tank signal directly.
  They typically include:

    1) Tone shaping (low/high shelves)
    2) High-pass filtering (remove rumble)
    3) Stereo width control
    4) Subtle saturation (glue/density)
    5) Limiting or safety clipping

  This stage is intentionally simple but powerful.
  It allows each mode to feel:
    - darker or brighter
    - wider or narrower
    - smoother or more aggressive

  Processing order (per sample):
  --------------------------------
    1) High-pass filter (rumble removal)
    2) Low shelf
    3) High shelf
    4) Stereo width (Mid/Side)
    5) Soft saturation
    6) Output level scaling

  Real-time safety:
    - No allocations in processBlock
    - Filters configured in prepare/setParams
*/

#include "Dsp.h"

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

    void updateFilters();
};
#pragma once
