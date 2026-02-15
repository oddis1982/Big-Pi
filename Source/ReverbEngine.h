#pragma once
/*
  =============================================================================
  ReverbEngine.h — Big Pi Reverb Platform Orchestrator (Kappa-level)
  =============================================================================

  This is the "brain" that connects Big Pi's modules:

    - EarlyReflections  (sparse initial echoes)
    - Diffusion         (input + late diffusion smoothing)
    - Tank              (dense late tail network)
    - TapPatterns       (how we listen to the tank)
    - OutputStage       (final tone, width, saturation)

  Modes:
  ------
  A Mode is a "recipe" defined in ModePresets:
    - tank tuning (delay scale, diffusion, modulation maps, decay coloration)
    - feature flags (shimmer/granular/etc. later)
    - suggested defaults

  Real-time use:
  --------------
  ReverbEngine is meant to be called from an audio callback:

    prepare(sampleRate, blockSize)
    setParams(p)   // when knobs change (safe to call per block)
    processBlock(inL,inR,outL,outR,n)

  Notes:
  ------
  - This engine is focused on the reverb core.
  - Dry/wet mixing happens here.
  - "Advanced blocks" like Shimmer/Granular will be added later as modules.

  Beginner-friendly warning:
  --------------------------
  A reverb is many interacting systems:
    - delay network stability
    - modulation
    - filtering
    - diffusion
    - stereo rendering
  If you change parameters wildly you can break stability (runaway feedback).
  We clamp risky values to safe ranges.
*/

#include <vector>
#include <array>
#include <cstdint>

#include "Dsp.h"
#include "EarlyReflections.h"
#include "OutputStage.h"

#include "Modes/Modes.h"
#include "Modes/ModePresets.h"

#include "ReverbCore/Diffusion.h"
#include "ReverbCore/Tank.h"
#include "ReverbCore/TapPatterns.h"

class ReverbEngine {
public:
    // --------------------------------------------------------------------------
    // Public parameter set
    // --------------------------------------------------------------------------

    struct Params {
        // Mode selection
        bigpi::Mode mode = bigpi::Mode::Sky;

        // Global wet mix (0 dry, 1 fully wet)
        float mix = 0.35f;

        // Predelay: time before reverb starts (ms)
        float predelayMs = 20.0f;

        // Decay: feedback amount (0..1). Higher => longer tail.
        float decay = 0.92f;

        // Feedback damping & high-pass inside tank
        float dampingHz = 9000.0f;
        float feedbackHpHz = 30.0f;

        // Tank modulation
        float modDepthMs = 6.0f;
        float modRateHz = 0.25f;

        // Jitter modulation (organic modulation)
        float modJitterEnable = 1.0f;
        float modJitterAmount = 0.35f;
        float modJitterRateHz = 0.35f;
        float modJitterSmoothMs = 80.0f;

        // Multiband decay coloration
        float fbXoverLoHz = 250.0f;
        float fbXoverHiHz = 3500.0f;
        float decayLowMul = 1.08f;
        float decayMidMul = 1.00f;
        float decayHighMul = 0.90f;

        // Input diffusion settings (can be per-mode controlled)
        int   inputDiffStages = 6;
        float inputDiffG = 0.72f;

        // Late diffusion refinement
        float lateDiffEnable = 1.0f;
        float lateDiffAmount = 0.60f;
        float lateDiffMinG = 0.45f;
        float lateDiffMaxG = 0.72f;

        // Early reflections parameters
        float erLevel = 0.30f;
        float erSize = 0.55f;
        float erDampHz = 9000.0f;
        float erWidth = 1.0f;

        // Output stage parameters
        float outHpHz = 20.0f;
        float outLowShelfHz = 200.0f;
        float outLowGainDb = 0.0f;

        float outHighShelfHz = 8000.0f;
        float outHighGainDb = 0.0f;

        float outWidth = 1.10f;
        float outDrive = 0.0f;
        float outLevel = 1.0f;

        // Freeze: forces infinite-ish sustain (feedback ~1). We'll handle safely.
        float freeze = 0.0f;

        // Simple ducking support (optional)
        float duckEnable = 0.0f;
        float duckThresholdDb = -28.0f;
        float duckDepthDb = 10.0f;

        // Loudness compensation (keeps wet consistent across decay changes)
        float loudCompEnable = 1.0f;
        float loudCompStrength = 0.50f;
        float loudCompMaxDb = 9.0f;
    };

    ReverbEngine() = default;

    // --------------------------------------------------------------------------
    // Lifecycle
    // --------------------------------------------------------------------------

    void prepare(float sampleRate, int blockSize);
    void reset();
    void setParams(const Params& p);

    // --------------------------------------------------------------------------
    // Processing
    // --------------------------------------------------------------------------

    void processBlock(const float* inL, const float* inR,
        float* outL, float* outR,
        int n);

private:
    float sr = 48000.0f;
    int   block = 64;
    bool  prepared = false;

    Params target{};

    // Current mode preset (cached so we don't recompute each sample)
    bigpi::ModeConfig modeCfg{};

    // Modules
    EarlyReflections er{};
    bigpi::core::Diffusion diffusion{};
    bigpi::core::Tank tank{};
    OutputStage outStage{};

    // Predelay: stereo delay lines
    dsp::DelayLine preL{}, preR{};

    // Modulation helpers
    dsp::MultiLFO lfos{};

    // Ducking helper
    dsp::EnvelopeFollower duckEnv{};

    // Temporary buffers for wet processing
    std::vector<float> wetL{};
    std::vector<float> wetR{};
    std::vector<float> erL{};
    std::vector<float> erR{};

    // Helpers
    void applyModePreset(bigpi::Mode m);
    float computeEffectiveDecay(float decay, float freeze01) const;
    float computeLoudnessCompDb(float decay01) const;
};
#pragma once
