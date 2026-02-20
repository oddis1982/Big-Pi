#pragma once
/*
  =============================================================================
  ReverbEngine.h — Big Pi Reverb Platform Orchestrator (Kappa+Cloud Mod)
  =============================================================================
*/

#include <vector>
#include <array>
#include <cstdint>
#include <algorithm> // std::min/std::max used in implementation

#include "dsp/common/Dsp.h"
#include "dsp/engines/tune_hall/EarlyReflections.h"
#include "dsp/engines/tune_hall/OutputStage.h"

#include "dsp/modes/Modes.h"
#include "dsp/modes/ModePresets.h"

#include "dsp/diffusion/Diffusion.h"
#include "dsp/tail/Tank.h"
#include "dsp/tail/TapPatterns.h"

class ReverbEngine {
public:
    struct Params {
        bigpi::Mode mode = bigpi::Mode::Hall;

        float mix = 0.35f;

        float predelayMs = 20.0f;

        float decay = 0.92f;

        float dampingHz = 9000.0f;
        float feedbackHpHz = 30.0f;

        float modDepthMs = 6.0f;
        float modRateHz = 0.25f;

        float modJitterEnable = 1.0f;
        float modJitterAmount = 0.35f;
        float modJitterRateHz = 0.35f;
        float modJitterSmoothMs = 80.0f;

        float fbXoverLoHz = 250.0f;
        float fbXoverHiHz = 3500.0f;
        float decayLowMul = 1.08f;
        float decayMidMul = 1.00f;
        float decayHighMul = 0.90f;

        int   inputDiffStages = 6;
        float inputDiffG = 0.72f;

        float lateDiffEnable = 1.0f;
        float lateDiffAmount = 0.60f;
        float lateDiffMinG = 0.45f;
        float lateDiffMaxG = 0.72f;

        float erLevel = 0.30f;
        float erSize = 0.55f;
        float erDampHz = 9000.0f;
        float erWidth = 1.0f;

        // ---------------------------------------------------------------------
        // Kappa+Cloud Mod — Step 1: decorrelated stereo tank injection depth
        // ---------------------------------------------------------------------
        float stereoDepth = 0.0f;

        // ---------------------------------------------------------------------
        // Kappa+Cloud Mod — Level 2: Cloudify modulation (Spin + Wander)
        // ---------------------------------------------------------------------
        float cloudEnable = 0.0f;
        float cloudSpinHz = 0.045f;
        float cloudWanderAmount = 0.55f;
        float cloudWanderRateHz = 0.08f;
        float cloudWanderSmoothMs = 500.0f;

        // ---------------------------------------------------------------------
        // Kappa+Cloud Mod — Step 3: Cloud front-end multitap "spray"
        // ---------------------------------------------------------------------
        float cloudFrontEnable = 0.0f;
        float cloudFrontAmount = 0.45f;
        float cloudFrontSizeMs = 22.0f;
        float cloudFrontWidth = 0.70f;

        // ---------------------------------------------------------------------
        // Kappa+Cloud Mod — Step 4–5 (combined):
        //  Step 4: Cloud-specific delay-line set (Sky)
        //  Step 5: Post-tank micro-smear
        // ---------------------------------------------------------------------
        float cloudDelaySetEnable = 1.0f;

        float cloudSmearEnable = 0.0f;
        float cloudSmearAmount = 0.35f;
        float cloudSmearTimeMs = 14.0f;
        float cloudSmearWidth = 0.75f;

        // ---------------------------------------------------------------------
        // Kappa+Cloud Mod — Step 6: Dynamic diffusion refinement (polish)
        //
        // dynDiffEnable:
        //   enables time-varying diffusion behavior
        //
        // dynDiffTailBoost:
        //   boosts input diffusion as the tank tail builds (0..1)
        //
        // dynDiffTransientReduce:
        //   reduces input diffusion on transients to preserve pick attack (0..1)
        //
        // dynDiffLateBoost:
        //   boosts late diffusion amount as tail builds (0..1)
        // ---------------------------------------------------------------------
        float dynDiffEnable = 1.0f;
        float dynDiffTailBoost = 0.40f;
        float dynDiffTransientReduce = 0.35f;
        float dynDiffLateBoost = 0.35f;

        float outHpHz = 20.0f;
        float outLowShelfHz = 200.0f;
        float outLowGainDb = 0.0f;

        float outHighShelfHz = 8000.0f;
        float outHighGainDb = 0.0f;

        float outWidth = 1.10f;
        float outDrive = 0.0f;
        float outLevel = 1.0f;

        float freeze = 0.0f;

        float duckEnable = 0.0f;
        float duckThresholdDb = -28.0f;
        float duckDepthDb = 10.0f;

        float loudCompEnable = 1.0f;
        float loudCompStrength = 0.50f;
        float loudCompMaxDb = 9.0f;
    };

    ReverbEngine() = default;

    void prepare(float sampleRate, int blockSize);
    void reset();
    void setParams(const Params& p);

    void processBlock(const float* inL, const float* inR,
        float* outL, float* outR,
        int n);

private:
    float sr = 48000.0f;
    int   block = 64;
    bool  prepared = false;

    Params target{};

    bigpi::ModeConfig modeCfg{};

    // -------------------------------------------------------------------------
    // Kappa+Cloud Mod — Step 1: MS decorrelated injection vectors
    // -------------------------------------------------------------------------
    std::array<float, bigpi::core::Tank::kMaxLines> injVec{};
    std::array<float, bigpi::core::Tank::kMaxLines> vM{};
    std::array<float, bigpi::core::Tank::kMaxLines> vS{};
    int lastStereoVecN = -1;
    void rebuildStereoVectors(int lines);

    EarlyReflections er{};
    bigpi::core::Diffusion diffusion{};
    bigpi::core::Tank tank{};
    OutputStage outStage{};

    dsp::DelayLine preL{}, preR{};

    // Step 5: post-tank micro-smear buffers (small, RT-safe)
    dsp::DelayLine smearL{}, smearR{};

    dsp::MultiLFO lfos{};

    dsp::EnvelopeFollower duckEnv{};

    // Step 6: dynamic diffusion helpers (RT-safe, no allocations)
    dsp::EnvelopeFollower diffFast{};
    dsp::EnvelopeFollower diffSlow{};
    float tailEnvSm = 0.0f;

    // REAL-TIME RULE:
    // These vectors must be sized ONLY in prepare(). processBlock() must not resize.
    std::vector<float> wetL{};
    std::vector<float> wetR{};
    std::vector<float> erL{};
    std::vector<float> erR{};

    void applyModePreset(bigpi::Mode m);
    float computeEffectiveDecay(float decay, float freeze01) const;
    float computeLoudnessCompDb(float decay01) const;
};