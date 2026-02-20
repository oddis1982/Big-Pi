// =========================== src/dsp/tail/Tank.h ============================
#pragma once
/*
  =============================================================================
  Tank.h � Big Pi Late Reverb Tank (FDN-like delay network)
  =============================================================================

  This module implements the "late reverb": the dense tail after early reflections.

  Conceptual model:
  -----------------
  A late reverb tank is basically:
    - several delay lines
    - cross-mixed by a matrix each sample
    - fed back with filters

  Big Pi Tank features:
  ---------------------
  1) 8 or 16 delay lines (Eco / HQ)
  2) Matrix mixing (Hadamard or Householder)
  3) Per-line HP and LP filters in the feedback loop
  4) Multiband decay coloration:
       - low/mid/high bands decay at different rates
  5) Fractional delay modulation:
       - per-line LFO modulation
  6) Jitter modulation:
       - smoothed random modulation on top of sinusoidal LFO
  7) Envelope follower:
       - measures internal tank energy (tail “age” proxy)
  8) Optional saturation inside feedback:
       - adds density and “glue”

  Real-time safety:
    - No allocations during processSample()
    - All buffers allocated in init()
*/

#include <array>
#include <cstdint>

#include "dsp/common/Dsp.h"
#include "dsp/tail/Matrices.h"

namespace bigpi::core {

    class Tank {
    public:
        static constexpr int kMaxLines = 16;

        // ----------------------------------------------------------------------
        // Configuration
        // ----------------------------------------------------------------------

        struct Config {
            // Number of active delay lines. Clamped internally to [1, kMaxLines].
            int lines = 16;

            MatrixType matrix = MatrixType::Householder;

            // Delay times for each line (samples)
            std::array<float, kMaxLines> delaySamp{};

            // Feedback filtering
            float fbHpHz = 30.0f;     // removes LF/DC build-up inside feedback loop
            float dampHz = 9000.0f;   // low-pass damping inside feedback loop

            // Multiband crossover points
            float xoverLoHz = 250.0f;
            float xoverHiHz = 3500.0f;

            // Multiband decay multipliers
            float decayLowMul = 1.08f;
            float decayMidMul = 1.00f;
            float decayHighMul = 0.90f;

            // Saturation inside feedback loop
            float drive = 1.2f;     // how hard to drive
            float satMix = 0.25f;   // 0 clean, 1 fully saturated feedback

            // Modulation
            float modDepthSamples = 0.0f; // already converted to samples by engine
            float modRateHz = 0.25f;

            // Per-line modulation scaling (maps)
            // IMPORTANT: These must be filled by presets/engine.
            // If left as all zeros, modulation will effectively do nothing.
            std::array<float, kMaxLines> modDepthMul{};
            std::array<float, kMaxLines> modRateMul{};

            // Jitter modulation (smoothed random)
            float jitterEnable = 1.0f;
            float jitterAmount = 0.35f;   // multiplier of modDepthSamples
            float jitterRateHz = 0.35f;
            float jitterSmoothMs = 80.0f;


            // ------------------------------------------------------------------
            // Kappa+Cloud Mod (Level 2): Cloudify modulation (spin + wander)
            // cloudEnable: enables slow spatial drift modulation
            // cloudSpinHz: global rotation speed (very slow)
            // cloudWanderAmount: per-line drift amount (0..1, scaled by modDepth)
            // cloudWanderRateHz: drift noise rate
            // cloudWanderSmoothMs: drift noise smoothing
            // ------------------------------------------------------------------
            float cloudEnable = 0.0f;
            float cloudSpinHz = 0.045f;
            float cloudWanderAmount = 0.55f;
            float cloudWanderRateHz = 0.08f;
            float cloudWanderSmoothMs = 500.0f;

            // Dynamic damping (optional)
            float dynEnable = 1.0f;
            float dynAmount = 0.65f;
            float dynMinHz = 3500.0f;
            float dynMaxHz = 12000.0f;
            float dynSensitivity = 3.0f;
            float dynAtkMs = 12.0f;
            float dynRelMs = 280.0f;
        };

        // ----------------------------------------------------------------------
        // Lifecycle
        // ----------------------------------------------------------------------

        Tank() = default;

        void init(float sampleRate, int maxDelaySamples, uint32_t seed);

        // Flush memory (clear delay lines and filter states)
        void clear();

        // Apply configuration (safe to call at block rate)
        void setConfig(const Config& c);

        // Access current config
        const Config& getConfig() const { return cfg; }

        // ----------------------------------------------------------------------
        // Processing
        // ----------------------------------------------------------------------

        void processSample(float inj,
            float baseDecay,
            dsp::MultiLFO& lfoBank,
            std::array<float, kMaxLines>& yOut);

        /*
          processSampleVec(injVec, baseDecay, lfoBank, yOut)
          --------------------------------------------------
          Kappa+Cloud Mod (Step 1): per-line injection vector.
          Only the first cfg.lines entries are read.
        */
        void processSampleVec(const std::array<float, kMaxLines>& injVec,
            float baseDecay,
            dsp::MultiLFO& lfoBank,
            std::array<float, kMaxLines>& yOut);

        // Envelope output (0..1-ish), used as a tail energy proxy.
        float getEnv01() const { return env01; }

    private:
        float sr = 48000.0f;
        bool inited = false;

        Config cfg{};

        // Delay lines: one per line
        std::array<dsp::DelayLine, kMaxLines> d{};

        // Feedback filters per line
        std::array<dsp::OnePoleHP, kMaxLines> hp{};
        std::array<dsp::OnePoleLP, kMaxLines> lp{};

        // Multiband split filters (cheap: one-pole approximations)
        std::array<dsp::OnePoleLP, kMaxLines> xLo{};
        std::array<dsp::OnePoleLP, kMaxLines> xHi{};

        // Jitter modulators (one per line)
        std::array<dsp::SmoothNoise, kMaxLines> jitter{};

        // Kappa+Cloud Mod (Level 2): slow drift modulators
        std::array<dsp::SmoothNoise, kMaxLines> cloudNoise{};
        std::array<float, kMaxLines> cloudPhaseOffset{};
        float cloudPhase = 0.0f;

        // Tail energy tracking
        dsp::EnvelopeFollower envFollower{};
        float env01 = 0.0f;

        // Smoothed dynamic damping cutoff
        float dynDampHzCurrent = 9000.0f;

        // ----------------------------------------------------------------------
        // Kappa upgrade: RT60-based decay gains (stable, line-length aware)
        // ----------------------------------------------------------------------
        float lastDecay01 = -1.0f; // invalid forces a recompute

        std::array<float, kMaxLines> fbGainLow{};
        std::array<float, kMaxLines> fbGainMid{};
        std::array<float, kMaxLines> fbGainHigh{};

        void updateDecayGains(float decay01);

        // Last outputs (debug/inspection; not required for sound)
        std::array<float, kMaxLines> lastY{};

        // Seed for deterministic variation
        uint32_t seed = 0x12345678u;

        float computeDynamicDampingHz(float staticDampHz, float env01Now);
    };

} // namespace bigpi::core