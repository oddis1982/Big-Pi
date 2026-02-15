#pragma once
/*
  =============================================================================
  Tank.h — Big Pi Late Reverb Tank (FDN-like delay network)
  =============================================================================

  This module implements the "late reverb": the dense tail after early reflections.

  Conceptual model:
  -----------------
  A late reverb tank is basically:
    - several delay lines
    - cross-mixed by a matrix each sample
    - fed back with filters

  Why this works:
    - Delay lines create echoes.
    - Feedback makes echoes repeat and decay over time.
    - Filters in the feedback path make decay frequency-dependent (damping).
    - Cross-mixing spreads energy quickly -> high echo density -> smooth tail.

  Big Pi Tank features (Kappa-level):
  ----------------------------------
  1) 8 or 16 delay lines (Eco / HQ)
  2) Matrix mixing (Hadamard or Householder) done externally
  3) Per-line HP and LP filters in the feedback loop
  4) Multiband decay coloration:
       - low/mid/high bands decay at different rates
       - helps “natural” tail (lows can ring longer, highs die faster)
  5) Fractional delay modulation:
       - modulates read position to reduce metallic ringing
       - per-line depth/rate maps
  6) Jitter modulation:
       - smoothed random modulation on top of sinusoidal LFO
       - adds organic motion without obvious pitch wobble
  7) Envelope follower:
       - measures internal tank energy (tail “age” proxy)
       - used by engine to drive time-varying diffusion / pattern morph / dyn damping
  8) Optional saturation inside feedback:
       - adds density and “glue”
       - prevents sterile digital tail

  Real-time safety:
    - No allocations during processSample()
    - All buffers allocated in init()
*/

#include <array>
#include <cstdint>
#include <algorithm>

#include "../Dsp.h"
#include "Matrices.h"

namespace bigpi::core {

    class Tank {
    public:
        static constexpr int kMaxLines = 16;

        // --------------------------------------------------------------------------
        // Configuration
        // --------------------------------------------------------------------------

        struct Config {
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
            float drive = 1.2f;   // how hard to drive
            float satMix = 0.25f;  // 0 clean, 1 fully saturated feedback

            // Modulation
            float modDepthSamples = 0.0f; // already converted to samples by engine
            float modRateHz = 0.25f;

            // Per-line modulation scaling (maps)
            std::array<float, kMaxLines> modDepthMul{};
            std::array<float, kMaxLines> modRateMul{};

            // Jitter modulation (smoothed random)
            float jitterEnable = 1.0f;
            float jitterAmount = 0.35f; // multiplier of modDepthSamples
            float jitterRateHz = 0.35f;
            float jitterSmoothMs = 80.0f;

            // Dynamic damping (optional)
            float dynEnable = 1.0f;
            float dynAmount = 0.65f;
            float dynMinHz = 3500.0f;
            float dynMaxHz = 12000.0f;
            float dynSensitivity = 3.0f;
            float dynAtkMs = 12.0f;
            float dynRelMs = 280.0f;
        };

        // --------------------------------------------------------------------------
        // Lifecycle
        // --------------------------------------------------------------------------

        Tank() = default;

        /*
          init(sampleRate, maxDelaySamples, seed)
          ---------------------------------------
          Allocates delay line buffers and initializes internal filters/LFO helpers.

          - maxDelaySamples must be >= maximum delaySamp you plan to use.
            (we usually allocate based on sampleRate and a worst-case max delay time)
        */
        void init(float sampleRate, int maxDelaySamples, uint32_t seed);

        // Flush memory (clear delay lines and filter states)
        void clear();

        // Apply configuration (safe to call at block rate)
        void setConfig(const Config& c);

        // Access current config
        const Config& getConfig() const { return cfg; }

        // --------------------------------------------------------------------------
        // Processing
        // --------------------------------------------------------------------------

        /*
          processSample(inj, baseDecay, lfoBank, yOut)
          -------------------------------------------
          Processes ONE sample through the tank.

          Inputs:
            inj       : injection sample (mono) that feeds the tank
            baseDecay : base feedback amount (0..1), before multiband multipliers
            lfoBank   : MultiLFO used for sinusoidal per-line modulation

          Outputs:
            yOut[i]   : the raw delay line outputs after reading delays

          Notes:
            - The caller (ReverbEngine) will:
                * mix yOut through matrix
                * render tap patterns from yOut to stereo
                * optionally apply more diffusion and output stage processing
            - We still do internal feedback filtering and delay write-back here.
        */
        void processSample(float inj,
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

        // Multiband split filters (very cheap: one-pole approximations)
        std::array<dsp::OnePoleLP, kMaxLines> xLo{};
        std::array<dsp::OnePoleLP, kMaxLines> xHi{};

        // Jitter modulators (one per line)
        std::array<dsp::SmoothNoise, kMaxLines> jitter{};

        // Tail energy tracking
        dsp::EnvelopeFollower envFollower{};
        float env01 = 0.0f;

        // For dynamic damping (if enabled)
        float dynDampHzCurrent = 9000.0f;

        // Internal helper state: last outputs
        std::array<float, kMaxLines> lastY{};

        // Seed for deterministic variation
        uint32_t seed = 0x12345678u;

        // Helpers
        static inline float msToSamples(float ms, float sr) {
            return ms * 0.001f * sr;
        }

        // Compute dynamic damping cutoff based on tank envelope
        float computeDynamicDampingHz(float staticDampHz, float env01Now);
    };

} // namespace bigpi::core
#pragma once
