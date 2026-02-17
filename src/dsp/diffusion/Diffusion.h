#pragma once
/*
  =============================================================================
  Diffusion.h — Big Pi Diffusion Module (input + late diffusion)
  =============================================================================

  Diffusion in reverbs:
  ---------------------
  When a sound reflects in a real space, it quickly becomes extremely dense:
    many overlapping reflections, arriving at slightly different times.

  In algorithmic reverbs, we create this density using "allpass diffusers":
    - Allpass filters preserve magnitude (roughly) but scramble phase.
    - Phase scrambling makes echoes overlap and smooth out.

  Big Pi uses diffusion in two places:

    1) Input diffusion:
         Applied soon after the input / early reflections.
         Purpose:
           - "smear" the input
           - prevent distinct echoes before the tank densifies

    2) Late diffusion refinement:
         Applied after the tank output (still inside the wet path).
         Purpose:
           - add subtle smoothness to the tail
           - reduce flutter / grain from modulation

  Time-varying diffusion:
  -----------------------
  The diffuser coefficient g can change depending on tail "age" or energy.
  This can create more natural build-up:
    - early: less diffusion (preserve articulation)
    - late: more diffusion (dense smooth tail)

  Real-time safety:
  -----------------
  - init() may allocate (sets up delay buffers)
  - processInput/processLate must not allocate
*/

#include <array>
#include <cstdint>
#include <algorithm>

#include "dsp/common/Dsp.h"

namespace bigpi::core {

    class Diffusion {
    public:
        // Maximum stages supported (kept small & fixed for real-time and simplicity).
        static constexpr int kMaxInputStages = 8;
        static constexpr int kLateStages = 3;

        // ----------------------------------------------------------------------
        // Configuration structures
        // ----------------------------------------------------------------------

        struct InputConfig {
            int   stages = 6;     // how many allpasses are active (0..kMaxInputStages)
            float g = 0.72f;      // allpass coefficient (diffusion strength)

            // Base delay times in ms for L/R (will be slightly offset for decorrelation)
            // These are short delays, typically ~1..20ms.
            std::array<float, kMaxInputStages> timesMsL{};
            std::array<float, kMaxInputStages> timesMsR{};
        };

        struct LateConfig {
            float minG = 0.45f;
            float maxG = 0.72f;

            // Base delay times (ms) for late diffusion stages
            std::array<float, kLateStages> timesMsL{};
            std::array<float, kLateStages> timesMsR{};
        };

        // ----------------------------------------------------------------------
        // Lifecycle
        // ----------------------------------------------------------------------

        Diffusion() = default;

        /*
          init(sampleRate, seed)
          ----------------------
          Allocates allpass buffers and initializes stage delay times.

          - seed ensures L/R delay offsets differ per instance if desired.
        */
        void init(float sampleRate, uint32_t seed);

        // Clears internal state (flushes diffusion memory).
        void clear();

        // ----------------------------------------------------------------------
        // Configuration
        // ----------------------------------------------------------------------

        // Safe to call at block-rate (e.g., when knobs change).
        void setInputConfig(const InputConfig& cfg);
        void setLateConfig(const LateConfig& cfg);

        // Time-varying diffusion control:
        // supply a "g" each sample (or per block). Keep this cheap.
        void setTimeVaryingG(float g) { tvG = g; }
        float getTimeVaryingG() const { return tvG; }

        // ----------------------------------------------------------------------
        // Processing
        // ----------------------------------------------------------------------

        // Applies the input diffusion chain in-place.
        void processInput(float& L, float& R);

        /*
          processLate(L, R, amount01)
          ---------------------------
          Applies late diffusion. amount01 crossfades between:
            0 => no late diffusion
            1 => fully late-diffused
        */
        void processLate(float& L, float& R, float amount01);

    private:
        float sr = 48000.0f;

        // Stored configs
        InputConfig inputCfg{};
        LateConfig  lateCfg{};

        // Allpass chains
        std::array<dsp::Allpass, kMaxInputStages> inL{};
        std::array<dsp::Allpass, kMaxInputStages> inR{};

        std::array<dsp::Allpass, kLateStages> lateL{};
        std::array<dsp::Allpass, kLateStages> lateR{};

        // How many input stages are active
        int activeInputStages = 6;

        // Time-varying diffusion coefficient (optional modulation source)
        float tvG = 0.72f;

        bool inited = false;

        static int clampInputStages(int s) {
            return std::max(0, std::min(s, kMaxInputStages));
        }

        static float clamp01(float x) {
            return std::max(0.0f, std::min(1.0f, x));
        }
    };

} // namespace bigpi::core
