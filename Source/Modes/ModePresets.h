#pragma once
/*
  =============================================================================
  ModePresets.h — Big Pi Mode Configuration System
  =============================================================================

  This file defines how a Mode maps to:

    - Tank topology settings
    - Diffusion configuration
    - Modulation maps
    - Feature flags (pitch block, granular block, etc.)
    - Default parameter suggestions

  Important concept:
  ------------------
  A Mode is NOT a separate engine.
  It is a "recipe" that configures the core reverb platform.

  This allows:
    - Adding new modes without rewriting DSP
    - Keeping engine modular and maintainable
    - Clean separation of behavior vs processing

  Implementation note:
  --------------------
  ModePresets.cpp will return a ModeConfig object for a given Mode.
*/

#include <array>
#include "../Dsp.h"
#include "Modes.h"

namespace bigpi {

    // ============================================================================
    // Feature Flags (Optional DSP Blocks)
    // ============================================================================

    struct ModeFeatures {
        bool usePitchBlock = false;  // Shimmer
        bool useGranularBlock = false;  // Granular
        bool useMagneticBlock = false;  // Magnetic
        bool useSingularity = false;  // Blackhole-style warp
        bool useSpringModel = false;  // Physical spring structure
        bool useBlossomEnv = false;  // Bloom-style swell shaping
    };

    // ============================================================================
    // Tank Configuration (Core Reverb Topology)
    // ============================================================================

    struct TankConfig {

        // Delay network structure
        int   delayLines = 16;    // 8 (Eco) or 16 (HQ)
        float delayScale = 1.0f;  // scales base delay set
        bool  useHouseholder = true;  // else Hadamard

        // Diffusion (input)
        int   inputDiffStages = 6;
        float inputDiffG = 0.72f;

        // Late diffusion refinement
        float lateDiffMinG = 0.45f;
        float lateDiffMaxG = 0.72f;

        // Modulation base
        float modDepthMs = 6.0f;
        float modRateHz = 0.25f;

        // Multiband decay coloration
        float decayLowMul = 1.05f;
        float decayMidMul = 1.00f;
        float decayHighMul = 0.90f;

        // Tap pattern selection (late output)
        int tapPattern = 0;
        int tapPatternLate = 1;

        // Per-line modulation multipliers
        std::array<float, 16> modDepthMul{};
        std::array<float, 16> modRateMul{};
    };

    // ============================================================================
    // Complete Mode Configuration
    // ============================================================================

    struct ModeConfig {

        Mode mode;

        TankConfig tank;

        ModeFeatures features;

        // Suggested default parameters (UI defaults)
        float defaultMix = 0.35f;
        float defaultDecay = 0.92f;
        float defaultDamping = 9000.0f;
        float defaultPreDelay = 20.0f;

        float defaultERLevel = 0.30f;
        float defaultERSize = 0.55f;
    };

    // ============================================================================
    // API
    // ============================================================================

    /*
      getModePreset(mode)

      Returns a fully populated ModeConfig.

      This is the single source of truth for:
        - How Room differs from Hall
        - How Sky differs from Cathedral
        - Which modes activate shimmer/pitch/etc.
    */
    ModeConfig getModePreset(Mode m);

} // namespace bigpi
#pragma once
