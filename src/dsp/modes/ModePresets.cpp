#include "ModePresets.h"

/*
  =============================================================================
  ModePresets.cpp — Big Pi Mode Recipes (updated: Hall tuning pass)
  =============================================================================
*/

namespace bigpi {

    // Helper to fill default modulation maps with a pleasing spread.
    static void fillModMap_Default(std::array<float, 16>& depthMul,
        std::array<float, 16>& rateMul) {
        for (int i = 0; i < 16; ++i) {
            float t = (16 == 1) ? 0.0f : float(i) / 15.0f;
            depthMul[i] = 0.85f + 0.30f * t; // 0.85..1.15
            rateMul[i] = 0.80f + 0.40f * t; // 0.80..1.20
        }
    }

    static void fillModMap_Plate(std::array<float, 16>& depthMul,
        std::array<float, 16>& rateMul) {
        for (int i = 0; i < 16; ++i) {
            float t = (16 == 1) ? 0.0f : float(i) / 15.0f;
            depthMul[i] = 0.92f + 0.16f * t; // 0.92..1.08
            rateMul[i] = 0.90f + 0.20f * t; // 0.90..1.10
        }
    }

    static void fillModMap_Sky(std::array<float, 16>& depthMul,
        std::array<float, 16>& rateMul) {
        for (int i = 0; i < 16; ++i) {
            float t = (16 == 1) ? 0.0f : float(i) / 15.0f;
            depthMul[i] = 0.75f + 0.50f * t; // 0.75..1.25
            rateMul[i] = 0.70f + 0.60f * t; // 0.70..1.30
        }
    }

    static void fillModMap_Vintage(std::array<float, 16>& depthMul,
        std::array<float, 16>& rateMul) {
        for (int i = 0; i < 16; ++i) {
            float t = (16 == 1) ? 0.0f : float(i) / 15.0f;
            depthMul[i] = 0.85f + 0.25f * t; // 0.85..1.10
            rateMul[i] = 0.60f + 0.30f * t; // 0.60..0.90
        }
    }

    ModeConfig getModePreset(Mode m) {
        ModeConfig cfg{};
        cfg.mode = m;

        // Global defaults (sensible Kappa baseline)
        cfg.tank.delayLines = 16;
        cfg.tank.delayScale = 1.0f;
        cfg.tank.useHouseholder = true;

        cfg.tank.inputDiffStages = 6;
        cfg.tank.inputDiffG = 0.72f;

        cfg.tank.lateDiffMinG = 0.45f;
        cfg.tank.lateDiffMaxG = 0.72f;

        cfg.tank.modDepthMs = 6.0f;
        cfg.tank.modRateHz = 0.25f;

        cfg.tank.decayLowMul = 1.05f;
        cfg.tank.decayMidMul = 1.00f;
        cfg.tank.decayHighMul = 0.90f;

        cfg.tank.tapPattern = 0;
        cfg.tank.tapPatternLate = 1;

        fillModMap_Default(cfg.tank.modDepthMul, cfg.tank.modRateMul);

        cfg.defaultMix = 0.35f;
        cfg.defaultDecay = 0.92f;
        cfg.defaultDamping = 9000.0f;
        cfg.defaultPreDelay = 20.0f;

        cfg.defaultERLevel = 0.30f;
        cfg.defaultERSize = 0.55f;

        switch (m) {

        case Mode::Room: {
            cfg.tank.delayScale = 0.78f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 5;
            cfg.tank.inputDiffG = 0.70f;

            cfg.tank.modDepthMs = 4.5f;
            cfg.tank.modRateHz = 0.35f;

            cfg.tank.decayLowMul = 1.02f;
            cfg.tank.decayHighMul = 0.92f;

            cfg.defaultPreDelay = 10.0f;
            cfg.defaultDecay = 0.86f;
            cfg.defaultDamping = 11000.0f;

            cfg.defaultERLevel = 0.35f;
            cfg.defaultERSize = 0.45f;
        } break;

        case Mode::Hall: {
            // Hall tuning pass (Roadmap 1.0 / Phase 1)
            cfg.tank.delayScale = 1.15f;
            cfg.tank.useHouseholder = true;

            // Softer onset: slightly lower diffusion coefficient
            cfg.tank.inputDiffStages = 6;
            cfg.tank.inputDiffG = 0.68f;

            // Late diffusion range
            cfg.tank.lateDiffMinG = 0.48f;
            cfg.tank.lateDiffMaxG = 0.74f;

            // Slow, subtle modulation (avoid chorus wobble)
            cfg.tank.modDepthMs = 4.5f;
            cfg.tank.modRateHz = 0.18f;

            // Tonal realism: lows linger, highs decay faster
            cfg.tank.decayLowMul = 1.12f;
            cfg.tank.decayMidMul = 1.00f;
            cfg.tank.decayHighMul = 0.86f;

            // Suggested UI defaults
            cfg.defaultPreDelay = 25.0f;
            cfg.defaultDecay = 0.93f;
            cfg.defaultDamping = 9000.0f;

            cfg.defaultERLevel = 0.24f;
            cfg.defaultERSize = 0.70f;

            fillModMap_Default(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::Cathedral: {
            cfg.tank.delayScale = 1.35f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 7;
            cfg.tank.inputDiffG = 0.75f;

            cfg.tank.modDepthMs = 7.5f;
            cfg.tank.modRateHz = 0.18f;

            cfg.tank.decayLowMul = 1.12f;
            cfg.tank.decayHighMul = 0.82f;

            cfg.defaultPreDelay = 35.0f;
            cfg.defaultDecay = 0.95f;
            cfg.defaultDamping = 7500.0f;

            cfg.defaultERLevel = 0.22f;
            cfg.defaultERSize = 0.75f;
        } break;

        case Mode::Plate: {
            cfg.tank.delayScale = 0.95f;
            cfg.tank.useHouseholder = false;

            cfg.tank.inputDiffStages = 7;
            cfg.tank.inputDiffG = 0.77f;

            cfg.tank.modDepthMs = 5.0f;
            cfg.tank.modRateHz = 0.30f;

            cfg.tank.decayLowMul = 1.00f;
            cfg.tank.decayHighMul = 0.93f;

            cfg.defaultPreDelay = 5.0f;
            cfg.defaultDecay = 0.90f;
            cfg.defaultDamping = 12000.0f;

            cfg.defaultERLevel = 0.20f;
            cfg.defaultERSize = 0.40f;

            fillModMap_Plate(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::Spring: {
            cfg.features.useSpringModel = true;

            cfg.tank.delayScale = 0.80f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 4;
            cfg.tank.inputDiffG = 0.66f;

            cfg.tank.modDepthMs = 2.5f;
            cfg.tank.modRateHz = 0.45f;

            cfg.tank.decayLowMul = 0.98f;
            cfg.tank.decayHighMul = 0.95f;

            cfg.defaultPreDelay = 0.0f;
            cfg.defaultDecay = 0.80f;
            cfg.defaultDamping = 14000.0f;

            cfg.defaultERLevel = 0.15f;
            cfg.defaultERSize = 0.35f;
        } break;

        case Mode::Vintage: {
            cfg.tank.delayScale = 1.05f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 6;
            cfg.tank.inputDiffG = 0.72f;

            cfg.tank.modDepthMs = 6.0f;
            cfg.tank.modRateHz = 0.16f;

            cfg.tank.decayLowMul = 1.06f;
            cfg.tank.decayHighMul = 0.86f;

            cfg.defaultPreDelay = 18.0f;
            cfg.defaultDecay = 0.90f;
            cfg.defaultDamping = 8200.0f;

            cfg.defaultERLevel = 0.26f;
            cfg.defaultERSize = 0.55f;

            fillModMap_Vintage(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::Sky: {
            cfg.tank.delayScale = 1.20f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 8;
            cfg.tank.inputDiffG = 0.78f;

            cfg.tank.modDepthMs = 8.0f;
            cfg.tank.modRateHz = 0.14f;

            cfg.tank.decayLowMul = 1.10f;
            cfg.tank.decayHighMul = 0.84f;

            cfg.defaultPreDelay = 28.0f;
            cfg.defaultDecay = 0.95f;
            cfg.defaultDamping = 9000.0f;

            cfg.defaultERLevel = 0.18f;
            cfg.defaultERSize = 0.65f;

            fillModMap_Sky(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::Blossom: {
            cfg.features.useBlossomEnv = true;

            cfg.tank.delayScale = 1.12f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 7;
            cfg.tank.inputDiffG = 0.76f;

            cfg.tank.modDepthMs = 7.0f;
            cfg.tank.modRateHz = 0.18f;

            cfg.tank.decayLowMul = 1.10f;
            cfg.tank.decayHighMul = 0.86f;

            cfg.defaultPreDelay = 20.0f;
            cfg.defaultDecay = 0.94f;
            cfg.defaultDamping = 9000.0f;

            cfg.defaultERLevel = 0.20f;
            cfg.defaultERSize = 0.60f;
        } break;

        case Mode::Shimmer: {
            cfg.features.usePitchBlock = true;

            cfg.tank.delayScale = 1.25f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 8;
            cfg.tank.inputDiffG = 0.78f;

            cfg.tank.modDepthMs = 7.5f;
            cfg.tank.modRateHz = 0.14f;

            cfg.tank.decayLowMul = 1.06f;
            cfg.tank.decayHighMul = 0.82f;

            cfg.defaultPreDelay = 30.0f;
            cfg.defaultDecay = 0.95f;
            cfg.defaultDamping = 8500.0f;

            cfg.defaultERLevel = 0.16f;
            cfg.defaultERSize = 0.62f;

            fillModMap_Sky(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::Magnetic: {
            cfg.features.useMagneticBlock = true;

            cfg.tank.delayScale = 1.00f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 6;
            cfg.tank.inputDiffG = 0.72f;

            cfg.tank.modDepthMs = 5.5f;
            cfg.tank.modRateHz = 0.22f;

            cfg.tank.decayLowMul = 1.04f;
            cfg.tank.decayHighMul = 0.88f;

            cfg.defaultPreDelay = 10.0f;
            cfg.defaultDecay = 0.90f;
            cfg.defaultDamping = 10000.0f;

            cfg.defaultERLevel = 0.18f;
            cfg.defaultERSize = 0.55f;
        } break;

        case Mode::Granular: {
            cfg.features.useGranularBlock = true;

            cfg.tank.delayScale = 1.10f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 7;
            cfg.tank.inputDiffG = 0.75f;

            cfg.tank.modDepthMs = 8.0f;
            cfg.tank.modRateHz = 0.16f;

            cfg.defaultPreDelay = 10.0f;
            cfg.defaultDecay = 0.92f;
            cfg.defaultDamping = 10000.0f;
        } break;

        case Mode::Singularity: {
            cfg.features.useSingularity = true;

            cfg.tank.delayScale = 1.45f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 8;
            cfg.tank.inputDiffG = 0.79f;

            cfg.tank.modDepthMs = 9.0f;
            cfg.tank.modRateHz = 0.12f;

            cfg.tank.decayLowMul = 1.15f;
            cfg.tank.decayHighMul = 0.78f;

            cfg.defaultPreDelay = 30.0f;
            cfg.defaultDecay = 0.96f;
            cfg.defaultDamping = 8200.0f;

            fillModMap_Sky(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        case Mode::MicroCosmic: {
            cfg.features.useGranularBlock = true;

            cfg.tank.delayScale = 1.20f;
            cfg.tank.useHouseholder = true;

            cfg.tank.inputDiffStages = 7;
            cfg.tank.inputDiffG = 0.76f;

            cfg.tank.modDepthMs = 8.5f;
            cfg.tank.modRateHz = 0.14f;

            cfg.defaultPreDelay = 12.0f;
            cfg.defaultDecay = 0.94f;
            cfg.defaultDamping = 9500.0f;

            fillModMap_Sky(cfg.tank.modDepthMul, cfg.tank.modRateMul);
        } break;

        default:
        case Mode::Count:
            break;
        }

        cfg.tank.inputDiffStages = std::max(0, std::min(cfg.tank.inputDiffStages, 8));
        return cfg;
    }

} // namespace bigpi
