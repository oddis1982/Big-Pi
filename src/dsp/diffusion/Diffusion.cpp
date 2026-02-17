#include "Diffusion.h"

/*
  =============================================================================
  Diffusion.cpp — Big Pi Diffusion Module (implementation)
  =============================================================================
*/

namespace bigpi::core {

    static inline float msToSamples(float ms, float sr) {
        return ms * 0.001f * sr;
    }

    void Diffusion::init(float sampleRate, uint32_t seed) {
        sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;

        // Allocate buffers large enough to hold our max diffusion delay.
        // Input diffusion delay is small (< ~25ms).
        // Late diffusion also small (< ~20ms).
        const int maxDelaySamples = std::max(16, int(sr * 0.030f)); // 30ms safe cap

        for (int i = 0; i < kMaxInputStages; ++i) {
            inL[i].init(maxDelaySamples);
            inR[i].init(maxDelaySamples);
        }
        for (int i = 0; i < kLateStages; ++i) {
            lateL[i].init(maxDelaySamples);
            lateR[i].init(maxDelaySamples);
        }

        // Default delay time patterns (ms).
        // We deliberately choose non-identical L/R times to decorrelate stereo.
        // These are short and not tuned to musical pitch to avoid ringing.

        std::array<float, kMaxInputStages> baseTimes = {
          1.2f, 2.1f, 3.7f, 5.9f, 8.6f, 12.1f, 16.4f, 20.0f
        };

        // Use seed to introduce tiny offsets (deterministic)
        float seedN = float((seed % 1000u)) / 1000.0f; // 0..0.999
        float offA = 0.08f + 0.10f * seedN;
        float offB = 0.11f + 0.12f * (1.0f - seedN);

        for (int i = 0; i < kMaxInputStages; ++i) {
            inputCfg.timesMsL[i] = baseTimes[i] + (i & 1 ? offA : offB);
            inputCfg.timesMsR[i] = baseTimes[i] + (i & 1 ? offB : offA);
        }

        // Late diffusion times (3 stages)
        lateCfg.timesMsL = { 4.2f + offA, 7.3f + offB, 11.5f + offA };
        lateCfg.timesMsR = { 4.8f + offB, 6.9f + offA, 12.1f + offB };

        // Default coefficients
        inputCfg.stages = 6;
        inputCfg.g = 0.72f;

        lateCfg.minG = 0.45f;
        lateCfg.maxG = 0.72f;

        tvG = inputCfg.g;

        // Apply configs to allpasses
        setInputConfig(inputCfg);
        setLateConfig(lateCfg);

        clear();
        inited = true;
    }

    void Diffusion::clear() {
        for (int i = 0; i < kMaxInputStages; ++i) {
            inL[i].clear();
            inR[i].clear();
        }
        for (int i = 0; i < kLateStages; ++i) {
            lateL[i].clear();
            lateR[i].clear();
        }
    }

    void Diffusion::setInputConfig(const InputConfig& cfg) {
        inputCfg = cfg;

        // NOTE: header helper name is clampInputStages()
        activeInputStages = clampInputStages(inputCfg.stages);

        for (int i = 0; i < kMaxInputStages; ++i) {
            float msL = inputCfg.timesMsL[i];
            float msR = inputCfg.timesMsR[i];

            inL[i].delaySamp = msToSamples(msL, sr);
            inR[i].delaySamp = msToSamples(msR, sr);

            inL[i].g = inputCfg.g;
            inR[i].g = inputCfg.g;
        }

        tvG = inputCfg.g;
    }

    void Diffusion::setLateConfig(const LateConfig& cfg) {
        lateCfg = cfg;

        for (int i = 0; i < kLateStages; ++i) {
            lateL[i].delaySamp = msToSamples(lateCfg.timesMsL[i], sr);
            lateR[i].delaySamp = msToSamples(lateCfg.timesMsR[i], sr);

            // Safe default; actual g can be changed per call in processLate()
            lateL[i].g = lateCfg.maxG;
            lateR[i].g = lateCfg.maxG;
        }
    }

    void Diffusion::processInput(float& L, float& R) {
        if (!inited || activeInputStages <= 0) return;

        // External time-varying g (optional). Clamp to safe range.
        float g = dsp::clampf(tvG, 0.30f, 0.85f);

        float xL = L;
        float xR = R;

        for (int i = 0; i < activeInputStages; ++i) {
            inL[i].g = g;
            inR[i].g = g;

            xL = inL[i].process(xL);
            xR = inR[i].process(xR);
        }

        L = xL;
        R = xR;
    }

    void Diffusion::processLate(float& L, float& R, float amount01) {
        if (!inited) return;

        amount01 = dsp::clampf(amount01, 0.0f, 1.0f);
        if (amount01 <= 0.0001f) return;

        float g = lateCfg.minG + (lateCfg.maxG - lateCfg.minG) * amount01;
        g = dsp::clampf(g, 0.25f, 0.85f);

        float dL = L;
        float dR = R;

        for (int i = 0; i < kLateStages; ++i) {
            lateL[i].g = g;
            lateR[i].g = g;

            dL = lateL[i].process(dL);
            dR = lateR[i].process(dR);
        }

        // Crossfade between dry and diffused
        L = (1.0f - amount01) * L + amount01 * dL;
        R = (1.0f - amount01) * R + amount01 * dR;
    }

} // namespace bigpi::core
