#include "EarlyReflections.h"

#include <algorithm> // std::max
#include <cmath>     // std::exp

/*
  =============================================================================
  EarlyReflections.cpp — Big Pi Early Reflection Generator (implementation)
  =============================================================================
*/

static inline float msToSamples(float ms, float sr) {
    return ms * 0.001f * sr;
}

void EarlyReflections::prepare(float sampleRate) {
    sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;

    // Early reflections rarely need more than ~80ms buffer.
    // We allocate 100ms for safety.
    int maxSamples = std::max(16, int(sr * 0.10f));

    delayL.init(maxSamples);
    delayR.init(maxSamples);

    // Smoothing times chosen to prevent zipper noise but remain responsive.
    levelSm.setTimeMs(80.0f, sr);
    sizeSm.setTimeMs(120.0f, sr);
    dampSm.setTimeMs(120.0f, sr);
    widthSm.setTimeMs(120.0f, sr);

    reset();
}

void EarlyReflections::reset() {
    delayL.clear();
    delayR.clear();

    dampL.clear();
    dampR.clear();

    // Initialize smoothers to current targets so we don't "glide from zero".
    levelSm.setInstant(target.level);
    sizeSm.setInstant(target.size);
    dampSm.setInstant(target.dampHz);
    widthSm.setInstant(target.width);
}

void EarlyReflections::setParams(const Params& p) {
    target = p;
}

void EarlyReflections::processBlock(const float* inL, const float* inR,
    float* outL, float* outR,
    int n)
{
    for (int i = 0; i < n; ++i) {
        // --------------------------------------------------------------------
        // 1) Smooth parameters
        // --------------------------------------------------------------------
        float level = dsp::clampf(levelSm.process(target.level), 0.0f, 1.0f);
        float size = dsp::clampf(sizeSm.process(target.size), 0.1f, 2.0f);
        float dampHz = dsp::clampf(dampSm.process(target.dampHz), 500.0f, 20000.0f);
        float width = dsp::clampf(widthSm.process(target.width), 0.0f, 2.5f);

        // PERFORMANCE FIX:
        // OnePoleLP::setCutoff() uses exp(), which is expensive.
        // Instead, compute the coefficient once and assign it.
        //
        // OnePoleLP uses: a = exp(-2*pi*hz/sr)
        dampHz = dsp::clampf(dampHz, 5.0f, 0.49f * sr);
        float aLP = std::exp(-2.0f * dsp::kPi * dampHz / sr);
        dampL.a = aLP;
        dampR.a = aLP;

        // --------------------------------------------------------------------
        // 2) Read input and write into delay
        // --------------------------------------------------------------------
        float xL = inL[i];
        float xR = inR[i];

        delayL.push(xL);
        delayR.push(xR);

        // --------------------------------------------------------------------
        // 3) Multi-tap read
        // --------------------------------------------------------------------
        float erL = 0.0f;
        float erR = 0.0f;

        for (int t = 0; t < kNumTaps; ++t) {
            float dSampL = msToSamples(kTapTimesMs[t] * size, sr);

            // Slight decorrelation for R tap times so stereo ER doesn't collapse
            float dSampR = msToSamples(kTapTimesMs[t] * size * 1.10f, sr);

            float tapL = delayL.readFracCubic(dSampL);
            float tapR = delayR.readFracCubic(dSampR);

            erL += tapL * kTapGains[t];
            erR += tapR * kTapGains[t];
        }

        // --------------------------------------------------------------------
        // 4) Damping (low-pass)
        // --------------------------------------------------------------------
        erL = dampL.process(erL);
        erR = dampR.process(erR);

        // --------------------------------------------------------------------
        // 5) Stereo width (Mid/Side)
        // --------------------------------------------------------------------
        float M = 0.5f * (erL + erR);
        float S = 0.5f * (erL - erR);

        S *= width;

        erL = M + S;
        erR = M - S;

        // --------------------------------------------------------------------
        // 6) Apply level
        // --------------------------------------------------------------------
        erL *= level;
        erR *= level;

        outL[i] = erL;
        outR[i] = erR;
    }
}

