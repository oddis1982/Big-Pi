#include "OutputStage.h"

#include <algorithm> // std::max, std::min
#include <cmath>     // std::abs

/*
  =============================================================================
  OutputStage.cpp — Big Pi Final Wet Processing (implementation)
  =============================================================================
*/

static inline float killDenorm(float x) {
    // Prevent denormals (tiny subnormal floats) that can cause CPU spikes
    return (std::abs(x) < 1e-20f) ? 0.0f : x;
}

void OutputStage::prepare(float sampleRate) {
    sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;

    // Smoothers to prevent clicks when UI changes.
    widthSm.setTimeMs(80.0f, sr);
    driveSm.setTimeMs(120.0f, sr);
    levelSm.setTimeMs(120.0f, sr);

    widthSm.setInstant(target.width);
    driveSm.setInstant(target.drive);
    levelSm.setInstant(target.level);

    updateFilters();
    reset();

    prepared = true;
}

void OutputStage::reset() {
    hpL.clear(); hpR.clear();
    lowL.clear(); lowR.clear();
    highL.clear(); highR.clear();

    widthSm.setInstant(target.width);
    driveSm.setInstant(target.drive);
    levelSm.setInstant(target.level);
}

void OutputStage::setParams(const Params& p) {
    target = p;
    updateFilters();
}

void OutputStage::updateFilters() {
    // Clamp params to safe ranges.
    float hpHz = std::max(5.0f, std::min(target.hpHz, 0.49f * sr));

    float lowHz = std::max(5.0f, std::min(target.lowShelfHz, 0.49f * sr));
    float highHz = std::max(5.0f, std::min(target.highShelfHz, 0.49f * sr));

    float lowDb = std::max(-24.0f, std::min(target.lowGainDb, 24.0f));
    float highDb = std::max(-24.0f, std::min(target.highGainDb, 24.0f));

    // High-pass
    hpL.setHighPass(hpHz, 0.707f, sr);
    hpR.setHighPass(hpHz, 0.707f, sr);

    // Shelves
    lowL.setLowShelf(lowHz, lowDb, 0.9f, sr);
    lowR.setLowShelf(lowHz, lowDb, 0.9f, sr);

    highL.setHighShelf(highHz, highDb, 0.9f, sr);
    highR.setHighShelf(highHz, highDb, 0.9f, sr);
}

void OutputStage::processBlock(float* wetL, float* wetR, int n) {
    if (!prepared) return;

    for (int i = 0; i < n; ++i) {
        // Smooth parameters (per-sample smoothing prevents zipper noise)
        float width = dsp::clampf(widthSm.process(target.width), 0.0f, 2.5f);
        float drive = dsp::clampf(driveSm.process(target.drive), 0.0f, 6.0f);
        float level = dsp::clampf(levelSm.process(target.level), 0.0f, 2.0f);

        float L = wetL[i];
        float R = wetR[i];

        // 1) High-pass
        L = hpL.process(L);
        R = hpR.process(R);

        // 2) Low shelf + High shelf
        L = lowL.process(L);
        R = lowR.process(R);

        L = highL.process(L);
        R = highR.process(R);

        // 3) Stereo width (Mid/Side)
        float M = 0.5f * (L + R);
        float S = 0.5f * (L - R);

        S *= width;

        L = M + S;
        R = M - S;

        // 4) Soft saturation (optional)
        if (drive > 0.0001f) {
            L = dsp::softSat(L, drive);
            R = dsp::softSat(R, drive);
        }

        // 5) Final level
        L *= level;
        R *= level;

        // Denormal guard at the end (important for long tails)
        wetL[i] = killDenorm(L);
        wetR[i] = killDenorm(R);
    }
}

