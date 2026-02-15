#include "OutputStage.h"

/*
  =============================================================================
  OutputStage.cpp — Big Pi Final Wet Processing (implementation)
  =============================================================================

  This stage happens AFTER:
    - early reflections
    - late reverb tank output
    - (optional) late diffusion refinement

  The goal is "mix-ready wet":
    - remove rumble (high-pass)
    - shape tone (shelves)
    - control stereo width
    - add gentle saturation (optional)
    - apply final level

  This stage is intentionally not too fancy:
    high-end reverbs often keep output shaping simple but well-tuned.

  Real-time safe:
    - No allocations
    - Small filters and smoothers only
*/

void OutputStage::prepare(float sampleRate) {
    sr = sampleRate;

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

    // Filter coefficients can be updated immediately (safe + small).
    updateFilters();
}

void OutputStage::updateFilters() {
    // High-pass: remove low rumble from wet output.
    // We use a biquad HP for a cleaner cutoff than a one-pole.
    hpL.setHighPass(target.hpHz, 0.707f, sr);
    hpR.setHighPass(target.hpHz, 0.707f, sr);

    // Shelves: gentle broad tone shaping.
    lowL.setLowShelf(target.lowShelfHz, target.lowGainDb, 0.9f, sr);
    lowR.setLowShelf(target.lowShelfHz, target.lowGainDb, 0.9f, sr);

    highL.setHighShelf(target.highShelfHz, target.highGainDb, 0.9f, sr);
    highR.setHighShelf(target.highShelfHz, target.highGainDb, 0.9f, sr);
}

void OutputStage::processBlock(float* wetL, float* wetR, int n) {
    if (!prepared) return;

    for (int i = 0; i < n; ++i) {

        // Smooth parameters
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

        // 3) Stereo width in Mid/Side domain
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

        wetL[i] = L;
        wetR[i] = R;
    }
}
