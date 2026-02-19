﻿﻿#include "Tank.h"

/*
  =============================================================================
  Tank.cpp — Big Pi Late Reverb Tank (implementation)
  =============================================================================
*/

#include <algorithm> // std::min, std::max
#include <cmath>     // std::abs, std::exp

namespace bigpi::core {

    // ----------------------------------------------------------------------
    // Kappa upgrade: RT60-based decay model
    // ----------------------------------------------------------------------
    //
    // Instead of treating the decay knob as a direct feedback gain, we treat it
    // as a target RT60 (seconds). For each delay line, we compute the feedback
    // gain that yields -60 dB after RT60 seconds:
    //
    //   gain = 0.001 ^ (delaySeconds / rt60Seconds)
    //
    // This is a classic, stable approach used in many high-quality reverbs.
    // It prevents runaway feedback even when low/mid/high multipliers are > 1,
    // because those multipliers scale *time* (RT60), not *gain*.
    //
    // Mapping of decay01 -> RT60:
    //   0.0 -> ~0.2 s (very short)
    //   1.0 -> ~12  s (long)
    //
    // (You can later re-tune these endpoints to taste.)
    static inline float decay01ToRt60Sec(float decay01) {
        decay01 = dsp::clampf(decay01, 0.0f, 1.0f);
        const float rt60Min = 0.2f;
        const float rt60Max = 12.0f;
        const float ratio = rt60Max / rt60Min;
        return rt60Min * std::pow(ratio, decay01);
    }

    static inline float rt60ToFeedbackGain(float delaySec, float rt60Sec) {
        rt60Sec = std::max(0.01f, rt60Sec);
        delaySec = std::max(0.0f, delaySec);
        return std::exp(std::log(0.001f) * (delaySec / rt60Sec));
    }

    void Tank::init(float sampleRate, int maxDelaySamples, uint32_t s) {
        sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;
        seed = (s == 0 ? 1u : s);

        maxDelaySamples = std::max(8, maxDelaySamples);

        for (int i = 0; i < kMaxLines; ++i) {
            d[i].init(maxDelaySamples);

            jitter[i].setSampleRate(sr);
            jitter[i].seed(seed + 0x9E3779B9u * uint32_t(i + 1));
            jitter[i].setRateHz(0.35f);
            jitter[i].setSmoothMs(80.0f);

            hp[i].clear();
            lp[i].clear();
            xLo[i].clear();
            xHi[i].clear();

            lastY[i] = 0.0f;
        }

        envFollower.setSampleRate(sr);
        envFollower.setAttackReleaseMs(12.0f, 280.0f);
        envFollower.clear();
        env01 = 0.0f;

        inited = true;
        clear();
    }

    void Tank::clear() {
        for (int i = 0; i < kMaxLines; ++i) {
            d[i].clear();

            hp[i].clear();
            lp[i].clear();
            xLo[i].clear();
            xHi[i].clear();

            jitter[i].clear();
            lastY[i] = 0.0f;
        }

        envFollower.clear();
        env01 = 0.0f;

        dynDampHzCurrent = cfg.dampHz;
    }

    void Tank::setConfig(const Config& c) {
        cfg = c;

        cfg.lines = std::max(1, std::min(cfg.lines, kMaxLines));

        // Keep config values in sane ranges (prevents surprising behavior)
        cfg.fbHpHz = dsp::clampf(cfg.fbHpHz, 5.0f, 0.49f * sr);
        cfg.dampHz = dsp::clampf(cfg.dampHz, 20.0f, 0.49f * sr);

        cfg.xoverLoHz = dsp::clampf(cfg.xoverLoHz, 30.0f, 0.49f * sr);
        cfg.xoverHiHz = dsp::clampf(cfg.xoverHiHz, cfg.xoverLoHz + 10.0f, 0.49f * sr);

        cfg.drive = dsp::clampf(cfg.drive, 0.0f, 10.0f);
        cfg.satMix = dsp::clampf(cfg.satMix, 0.0f, 1.0f);

        cfg.modRateHz = dsp::clampf(cfg.modRateHz, 0.01f, 20.0f);
        cfg.modDepthSamples = dsp::clampf(cfg.modDepthSamples, 0.0f, 2000.0f); // safety cap

        cfg.jitterEnable = dsp::clampf(cfg.jitterEnable, 0.0f, 1.0f);
        cfg.jitterAmount = dsp::clampf(cfg.jitterAmount, 0.0f, 2.0f);
        cfg.jitterRateHz = dsp::clampf(cfg.jitterRateHz, 0.01f, 20.0f);
        cfg.jitterSmoothMs = dsp::clampf(cfg.jitterSmoothMs, 1.0f, 2000.0f);

        cfg.dynEnable = dsp::clampf(cfg.dynEnable, 0.0f, 1.0f);
        cfg.dynAmount = dsp::clampf(cfg.dynAmount, 0.0f, 1.0f);
        cfg.dynSensitivity = dsp::clampf(cfg.dynSensitivity, 0.0f, 10.0f);

        cfg.dynMinHz = dsp::clampf(cfg.dynMinHz, 50.0f, 0.49f * sr);
        cfg.dynMaxHz = dsp::clampf(cfg.dynMaxHz, cfg.dynMinHz, 0.49f * sr);
        cfg.dynAtkMs = dsp::clampf(cfg.dynAtkMs, 0.1f, 2000.0f);
        cfg.dynRelMs = dsp::clampf(cfg.dynRelMs, 0.1f, 5000.0f);

        // Update filters once per config change (cheap, safe)
        for (int i = 0; i < cfg.lines; ++i) {
            hp[i].setCutoff(cfg.fbHpHz, sr);
            lp[i].setCutoff(cfg.dampHz, sr);

            xLo[i].setCutoff(cfg.xoverLoHz, sr);
            xHi[i].setCutoff(cfg.xoverHiHz, sr);

            jitter[i].setRateHz(cfg.jitterRateHz);
            jitter[i].setSmoothMs(cfg.jitterSmoothMs);
        }

        envFollower.setAttackReleaseMs(cfg.dynAtkMs, cfg.dynRelMs);

        // Keep smoother state inside sane bounds
        dynDampHzCurrent = cfg.dampHz;

        // Decay model cache invalidation (forces RT60 gain recompute)
        lastDecay01 = -1.0f;
    }

    float Tank::computeDynamicDampingHz(float staticDampHz, float env01Now) {
        float e = dsp::clampf(env01Now * cfg.dynSensitivity, 0.0f, 1.0f);

        // loud -> dynMinHz (darker), quiet -> dynMaxHz (brighter)
        float dynHz = cfg.dynMaxHz + (cfg.dynMinHz - cfg.dynMaxHz) * e;

        float amt = dsp::clampf(cfg.dynAmount, 0.0f, 1.0f);
        float outHz = (1.0f - amt) * staticDampHz + amt * dynHz;

        return dsp::clampf(outHz, 20.0f, 0.49f * sr);
    }

    void Tank::updateDecayGains(float decay01) {
        // Only recompute when the decay control changes.
        // (This is called per-sample, so caching matters.)
        if (decay01 == lastDecay01) return;
        lastDecay01 = decay01;

        // 1) Map knob to base RT60 in seconds.
        const float rt60Base = decay01ToRt60Sec(decay01);

        // 2) Interpret the band's "multipliers" as RT60 time scalers.
        //    Example: decayLowMul = 1.12 means "low band decays 12% longer".
        const float rt60Low = rt60Base * std::max(0.10f, cfg.decayLowMul);
        const float rt60Mid = rt60Base * std::max(0.10f, cfg.decayMidMul);
        const float rt60High = rt60Base * std::max(0.10f, cfg.decayHighMul);

        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        for (int i = 0; i < N; ++i) {
            // Use the nominal delay length (without modulation) to derive gain.
            // This keeps behavior stable and avoids recalculating per-line gains
            // every time the LFO changes the fractional delay.
            const float delaySec = std::max(1.0f, cfg.delaySamp[i]) / sr;

            fbGainLow[i] = rt60ToFeedbackGain(delaySec, rt60Low);
            fbGainMid[i] = rt60ToFeedbackGain(delaySec, rt60Mid);
            fbGainHigh[i] = rt60ToFeedbackGain(delaySec, rt60High);

            // Extra safety clamp: never allow unity gain.
            fbGainLow[i] = dsp::clampf(fbGainLow[i], 0.0f, 0.9997f);
            fbGainMid[i] = dsp::clampf(fbGainMid[i], 0.0f, 0.9997f);
            fbGainHigh[i] = dsp::clampf(fbGainHigh[i], 0.0f, 0.9997f);
        }
    }

    void Tank::processSample(float inj,
        float baseDecay,
        dsp::MultiLFO& lfoBank,
        std::array<float, kMaxLines>& yOut)
    {
        if (!inited) {
            yOut.fill(0.0f);
            return;
        }

        // Backwards-compatible path: uniform injection across lines.
        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        std::array<float, kMaxLines> injVec{};
        injVec.fill(0.0f);

        const float injPerLine = inj / float(N);
        for (int i = 0; i < N; ++i) injVec[i] = injPerLine;

        processSampleVec(injVec, baseDecay, lfoBank, yOut);
    }

    void Tank::processSampleVec(const std::array<float, kMaxLines>& injVec,
        float baseDecay,
        dsp::MultiLFO& lfoBank,
        std::array<float, kMaxLines>& yOut)
    {
        if (!inited) {
            yOut.fill(0.0f);
            return;
        }

        // Extra safety: cfg.lines should be clamped in setConfig(),
        // but guard here because this is the hottest function.
        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        baseDecay = dsp::clampf(baseDecay, 0.0f, 0.9995f);

        // ----------------------------------------------------------------------
        // 1) Read delay line outputs with fractional delay modulation
        // ----------------------------------------------------------------------
        std::array<float, kMaxLines> y{};
        y.fill(0.0f);

        float peakAbs = 0.0f;

        for (int i = 0; i < N; ++i) {
            float depthMul = cfg.modDepthMul[i];
            float rateMul = cfg.modRateMul[i];

            float lfo = lfoBank.process(i, cfg.modRateHz * rateMul);

            float jit = 0.0f;
            if (cfg.jitterEnable > 0.0001f) {
                jit = jitter[i].process();
            }

            float mod = cfg.modDepthSamples
                * (lfo * depthMul + cfg.jitterEnable * cfg.jitterAmount * jit);

            float delay = cfg.delaySamp[i] + mod;

            // Avoid reading at ~0 delay (read head ≈ write head).
            delay = std::max(1.0f, delay);

            float yi = d[i].readFracCubic(delay);

            y[i] = yi;
            yOut[i] = yi;

            // Currently unused, but kept for future debugging/visualization
            lastY[i] = yi;

            peakAbs = std::max(peakAbs, std::abs(yi));
        }

        // Envelope follower (tail energy proxy)
        float e = envFollower.process(peakAbs);
        env01 = dsp::clampf(e * 2.0f, 0.0f, 1.0f);

        // ----------------------------------------------------------------------
        // 2) Mix through matrix (density)
        // ----------------------------------------------------------------------
        mix(y, N, cfg.matrix);

        // ----------------------------------------------------------------------
        // 3) Dynamic damping (compute ONCE per sample, apply cheaply)
        // ----------------------------------------------------------------------
        float dampHzEffective = dsp::clampf(cfg.dampHz, 20.0f, 0.49f * sr);
        if (cfg.dynEnable > 0.0001f) {
            dampHzEffective = computeDynamicDampingHz(cfg.dampHz, env01);
        }

        // Smooth damping to avoid zippering
        dynDampHzCurrent = 0.995f * dynDampHzCurrent + 0.005f * dampHzEffective;

        // Compute 1-pole coefficient ONCE and apply to each line filter.
        float aLP = std::exp(-2.0f * dsp::kPi * dynDampHzCurrent / sr);
        for (int i = 0; i < N; ++i) {
            lp[i].a = aLP;
        }

        // ----------------------------------------------------------------------
        // 4) Feedback writeback: HP -> LP -> multiband RT60 decay -> optional saturation
        // ----------------------------------------------------------------------
        //
        // Kappa upgrade: compute stable per-line feedback gains from RT60.
        // This prevents the "slow build then infinite sustain" problem that
        // happens when baseDecay * bandMul approaches 1.0.
        updateDecayGains(baseDecay);

        for (int i = 0; i < N; ++i) {
            float fb = y[i];

            fb = hp[i].process(fb);
            fb = lp[i].process(fb);

            float low = xLo[i].process(fb);
            float lowMid = xHi[i].process(fb);

            float mid = lowMid - low;
            float high = fb - lowMid;

            // Apply per-band gains derived from RT60 (time), not direct gain multipliers.
            float fbColored =
                low * fbGainLow[i] +
                mid * fbGainMid[i] +
                high * fbGainHigh[i];

            float sat = dsp::softSat(fbColored, cfg.drive);
            float fbFinal = (1.0f - cfg.satMix) * fbColored + cfg.satMix * sat;

            d[i].push(injVec[i] + fbFinal);
        }
    }

} // namespace bigpi::core
