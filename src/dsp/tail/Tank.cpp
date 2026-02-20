#include "Tank.h"

/*
  =============================================================================
  Tank.cpp — Big Pi Late Reverb Tank (implementation)
  =============================================================================
*/

#include <algorithm> // std::min, std::max
#include <cmath>     // std::abs, std::exp

namespace bigpi::core {

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

            cloudNoise[i].setSampleRate(sr);
            cloudNoise[i].seed(seed + 0x7F4A7C15u * uint32_t(i + 1));
            cloudNoise[i].setRateHz(0.08f);
            cloudNoise[i].setSmoothMs(500.0f);

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


        // Kappa+Cloud Mod (Level 2): deterministic phase offsets for "spin"
        // We generate a shuffled set of offsets in [0, 2π).
        {
            std::array<int, kMaxLines> idx{};
            for (int i = 0; i < kMaxLines; ++i) idx[i] = i;

            uint32_t x = seed ^ 0xA511E9B3u;
            auto rnd = [&]() -> uint32_t {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                return x;
                };

            for (int i = kMaxLines - 1; i > 0; --i) {
                int j = int(rnd() % uint32_t(i + 1));
                std::swap(idx[i], idx[j]);
            }

            for (int i = 0; i < kMaxLines; ++i) {
                cloudPhaseOffset[idx[i]] = (2.0f * dsp::kPi) * (float(i) / float(kMaxLines));
            }
        }

        cloudPhase = 0.0f;

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
            cloudNoise[i].clear();
            lastY[i] = 0.0f;
        }

        envFollower.clear();
        env01 = 0.0f;

        dynDampHzCurrent = cfg.dampHz;
        cloudPhase = 0.0f;
    }

    void Tank::setConfig(const Config& c) {
        cfg = c;

        cfg.lines = std::max(1, std::min(cfg.lines, kMaxLines));

        cfg.fbHpHz = dsp::clampf(cfg.fbHpHz, 5.0f, 0.49f * sr);
        cfg.dampHz = dsp::clampf(cfg.dampHz, 20.0f, 0.49f * sr);

        cfg.xoverLoHz = dsp::clampf(cfg.xoverLoHz, 30.0f, 0.49f * sr);
        cfg.xoverHiHz = dsp::clampf(cfg.xoverHiHz, cfg.xoverLoHz + 10.0f, 0.49f * sr);

        cfg.drive = dsp::clampf(cfg.drive, 0.0f, 10.0f);
        cfg.satMix = dsp::clampf(cfg.satMix, 0.0f, 1.0f);

        cfg.modRateHz = dsp::clampf(cfg.modRateHz, 0.01f, 20.0f);
        cfg.modDepthSamples = dsp::clampf(cfg.modDepthSamples, 0.0f, 2000.0f);

        cfg.jitterEnable = dsp::clampf(cfg.jitterEnable, 0.0f, 1.0f);
        cfg.jitterAmount = dsp::clampf(cfg.jitterAmount, 0.0f, 2.0f);
        cfg.jitterRateHz = dsp::clampf(cfg.jitterRateHz, 0.01f, 20.0f);
        cfg.jitterSmoothMs = dsp::clampf(cfg.jitterSmoothMs, 1.0f, 2000.0f);


        // Kappa+Cloud Mod (Level 2): Cloudify modulation clamps
        cfg.cloudEnable = dsp::clampf(cfg.cloudEnable, 0.0f, 1.0f);
        cfg.cloudSpinHz = dsp::clampf(cfg.cloudSpinHz, 0.0f, 1.0f); // 1 Hz is already very fast for "spin"
        cfg.cloudWanderAmount = dsp::clampf(cfg.cloudWanderAmount, 0.0f, 2.0f);
        cfg.cloudWanderRateHz = dsp::clampf(cfg.cloudWanderRateHz, 0.0f, 2.0f);
        cfg.cloudWanderSmoothMs = dsp::clampf(cfg.cloudWanderSmoothMs, 1.0f, 5000.0f);


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

            cloudNoise[i].setRateHz(cfg.cloudWanderRateHz);
            cloudNoise[i].setSmoothMs(cfg.cloudWanderSmoothMs);
        }

        envFollower.setAttackReleaseMs(cfg.dynAtkMs, cfg.dynRelMs);

        dynDampHzCurrent = cfg.dampHz;

        lastDecay01 = -1.0f;
    }

    float Tank::computeDynamicDampingHz(float staticDampHz, float env01Now) {
        float e = dsp::clampf(env01Now * cfg.dynSensitivity, 0.0f, 1.0f);

        float dynHz = cfg.dynMaxHz + (cfg.dynMinHz - cfg.dynMaxHz) * e;

        float amt = dsp::clampf(cfg.dynAmount, 0.0f, 1.0f);
        float outHz = (1.0f - amt) * staticDampHz + amt * dynHz;

        return dsp::clampf(outHz, 20.0f, 0.49f * sr);
    }

    void Tank::updateDecayGains(float decay01) {
        if (decay01 == lastDecay01) return;
        lastDecay01 = decay01;

        const float rt60Base = decay01ToRt60Sec(decay01);

        const float rt60Low = rt60Base * std::max(0.10f, cfg.decayLowMul);
        const float rt60Mid = rt60Base * std::max(0.10f, cfg.decayMidMul);
        const float rt60High = rt60Base * std::max(0.10f, cfg.decayHighMul);

        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        for (int i = 0; i < N; ++i) {
            const float delaySec = std::max(1.0f, cfg.delaySamp[i]) / sr;

            fbGainLow[i] = rt60ToFeedbackGain(delaySec, rt60Low);
            fbGainMid[i] = rt60ToFeedbackGain(delaySec, rt60Mid);
            fbGainHigh[i] = rt60ToFeedbackGain(delaySec, rt60High);

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

        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        // Kappa+Cloud Mod (Level 2): advance global "spin" phase once per sample
        if (cfg.cloudEnable > 0.0001f && cfg.cloudSpinHz > 0.0f) {
            cloudPhase += (2.0f * dsp::kPi) * (cfg.cloudSpinHz / sr);
            if (cloudPhase >= 2.0f * dsp::kPi) cloudPhase -= 2.0f * dsp::kPi;
        }


        baseDecay = dsp::clampf(baseDecay, 0.0f, 0.9995f);

        std::array<float, kMaxLines> y{};
        y.fill(0.0f);

        float peakAbs = 0.0f;

        for (int i = 0; i < N; ++i) {
            float depthMul = cfg.modDepthMul[i];
            float rateMul = cfg.modRateMul[i];

            float lfo = 0.0f;
            if (cfg.cloudEnable > 0.0001f) {
                // Cloudify: slow rotating phase field instead of per-line sinus LFO
                lfo = std::sin(cloudPhase + cloudPhaseOffset[i]);
            }
            else {
                lfo = lfoBank.process(i, cfg.modRateHz * rateMul);
            }

            float jit = 0.0f;
            if (cfg.jitterEnable > 0.0001f) {
                jit = jitter[i].process();
            }

            float wander = 0.0f;
            if (cfg.cloudEnable > 0.0001f) {
                wander = cfg.cloudWanderAmount * cloudNoise[i].process();
            }

            float mod = cfg.modDepthSamples
                * (lfo * depthMul + cfg.jitterEnable * cfg.jitterAmount * jit + wander * depthMul);

            float delay = cfg.delaySamp[i] + mod;

            // Avoid reading at ~0 delay (read head ≈ write head).
            delay = std::max(1.0f, delay);

            float yi = d[i].readFracCubic(delay);

            y[i] = yi;
            yOut[i] = yi;

            lastY[i] = yi;

            peakAbs = std::max(peakAbs, std::abs(yi));
        }

        float e = envFollower.process(peakAbs);
        env01 = dsp::clampf(e * 2.0f, 0.0f, 1.0f);

        mix(y, N, cfg.matrix);

        float dampHzEffective = dsp::clampf(cfg.dampHz, 20.0f, 0.49f * sr);
        if (cfg.dynEnable > 0.0001f) {
            dampHzEffective = computeDynamicDampingHz(cfg.dampHz, env01);
        }

        dynDampHzCurrent = 0.995f * dynDampHzCurrent + 0.005f * dampHzEffective;

        float aLP = std::exp(-2.0f * dsp::kPi * dynDampHzCurrent / sr);
        for (int i = 0; i < N; ++i) {
            lp[i].a = aLP;
        }

        updateDecayGains(baseDecay);

        const float injPerLine = inj / float(N);
        for (int i = 0; i < N; ++i) {
            float fb = y[i];

            fb = hp[i].process(fb);
            fb = lp[i].process(fb);

            float low = xLo[i].process(fb);
            float lowMid = xHi[i].process(fb);

            float mid = lowMid - low;
            float high = fb - lowMid;

            float fbColored =
                low * fbGainLow[i] +
                mid * fbGainMid[i] +
                high * fbGainHigh[i];

            float sat = dsp::softSat(fbColored, cfg.drive);
            float fbFinal = (1.0f - cfg.satMix) * fbColored + cfg.satMix * sat;

            d[i].push(injPerLine + fbFinal);
        }
    }


    /*
      Kappa+Cloud Mod (Step 1): per-line injection entrypoint.
      ReverbEngine uses this to inject a decorrelated MS vector.
    */
    void Tank::processSampleVec(const std::array<float, kMaxLines>& injVec,
        float baseDecay,
        dsp::MultiLFO& lfoBank,
        std::array<float, kMaxLines>& yOut)
    {
        if (!inited) {
            yOut.fill(0.0f);
            return;
        }

        const int N = std::max(1, std::min(cfg.lines, kMaxLines));

        // Kappa+Cloud Mod (Level 2): advance global "spin" phase once per sample
        if (cfg.cloudEnable > 0.0001f && cfg.cloudSpinHz > 0.0f) {
            cloudPhase += (2.0f * dsp::kPi) * (cfg.cloudSpinHz / sr);
            if (cloudPhase >= 2.0f * dsp::kPi) cloudPhase -= 2.0f * dsp::kPi;
        }


        baseDecay = dsp::clampf(baseDecay, 0.0f, 0.9995f);

        std::array<float, kMaxLines> y{};
        y.fill(0.0f);

        float peakAbs = 0.0f;

        for (int i = 0; i < N; ++i) {
            float depthMul = cfg.modDepthMul[i];
            float rateMul = cfg.modRateMul[i];

            float lfo = 0.0f;
            if (cfg.cloudEnable > 0.0001f) {
                // Cloudify: slow rotating phase field instead of per-line sinus LFO
                lfo = std::sin(cloudPhase + cloudPhaseOffset[i]);
            }
            else {
                lfo = lfoBank.process(i, cfg.modRateHz * rateMul);
            }

            float jit = 0.0f;
            if (cfg.jitterEnable > 0.0001f) {
                jit = jitter[i].process();
            }

            float wander = 0.0f;
            if (cfg.cloudEnable > 0.0001f) {
                wander = cfg.cloudWanderAmount * cloudNoise[i].process();
            }

            float mod = cfg.modDepthSamples
                * (lfo * depthMul + cfg.jitterEnable * cfg.jitterAmount * jit + wander * depthMul);

            float delay = cfg.delaySamp[i] + mod;

            // Avoid reading at ~0 delay (read head ≈ write head).
            delay = std::max(1.0f, delay);

            float yi = d[i].readFracCubic(delay);

            y[i] = yi;
            yOut[i] = yi;

            lastY[i] = yi;

            peakAbs = std::max(peakAbs, std::abs(yi));
        }

        float e = envFollower.process(peakAbs);
        env01 = dsp::clampf(e * 2.0f, 0.0f, 1.0f);

        mix(y, N, cfg.matrix);

        float dampHzEffective = dsp::clampf(cfg.dampHz, 20.0f, 0.49f * sr);
        if (cfg.dynEnable > 0.0001f) {
            dampHzEffective = computeDynamicDampingHz(cfg.dampHz, env01);
        }

        dynDampHzCurrent = 0.995f * dynDampHzCurrent + 0.005f * dampHzEffective;

        float aLP = std::exp(-2.0f * dsp::kPi * dynDampHzCurrent / sr);
        for (int i = 0; i < N; ++i) {
            lp[i].a = aLP;
        }

        updateDecayGains(baseDecay);

        for (int i = 0; i < N; ++i) {
            float fb = y[i];

            fb = hp[i].process(fb);
            fb = lp[i].process(fb);

            float low = xLo[i].process(fb);
            float lowMid = xHi[i].process(fb);

            float mid = lowMid - low;
            float high = fb - lowMid;

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
