#include "dsp/engines/tune_hall/ReverbEngine.h"

#include <algorithm> // std::min, std::max
#include <array>     // std::array
#include <cmath>     // std::abs
#include <cstdint>   // uint32_t

static inline float msToSamples(float ms, float sr) {
    return ms * 0.001f * sr;
}

// -----------------------------------------------------------------------------
// Kappa+Cloud Mod (Step 1): build MS injection vectors for current line count
// -----------------------------------------------------------------------------
void ReverbEngine::rebuildStereoVectors(int lines) {
    lines = std::max(1, std::min(lines, bigpi::core::Tank::kMaxLines));
    if (lines == lastStereoVecN) return;
    lastStereoVecN = lines;

    injVec.fill(0.0f);
    vM.fill(0.0f);
    vS.fill(0.0f);

    const float invN = 1.0f / float(lines);

    // Mid vector: uniform (sum = 1.0)
    for (int i = 0; i < lines; ++i) vM[i] = invN;

    // Deterministic shuffle so each mode has stable decorrelation
    std::array<int, bigpi::core::Tank::kMaxLines> idx{};
    for (int i = 0; i < lines; ++i) idx[i] = i;

    uint32_t x = 0xC10UD00Du ^ (uint32_t(int(target.mode)) * 0x9E3779B9u);
    auto rnd = [&]() -> uint32_t {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return x;
        };

    for (int i = lines - 1; i > 0; --i) {
        int j = int(rnd() % uint32_t(i + 1));
        std::swap(idx[i], idx[j]);
    }

    // Side vector: balanced +/- (sum ≈ 0). For odd N, one extra negative.
    const int posCount = lines / 2;
    for (int k = 0; k < lines; ++k) {
        const int i = idx[k];
        const float sgn = (k < posCount) ? 1.0f : -1.0f;
        vS[i] = sgn * invN;
    }
}

void ReverbEngine::prepare(float sampleRate, int blockSize) {
    sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;
    block = std::max(1, blockSize);

    // Pre-allocate block buffers (real-time safe)
    wetL.assign(block, 0.0f);
    wetR.assign(block, 0.0f);
    erL.assign(block, 0.0f);
    erR.assign(block, 0.0f);

    er.prepare(sr);
    outStage.prepare(sr);

    // Predelay buffer also doubles as the source for Cloud front-end multitap spray.
    const int preMax = std::max(16, int(sr * 0.20f)); // 200 ms
    preL.init(preMax);
    preR.init(preMax);

    // Step 5: post-tank smear buffer (micro-delay taps)
    const int smearMax = std::max(16, int(sr * 0.060f)); // 60 ms
    smearL.init(smearMax);
    smearR.init(smearMax);

    diffusion.init(sr, 0xB16B00B5u);

    lfos.init(16, sr);

    duckEnv.setSampleRate(sr);
    duckEnv.setAttackReleaseMs(8.0f, 120.0f);
    duckEnv.clear();

    const int maxTankDelay = std::max(64, int(sr * 2.5f));
    tank.init(sr, maxTankDelay, 0xC0FFEEu);

    // Apply preset defaults into target + tank config
    applyModePreset(target.mode);

    prepared = true;
    reset();
}

void ReverbEngine::reset() {
    if (!prepared) return;

    preL.clear();
    preR.clear();

    smearL.clear();
    smearR.clear();

    er.reset();
    diffusion.clear();
    tank.clear();
    outStage.reset();

    duckEnv.clear();
}

void ReverbEngine::setParams(const Params& p) {
    const bool modeChanged = (p.mode != target.mode);
    target = p;

    if (modeChanged) {
        applyModePreset(target.mode);
    }

    // Early reflections
    EarlyReflections::Params erp;
    erp.level = target.erLevel;
    erp.size = target.erSize;
    erp.dampHz = target.erDampHz;
    erp.width = target.erWidth;
    er.setParams(erp);

    // Output stage
    OutputStage::Params op;
    op.hpHz = target.outHpHz;
    op.lowShelfHz = target.outLowShelfHz;
    op.lowGainDb = target.outLowGainDb;
    op.highShelfHz = target.outHighShelfHz;
    op.highGainDb = target.outHighGainDb;
    op.width = target.outWidth;
    op.drive = target.outDrive;
    op.level = target.outLevel;
    outStage.setParams(op);

    // Diffusion
    bigpi::core::Diffusion::InputConfig inCfg = {};
    inCfg.stages = target.inputDiffStages;
    inCfg.g = target.inputDiffG;
    diffusion.setInputConfig(inCfg);

    bigpi::core::Diffusion::LateConfig lateCfg = {};
    lateCfg.minG = target.lateDiffMinG;
    lateCfg.maxG = target.lateDiffMaxG;
    diffusion.setLateConfig(lateCfg);

    // Tank
    bigpi::core::Tank::Config tc = tank.getConfig();

    tc.fbHpHz = target.feedbackHpHz;
    tc.dampHz = target.dampingHz;

    tc.xoverLoHz = target.fbXoverLoHz;
    tc.xoverHiHz = target.fbXoverHiHz;

    tc.decayLowMul = target.decayLowMul;
    tc.decayMidMul = target.decayMidMul;
    tc.decayHighMul = target.decayHighMul;

    tc.modDepthSamples = msToSamples(target.modDepthMs, sr);
    tc.modRateHz = target.modRateHz;

    tc.jitterEnable = target.modJitterEnable;
    tc.jitterAmount = target.modJitterAmount;
    tc.jitterRateHz = target.modJitterRateHz;
    tc.jitterSmoothMs = target.modJitterSmoothMs;

    // Cloudify modulation controls
    tc.cloudEnable = target.cloudEnable;
    tc.cloudSpinHz = target.cloudSpinHz;
    tc.cloudWanderAmount = target.cloudWanderAmount;
    tc.cloudWanderRateHz = target.cloudWanderRateHz;
    tc.cloudWanderSmoothMs = target.cloudWanderSmoothMs;

    tank.setConfig(tc);
}

void ReverbEngine::applyModePreset(bigpi::Mode m) {
    modeCfg = bigpi::getModePreset(m);

    bigpi::core::Tank::Config tc = tank.getConfig();
    tc.lines = modeCfg.tank.delayLines;

    // ensure vectors match tank line count
    rebuildStereoVectors(tc.lines);

    tc.matrix = modeCfg.tank.useHouseholder
        ? bigpi::core::MatrixType::Householder
        : bigpi::core::MatrixType::Hadamard;

    // Base delay sets (ms)
    static const float baseMs16_Default[16] = {
      29.7f, 37.1f, 41.1f, 43.7f,
      53.9f, 59.5f, 61.7f, 71.3f,
      79.9f, 89.7f, 97.3f, 101.9f,
      107.9f, 115.1f, 123.7f, 131.9f
    };

    static const float baseMs16_Hall[16] = {
      31.7f, 37.9f, 41.3f, 43.1f,
      53.3f, 59.9f, 61.1f, 71.7f,
      79.3f, 89.1f, 97.9f, 103.3f,
      109.7f, 117.1f, 125.9f, 137.3f
    };

    // Step 4: Cloud delay-line set (designed to be more "de-correlated" and less clustered)
    // - avoids near-harmonic groupings
    // - spreads energy more evenly across a wider window
    // - still sits in a plausible late-reverb range
    static const float baseMs16_Cloud[16] = {
      27.9f, 33.6f, 39.2f, 44.9f,
      51.7f, 57.4f, 63.8f, 70.9f,
      78.1f, 86.6f, 95.8f, 104.7f,
      114.3f, 124.9f, 136.8f, 149.7f
    };

    const float* baseSet = baseMs16_Default;
    if (m == bigpi::Mode::Hall) baseSet = baseMs16_Hall;

    // If Sky and enabled, use Cloud delay set.
    if (m == bigpi::Mode::Sky && target.cloudDelaySetEnable > 0.0001f) {
        baseSet = baseMs16_Cloud;
    }

    for (int i = 0; i < 16; ++i) {
        const float ms = baseSet[i] * modeCfg.tank.delayScale;
        tc.delaySamp[i] = msToSamples(ms, sr);
    }

    tc.modDepthMul = modeCfg.tank.modDepthMul;
    tc.modRateMul = modeCfg.tank.modRateMul;

    // Preset-driven parameters (stored in target)
    target.inputDiffStages = modeCfg.tank.inputDiffStages;
    target.inputDiffG = modeCfg.tank.inputDiffG;

    target.lateDiffMinG = modeCfg.tank.lateDiffMinG;
    target.lateDiffMaxG = modeCfg.tank.lateDiffMaxG;

    target.modDepthMs = modeCfg.tank.modDepthMs;
    target.modRateHz = modeCfg.tank.modRateHz;

    target.decayLowMul = modeCfg.tank.decayLowMul;
    target.decayMidMul = modeCfg.tank.decayMidMul;
    target.decayHighMul = modeCfg.tank.decayHighMul;

    // Mode suggested defaults
    target.mix = modeCfg.defaultMix;
    target.decay = modeCfg.defaultDecay;
    target.dampingHz = modeCfg.defaultDamping;
    target.predelayMs = modeCfg.defaultPreDelay;

    target.erLevel = modeCfg.defaultERLevel;
    target.erSize = modeCfg.defaultERSize;

    // Cloudify defaults (Sky)
    if (m == bigpi::Mode::Sky) {
        target.cloudEnable = 1.0f;
        target.cloudSpinHz = 0.045f;
        target.cloudWanderAmount = 0.55f;
        target.cloudWanderRateHz = 0.08f;
        target.cloudWanderSmoothMs = 500.0f;
    }
    else {
        target.cloudEnable = 0.0f;
    }

    // Step 3 defaults (Sky)
    if (m == bigpi::Mode::Sky) {
        target.cloudFrontEnable = 1.0f;
        target.cloudFrontAmount = 0.48f;
        target.cloudFrontSizeMs = 24.0f;
        target.cloudFrontWidth = 0.75f;
    }
    else {
        target.cloudFrontEnable = 0.0f;
    }

    // Step 4–5 defaults (Sky)
    if (m == bigpi::Mode::Sky) {
        target.cloudDelaySetEnable = 1.0f;

        target.cloudSmearEnable = 1.0f;
        target.cloudSmearAmount = 0.34f;
        target.cloudSmearTimeMs = 14.0f;
        target.cloudSmearWidth = 0.80f;
    }
    else {
        target.cloudSmearEnable = 0.0f;
    }

    // Push configs now
    tank.setConfig(tc);

    EarlyReflections::Params erp;
    erp.level = target.erLevel;
    erp.size = target.erSize;
    erp.dampHz = target.erDampHz;
    erp.width = target.erWidth;
    er.setParams(erp);

    OutputStage::Params op;
    op.hpHz = target.outHpHz;
    op.lowShelfHz = target.outLowShelfHz;
    op.lowGainDb = target.outLowGainDb;
    op.highShelfHz = target.outHighShelfHz;
    op.highGainDb = target.outHighGainDb;
    op.width = target.outWidth;
    op.drive = target.outDrive;
    op.level = target.outLevel;
    outStage.setParams(op);

    bigpi::core::Diffusion::InputConfig inCfg = {};
    inCfg.stages = target.inputDiffStages;
    inCfg.g = target.inputDiffG;
    diffusion.setInputConfig(inCfg);

    bigpi::core::Diffusion::LateConfig lateCfg = {};
    lateCfg.minG = target.lateDiffMinG;
    lateCfg.maxG = target.lateDiffMaxG;
    diffusion.setLateConfig(lateCfg);

    // Ensure tank config includes non-preset-owned fields too
    bigpi::core::Tank::Config tc2 = tank.getConfig();
    tc2.xoverLoHz = target.fbXoverLoHz;
    tc2.xoverHiHz = target.fbXoverHiHz;

    tc2.decayLowMul = target.decayLowMul;
    tc2.decayMidMul = target.decayMidMul;
    tc2.decayHighMul = target.decayHighMul;

    tc2.modDepthSamples = msToSamples(target.modDepthMs, sr);
    tc2.modRateHz = target.modRateHz;

    tc2.jitterEnable = target.modJitterEnable;
    tc2.jitterAmount = target.modJitterAmount;
    tc2.jitterRateHz = target.modJitterRateHz;
    tc2.jitterSmoothMs = target.modJitterSmoothMs;

    tc2.cloudEnable = target.cloudEnable;
    tc2.cloudSpinHz = target.cloudSpinHz;
    tc2.cloudWanderAmount = target.cloudWanderAmount;
    tc2.cloudWanderRateHz = target.cloudWanderRateHz;
    tc2.cloudWanderSmoothMs = target.cloudWanderSmoothMs;

    tc2.fbHpHz = target.feedbackHpHz;
    tc2.dampHz = target.dampingHz;

    tank.setConfig(tc2);
}

float ReverbEngine::computeEffectiveDecay(float decay, float freeze01) const {
    freeze01 = dsp::clampf(freeze01, 0.0f, 1.0f);
    decay = dsp::clampf(decay, 0.0f, 0.9995f);
    const float frozen = 0.9993f;
    return (1.0f - freeze01) * decay + freeze01 * frozen;
}

float ReverbEngine::computeLoudnessCompDb(float decay01) const {
    decay01 = dsp::clampf(decay01, 0.0f, 1.0f);

    const float strength = dsp::clampf(target.loudCompStrength, 0.0f, 1.0f);
    const float maxDb = dsp::clampf(target.loudCompMaxDb, 0.0f, 24.0f);

    return -maxDb * strength * decay01;
}

void ReverbEngine::processBlock(const float* inL, const float* inR,
    float* outL, float* outR,
    int n)
{
    if (!prepared) return;
    if (n <= 0) return;

    // Step 3: Cloud front-end multitap pattern (fixed, RT-safe)
    static constexpr int kCloudTaps = 8;
    static constexpr float kTapPos[kCloudTaps] = {
        0.06f, 0.12f, 0.20f, 0.31f, 0.45f, 0.62f, 0.80f, 1.00f
    };
    static constexpr float kTapGain[kCloudTaps] = {
        0.90f, 0.78f, 0.66f, 0.56f, 0.48f, 0.40f, 0.34f, 0.28f
    };
    static constexpr float kTapSign[kCloudTaps] = {
        +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f
    };

    // Step 5: post-tank smear taps (fixed pattern)
    static constexpr int kSmearTaps = 6;
    static constexpr float kSmearPos[kSmearTaps] = { 0.15f, 0.28f, 0.42f, 0.58f, 0.76f, 1.00f };
    static constexpr float kSmearGain[kSmearTaps] = { 0.88f, 0.70f, 0.56f, 0.45f, 0.36f, 0.30f };
    static constexpr float kSmearSign[kSmearTaps] = { +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f };

    int pos = 0;
    while (pos < n) {
        const int chunk = std::min(block, n - pos);

        const float preSamp = msToSamples(dsp::clampf(target.predelayMs, 0.0f, 200.0f), sr);

        // Predelay stage (also fills the buffer used by cloud multitaps)
        for (int i = 0; i < chunk; ++i) {
            preL.push(inL[pos + i]);
            preR.push(inR[pos + i]);
            wetL[i] = preL.readFracCubic(preSamp);
            wetR[i] = preR.readFracCubic(preSamp);
        }

        // Early reflections
        er.processBlock(wetL.data(), wetR.data(), erL.data(), erR.data(), chunk);

        std::array<float, bigpi::core::Tank::kMaxLines> yVec{};

        const float effDecay = computeEffectiveDecay(target.decay, target.freeze);

        float loudDb = 0.0f;
        if (target.loudCompEnable > 0.0001f) loudDb = computeLoudnessCompDb(target.decay);
        const float loudGain = dsp::dbToLin(loudDb);

        const float duckDepthLin = dsp::dbToLin(-dsp::clampf(target.duckDepthDb, 0.0f, 36.0f));
        const float duckThreshLin = dsp::dbToLin(dsp::clampf(target.duckThresholdDb, -80.0f, 0.0f));

        // Set diffusion time-varying parameter once per chunk
        diffusion.setTimeVaryingG(target.inputDiffG);

        // Fetch tank config once per chunk
        const auto tcNow = tank.getConfig();
        rebuildStereoVectors(tcNow.lines);

        const float gS = dsp::clampf(target.stereoDepth, 0.0f, 1.0f);

        const float cfEnable = (target.cloudFrontEnable > 0.0001f) ? 1.0f : 0.0f;
        const float cfAmt = dsp::clampf(target.cloudFrontAmount, 0.0f, 1.0f) * cfEnable;
        const float cfSizeSamp = msToSamples(dsp::clampf(target.cloudFrontSizeMs, 0.0f, 120.0f), sr);
        const float cfWidth = dsp::clampf(target.cloudFrontWidth, 0.0f, 1.0f);
        const float widthSkewSamp = cfWidth * msToSamples(0.45f, sr); // up to ~0.45 ms

        // Step 5 controls (cache per chunk)
        const float smearOn = (target.cloudSmearEnable > 0.0001f) ? 1.0f : 0.0f;
        const float smearAmt = dsp::clampf(target.cloudSmearAmount, 0.0f, 1.0f) * smearOn;
        const float smearTimeSamp = msToSamples(dsp::clampf(target.cloudSmearTimeMs, 0.0f, 60.0f), sr);
        const float smearWidth = dsp::clampf(target.cloudSmearWidth, 0.0f, 1.0f);
        const float smearSkewSamp = smearWidth * msToSamples(0.60f, sr); // up to ~0.6 ms

        for (int i = 0; i < chunk; ++i) {
            const float pL = wetL[i];
            const float pR = wetR[i];

            const float eL = erL[i];
            const float eR = erR[i];

            // Step 3: Cloud front-end multitap spray from predelay buffer
            float sprayL = 0.0f;
            float sprayR = 0.0f;

            if (cfAmt > 0.0f && cfSizeSamp > 0.0f) {
                for (int t = 0; t < kCloudTaps; ++t) {
                    const float dt = kTapPos[t] * cfSizeSamp;
                    const float sign = kTapSign[t];
                    const float skew = sign * widthSkewSamp;

                    const float dL = std::max(1.0f, preSamp + dt + skew);
                    const float dR = std::max(1.0f, preSamp + dt - skew);

                    const float tapL = preL.readFracCubic(dL);
                    const float tapR = preR.readFracCubic(dR);

                    sprayL += kTapGain[t] * tapL;
                    sprayR += kTapGain[t] * tapR;
                }

                // conservative normalization
                sprayL *= 0.22f;
                sprayR *= 0.22f;
            }

            // Build injection
            float injL = pL + eL * 0.65f + cfAmt * sprayL;
            float injR = pR + eR * 0.65f + cfAmt * sprayR;

            diffusion.processInput(injL, injR);

            // Step 1: MS decorrelated vector injection into tank
            const float M = 0.5f * (injL + injR);
            const float S = 0.5f * (injL - injR);

            for (int li = 0; li < tcNow.lines; ++li) {
                injVec[li] = (M * vM[li]) + (S * gS) * vS[li];
            }

            tank.processSampleVec(injVec, effDecay, lfos, yVec);

            float tailL = 0.0f, tailR = 0.0f;
            bigpi::core::renderTapPattern(yVec, tcNow.lines, modeCfg.tank.tapPattern, tailL, tailR);

            // -----------------------------------------------------------------
            // Step 5: Optional post-tank micro-smear
            // - push current tail into a short buffer
            // - read a few micro-taps and mix back in
            // - placed before late diffusion for a smoother "cloud" bloom
            // -----------------------------------------------------------------
            if (smearAmt > 0.0f && smearTimeSamp > 0.0f) {
                smearL.push(tailL);
                smearR.push(tailR);

                float sL = 0.0f;
                float sR = 0.0f;

                for (int t = 0; t < kSmearTaps; ++t) {
                    const float dt = kSmearPos[t] * smearTimeSamp;
                    const float sign = kSmearSign[t];
                    const float skew = sign * smearSkewSamp;

                    const float dL = std::max(1.0f, dt + skew);
                    const float dR = std::max(1.0f, dt - skew);

                    sL += kSmearGain[t] * smearL.readFracCubic(dL);
                    sR += kSmearGain[t] * smearR.readFracCubic(dR);
                }

                // normalization: keep subtle and stable
                sL *= 0.20f;
                sR *= 0.20f;

                tailL = (1.0f - smearAmt) * tailL + smearAmt * (tailL + sL);
                tailR = (1.0f - smearAmt) * tailR + smearAmt * (tailR + sR);
            }
            else {
                // keep smear buffers “warm” but stable if enabled later mid-stream
                smearL.push(tailL);
                smearR.push(tailR);
            }

            // Late diffusion
            if (target.lateDiffEnable > 0.0001f) {
                diffusion.processLate(tailL, tailR, dsp::clampf(target.lateDiffAmount, 0.0f, 1.0f));
            }

            float wetOutL = tailL + eL;
            float wetOutR = tailR + eR;

            wetOutL *= loudGain;
            wetOutR *= loudGain;

            float duckGain = 1.0f;
            if (target.duckEnable > 0.0001f) {
                const float inMono = 0.5f * (std::abs(inL[pos + i]) + std::abs(inR[pos + i]));
                const float env = duckEnv.process(inMono);

                if (env > duckThreshLin) {
                    const float denom = std::max(1e-6f, (1.0f - duckThreshLin));
                    const float over = dsp::clampf((env - duckThreshLin) / denom, 0.0f, 1.0f);
                    duckGain = (1.0f - over) + over * duckDepthLin;
                }
            }

            wetL[i] = wetOutL * duckGain;
            wetR[i] = wetOutR * duckGain;
        }

        outStage.processBlock(wetL.data(), wetR.data(), chunk);

        const float mix = dsp::clampf(target.mix, 0.0f, 1.0f);

        for (int i = 0; i < chunk; ++i) {
            const float dryL = inL[pos + i];
            const float dryR = inR[pos + i];

            const float wL = wetL[i];
            const float wR = wetR[i];

            outL[pos + i] = (1.0f - mix) * dryL + mix * wL;
            outR[pos + i] = (1.0f - mix) * dryR + mix * wR;
        }

        pos += chunk;
    }
}