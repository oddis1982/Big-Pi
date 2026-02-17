#include "dsp/engines/tune_hall/ReverbEngine.h"

#include <algorithm> // std::min, std::max
#include <array>     // std::array
#include <cmath>     // std::abs

static inline float msToSamples(float ms, float sr) {
    return ms * 0.001f * sr;
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

    int preMax = std::max(16, int(sr * 0.20f));
    preL.init(preMax);
    preR.init(preMax);

    diffusion.init(sr, 0xB16B00B5u);

    lfos.init(16, sr);

    duckEnv.setSampleRate(sr);
    duckEnv.setAttackReleaseMs(8.0f, 120.0f);
    duckEnv.clear();

    int maxTankDelay = std::max(64, int(sr * 2.5f));
    tank.init(sr, maxTankDelay, 0xC0FFEEu);

    // Apply preset defaults into target + tank config
    applyModePreset(target.mode);

    // IMPORTANT:
    // reset() must work during prepare(). It should not be gated by prepared==true.
    prepared = true;
    reset();
}

void ReverbEngine::reset() {
    // Always reset if prepare() has been called. (prepare() sets prepared=true)
    if (!prepared) return;

    preL.clear();
    preR.clear();

    er.reset();
    diffusion.clear();
    tank.clear();
    outStage.reset();

    duckEnv.clear();
}

void ReverbEngine::setParams(const Params& p) {
    bool modeChanged = (p.mode != target.mode);
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

    tank.setConfig(tc);
}

void ReverbEngine::applyModePreset(bigpi::Mode m) {
    modeCfg = bigpi::getModePreset(m);

    bigpi::core::Tank::Config tc = tank.getConfig();
    tc.lines = modeCfg.tank.delayLines;

    tc.matrix = modeCfg.tank.useHouseholder
        ? bigpi::core::MatrixType::Householder
        : bigpi::core::MatrixType::Hadamard;

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

    const float* baseSet = baseMs16_Default;
    if (m == bigpi::Mode::Hall) baseSet = baseMs16_Hall;

    for (int i = 0; i < 16; ++i) {
        float ms = baseSet[i] * modeCfg.tank.delayScale;
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

    if (m == bigpi::Mode::Hall) {
        target.fbXoverLoHz = 220.0f;
        target.fbXoverHiHz = 3800.0f;

        target.erDampHz = 8500.0f;
        target.erWidth = 1.20f;

        target.modJitterEnable = 1.0f;
        target.modJitterAmount = 0.25f;
        target.modJitterRateHz = 0.30f;
        target.modJitterSmoothMs = 90.0f;

        target.outHpHz = 22.0f;

        target.outLowShelfHz = 180.0f;
        target.outLowGainDb = +1.0f;

        target.outHighShelfHz = 7000.0f;
        target.outHighGainDb = -1.5f;

        target.outWidth = 1.15f;
        target.outDrive = 0.0f;
    }

    // Push configs NOW so mode switches take effect immediately
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

    // Ensure tank config includes the non-preset owned fields too
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

    tc2.fbHpHz = target.feedbackHpHz;
    tc2.dampHz = target.dampingHz;

    tank.setConfig(tc2);
}

float ReverbEngine::computeEffectiveDecay(float decay, float freeze01) const {
    freeze01 = dsp::clampf(freeze01, 0.0f, 1.0f);
    decay = dsp::clampf(decay, 0.0f, 0.9995f);
    float frozen = 0.9993f;
    return (1.0f - freeze01) * decay + freeze01 * frozen;
}

float ReverbEngine::computeLoudnessCompDb(float decay01) const {
    decay01 = dsp::clampf(decay01, 0.0f, 1.0f);

    float strength = dsp::clampf(target.loudCompStrength, 0.0f, 1.0f);
    float maxDb = dsp::clampf(target.loudCompMaxDb, 0.0f, 24.0f);

    return -maxDb * strength * decay01;
}

void ReverbEngine::processBlock(const float* inL, const float* inR,
    float* outL, float* outR,
    int n)
{
    if (!prepared) return;
    if (n <= 0) return;

    // Real-time safety: never resize buffers here.
    // If n > block (allocated in prepare), process in chunks.
    int pos = 0;
    while (pos < n) {
        const int chunk = std::min(block, n - pos);

        const float preSamp = msToSamples(dsp::clampf(target.predelayMs, 0.0f, 200.0f), sr);

        // Predelay stage
        for (int i = 0; i < chunk; ++i) {
            preL.push(inL[pos + i]);
            preR.push(inR[pos + i]);
            wetL[i] = preL.readFracCubic(preSamp);
            wetR[i] = preR.readFracCubic(preSamp);
        }

        // Early reflections
        er.processBlock(wetL.data(), wetR.data(), erL.data(), erR.data(), chunk);

        std::array<float, bigpi::core::kMaxLines> yVec{};

        const float effDecay = computeEffectiveDecay(target.decay, target.freeze);

        float loudDb = 0.0f;
        if (target.loudCompEnable > 0.0001f) loudDb = computeLoudnessCompDb(target.decay);
        const float loudGain = dsp::dbToLin(loudDb);

        const float duckDepthLin = dsp::dbToLin(-dsp::clampf(target.duckDepthDb, 0.0f, 36.0f));
        const float duckThreshLin = dsp::dbToLin(dsp::clampf(target.duckThresholdDb, -80.0f, 0.0f));

        // Control parameter: set once per chunk (not per-sample)
        diffusion.setTimeVaryingG(target.inputDiffG);

        // Cache tank config once per chunk (avoids repeated getConfig())
        const auto tc = tank.getConfig();

        for (int i = 0; i < chunk; ++i) {
            const float pL = wetL[i];
            const float pR = wetR[i];

            const float eL = erL[i];
            const float eR = erR[i];

            float injL = pL + eL * 0.65f;
            float injR = pR + eR * 0.65f;

            diffusion.processInput(injL, injR);

            const float injMono = 0.5f * (injL + injR);

            tank.processSample(injMono, effDecay, lfos, yVec);

            float tailL = 0.0f, tailR = 0.0f;
            bigpi::core::renderTapPattern(yVec, tc.lines, modeCfg.tank.tapPattern, tailL, tailR);

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

