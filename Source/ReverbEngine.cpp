#include "ReverbEngine.h"
#include "Version.h"

static inline float msToSamples(float ms, float sr) {
    return ms * 0.001f * sr;
}

void ReverbEngine::prepare(float sampleRate, int blockSize) {
    sr = sampleRate;
    block = std::max(1, blockSize);

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

    applyModePreset(target.mode);

    reset();
    prepared = true;
}

void ReverbEngine::reset() {
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

    // --------------------------------------------------------------------------
    // Delay line sets (ms) per mode
    //
    // NOTE:
    // We start with a general-purpose set for most modes, but for Hall we use
    // a dedicated set (Phase 1 tuning). This is a major quality lever.
    // --------------------------------------------------------------------------

    static const float baseMs16_Default[16] = {
      29.7f, 37.1f, 41.1f, 43.7f,
      53.9f, 59.5f, 61.7f, 71.3f,
      79.9f, 89.7f, 97.3f, 101.9f,
      107.9f, 115.1f, 123.7f, 131.9f
    };

    // Tuned Hall set (slightly different distribution to reduce metallicity)
    static const float baseMs16_Hall[16] = {
      31.7f, 37.9f, 41.3f, 43.1f,
      53.3f, 59.9f, 61.1f, 71.7f,
      79.3f, 89.1f, 97.9f, 103.3f,
      109.7f, 117.1f, 125.9f, 137.3f
    };

    const float* baseSet = baseMs16_Default;
    if (m == bigpi::Mode::Hall) {
        baseSet = baseMs16_Hall;
    }

    for (int i = 0; i < 16; ++i) {
        float ms = baseSet[i] * modeCfg.tank.delayScale;
        tc.delaySamp[i] = msToSamples(ms, sr);
    }

    // Copy modulation maps from preset
    tc.modDepthMul = modeCfg.tank.modDepthMul;
    tc.modRateMul = modeCfg.tank.modRateMul;

    // Apply tank tuning from preset (what TankConfig actually owns)
    target.inputDiffStages = modeCfg.tank.inputDiffStages;
    target.inputDiffG = modeCfg.tank.inputDiffG;

    target.lateDiffMinG = modeCfg.tank.lateDiffMinG;
    target.lateDiffMaxG = modeCfg.tank.lateDiffMaxG;

    target.modDepthMs = modeCfg.tank.modDepthMs;
    target.modRateHz = modeCfg.tank.modRateHz;

    target.decayLowMul = modeCfg.tank.decayLowMul;
    target.decayMidMul = modeCfg.tank.decayMidMul;
    target.decayHighMul = modeCfg.tank.decayHighMul;

    // --------------------------------------------------------------------------
    // Mode suggested defaults (UI-ish defaults)
    // --------------------------------------------------------------------------
    target.mix = modeCfg.defaultMix;
    target.decay = modeCfg.defaultDecay;
    target.dampingHz = modeCfg.defaultDamping;
    target.predelayMs = modeCfg.defaultPreDelay;

    target.erLevel = modeCfg.defaultERLevel;
    target.erSize = modeCfg.defaultERSize;

    // --------------------------------------------------------------------------
    // Hall-specific extra voicing (Phase 1)
    //
    // These parameters are not currently stored in ModeConfig/TankConfig,
    // so we set them here when Hall is selected.
    // --------------------------------------------------------------------------
    if (m == bigpi::Mode::Hall) {

        // Multiband crossover points (more realistic hall warmth)
        target.fbXoverLoHz = 220.0f;
        target.fbXoverHiHz = 3800.0f;

        // ER voicing
        target.erDampHz = 8500.0f;
        target.erWidth = 1.20f;

        // Jitter tuning: subtle organic motion without chorus wobble
        target.modJitterEnable = 1.0f;
        target.modJitterAmount = 0.25f;
        target.modJitterRateHz = 0.30f;
        target.modJitterSmoothMs = 90.0f;

        // Output voicing: gentle warmth + slightly darker top
        target.outHpHz = 22.0f;

        target.outLowShelfHz = 180.0f;
        target.outLowGainDb = +1.0f;

        target.outHighShelfHz = 7000.0f;
        target.outHighGainDb = -1.5f;

        target.outWidth = 1.15f;

        // Keep saturation off by default for Hall
        target.outDrive = 0.0f;
    }

    // --------------------------------------------------------------------------
    // Push updated configs into modules NOW
    // (So switching modes immediately updates sound even before next setParams)
    // --------------------------------------------------------------------------

    tank.setConfig(tc);

    EarlyReflections::Params erp;
    erp.level = target.erLevel;
    erp.size = target.erSize;
    erp.dampHz = target.erDampHz;
    erp.width = target.erWidth;
    er.setParams(erp);

    // Update output stage immediately too
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

    // Also update diffusion configs immediately
    bigpi::core::Diffusion::InputConfig inCfg = {};
    inCfg.stages = target.inputDiffStages;
    inCfg.g = target.inputDiffG;
    diffusion.setInputConfig(inCfg);

    bigpi::core::Diffusion::LateConfig lateCfg = {};
    lateCfg.minG = target.lateDiffMinG;
    lateCfg.maxG = target.lateDiffMaxG;
    diffusion.setLateConfig(lateCfg);

    // Finally: apply crossover/multiband/mod/jitter changes into tank config too
    // (these are owned by Tank::Config, not TankPreset)
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

    float attDb = -maxDb * strength * decay01;
    return attDb;
}

void ReverbEngine::processBlock(const float* inL, const float* inR,
    float* outL, float* outR,
    int n) {
    if (!prepared) return;

    if (n > block) {
        // For development harness, allow resize. For hard real-time, pre-allocate.
        block = n;
        wetL.assign(block, 0.0f);
        wetR.assign(block, 0.0f);
        erL.assign(block, 0.0f);
        erR.assign(block, 0.0f);
    }

    float preSamp = msToSamples(dsp::clampf(target.predelayMs, 0.0f, 200.0f), sr);

    for (int i = 0; i < n; ++i) {
        preL.push(inL[i]);
        preR.push(inR[i]);
        wetL[i] = preL.readFracCubic(preSamp);
        wetR[i] = preR.readFracCubic(preSamp);
    }

    er.processBlock(wetL.data(), wetR.data(), erL.data(), erR.data(), n);

    std::array<float, bigpi::core::kMaxLines> yVec{};

    float effDecay = computeEffectiveDecay(target.decay, target.freeze);

    float loudDb = 0.0f;
    if (target.loudCompEnable > 0.0001f) loudDb = computeLoudnessCompDb(target.decay);
    float loudGain = dsp::dbToLin(loudDb);

    float duckDepthLin = dsp::dbToLin(-dsp::clampf(target.duckDepthDb, 0.0f, 36.0f));
    float duckThreshLin = dsp::dbToLin(dsp::clampf(target.duckThresholdDb, -80.0f, 0.0f));

    for (int i = 0; i < n; ++i) {
        float pL = wetL[i];
        float pR = wetR[i];

        float eL = erL[i];
        float eR = erR[i];

        float injL = pL + eL * 0.65f;
        float injR = pR + eR * 0.65f;

        diffusion.setTimeVaryingG(target.inputDiffG);
        diffusion.processInput(injL, injR);

        float injMono = 0.5f * (injL + injR);

        tank.processSample(injMono, effDecay, lfos, yVec);

        float tailL = 0.0f, tailR = 0.0f;
        bigpi::core::renderTapPattern(yVec, tank.getConfig().lines, modeCfg.tank.tapPattern, tailL, tailR);

        if (target.lateDiffEnable > 0.0001f) {
            diffusion.processLate(tailL, tailR, dsp::clampf(target.lateDiffAmount, 0.0f, 1.0f));
        }

        float wetOutL = tailL + eL;
        float wetOutR = tailR + eR;

        wetOutL *= loudGain;
        wetOutR *= loudGain;

        float duckGain = 1.0f;
        if (target.duckEnable > 0.0001f) {
            float inMono = 0.5f * (std::abs(inL[i]) + std::abs(inR[i]));
            float env = duckEnv.process(inMono);

            if (env > duckThreshLin) {
                float over = dsp::clampf((env - duckThreshLin) / std::max(1e-6f, (1.0f - duckThreshLin)), 0.0f, 1.0f);
                duckGain = (1.0f - over) + over * duckDepthLin;
            }
        }

        wetOutL *= duckGain;
        wetOutR *= duckGain;

        wetL[i] = wetOutL;
        wetR[i] = wetOutR;
    }

    outStage.processBlock(wetL.data(), wetR.data(), n);

    float mix = dsp::clampf(target.mix, 0.0f, 1.0f);

    for (int i = 0; i < n; ++i) {
        float dryL = inL[i];
        float dryR = inR[i];

        float wL = wetL[i];
        float wR = wetR[i];

        outL[i] = (1.0f - mix) * dryL + mix * wL;
        outR[i] = (1.0f - mix) * dryR + mix * wR;
    }
}

