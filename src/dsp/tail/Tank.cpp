#include "Tank.h"

/*
  =============================================================================
  Tank.cpp — Big Pi Late Reverb Tank (implementation)
  =============================================================================

  This file processes ONE sample at a time through an FDN-like structure.

  Terminology:
    - "line" = one delay line in the network
    - "inj"  = injection sample (mono) fed into the tank each sample
    - "y[i]" = the output read from delay line i
    - "m[i]" = the mixed vector after matrix (computed externally by engine)
              but here we still write back using internally computed feedback
              signal based on mixed values.

  Architectural note:
    In our modular architecture:
      - Tank reads delay lines, applies modulation, returns y[]
      - ReverbEngine applies matrix mixing and decides how to render taps
      - Then Tank receives the mixed values implicitly by using lastY + external
        matrix? (We will keep Tank self-contained by doing matrix here.)

  BUT:
    To keep files small and stable, we choose one clear approach:

      ✅ Tank handles:
         - read y[]
         - matrix mixing (Hadamard/Householder)
         - filter + decay + writeback

      ✅ Engine handles:
         - injection creation
         - output tap pattern rendering
         - diffusion + output stage + crossfades

  This keeps Tank cohesive and avoids duplicated mixing logic.

  Real-time safety:
    - All memory allocated in init()
    - processSample has no allocations
*/

namespace bigpi::core {

    void Tank::init(float sampleRate, int maxDelaySamples, uint32_t s) {
        sr = sampleRate;
        seed = (s == 0 ? 1u : s);

        // Allocate delay buffers.
        // Each delay line gets the same max size; we control actual delay via read offset.
        maxDelaySamples = std::max(8, maxDelaySamples);

        for (int i = 0; i < kMaxLines; ++i) {
            d[i].init(maxDelaySamples);

            // Prepare jitter noise sources (one per line)
            jitter[i].setSampleRate(sr);
            jitter[i].seed(seed + 0x9E3779B9u * uint32_t(i + 1)); // golden ratio-ish scramble
            jitter[i].setRateHz(0.35f);
            jitter[i].setSmoothMs(80.0f);

            // Clear filters
            hp[i].clear();
            lp[i].clear();
            xLo[i].clear();
            xHi[i].clear();

            lastY[i] = 0.0f;
        }

        // Envelope follower setup (tail energy proxy)
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

        // Clamp line count to safe range.
        cfg.lines = std::max(1, std::min(cfg.lines, kMaxLines));

        // Update feedback filters and split filters.
        // We do this here so per-sample loop stays cheaper.
        for (int i = 0; i < cfg.lines; ++i) {
            hp[i].setCutoff(cfg.fbHpHz, sr);
            lp[i].setCutoff(cfg.dampHz, sr);

            // Multiband split:
            // xLo: low-pass at xoverLoHz (gives "low band")
            // xHi: low-pass at xoverHiHz (gives "low+mid band")
            xLo[i].setCutoff(cfg.xoverLoHz, sr);
            xHi[i].setCutoff(cfg.xoverHiHz, sr);

            // Update jitter configuration
            jitter[i].setRateHz(cfg.jitterRateHz);
            jitter[i].setSmoothMs(cfg.jitterSmoothMs);
        }

        // Configure envelope follower for dynamic damping behavior.
        envFollower.setAttackReleaseMs(cfg.dynAtkMs, cfg.dynRelMs);
    }

    float Tank::computeDynamicDampingHz(float staticDampHz, float env01Now) {
        // Dynamic damping is meant to emulate real spaces:
        // when the tank is “excited” (loud input), highs are damped more.
        //
        // env01Now is roughly in [0,1] but not strictly limited.
        // We clamp it to avoid extreme values.
        float e = dsp::clampf(env01Now * cfg.dynSensitivity, 0.0f, 1.0f);

        // Map envelope to cutoff: loud -> closer to dynMinHz (darker)
        // quiet -> closer to dynMaxHz (brighter)
        float dynHz = cfg.dynMaxHz + (cfg.dynMinHz - cfg.dynMaxHz) * e;

        // Blend between static damping and dynamic target.
        // dynAmount=0 => static
        // dynAmount=1 => fully dynamic
        float amt = dsp::clampf(cfg.dynAmount, 0.0f, 1.0f);

        return (1.0f - amt) * staticDampHz + amt * dynHz;
    }

    void Tank::processSample(float inj,
        float baseDecay,
        dsp::MultiLFO& lfoBank,
        std::array<float, kMaxLines>& yOut) {
        if (!inited) {
            yOut.fill(0.0f);
            return;
        }

        const int N = cfg.lines;

        // Clamp decay to a safe range.
        // Values too close to 1.0 can explode with mixing + saturation.
        baseDecay = dsp::clampf(baseDecay, 0.0f, 0.9995f);

        // --------------------------------------------------------------------------
        // 1) Read each delay line output with fractional delay modulation
        // --------------------------------------------------------------------------
        //
        // Why modulate fractional delay reads?
        //   Static delay networks can produce strong stationary resonances.
        //   Small modulation breaks up those resonances → smoother tail.
        //
        // We use two modulation sources:
        //   - sinusoidal LFO from MultiLFO (predictable periodic motion)
        //   - smoothed random jitter (organic non-periodic motion)
        //
        // Total modulation (samples):
        //   mod = modDepthSamples * (lfo * depthMul + jitterEnable * jitter * jitterAmount)
        //
        // Then we read at (delaySamp + mod).
        //
        std::array<float, kMaxLines> y{};
        y.fill(0.0f);

        float peakAbs = 0.0f;

        for (int i = 0; i < N; ++i) {
            float depthMul = cfg.modDepthMul[i];
            float rateMul = cfg.modRateMul[i];

            // Sinusoidal LFO in [-1,1]
            float lfo = lfoBank.process(i, cfg.modRateHz * rateMul);

            // Jitter noise in [-1,1], smoothed
            float jit = 0.0f;
            if (cfg.jitterEnable > 0.0001f) {
                jit = jitter[i].process();
            }

            float mod = cfg.modDepthSamples
                * (lfo * depthMul + cfg.jitterEnable * cfg.jitterAmount * jit);

            float delay = cfg.delaySamp[i] + mod;

            // Read the delay line output at this fractional delay position.
            float yi = d[i].readFracCubic(delay);

            y[i] = yi;
            yOut[i] = yi;
            lastY[i] = yi;

            peakAbs = std::max(peakAbs, std::abs(yi));
        }

        // Update tank envelope (tail energy proxy).
        // We feed envelope follower a representative energy measure.
        //
        // Using peakAbs is cheap; using RMS would be more accurate but cost more CPU.
        float e = envFollower.process(peakAbs);
        env01 = dsp::clampf(e * 2.0f, 0.0f, 1.0f); // scale to a handy 0..1 range

        // --------------------------------------------------------------------------
        // 2) Mix y[] through matrix (Hadamard or Householder)
        // --------------------------------------------------------------------------
        //
        // This is the key “density” mechanism: it spreads energy across lines.
        //
        mix(y, N, cfg.matrix);

        // --------------------------------------------------------------------------
        // 3) Compute dynamic damping cutoff (optional)
        // --------------------------------------------------------------------------
        //
        // Dynamic damping = brightness changes with excitation.
        // If enabled, compute a new effective dampHz based on env.
        float dampHzEffective = cfg.dampHz;
        if (cfg.dynEnable > 0.0001f) {
            dampHzEffective = computeDynamicDampingHz(cfg.dampHz, env01);
        }

        // Update low-pass filter cutoff per line.
        // (Yes, this is inside processSample; but it’s still cheap for N=8/16.
        // If you want later optimization, we can update per block and smooth.)
        for (int i = 0; i < N; ++i) {
            lp[i].setCutoff(dampHzEffective, sr);
        }

        // --------------------------------------------------------------------------
        // 4) Write back into delay lines (feedback + injection)
        // --------------------------------------------------------------------------
        //
        // Each line receives:
        //   x = inj + feedbackSignal
        //
        // feedbackSignal is:
        //   - the mixed signal y[i]
        //   - filtered (HP then LP)
        //   - split into low/mid/high bands
        //   - each band scaled by its decay multiplier
        //   - recombined
        //   - optionally saturated
        //
        // Why multiband decay?
        //   Real spaces absorb highs quickly, and lows can linger longer.
        //
        // We do:
        //   low  = LP(xLoHz)
        //   low+mid = LP(xHiHz)
        //   mid = (low+mid) - low
        //   high = input - (low+mid)
        //
        // Then:
        //   fb = low*decLow + mid*decMid + high*decHigh
        //
        // Where decLow = baseDecay*decayLowMul, etc.
        //
        float decLow = dsp::clampf(baseDecay * cfg.decayLowMul, 0.0f, 0.9997f);
        float decMid = dsp::clampf(baseDecay * cfg.decayMidMul, 0.0f, 0.9997f);
        float decHigh = dsp::clampf(baseDecay * cfg.decayHighMul, 0.0f, 0.9997f);

        // We distribute injection across lines so the tank builds faster and evenly.
        float injPerLine = inj / float(N);

        for (int i = 0; i < N; ++i) {
            float fb = y[i];

            // Feedback HP (remove rumble / DC)
            fb = hp[i].process(fb);

            // Feedback LP (damping - already configured to dampHzEffective)
            fb = lp[i].process(fb);

            // Multiband split
            float low = xLo[i].process(fb);
            float lowMid = xHi[i].process(fb);

            float mid = lowMid - low;
            float high = fb - lowMid;

            // Apply multiband decays
            float fbColored = low * decLow
                + mid * decMid
                + high * decHigh;

            // Optional saturation inside the feedback loop.
            // This can add "density" and prevent sterile tails.
            //
            // satMix:
            //   0 => clean feedback (fbColored)
            //   1 => fully saturated feedback
            float sat = dsp::softSat(fbColored, cfg.drive);
            float fbFinal = (1.0f - cfg.satMix) * fbColored + cfg.satMix * sat;

            // Combine injection + feedback
            float x = injPerLine + fbFinal;

            // Push into delay line (write head)
            d[i].push(x);
        }
    }

} // namespace bigpi::core
