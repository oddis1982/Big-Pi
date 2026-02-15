#pragma once
/*
  =============================================================================
  Dsp.h — Big Pi DSP Primitives (header-only, heavily annotated)
  =============================================================================

  This header provides the building blocks used across Big Pi:

    - Utility math (clamp, dB conversions)
    - Parameter smoothing (avoid clicks/zipper noise)
    - Filters:
        * One-pole LP/HP (cheap damping & rumble removal)
        * Biquad shelves (tone shaping in OutputStage)
    - Envelope follower (ducking, loudness comp, dynamic damping)
    - Allpass diffuser (diffusion / density)
    - DelayLine with cubic interpolation (fractional delay for modulation)
    - MultiLFO (per-line sinusoidal modulation)
    - SmoothNoise (smoothed random modulation for jitter)
    - StereoSpinner (slow random stereo rotation)
    - Soft saturation (tanh-based, “glue/density”)

  Why header-only?
    - Most functions are small and benefit from inlining.
    - No separate Dsp.cpp is required or expected.

  Scientific concepts (novice-friendly overview):
    - Sampling: audio is processed as discrete samples x[n]
    - Filters are difference equations (depend on past values)
    - Smoothing is a low-pass filter applied to control signals
    - Fractional delay requires interpolation (delay time not an integer)
    - Diffusion uses allpass networks to scramble phase without big EQ change
    - Modulation reduces stationary resonances (less metallic ringing)
    - Nonlinearity creates harmonics; gentle saturation can add density

  Real-time safety:
    - No allocations inside per-sample processing
    - Avoid expensive operations inside tight loops where possible
*/

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace dsp {

    // ============================================================================
    // Basic helpers
    // ============================================================================

    inline float clampf(float x, float lo, float hi) {
        return std::max(lo, std::min(hi, x));
    }

    inline float dbToLin(float db) {
        // 20*log10(A) => A = 10^(db/20)
        return std::pow(10.0f, db / 20.0f);
    }

    inline float linToDb(float lin) {
        // Avoid log(0) -> -inf
        lin = std::max(lin, 1e-12f);
        return 20.0f * std::log10(lin);
    }

    /*
      curve01(x, shape)
      -----------------
      Remaps a 0..1 control curve so knobs feel more "musical".

      shape > 1  => more resolution near 0 (slow start)
      shape < 1  => more resolution near 1 (fast start)
    */
    inline float curve01(float x01, float shape) {
        x01 = clampf(x01, 0.0f, 1.0f);
        shape = clampf(shape, 0.05f, 10.0f);
        return std::pow(x01, shape);
    }

    // ============================================================================
    // Parameter smoothing (control-rate low-pass filter)
    // ============================================================================

    /*
      SmoothValue
      ----------
      Smoothly approaches a target value.

      Equation:
        y[n] = a*y[n-1] + (1-a)*x[n]

      Where "x[n]" is the target and "y[n]" is the smoothed value.

      Choosing a:
        a = exp(-1 / (T * sr))
      where:
        - T is a time constant (seconds)
        - sr is sample rate

      Intuition:
        Larger T => a closer to 1 => slower smoothing.
    */
    struct SmoothValue {
        float y = 0.0f;
        float a = 0.0f;
        float sr = 48000.0f;

        void setTimeMs(float ms, float sampleRate) {
            sr = sampleRate;
            float sec = std::max(ms, 0.001f) * 0.001f;
            a = std::exp(-1.0f / (sec * sr));
        }

        void setInstant(float v) { y = v; }

        float process(float target) {
            y = a * y + (1.0f - a) * target;
            return y;
        }
    };

    // ============================================================================
    // One-pole filters (cheap and stable)
    // ============================================================================

    /*
      OnePoleLP:
        z = a*z + (1-a)*x

      OnePoleHP:
        y = x - LP(x)

      Used heavily in reverbs because:
        - very cheap
        - stable
        - great for damping highs and removing rumble
    */
    struct OnePoleLP {
        float z = 0.0f;
        float a = 0.0f;

        void clear() { z = 0.0f; }

        void setCutoff(float hz, float sr) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            a = std::exp(-2.0f * float(M_PI) * hz / sr);
        }

        float process(float x) {
            z = a * z + (1.0f - a) * x;
            return z;
        }
    };

    struct OnePoleHP {
        float z = 0.0f;
        float a = 0.0f;

        void clear() { z = 0.0f; }

        void setCutoff(float hz, float sr) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            a = std::exp(-2.0f * float(M_PI) * hz / sr);
        }

        float process(float x) {
            z = a * z + (1.0f - a) * x; // low-pass state
            return x - z;              // high-pass output
        }
    };

    // ============================================================================
    // Biquad (RBJ cookbook style) — used for shelves in OutputStage
    // ============================================================================

    /*
      Biquad: second-order IIR filter.

      We use Direct Form II Transposed:
        y = b0*x + z1
        z1 = b1*x - a1*y + z2
        z2 = b2*x - a2*y

      Notes:
        - a0 is normalized to 1.0
        - coefficients computed using well-known RBJ formulas
    */
    struct Biquad {
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
        float a1 = 0.0f, a2 = 0.0f;
        float z1 = 0.0f, z2 = 0.0f;

        void clear() { z1 = 0.0f; z2 = 0.0f; }

        float process(float x) {
            float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }

        static void omega(float hz, float sr, float& w0, float& c, float& s) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            w0 = 2.0f * float(M_PI) * (hz / sr);
            c = std::cos(w0);
            s = std::sin(w0);
        }

        void setLowPass(float hz, float Q, float sr) {
            Q = clampf(Q, 0.1f, 10.0f);
            float w0, c, s; omega(hz, sr, w0, c, s);
            float alpha = s / (2.0f * Q);

            float bb0 = (1.0f - c) * 0.5f;
            float bb1 = (1.0f - c);
            float bb2 = (1.0f - c) * 0.5f;
            float aa0 = 1.0f + alpha;
            float aa1 = -2.0f * c;
            float aa2 = 1.0f - alpha;

            b0 = bb0 / aa0; b1 = bb1 / aa0; b2 = bb2 / aa0;
            a1 = aa1 / aa0; a2 = aa2 / aa0;
        }

        void setHighPass(float hz, float Q, float sr) {
            Q = clampf(Q, 0.1f, 10.0f);
            float w0, c, s; omega(hz, sr, w0, c, s);
            float alpha = s / (2.0f * Q);

            float bb0 = (1.0f + c) * 0.5f;
            float bb1 = -(1.0f + c);
            float bb2 = (1.0f + c) * 0.5f;
            float aa0 = 1.0f + alpha;
            float aa1 = -2.0f * c;
            float aa2 = 1.0f - alpha;

            b0 = bb0 / aa0; b1 = bb1 / aa0; b2 = bb2 / aa0;
            a1 = aa1 / aa0; a2 = aa2 / aa0;
        }

        void setLowShelf(float hz, float gainDb, float S, float sr) {
            S = clampf(S, 0.1f, 5.0f);
            float A = std::pow(10.0f, gainDb / 40.0f);
            float w0, c, s; omega(hz, sr, w0, c, s);

            float alpha = s / 2.0f * std::sqrt((A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
            float twoSqrtAalpha = 2.0f * std::sqrt(A) * alpha;

            float bb0 = A * ((A + 1.0f) - (A - 1.0f) * c + twoSqrtAalpha);
            float bb1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * c);
            float bb2 = A * ((A + 1.0f) - (A - 1.0f) * c - twoSqrtAalpha);
            float aa0 = (A + 1.0f) + (A - 1.0f) * c + twoSqrtAalpha;
            float aa1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * c);
            float aa2 = (A + 1.0f) + (A - 1.0f) * c - twoSqrtAalpha;

            b0 = bb0 / aa0; b1 = bb1 / aa0; b2 = bb2 / aa0;
            a1 = aa1 / aa0; a2 = aa2 / aa0;
        }

        void setHighShelf(float hz, float gainDb, float S, float sr) {
            S = clampf(S, 0.1f, 5.0f);
            float A = std::pow(10.0f, gainDb / 40.0f);
            float w0, c, s; omega(hz, sr, w0, c, s);

            float alpha = s / 2.0f * std::sqrt((A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
            float twoSqrtAalpha = 2.0f * std::sqrt(A) * alpha;

            float bb0 = A * ((A + 1.0f) + (A - 1.0f) * c + twoSqrtAalpha);
            float bb1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * c);
            float bb2 = A * ((A + 1.0f) + (A - 1.0f) * c - twoSqrtAalpha);
            float aa0 = (A + 1.0f) - (A - 1.0f) * c + twoSqrtAalpha;
            float aa1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * c);
            float aa2 = (A + 1.0f) - (A - 1.0f) * c - twoSqrtAalpha;

            b0 = bb0 / aa0; b1 = bb1 / aa0; b2 = bb2 / aa0;
            a1 = aa1 / aa0; a2 = aa2 / aa0;
        }
    };

    // ============================================================================
    // Envelope follower (for dynamics/ducking/loudness)
    // ============================================================================

    struct EnvelopeFollower {
        float sr = 48000.0f;
        float env = 0.0f;
        float aAtk = 0.0f;
        float aRel = 0.0f;

        void setSampleRate(float sampleRate) { sr = sampleRate; }

        void setAttackReleaseMs(float attackMs, float releaseMs) {
            attackMs = std::max(attackMs, 0.1f);
            releaseMs = std::max(releaseMs, 0.1f);

            float atkSec = attackMs * 0.001f;
            float relSec = releaseMs * 0.001f;

            aAtk = std::exp(-1.0f / (atkSec * sr));
            aRel = std::exp(-1.0f / (relSec * sr));
        }

        void clear() { env = 0.0f; }

        float process(float x) {
            float mag = std::abs(x);
            float a = (mag > env) ? aAtk : aRel;
            env = a * env + (1.0f - a) * mag;
            return env;
        }
    };

    // ============================================================================
    // Allpass diffuser (delay-based)
    // ============================================================================

    struct Allpass {
        std::vector<float> buf;
        int idx = 0;

        float g = 0.7f;
        float delaySamp = 200.0f;

        void init(int maxDelaySamples) {
            buf.assign(std::max(1, maxDelaySamples), 0.0f);
            idx = 0;
        }

        void clear() {
            std::fill(buf.begin(), buf.end(), 0.0f);
            idx = 0;
        }

        float process(float x) {
            int d = int(clampf(delaySamp, 1.0f, float(buf.size() - 1)));

            int r = idx - d;
            if (r < 0) r += (int)buf.size();

            float v = buf[r];

            // Classic allpass:
            // y = -g*x + v
            // buf[idx] = x + g*y
            float y = -g * x + v;
            buf[idx] = x + g * y;

            idx++;
            if (idx >= (int)buf.size()) idx = 0;

            return y;
        }
    };

    // ============================================================================
    // Delay line with cubic interpolation (fractional delay support)
    // ============================================================================

    struct DelayLine {
        std::vector<float> buf;
        int w = 0;

        void init(int maxSamples) {
            buf.assign(std::max(4, maxSamples), 0.0f);
            w = 0;
        }

        void clear() {
            std::fill(buf.begin(), buf.end(), 0.0f);
            w = 0;
        }

        void push(float x) {
            buf[w] = x;
            w++;
            if (w >= (int)buf.size()) w = 0;
        }

        float readFracCubic(float delaySamples) const {
            delaySamples = clampf(delaySamples, 0.0f, float(buf.size() - 4));

            float rp = float(w) - delaySamples;
            while (rp < 0.0f) rp += float(buf.size());
            while (rp >= float(buf.size())) rp -= float(buf.size());

            int i1 = int(rp);
            float f = rp - float(i1);

            int i0 = i1 - 1; if (i0 < 0) i0 += (int)buf.size();
            int i2 = i1 + 1; if (i2 >= (int)buf.size()) i2 -= (int)buf.size();
            int i3 = i1 + 2; if (i3 >= (int)buf.size()) i3 -= (int)buf.size();

            float y0 = buf[i0], y1 = buf[i1], y2 = buf[i2], y3 = buf[i3];

            // Cubic Hermite interpolation (smooth fractional delay)
            float m1 = 0.5f * (y2 - y0);
            float m2 = 0.5f * (y3 - y1);

            float f2 = f * f;
            float f3 = f2 * f;

            float h00 = 2.0f * f3 - 3.0f * f2 + 1.0f;
            float h10 = f3 - 2.0f * f2 + f;
            float h01 = -2.0f * f3 + 3.0f * f2;
            float h11 = f3 - f2;

            return h00 * y1 + h10 * m1 + h01 * y2 + h11 * m2;
        }
    };

    // ============================================================================
    // Multi LFO bank (sin oscillators for modulation)
    // ============================================================================

    struct MultiLFO {
        int count = 0;
        float sr = 48000.0f;

        std::vector<float> phase;
        std::vector<float> rateMul;

        void init(int n, float sampleRate) {
            count = std::max(1, n);
            sr = sampleRate;

            phase.assign(count, 0.0f);
            rateMul.assign(count, 1.0f);

            for (int i = 0; i < count; ++i) {
                float t = (count == 1) ? 0.0f : float(i) / float(count - 1);
                rateMul[i] = 0.85f + 0.30f * t;                 // spread rates
                phase[i] = (2.0f * float(M_PI)) * (t + 0.13f); // spread phases
            }
        }

        float process(int i, float baseRateHz) {
            i = std::max(0, std::min(i, count - 1));

            float hz = baseRateHz * rateMul[i];
            float inc = (2.0f * float(M_PI) * hz) / sr;

            float y = std::sin(phase[i]);

            phase[i] += inc;
            if (phase[i] >= 2.0f * float(M_PI)) phase[i] -= 2.0f * float(M_PI);

            return y; // [-1, 1]
        }
    };

    // ============================================================================
    // Soft saturation (tanh-based “glue”)
    // ============================================================================

    inline float softSat(float x, float drive) {
        drive = clampf(drive, 0.0f, 10.0f);

        float y = x * (1.0f + drive);
        y = std::tanh(y);

        // Normalize to keep level predictable as drive changes.
        float norm = 1.0f / std::tanh(1.0f + drive);
        return y * norm;
    }

    // ============================================================================
    // SmoothNoise (smoothed random modulation source)
    // ============================================================================

    struct SmoothNoise {
        float sr = 48000.0f;
        uint32_t rng = 0x12345678u;

        float y = 0.0f;
        float target = 0.0f;

        float rateHz = 0.5f;
        int samplesToNext = 1;

        float a = 0.0f;

        void setSampleRate(float sampleRate) { sr = sampleRate; }

        void setRateHz(float hz) {
            rateHz = clampf(hz, 0.01f, 20.0f);
            int period = int(sr / rateHz);
            samplesToNext = std::max(1, period);
        }

        void setSmoothMs(float ms) {
            ms = std::max(ms, 0.1f);
            float sec = ms * 0.001f;
            a = std::exp(-1.0f / (sec * sr));
        }

        void seed(uint32_t s) { rng = (s == 0 ? 1u : s); }

        void clear() {
            y = 0.0f;
            target = 0.0f;
            samplesToNext = 1;
        }

        float nextRandBipolar() {
            // Fast LCG RNG (good enough for modulation noise)
            rng = 1664525u * rng + 1013904223u;

            // Convert bits -> float in [0,1)
            uint32_t r = (rng >> 9) | 0x3F800000u;
            float f = (*(float*)&r) - 1.0f;

            return 2.0f * f - 1.0f;
        }

        float process() {
            samplesToNext--;
            if (samplesToNext <= 0) {
                target = nextRandBipolar();
                int period = int(sr / rateHz);
                samplesToNext = std::max(1, period);
            }

            y = a * y + (1.0f - a) * target;
            return clampf(y, -1.0f, 1.0f);
        }
    };

    // ============================================================================
    // StereoSpinner (slow random stereo rotation)
    // ============================================================================

    struct StereoSpinner {
        float sr = 48000.0f;

        SmoothNoise noise;
        float smoothed = 0.0f;
        float a = 0.0f;

        void init(float sampleRate, uint32_t seed) {
            sr = sampleRate;

            noise.setSampleRate(sr);
            noise.seed(seed);

            noise.setRateHz(0.15f);
            noise.setSmoothMs(250.0f);

            setSmoothMs(250.0f);
            smoothed = 0.0f;
        }

        void setRateHz(float hz) { noise.setRateHz(hz); }
        void setNoiseSmoothMs(float ms) { noise.setSmoothMs(ms); }

        void setSmoothMs(float ms) {
            ms = std::max(ms, 1.0f);
            float sec = ms * 0.001f;
            a = std::exp(-1.0f / (sec * sr));
        }

        void clear() { noise.clear(); smoothed = 0.0f; }

        float process() {
            float n = noise.process();
            smoothed = a * smoothed + (1.0f - a) * n;
            return clampf(smoothed, -1.0f, 1.0f);
        }

        static void rotate(float& L, float& R, float angleRad) {
            float c = std::cos(angleRad);
            float s = std::sin(angleRad);
            float newL = c * L - s * R;
            float newR = s * L + c * R;
            L = newL;
            R = newR;
        }
    };

} // namespace dsp
#pragma once
