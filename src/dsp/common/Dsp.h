#pragma once
/*
  =============================================================================
  Dsp.h — Big Pi DSP Primitives (header-only, heavily annotated)
  =============================================================================
*/

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring> // std::memcpy

namespace dsp {

    // ============================================================================
    // Constants
    // ============================================================================

    constexpr float kPi = 3.14159265358979323846f;

    // ============================================================================
    // Basic helpers
    // ============================================================================

    template <typename T>
    inline T clamp(T x, T lo, T hi) {
        return std::max(lo, std::min(hi, x));
    }

    // Convenience overload for float.
    // (We keep this because most DSP parameters are float.)
    inline float clampf(float x, float lo, float hi) {
        return clamp<float>(x, lo, hi);
    }

    // Handy for UI parameters.
    inline float clamp01(float x) {
        return clampf(x, 0.0f, 1.0f);
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

    inline float curve01(float x01, float shape) {
        x01 = clampf(x01, 0.0f, 1.0f);
        shape = clampf(shape, 0.05f, 10.0f);
        return std::pow(x01, shape);
    }

    // Denormal helper (optional but useful for reverbs)
    inline float killDenorm(float x) {
        return (std::abs(x) < 1e-20f) ? 0.0f : x;
    }

    // ============================================================================
    // Parameter smoothing
    // ============================================================================

    struct SmoothValue {
        float y = 0.0f;
        float a = 0.0f;
        float sr = 48000.0f;

        void setTimeMs(float ms, float sampleRate) {
            sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;
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
    // One-pole filters
    // ============================================================================

    struct OnePoleLP {
        float z = 0.0f;
        float a = 0.0f;

        void clear() { z = 0.0f; }

        void setCutoff(float hz, float sr) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            a = std::exp(-2.0f * kPi * hz / sr);
        }

        float process(float x) {
            z = a * z + (1.0f - a) * x;
            z = killDenorm(z);
            return z;
        }
    };

    struct OnePoleHP {
        float z = 0.0f;
        float a = 0.0f;

        void clear() { z = 0.0f; }

        void setCutoff(float hz, float sr) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            a = std::exp(-2.0f * kPi * hz / sr);
        }

        float process(float x) {
            z = a * z + (1.0f - a) * x;
            z = killDenorm(z);
            return x - z;
        }
    };

    // ============================================================================
    // Biquad (DF2T)
    // ============================================================================

    struct Biquad {
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
        float a1 = 0.0f, a2 = 0.0f;
        float z1 = 0.0f, z2 = 0.0f;

        void clear() { z1 = 0.0f; z2 = 0.0f; }

        float process(float x) {
            float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;

            z1 = killDenorm(z1);
            z2 = killDenorm(z2);
            return y;
        }

        static void omega(float hz, float sr, float& w0, float& c, float& s) {
            hz = clampf(hz, 5.0f, 0.49f * sr);
            w0 = 2.0f * kPi * (hz / sr);
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
    // Envelope follower
    // ============================================================================

    struct EnvelopeFollower {
        float sr = 48000.0f;
        float env = 0.0f;
        float aAtk = 0.0f;
        float aRel = 0.0f;

        void setSampleRate(float sampleRate) {
            sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;
        }

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
            env = killDenorm(env);
            return env;
        }
    };

    // ============================================================================
    // Allpass diffuser
    // ============================================================================

    struct Allpass {
        std::vector<float> buf;
        int idx = 0;

        float g = 0.7f;
        float delaySamp = 200.0f;

        // Real-time rule: call init() only in prepare(), not in per-sample code.
        void init(int maxDelaySamples) {
            buf.assign(std::max(1, maxDelaySamples), 0.0f);
            idx = 0;
        }

        void clear() {
            std::fill(buf.begin(), buf.end(), 0.0f);
            idx = 0;
        }

        float process(float x) {
            if (buf.size() < 2) return x;

            int d = int(clampf(delaySamp, 1.0f, float(buf.size() - 1)));

            int r = idx - d;
            if (r < 0) r += (int)buf.size();

            float v = buf[r];

            float y = -g * x + v;
            buf[idx] = x + g * y;

            idx++;
            if (idx >= (int)buf.size()) idx = 0;

            return y;
        }
    };

    // ============================================================================
    // Delay line with cubic interpolation
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
            if (buf.empty()) return;
            buf[w] = x;
            w++;
            if (w >= (int)buf.size()) w = 0;
        }

        float readFracCubic(float delaySamples) const {
            if (buf.size() < 4) return 0.0f;

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
    // Multi LFO bank
    // ============================================================================

    struct MultiLFO {
        int count = 0;
        float sr = 48000.0f;

        std::vector<float> phase;
        std::vector<float> rateMul;

        void init(int n, float sampleRate) {
            count = std::max(1, n);
            sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;

            phase.assign(count, 0.0f);
            rateMul.assign(count, 1.0f);

            for (int i = 0; i < count; ++i) {
                float t = (count == 1) ? 0.0f : float(i) / float(count - 1);
                rateMul[i] = 0.85f + 0.30f * t;
                phase[i] = (2.0f * kPi) * (t + 0.13f);
            }
        }

        float process(int i, float baseRateHz) {
            i = std::max(0, std::min(i, count - 1));

            float hz = baseRateHz * rateMul[i];
            float inc = (2.0f * kPi * hz) / sr;

            float y = std::sin(phase[i]);

            phase[i] += inc;
            if (phase[i] >= 2.0f * kPi) phase[i] -= 2.0f * kPi;

            return y;
        }
    };

    // ============================================================================
    // Soft saturation
    // ============================================================================

    inline float softSat(float x, float drive) {
        drive = clampf(drive, 0.0f, 10.0f);

        float y = x * (1.0f + drive);
        y = std::tanh(y);

        float norm = 1.0f / std::tanh(1.0f + drive);
        return y * norm;
    }

    // ============================================================================
    // SmoothNoise
    // ============================================================================

    struct SmoothNoise {
        float sr = 48000.0f;
        uint32_t rng = 0x12345678u;

        float y = 0.0f;
        float target = 0.0f;

        float rateHz = 0.5f;
        int samplesToNext = 1;

        float a = 0.0f;

        void setSampleRate(float sampleRate) {
            sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;
        }

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
            rng = 1664525u * rng + 1013904223u;

            // Safe bit-cast to float in [1, 2)
            uint32_t bits = (rng >> 9) | 0x3F800000u;
            float f;
            std::memcpy(&f, &bits, sizeof(float));
            f -= 1.0f;               // [0, 1)
            return 2.0f * f - 1.0f;  // [-1, 1)
        }

        float process() {
            samplesToNext--;
            if (samplesToNext <= 0) {
                target = nextRandBipolar();
                int period = int(sr / rateHz);
                samplesToNext = std::max(1, period);
            }

            y = a * y + (1.0f - a) * target;
            y = killDenorm(y);
            return clampf(y, -1.0f, 1.0f);
        }
    };

    // ============================================================================
    // StereoSpinner
    // ============================================================================

    struct StereoSpinner {
        float sr = 48000.0f;

        SmoothNoise noise;
        float smoothed = 0.0f;
        float a = 0.0f;

        void init(float sampleRate, uint32_t seed) {
            sr = (sampleRate <= 1.0f) ? 48000.0f : sampleRate;

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
            smoothed = killDenorm(smoothed);
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
