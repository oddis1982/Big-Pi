#include "TapPatterns.h"

/*
  =============================================================================
  TapPatterns.cpp — Big Pi Tank Output Tap Patterns (implementation)
  =============================================================================

  Patterns:
    We define a few "good sounding" patterns that combine delay-line outputs.

  Goals:
    - Create decorrelated stereo (L and R not identical)
    - Avoid too much cancellation (keep energy stable)
    - Provide different "characters" (wide/centered/airy)

  Implementation style:
    - Each pattern is expressed as index/sign lists.
    - We normalize by number of taps to keep levels consistent.

  Note:
    This is intentionally simple. The overall reverb sound character is
    dominated by the tank, diffusion, modulation, and damping.
*/

namespace bigpi::core {

    // Helper: safe modulo that works for negative values.
    static inline int wrapIndex(int i, int n) {
        if (n <= 0) return 0;
        i %= n;
        if (i < 0) i += n;
        return i;
    }

    /*
      We define patterns using a base "tap index list" and sign alternation.
      For stereo decorrelation, we use different index sets for L and R.
    */

    // Pattern 0: Wide balanced (good default)
    static void pattern0(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        // Choose taps spread around the network
        // Use sign flips to reduce correlation.
        static const int tapsL[] = { 0, 2, 5, 7, 9, 12, 14 };
        static const int tapsR[] = { 1, 3, 4, 6, 10, 13, 15 };

        float sumL = 0.0f, sumR = 0.0f;

        for (int t = 0; t < (int)(sizeof(tapsL) / sizeof(tapsL[0])); ++t) {
            int idx = wrapIndex(tapsL[t], lines);
            float s = (t & 1) ? -1.0f : 1.0f;
            sumL += s * y[idx];
        }

        for (int t = 0; t < (int)(sizeof(tapsR) / sizeof(tapsR[0])); ++t) {
            int idx = wrapIndex(tapsR[t], lines);
            float s = (t & 1) ? 1.0f : -1.0f; // opposite sign sequence to decorrelate
            sumR += s * y[idx];
        }

        // Normalize by number of taps.
        L = sumL * (1.0f / 7.0f);
        R = sumR * (1.0f / 7.0f);
    }

    // Pattern 1: More centered (less extreme width)
    static void pattern1(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        static const int taps[] = { 0, 3, 5, 8, 11, 13 };

        float sumL = 0.0f, sumR = 0.0f;

        for (int t = 0; t < (int)(sizeof(taps) / sizeof(taps[0])); ++t) {
            int idx = wrapIndex(taps[t], lines);

            // L uses alternating sign; R uses same taps but shifted sign pattern
            float sL = (t & 1) ? -1.0f : 1.0f;
            float sR = (t & 1) ? 1.0f : -1.0f;

            sumL += sL * y[idx];
            sumR += sR * y[idx];
        }

        L = sumL * (1.0f / 6.0f);
        R = sumR * (1.0f / 6.0f);
    }

    // Pattern 2: Airy / scattered (lighter, more “sparkly”)
    static void pattern2(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        static const int tapsL[] = { 2, 6, 9, 12 };
        static const int tapsR[] = { 1, 7, 10, 15 };

        float sumL = 0.0f, sumR = 0.0f;

        for (int t = 0; t < 4; ++t) {
            int idx = wrapIndex(tapsL[t], lines);
            float s = (t & 1) ? -1.0f : 1.0f;
            sumL += s * y[idx];
        }

        for (int t = 0; t < 4; ++t) {
            int idx = wrapIndex(tapsR[t], lines);
            float s = (t & 1) ? 1.0f : -1.0f;
            sumR += s * y[idx];
        }

        L = sumL * 0.25f;
        R = sumR * 0.25f;
    }

    // Pattern 3: Very wide / aggressive decorrelation
    static void pattern3(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        // Use many taps and heavy sign flipping.
        float sumL = 0.0f, sumR = 0.0f;

        // L sums even indices, R sums odd indices (with alternating signs)
        int taps = 0;

        for (int i = 0; i < lines; ++i) {
            float s = (i & 1) ? -1.0f : 1.0f;

            if ((i & 1) == 0) {
                sumL += s * y[i];
                taps++;
            }
            else {
                sumR += (-s) * y[i]; // opposite sign relationship
            }
        }

        // Normalize:
        float norm = (taps > 0) ? (1.0f / float(taps)) : 1.0f;
        L = sumL * norm;
        R = sumR * norm;
    }

    void renderTapPattern(const std::array<float, kMaxLines>& y,
        int lines,
        int patternId,
        float& wetL,
        float& wetR) {
        lines = std::max(1, std::min(lines, kMaxLines));

        // Normalize patternId so any integer maps to a valid pattern.
        int pid = patternId % 4;
        if (pid < 0) pid += 4;

        wetL = 0.0f;
        wetR = 0.0f;

        switch (pid) {
        default:
        case 0: pattern0(y, lines, wetL, wetR); break;
        case 1: pattern1(y, lines, wetL, wetR); break;
        case 2: pattern2(y, lines, wetL, wetR); break;
        case 3: pattern3(y, lines, wetL, wetR); break;
        }
    }

} // namespace bigpi::core
