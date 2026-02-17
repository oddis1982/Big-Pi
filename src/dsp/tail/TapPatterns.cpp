#include "TapPatterns.h"

#include <algorithm> // std::min, std::max

/*
  =============================================================================
  TapPatterns.cpp — Big Pi Tank Output Tap Patterns (implementation)
  =============================================================================
*/

namespace bigpi::core {

    // Helper: safe modulo that works for negative values.
    static inline int wrapIndex(int i, int n) {
        if (n <= 0) return 0;
        i %= n;
        if (i < 0) i += n;
        return i;
    }

    // Pattern 0: Wide balanced (good default)
    static void pattern0(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        static const int tapsL[] = { 0, 2, 5, 7, 9, 12, 14 };
        static const int tapsR[] = { 1, 3, 4, 6, 10, 13, 15 };

        float sumL = 0.0f, sumR = 0.0f;

        const int nL = int(sizeof(tapsL) / sizeof(tapsL[0]));
        const int nR = int(sizeof(tapsR) / sizeof(tapsR[0]));

        for (int t = 0; t < nL; ++t) {
            int idx = wrapIndex(tapsL[t], lines);
            float s = (t & 1) ? -1.0f : 1.0f;
            sumL += s * y[idx];
        }

        for (int t = 0; t < nR; ++t) {
            int idx = wrapIndex(tapsR[t], lines);
            float s = (t & 1) ? 1.0f : -1.0f; // opposite sign sequence
            sumR += s * y[idx];
        }

        // Normalize by number of taps.
        L = sumL * (1.0f / float(nL));
        R = sumR * (1.0f / float(nR));
    }

    // Pattern 1: More centered (less extreme width)
    static void pattern1(const std::array<float, kMaxLines>& y, int lines, float& L, float& R) {
        static const int taps[] = { 0, 3, 5, 8, 11, 13 };
        const int n = int(sizeof(taps) / sizeof(taps[0]));

        float sumL = 0.0f, sumR = 0.0f;

        for (int t = 0; t < n; ++t) {
            int idx = wrapIndex(taps[t], lines);

            float sL = (t & 1) ? -1.0f : 1.0f;
            float sR = (t & 1) ? 1.0f : -1.0f;

            sumL += sL * y[idx];
            sumR += sR * y[idx];
        }

        L = sumL * (1.0f / float(n));
        R = sumR * (1.0f / float(n));
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
        float sumL = 0.0f, sumR = 0.0f;

        int tapsL = 0;
        int tapsR = 0;

        // L sums even indices, R sums odd indices (with opposite sign relationship)
        for (int i = 0; i < lines; ++i) {
            float s = (i & 1) ? -1.0f : 1.0f;

            if ((i & 1) == 0) {
                sumL += s * y[i];
                tapsL++;
            }
            else {
                sumR += (-s) * y[i];
                tapsR++;
            }
        }

        float normL = (tapsL > 0) ? (1.0f / float(tapsL)) : 1.0f;
        float normR = (tapsR > 0) ? (1.0f / float(tapsR)) : 1.0f;

        L = sumL * normL;
        R = sumR * normR;
    }

    void renderTapPattern(const std::array<float, kMaxLines>& y,
        int lines,
        int patternId,
        float& wetL,
        float& wetR)
    {
        lines = std::max(1, std::min(lines, kMaxLines));

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
