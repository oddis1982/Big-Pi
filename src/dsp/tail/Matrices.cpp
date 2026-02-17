#include "Matrices.h"

#include <algorithm> // std::min, std::max
#include <cmath>     // std::sqrt

/*
  =============================================================================
  Matrices.cpp — Big Pi Mixing Matrices (implementation)
  =============================================================================
*/

namespace bigpi::core {

    static inline bool isPowerOfTwo(int x) {
        return (x > 0) && ((x & (x - 1)) == 0);
    }

    void hadamardMix(std::array<float, kMaxLines>& v, int lines) {
        lines = std::max(1, std::min(lines, kMaxLines));

        // Hadamard transform is properly defined for power-of-two sizes.
        // Big Pi uses 8 or 16. If misconfigured, fall back to a safe mixer.
        if (!isPowerOfTwo(lines)) {
            householderMix(v, lines);
            return;
        }

        // In-place fast Walsh-Hadamard transform (FWHT)
        for (int step = 1; step < lines; step <<= 1) {
            for (int i = 0; i < lines; i += (step << 1)) {
                for (int j = 0; j < step; ++j) {
                    float a = v[i + j];
                    float b = v[i + j + step];
                    v[i + j] = a + b;
                    v[i + j + step] = a - b;
                }
            }
        }

        // Normalize to keep overall energy roughly constant.
        // scale = 1/sqrt(N)
        float scale = 1.0f;
        if (lines == 8)      scale = 0.3535533905932738f; // 1/sqrt(8)
        else if (lines == 16) scale = 0.25f;              // 1/sqrt(16)
        else                  scale = 1.0f / std::sqrt(float(lines));

        for (int i = 0; i < lines; ++i) {
            v[i] *= scale;
        }
    }

    void householderMix(std::array<float, kMaxLines>& v, int lines) {
        lines = std::max(1, std::min(lines, kMaxLines));

        float sum = 0.0f;
        for (int i = 0; i < lines; ++i) sum += v[i];

        float mean = sum / float(lines);

        // Householder reflection with u=[1,1,...,1]:
        // y[i] = x[i] - 2*mean
        for (int i = 0; i < lines; ++i) {
            v[i] = v[i] - 2.0f * mean;
        }
    }

} // namespace bigpi::core
