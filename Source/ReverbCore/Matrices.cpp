#include "Matrices.h"

/*
  =============================================================================
  Matrices.cpp — Big Pi Mixing Matrices (implementation)
  =============================================================================

  These matrix mixes are run inside the reverb tank every sample.

  Key design goal:
    - Mix delay line outputs without changing total energy drastically.

  If mixing changes energy too much, you get:
    - unexpected gain build-up (possible runaway)
    - or tail collapsing (too quiet)

  For Hadamard:
    - The raw Hadamard transform increases magnitude by sqrt(N).
    - So we scale by 1/sqrt(N) to keep energy roughly preserved.

  For Householder:
    - The reflection is orthogonal (energy-preserving) by construction.
    - We compute mean and apply y[i] = x[i] - 2*mean.

  Notes on numerical safety:
    - lines is expected to be 8 or 16.
    - if lines is not a supported size, we still do something sensible.
*/

namespace bigpi::core {

    void hadamardMix(std::array<float, kMaxLines>& v, int lines) {
        lines = std::max(1, std::min(lines, kMaxLines));

        // Hadamard transform is best-defined for power-of-two sizes.
        // Big Pi uses 8 or 16.
        // We’ll perform a standard in-place fast Walsh-Hadamard transform (FWHT).
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
        float scale = 1.0f / std::sqrt(float(lines));
        for (int i = 0; i < lines; ++i) {
            v[i] *= scale;
        }
    }

    void householderMix(std::array<float, kMaxLines>& v, int lines) {
        lines = std::max(1, std::min(lines, kMaxLines));

        // Compute mean of the vector.
        float sum = 0.0f;
        for (int i = 0; i < lines; ++i) sum += v[i];

        float mean = sum / float(lines);

        // Householder reflection with u=[1,1,...,1]:
        // y[i] = x[i] - 2*mean
        for (int i = 0; i < lines; ++i) {
            v[i] = v[i] - 2.0f * mean;
        }

        // Householder is orthogonal (energy-preserving), so no scaling needed.
    }

} // namespace bigpi::core
