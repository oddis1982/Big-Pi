#pragma once
/*
  =============================================================================
  Matrices.h — Big Pi Mixing Matrices (FDN cross-mix)
  =============================================================================

  In Big Pi, the late reverb is built from multiple delay lines in a feedback
  network (an FDN-like structure). Each sample, we:

    1) read each delay line output -> y[i]
    2) mix y[] through a matrix -> m[]
    3) filter and apply feedback -> write back into delay lines

  Why mixing matrices matter:
  --------------------------
  If each delay line fed back only to itself, the system would produce strong,
  repeating resonances (metallic ringing). A mixing matrix spreads energy across
  lines so that reflections become dense and smooth.

  Two classic choices:
    - Hadamard matrix:
        * very diffusive, good at energy spreading
    - Householder matrix:
        * energy-preserving and diffusive
        * tends to sound slightly “smoother” in many reverbs
*/

#include <array>
#include <algorithm>

namespace bigpi::core {

    // We support up to 16 delay lines in HQ mode.
    // (C++17 inline constexpr avoids multiple-definition surprises across TUs.)
    inline constexpr int kMaxLines = 16;

    // ============================================================================
    // Hadamard mix
    // ============================================================================

    // Applies an in-place Hadamard transform to the first `lines` elements.
    // NOTE: Hadamard requires lines to be a power of two (8 or 16 in our project).
    void hadamardMix(std::array<float, kMaxLines>& v, int lines);

    // ============================================================================
    // Householder mix
    // ============================================================================

    // Applies a Householder reflection with u = [1,1,...,1].
    void householderMix(std::array<float, kMaxLines>& v, int lines);

    // ============================================================================
    // Convenience wrapper
    // ============================================================================

    enum class MatrixType : int {
        Hadamard = 0,
        Householder = 1
    };

    inline void mix(std::array<float, kMaxLines>& v, int lines, MatrixType type) {
        // Safety clamp: prevents out-of-range access if caller misconfigures.
        lines = std::max(0, std::min(lines, kMaxLines));

        if (lines <= 1) return;

        if (type == MatrixType::Hadamard) hadamardMix(v, lines);
        else householderMix(v, lines);
    }

} // namespace bigpi::core

