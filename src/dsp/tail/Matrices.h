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
        * can feel bright/plate-like when tuned that way
    - Householder matrix:
        * also energy-preserving and diffusive
        * tends to sound slightly “smoother” in many reverbs

  Performance:
    Both operations here are O(N log N) or O(N) style operations for N=8 or 16,
    and are perfectly fine for real-time audio.
*/

#include <array>
#include <algorithm>
#include <cmath>

namespace bigpi::core {

    // We support up to 16 delay lines in HQ mode.
    static constexpr int kMaxLines = 16;

    // ============================================================================
    // Hadamard mix
    // ============================================================================

    /*
      hadamardMix(v, lines)
      ---------------------
      Applies an in-place Hadamard transform to the first `lines` elements.

      Property:
        - For power-of-two sizes (8 or 16), Hadamard is orthogonal up to a scale.
        - If we scale by 1/sqrt(N), it becomes energy-preserving.

      Why this helps in reverb:
        - It produces strong diffusion (rapid echo densification).
    */
    void hadamardMix(std::array<float, kMaxLines>& v, int lines);

    // ============================================================================
    // Householder mix
    // ============================================================================

    /*
      householderMix(v, lines)
      ------------------------
      Applies a Householder reflection:

          y = x - 2 * (u^T x) / (u^T u) * u

      For our special case, choose u = [1,1,1,...,1].
      That makes it:

          mean = sum(x)/N
          y[i] = x[i] - 2*mean

      This is extremely cheap (just sum + subtract).

      Properties:
        - Orthogonal (energy-preserving)
        - Strong mixing effect

      Why this helps:
        - Similar to Hadamard, but with a different “spread” character.
    */
    void householderMix(std::array<float, kMaxLines>& v, int lines);

    // ============================================================================
    // Convenience wrapper
    // ============================================================================

    enum class MatrixType : int {
        Hadamard = 0,
        Householder = 1
    };

    inline void mix(std::array<float, kMaxLines>& v, int lines, MatrixType type) {
        if (type == MatrixType::Hadamard) hadamardMix(v, lines);
        else householderMix(v, lines);
    }

} // namespace bigpi::core
#pragma once
