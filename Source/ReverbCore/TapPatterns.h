#pragma once
/*
  =============================================================================
  TapPatterns.h — Big Pi Tank Output Tap Patterns
  =============================================================================

  The reverb tank produces multiple delay-line outputs y[i].

  To produce stereo output (wetL, wetR), we don't just take one line.
  We "listen" to a mixture of lines.

  Why tap patterns matter:
  ------------------------
  - Different tap combinations change:
      * stereo image
      * density impression
      * tonal coloration
      * perceived size

  - By changing patterns over time (pattern morph), we can add subtle movement
    and prevent static "standing wave" impressions.

  This file provides:
    - a function that maps delay outputs y[] -> wetL/wetR for a chosen pattern
    - a function for morphing between patterns (future-friendly)

  Note:
    Patterns here are intentionally simple and deterministic.
    The "secret sauce" comes from:
      - mixing matrix
      - diffusion
      - modulation
      - frequency-dependent decay
*/

#include <array>
#include <algorithm>

namespace bigpi::core {

    static constexpr int kMaxLines = 16;

    // ============================================================================
    // Pattern selection and rendering
    // ============================================================================

    /*
      renderTapPattern(y, lines, patternId, wetL, wetR)
      ------------------------------------------------
      - y: delay-line outputs (size 16, but only first `lines` are valid)
      - lines: 8 or 16
      - patternId: selects a pre-defined mix pattern
      - wetL/wetR: outputs

      Implementation idea:
        - Each pattern is a small list of indices and signs.
        - We sum selected lines with sign flips to create decorrelated stereo.
    */
    void renderTapPattern(const std::array<float, kMaxLines>& y,
        int lines,
        int patternId,
        float& wetL,
        float& wetR);

    // ============================================================================
    // Pattern morph helper
    // ============================================================================

    /*
      renderMorphingPattern(...)
      -------------------------
      Blends between two patterns.

      morph01:
        0.0 => patternA only
        1.0 => patternB only

      This supports “pattern morphing” features (Delta.9-ish idea):
        - subtle evolution as tail “ages”
        - movement without pitch wobble
    */
    inline void renderMorphingPattern(const std::array<float, kMaxLines>& y,
        int lines,
        int patternA,
        int patternB,
        float morph01,
        float& wetL,
        float& wetR) {
        morph01 = std::max(0.0f, std::min(1.0f, morph01));

        float aL = 0.0f, aR = 0.0f;
        float bL = 0.0f, bR = 0.0f;

        renderTapPattern(y, lines, patternA, aL, aR);
        renderTapPattern(y, lines, patternB, bL, bR);

        wetL = (1.0f - morph01) * aL + morph01 * bL;
        wetR = (1.0f - morph01) * aR + morph01 * bR;
    }

} // namespace bigpi::core
#pragma once
