#pragma once
/*
  =============================================================================
  TapPatterns.h — Big Pi Tank Output Tap Patterns
  =============================================================================
*/

#include <array>
#include <algorithm>

#include "dsp/tail/Matrices.h" // provides bigpi::core::kMaxLines

namespace bigpi::core {

    // ============================================================================
    // Pattern selection and rendering
    // ============================================================================

    /*
      renderTapPattern(y, lines, patternId, wetL, wetR)
      ------------------------------------------------
      - y: delay-line outputs (size kMaxLines, but only first `lines` are valid)
      - lines: typically 8 or 16
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

    inline void renderMorphingPattern(const std::array<float, kMaxLines>& y,
        int lines,
        int patternA,
        int patternB,
        float morph01,
        float& wetL,
        float& wetR)
    {
        morph01 = std::max(0.0f, std::min(1.0f, morph01));

        // Safety clamp
        lines = std::max(0, std::min(lines, kMaxLines));
        if (lines <= 0) { wetL = 0.0f; wetR = 0.0f; return; }

        float aL = 0.0f, aR = 0.0f;
        float bL = 0.0f, bR = 0.0f;

        renderTapPattern(y, lines, patternA, aL, aR);
        renderTapPattern(y, lines, patternB, bL, bR);

        wetL = (1.0f - morph01) * aL + morph01 * bL;
        wetR = (1.0f - morph01) * aR + morph01 * bR;
    }

} // namespace bigpi::core

