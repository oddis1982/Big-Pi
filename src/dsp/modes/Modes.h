#pragma once
/*
  =============================================================================
  Modes.h — Big Pi Mode Definitions
  =============================================================================

  This file defines the complete list of Big Pi modes.

  We separate modes conceptually into three categories:

  1) Staple reverbs (bread-and-butter spaces)
     - Room
     - Hall
     - Cathedral
     - Plate
     - Spring
     - Vintage (PCM70-style)

  2) Atmospheric / Signature modes
     - Sky (Cloud-inspired)
     - Blossom (Bloom-inspired)
     - Shimmer
     - Magnetic (Magneto-inspired)

  3) Experimental / Advanced engines
     - Granular
     - Singularity (Blackhole-inspired)
     - MicroCosmic (Microcosm-inspired)

  Important architectural decision:
  ----------------------------------
  A Mode is NOT a separate reverb engine.
  A Mode is a configuration + optional DSP blocks.

  Later:
    ModePresets.cpp will map each mode to:
      - Tank configuration
      - Diffusion setup
      - Modulation maps
      - Optional feature flags (pitch block, granular block, etc.)
*/

namespace bigpi {

    // ============================================================================
    // Mode Enumeration
    // ============================================================================

    enum class Mode : int {

        // --------------------------------------------------------------------------
        // Staple reverbs
        // --------------------------------------------------------------------------

        Room = 0,
        Hall,
        Cathedral,
        Plate,
        Spring,
        Vintage,

        // --------------------------------------------------------------------------
        // Atmospheric / spacious
        // --------------------------------------------------------------------------

        Sky,        // BigSky Cloud-inspired
        Blossom,    // Bloom-style swell reverb
        Shimmer,    // Pitch-shifted feedback
        Magnetic,   // Tape-space hybrid

        // --------------------------------------------------------------------------
        // Experimental / special engines
        // --------------------------------------------------------------------------

        Granular,
        Singularity,
        MicroCosmic,

        Count
    };

    // ============================================================================
    // Mode Name Helper
    // ============================================================================

    inline const char* modeToString(Mode m) {
        switch (m) {

        case Mode::Room:        return "Room";
        case Mode::Hall:        return "Hall";
        case Mode::Cathedral:   return "Cathedral";
        case Mode::Plate:       return "Plate";
        case Mode::Spring:      return "Spring";
        case Mode::Vintage:     return "Vintage";

        case Mode::Sky:         return "Sky";
        case Mode::Blossom:     return "Blossom";
        case Mode::Shimmer:     return "Shimmer";
        case Mode::Magnetic:    return "Magnetic";

        case Mode::Granular:    return "Granular";
        case Mode::Singularity: return "Singularity";
        case Mode::MicroCosmic: return "MicroCosmic";

        case Mode::Count:       return "Count";

        default:                return "Unknown";
        }
    }

    // ============================================================================
    // Mode Category Helpers
    // ============================================================================

    inline bool isStaple(Mode m) {
        return m == Mode::Room
            || m == Mode::Hall
            || m == Mode::Cathedral
            || m == Mode::Plate
            || m == Mode::Spring
            || m == Mode::Vintage;
    }

    inline bool isAtmospheric(Mode m) {
        return m == Mode::Sky
            || m == Mode::Blossom
            || m == Mode::Shimmer
            || m == Mode::Magnetic;
    }

    inline bool isExperimental(Mode m) {
        return m == Mode::Granular
            || m == Mode::Singularity
            || m == Mode::MicroCosmic;
    }

} // namespace bigpi

