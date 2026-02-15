#pragma once
/*
  =============================================================================
  Version.h — Big Pi build stamp
  =============================================================================

  Why this exists:
    Big Pi is developed iteratively (Alpha → Delta → Kappa…).
    When you run the test harness, it should print the build ID so you can
    instantly confirm you’re running the code you think you are.

  Workflow rule:
    Every time we ship a new “bundle” iteration, we update REVERB_BUILD_ID.

  Current state:
    This modular project starts at the latest Kappa-level feature set we built
    (multiband decay coloration, late diffusion refinement, per-mode mod maps,
     loudness compensation, etc.).
*/

#define REVERB_BUILD_ID "kappa.modular.1"
#pragma once
