# Big Pi Development Checklist

Based on `docs/RoadMap.txt` current status (Phase 0 complete, entering Phase 1).

## Current Focus — Phase 1 (Staple Modes)

### Mode definitions
- [ ] Finalize **Room** delay set + diffusion map
- [ ] Finalize **Hall** delay set + diffusion map
- [ ] Finalize **Cathedral** delay set + diffusion map
- [ ] Finalize **Plate** delay set + diffusion map
- [ ] Finalize **Vintage (PCM-style)** delay set + diffusion map
- [ ] Finalize **Sky (Cloud-ish)** delay set + diffusion map

### Per-mode voicing
- [ ] Tune per-mode modulation maps
- [ ] Tune multiband decay behavior per mode
- [ ] Redesign/retune early reflections per mode
- [ ] Tune OutputStage voicing per mode

### Global refinement for Phase 1
- [ ] Refine dynamic damping behavior
- [ ] Set stereo width defaults per mode
- [ ] Tune loudness compensation between presets

### Phase 1 exit criteria
- [ ] Listening pass: all staple modes are coherent and musical
- [ ] CPU sanity check across all staple modes
- [ ] Regression WAV renders updated

---

## Next Up — Phase 2 (Core Engine Quality Lift)
- [ ] Add smoothing for sensitive parameters
- [ ] Implement dynamic diffusion mapping via tank envelope
- [ ] Add optional OutputStage safety limiter
- [ ] Improve predelay filtering
- [ ] Enhance ducking curve
- [ ] Add denormal protection
- [ ] Add CPU profiling per mode
- [ ] Define block-level performance budget

---

## Future Milestones (Placeholder)
- [ ] Phase 3: Spring mode
- [ ] Phase 4: Blossom mode
- [ ] Phase 5: Shimmer mode
- [ ] Phase 6: Magnetic mode
- [ ] Phase 7: Granular mode
- [ ] Phase 8: Singularity mode
- [ ] Phase 9: MicroCosmos mode (specified, not implemented)
- [ ] Phase 10: Productization

---

## Continuous Tasks (All Phases)
- [ ] Listening regression WAV renders
- [ ] CPU profiling
- [ ] Stability checks
- [ ] Parameter safety clamps
- [ ] Documentation updates
- [ ] Preserve modular separation
